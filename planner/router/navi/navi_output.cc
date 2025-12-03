#include "onboard/planner/router/navi/navi_output.h"

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type

#include <algorithm>
#include <iterator>
#include <optional>
#include <utility>

#include "google/protobuf/repeated_ptr_field.h"

#include "onboard/maps/map_or_die_macros.h"
#include "onboard/maps/proto/semantic_map.pb.h"
#include "onboard/maps/semantic_map_util.h"

namespace qcraft::planner::route {
namespace {
NaviActionProto MakeNavAction(
    const std::optional<NaviActionProto::NaviActionType>& nav_action_type,
    const std::optional<NaviActionProto::SubNaviActionType>& sub_action_type,
    const Vec2d& point, mapping::SectionId /*section_id*/) {
  NaviActionProto nav_action_proto;
  if (nav_action_type.has_value()) {
    nav_action_proto.set_navi_action_type(*nav_action_type);
  }
  if (sub_action_type.has_value()) {
    nav_action_proto.set_sub_action_type(*sub_action_type);
  }
  point.ToProto(nav_action_proto.mutable_start_point());
  return nav_action_proto;
}

std::optional<NaviActionProto::NaviActionType> GetNavActionType(
    const mapping::LaneProto& lane_proto,
    const mapping::LaneProto* last_lane_proto_ptr) {
  if (last_lane_proto_ptr != nullptr &&
      lane_proto.direction() == last_lane_proto_ptr->direction()) {
    return std::nullopt;
  }
  switch (lane_proto.direction()) {
    case mapping::LaneProto::STRAIGHT:
      if (lane_proto.is_in_intersection()) {
        return NaviActionProto::STRAIGHT;
      }
      break;
    case mapping::LaneProto::LEFT_TURN:
      return NaviActionProto::TURN_LEFT;

    case mapping::LaneProto::RIGHT_TURN:
      return NaviActionProto::TURN_RIGHT;

    case mapping::LaneProto::UTURN:
      return NaviActionProto::U_TURN;
  }
  return std::nullopt;
}

std::optional<NaviActionProto::SubNaviActionType> GetSubNavActionType(
    const mapping::LaneProto& lane_proto,
    const mapping::LaneProto* last_lane_proto) {
  if ((lane_proto.type() == mapping::LaneProto_Type_RAMP ||
       lane_proto.type() == mapping::LaneProto_Type_RAMP_JCT) &&
      last_lane_proto != nullptr &&
      (last_lane_proto->type() != mapping::LaneProto_Type_RAMP ||
       lane_proto.type() == mapping::LaneProto_Type_RAMP_JCT)) {
    return NaviActionProto::ENTER_RAMP;
  }
  return std::nullopt;
}

absl::StatusOr<std::vector<NaviActionInfo>> GenDestinationNaviAction(
    const mapping::v2::SemanticMapManager& smm,
    const std::vector<mapping::ElementId>& lane_ids, const Vec2d& destination,
    const double start_fraction, const double end_fraction) {
  std::vector<NaviActionInfo> navi_action_infos;
  NaviActionInfo cur_navi_action_info;
  SMM_LANE_PROTO_OR_ERROR(lane_proto, smm, lane_ids[0]);
  cur_navi_action_info.navi_action =
      MakeNavAction(NaviActionProto::DESTINATION, std::nullopt, destination,
                    mapping::SectionId(lane_proto->section_id()));
  SMM_SECTION_PROTO_OR_ERROR(section_proto, smm, lane_proto->section_id());
  cur_navi_action_info.length =
      section_proto->average_length() * (end_fraction - start_fraction);
  navi_action_infos.push_back(std::move(cur_navi_action_info));
  return navi_action_infos;
}

}  // namespace

// TODO(xiang): Need to merge some straight navi infos beform transform navi
// infos. Now it is just std::transform the object.
template <typename NaviInfoIt>
void MergeNaviAction(
    NaviInfoIt first, NaviInfoIt last,
    google::protobuf::RepeatedPtrField<NaviSegmentProto>* navi_segments) {
  navi_segments->Clear();
  navi_segments->Reserve(std::distance(first, last));
  for (auto it = first; it != last; ++it) {
    auto* navi_segment_ptr = navi_segments->Add();
    navi_segment_ptr->set_length(it->length);
    *navi_segment_ptr->add_navi_actions() = it->navi_action;
  }
}

NaviPreviewProto CreateNaviPreview(
    const std::vector<NaviActionInfo>& navi_action_infos, int64_t update_id) {
  NaviPreviewProto navi_preview;
  MergeNaviAction(navi_action_infos.cbegin(), navi_action_infos.cend(),
                  navi_preview.mutable_navi_segments());
  navi_preview.set_update_id(update_id);
  return navi_preview;
}

NaviInstructionProto CreateCurrentNaviInstruction(
    const std::vector<NaviActionInfo>& navi_action_infos, int64_t update_id) {
  NaviInstructionProto navi_instruction_proto;
  if (navi_action_infos.empty()) {
    return navi_instruction_proto;
  }
  const auto& navi_action_info = navi_action_infos.front();
  navi_instruction_proto.set_distance(navi_action_info.length);
  navi_instruction_proto.set_update_id(update_id);
  *navi_instruction_proto.mutable_navi_action() = navi_action_info.navi_action;
  if (navi_action_infos.size() == 1) {
    navi_instruction_proto.set_is_last_navi_action(true);
  }
  return navi_instruction_proto;
}

absl::StatusOr<std::vector<NaviActionInfo>> CreateNaviActionInfoList(
    const mapping::v2::SemanticMapManager& smm,
    const std::vector<mapping::ElementId>& lane_ids, double start_fraction,
    double end_fraction, const Vec2d& destination) {
  std::vector<NaviActionInfo> navi_action_infos;
  NaviActionInfo cur_navi_action_info;

  if (lane_ids.size() == 1) {
    return GenDestinationNaviAction(smm, lane_ids, destination, start_fraction,
                                    end_fraction);
  }

  SMM_LANE_PROTO_OR_ERROR(final_lane_proto, smm, lane_ids.back());
  const auto final_section_id = final_lane_proto->section_id();

  SMM_LANE_PROTO_OR_ERROR(first_lane_proto, smm, lane_ids.front());
  const auto first_section_id = first_lane_proto->section_id();

  double length = 0.0;
  const mapping::LaneProto* last_lane_proto_ptr = nullptr;
  for (int i = 0; i < lane_ids.size(); ++i) {
    const auto lane_id = lane_ids[i];
    SMM_LANE_PROTO_OR_CONTINUE(lane_proto, smm, lane_id);
    auto action_type_op = GetNavActionType(*lane_proto, last_lane_proto_ptr);
    auto sub_type_op = GetSubNavActionType(*lane_proto, last_lane_proto_ptr);
    const bool has_lc =
        last_lane_proto_ptr == nullptr
            ? false
            : !mapping::IsOutgoingLane(smm, *last_lane_proto_ptr, lane_id);
    last_lane_proto_ptr = lane_proto;
    if (i == lane_ids.size() - 1) {
      // Reset the last navi action to DESTINATION
      action_type_op = NaviActionProto::DESTINATION;
    }
    if (!action_type_op.has_value() && !sub_type_op.has_value()) {
      // No need to split navi segment
      if (!has_lc) {
        const double cur_start_frac =
            lane_proto->section_id() == first_section_id ? start_fraction : 0.0;
        const double cur_end_frac =
            lane_proto->section_id() == final_section_id ? end_fraction : 1.0;
        SMM_SECTION_PROTO_OR_CONTINUE(section_proto, smm,
                                      lane_proto->section_id());
        length +=
            section_proto->average_length() * (cur_end_frac - cur_start_frac);
      }
      continue;
    }

    // A new navi segment
    const auto& geo_point = lane_proto->polyline().points()[0];

    cur_navi_action_info.navi_action =
        MakeNavAction(action_type_op, sub_type_op,
                      {geo_point.longitude(), geo_point.latitude()},
                      mapping::SectionId(lane_proto->section_id()));
    cur_navi_action_info.length = length;
    navi_action_infos.push_back(cur_navi_action_info);
    length = 0;
    cur_navi_action_info.length = 0.0;
    cur_navi_action_info.navi_action.Clear();
  }
  return navi_action_infos;
}

}  // namespace qcraft::planner::route
