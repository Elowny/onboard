#include "onboard/planner/router/noa/route_mpp.h"

#include <float.h>

#include <algorithm>
#include <cstddef>
#include <iterator>
#include <memory>
#include <ostream>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

// IWYU pragma: no_include <boost/date_time/time_defs.hpp>
#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/hash/hash.h"
#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_join.h"
#include "glog/logging.h"
#include "google/protobuf/repeated_ptr_field.h"

#include "common/proto/lane_point.pb.h"

#include "onboard/container/strong_int.h"
#include "onboard/lite/logging.h"
#include "onboard/lite/qissue_trans.h"
#include "onboard/maps/ehr/proto/extension.pb.h"
#include "onboard/maps/ehr/proto/odd.pb.h"
#include "onboard/maps/lane_point.h"
#include "onboard/maps/map_or_die_macros.h"
#include "onboard/maps/proto/semantic_map.pb.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/maps/v2/semantic_map_object.h"
#include "onboard/planner/router/navi/route_navi_info.h"
#include "onboard/planner/router/noa/hdmap_util.h"
#include "onboard/planner/router/road_conditions_process.h"
#include "onboard/planner/router/route_manager_util.h"
#include "onboard/planner/router/route_speed_limit_util.h"
#include "onboard/planner/router/router_defs.h"
#include "onboard/proto/adasis.pb.h"
#include "onboard/utils/source_location.h"
#include "onboard/utils/status_macros.h"

namespace qcraft::planner::route::noa {

namespace {

absl::StatusOr<RouteSectionSequenceProto> TruncateRouteMpp(
    const RouteSectionSequenceProto& route_section_sequence, int64_t section_id,
    double fraction) {
  const auto& sections = route_section_sequence.section_id();
  auto find_it = std::find(sections.begin(), sections.end(), section_id);
  if (find_it == sections.end()) {
    return absl::NotFoundError(
        absl::StrCat("Cannot find the section ", section_id));
  }

  RouteSectionSequenceProto cur_route;
  cur_route.mutable_section_id()->Reserve(
      std::distance(find_it, sections.end()));
  for (auto it = find_it; it != sections.end(); ++it) {
    cur_route.add_section_id(*it);
  }
  // Ignore the fraction check.
  cur_route.set_start_fraction(fraction);
  cur_route.set_end_fraction(route_section_sequence.end_fraction());
  return cur_route;
}

absl::StatusOr<std::pair<std::size_t, RouteSectionSequenceProto>>
UpdateCurrentRoute(
    const std::vector<PointToLaneT<mapping::v2::Lane>>& matched_lanes,
    const RouteSectionSequenceProto& mpp_route_section) {
  for (std::size_t i = 0; i < matched_lanes.size(); ++i) {
    const auto& matched_lane = matched_lanes[i];
    const auto nearest_section_id =
        matched_lane.lane_info->proto().section_id();
    if (auto cur_sections = TruncateRouteMpp(
            mpp_route_section, nearest_section_id, matched_lane.fraction);
        !cur_sections.ok()) {
      VLOG(3) << "The current matched lane cannot project to mpp :"
              << matched_lane.DebugString();
    } else {
      return std::pair<std::size_t, RouteSectionSequenceProto>(
          i, std::move(cur_sections).value());
    }
  }
  return absl::NotFoundError(
      absl::StrCat("Cannot update route by mpp, not found the section: ",
                   absl::StrJoin(mpp_route_section.section_id(), ",")));
}

std::vector<mapping::ElementId> AddNavInfoAvoidLanes(
    const mapping::v2::SemanticMapManager& smm,
    const RouteSectionSequenceProto& route_sections,
    const RestrictProto& restrict_proto) {
  absl::flat_hash_map<int64_t, int64_t> section_identifiers;
  for (auto section_id : route_sections.section_id()) {
    auto section = smm.FindSection(mapping::SectionId(section_id));
    if (section != nullptr && section->proto().lanes_size() > 0) {
      SMM_LANE_PROTO_OR_CONTINUE(lane_proto, smm, section->proto().lanes()[0]);
      if (lane_proto->has_third_party_lane_extend() &&
          lane_proto->third_party_lane_extend().has_lane_model_identifier()) {
        section_identifiers.insert(
            {lane_proto->third_party_lane_extend().lane_model_identifier(),
             section_id});
      }
    }
  }
  VLOG(3) << "section_identifiers: "
          << absl::StrJoin(section_identifiers, ",", absl::PairFormatter("="));
  std::vector<mapping::ElementId> avoid_lanes;
  for (const auto& restrict_lane : restrict_proto.restrict_lanes()) {
    if (const auto find_it =
            section_identifiers.find(restrict_lane.section_id());
        find_it != section_identifiers.end()) {
      auto section = smm.FindSection(mapping::SectionId(find_it->second));
      if (section == nullptr) {
        QLOG(INFO) << "Cannot find section: " << find_it->second
                   << ", lane model identifier: " << find_it->first;
        continue;
      }
      for (const auto& lane_weakptr : section->lanes()) {
        if (auto lane = lane_weakptr.lock();
            lane != nullptr && lane->proto().has_third_party_lane_extend() &&
            lane->proto().third_party_lane_extend().lane_identifier() ==
                restrict_lane.lane_seq()) {
          VLOG(3) << "avoid lane modifier: " << find_it->first
                  << ", section_id: " << find_it->second
                  << ", lane_id: " << lane->id();
          avoid_lanes.push_back(mapping::ElementId(lane->id()));
        }
      }
    }
  }
  return avoid_lanes;
}

}  // namespace

NoaRouteMpp MppToNoaRoute(const mapping::v2::SemanticMapManager& v2smm,
                          absl::Span<const std::int64_t> sections,
                          int update_id, bool route_trunc_mpp_by_nav) {
  NoaRouteMpp noa_route;
  double noa_length = 0.0;
  auto* route = &(noa_route.route);
  route->set_update_id(update_id);
  auto* section_sequence_proto = route->mutable_route_section_sequence();
  section_sequence_proto->set_start_fraction(0.0);
  section_sequence_proto->set_end_fraction(1.0);
  section_sequence_proto->mutable_section_id()->Reserve(sections.size());
  section_sequence_proto->mutable_destination()->set_lane_id(-1);
  section_sequence_proto->mutable_destination()->set_fraction(0.0);

  int noa_section_count = 0;
  bool find_section = false;
  for (const auto section_id : sections) {
    auto section_ptr = v2smm.FindSection(mapping::SectionId(section_id));
    if (section_ptr == nullptr) {
      if (find_section) {
        break;
      }
      continue;
    } else {
      find_section = true;
      const auto& section = *section_ptr;
      if (!HasRouteGuideAttrBySection(section.proto()) &&
          route_trunc_mpp_by_nav) {
        VLOG(3) << "Has no route guidance info, section_id:" << section_id;
        break;
      } else {
        VLOG(3) << "Has route guidance info.";
      }
      noa_length += section.proto().average_length();
      section_sequence_proto->add_section_id(section_id);
      ++noa_section_count;
    }
  }
  noa_route.noa_length = static_cast<int>(noa_length);
  noa_route.is_noa_end = (noa_section_count == sections.size());
  // CreateCompositeLanePathBySections();
  // RouteSectionsFromCompositeLanePath(smm, clp)
  // TODO(xiang): Complement other fields.
  return noa_route;
}

absl::StatusOr<RouteManagerOutputProto> CalcNavInfoFromMpp(
    const mapping::v2::SemanticMapSpatialIndex& smm_index,
    const MppSectionsProto& mpp, const Vec2d& global,
    const std::optional<double>& heading, const int64_t update_id,
    const std::optional<double>& av_speed,
    const std::optional<double>& look_ahead_time, bool route_trunc_mpp_by_nav,
    const RestrictProto& restrict_proto) {
  if (!mpp.has_ego_section_id()) {
    return absl::FailedPreconditionError("The mpp is invalid.");
  }
  const mapping::v2::SemanticMapManager& v2smm = *smm_index.semantic_manager();
  // PointToNearestLane
  const double max_distance = kSearchLaneRadius;
  std::vector<PointToLaneT<mapping::v2::Lane>> matched_lanes;
  if (heading.has_value()) {
    matched_lanes = route::noa::PointToNearLanesWithHeading(
        smm_index, global, /*z=*/0.0, max_distance, *heading,
        /*max_heading_diff=*/0.8);
  } else {
    matched_lanes =
        route::noa::PointToNearLanes(smm_index, global, max_distance);
  }
  if (matched_lanes.empty()) {
    return absl::NotFoundError(absl::StrCat("Cannot find nearest lanes, ",
                                            global.DebugStringFullPrecision(),
                                            " radius:", max_distance));
  }

  // Truncate mpp section and infer mpp path.
  std::unordered_set<int64_t> av_nearest_sections;
  for (const auto& pt_lane : matched_lanes) {
    av_nearest_sections.insert(pt_lane.lane_info->proto().section_id());
  }
  VLOG(3) << "Nearest sections:" << absl::StrJoin(av_nearest_sections, ", ")
          << ", total matched lanes size: " << matched_lanes.size();
  std::vector<int64_t> trunc_mpp_sections;
  bool find_first_section = false;
  for (const auto section_id : mpp.section_ids()) {
    if (!find_first_section && av_nearest_sections.count(section_id) > 0) {
      find_first_section = true;
    }
    if (find_first_section) {
      trunc_mpp_sections.push_back(section_id);
    }
  }

  if (!find_first_section) {
    return absl::FailedPreconditionError(absl::StrCat(
        "The mpp cannot match av ego section, ego section: ",
        absl::StrJoin(av_nearest_sections, ", "),
        ". mpp sections: ", absl::StrJoin(mpp.section_ids(), ", ")));
  }

  VLOG(3) << "Truncate mpp sections, origin mpp: " << mpp.section_ids().size()
          << ", after, trunc_size: " << trunc_mpp_sections.size();

  route::noa::NoaRouteMpp noa_route_mpp =
      route::noa::MppToNoaRoute(v2smm, absl::MakeSpan(trunc_mpp_sections),
                                update_id, route_trunc_mpp_by_nav);
  if (noa_route_mpp.route.route_section_sequence().section_id().empty()) {
    return absl::FailedPreconditionError(absl::StrCat(
        "The mpp cannot be found in map at NOA mode. truncate mpp: ",
        absl::StrJoin(trunc_mpp_sections, ",")));
  }
  VLOG(3) << "mpp to noa route: "
          << noa_route_mpp.route.route_section_sequence().ShortDebugString();

  const auto& global_sections = noa_route_mpp.route.route_section_sequence();
  ASSIGN_OR_RETURN(auto lane_and_sections,
                   UpdateCurrentRoute(matched_lanes, global_sections));

  RouteManagerOutputProto rm_output;
  // Add Navinfo avoid lanes and odd.
  auto avoid_lanes =
      AddNavInfoAvoidLanes(v2smm, lane_and_sections.second, restrict_proto);
  if (!avoid_lanes.empty()) {
    rm_output.mutable_avoid_lanes()->Reserve(avoid_lanes.size());
    std::for_each(avoid_lanes.begin(), avoid_lanes.end(),
                  [&rm_output](auto id) {
                    rm_output.mutable_avoid_lanes()->Add(id.value());
                  });
  }
  QLOG_EVERY_N_SEC(INFO, 3)
      << "AddNavInfoAvoidLanes: " << absl::StrJoin(avoid_lanes, ",");
  route::UpdateAvoidLanes(v2smm, lane_and_sections.second.section_id(),
                          rm_output.mutable_avoid_lanes());
  absl::flat_hash_set<mapping::ElementId> avoid_lanes_set(
      rm_output.avoid_lanes().begin(), rm_output.avoid_lanes().end());

  // Merge avoid lanes with odd.
  auto odd_lane_segments =
      ParseDynamicOddAlongMpp(v2smm, lane_and_sections.second.section_id());
  for (const auto lane_id : avoid_lanes_set) {
    auto& lane_segment = odd_lane_segments.lane_segments_map[lane_id];
    lane_segment.start_fraction = 0.0;
    lane_segment.end_fraction = 1.0;
  }
  odd_lane_segments.ToProto(rm_output.mutable_avoid_lane_segments());

  const auto& pt_lane = matched_lanes[std::get<std::size_t>(lane_and_sections)];
  const auto start_lp = mapping::LanePoint(
      mapping::ElementId(pt_lane.lane_info->proto().id()), pt_lane.fraction);

  // Process dynamic odd to avoid lanes.
  *rm_output.mutable_avoid_lanes() = route::UpdateDynamicOddAvoidLanes(
      v2smm, odd_lane_segments, global_sections,
      *rm_output.mutable_avoid_lanes());
  avoid_lanes_set = {rm_output.avoid_lanes().begin(),
                     rm_output.avoid_lanes().end()};

  auto route_nav_info = CalcCurRouteNaviInfo(
      v2smm, global_sections, lane_and_sections.second, avoid_lanes_set,
      start_lp, kNaviInfoBackwardExtendLength, kNaviInfoPreviewDistance);

  if (route_nav_info.ok()) {
    route_nav_info->ToProto(rm_output.mutable_route_navi_info());
    VLOG(3) << "Calc nav info is ok, update_id: " << update_id;
  } else {
    VLOG(3) << "Calc nav failed, update_id: " << update_id;
  }
  rm_output.set_update_id(update_id);
  *rm_output.mutable_route_sections_from_current() =
      std::move(lane_and_sections.second);
  rm_output.set_route_status(RouteManagerOutputProto::NORMAL);
  if (av_speed.has_value() && look_ahead_time.has_value()) {
    constexpr double kFrontMaxTime = 5.0;  // second.
    const double look_ahead_dist =
        *av_speed * std::min(*look_ahead_time, kFrontMaxTime);
    const auto speed_limit_opt = FindFrontOverwrittenSpeedLimit(
        v2smm, rm_output.route_sections_from_current(), look_ahead_dist);

    const double front_look_ahead_dist = *av_speed * kFrontMaxTime;
    const auto front_speed_limit_opt = FindFrontOverwrittenSpeedLimit(
        v2smm, rm_output.route_sections_from_current(), front_look_ahead_dist);
    const double front_speed_limit =
        front_speed_limit_opt.has_value() ? *front_speed_limit_opt : DBL_MAX;

    if (speed_limit_opt.has_value()) {
      if (*speed_limit_opt <= front_speed_limit) {
        rm_output.set_recommend_lane_speed_limit(*speed_limit_opt);
      } else {
        rm_output.set_recommend_lane_speed_limit(front_speed_limit);
      }
    }
  }
  *rm_output.mutable_overlap_sections() = mpp.overlapping_sections();
  rm_output.set_overlap_sections_precomputed(
      mpp.has_overlapping_sections_precomputed()
          ? mpp.overlapping_sections_precomputed()
          : false);
  return rm_output;
}

AvoidLaneSegmentMap ParseDynamicOddAlongMpp(
    const mapping::v2::SemanticMapManager& v2smm,
    absl::Span<const int64_t> sections_id) {
  AvoidLaneSegmentMap avoid_lane_map;
  auto& segments_map = avoid_lane_map.lane_segments_map;
  for (const auto section_id : sections_id) {
    SMM_SECTION_PROTO_OR_CONTINUE(section, v2smm, section_id);
    if (!section->has_odd()) continue;
    for (const auto& custom_event : section->odd().custom_events()) {
      if (!custom_event.has_type() ||
          custom_event.type() != mapping::CustomEvent::AVOID_LANE_SEGMENTS) {
        continue;
      }
      std::vector<int64_t> odd_lanes;
      odd_lanes.reserve(section->lanes_size());
      if (custom_event.lane_number().empty()) {
        // All lanes are avoid.
        odd_lanes.insert(odd_lanes.end(), section->lanes().begin(),
                         section->lanes().end());
      } else {
        for (const auto lane_number : custom_event.lane_number()) {
          const int lane_idx =
              std::clamp(lane_number - 1, 0, section->lanes_size() - 1);
          odd_lanes.push_back(section->lanes(lane_idx));
        }
      }
      for (const auto lane_id : odd_lanes) {
        auto& lane_segment = segments_map[mapping::ElementId(lane_id)];
        lane_segment.start_fraction = custom_event.has_start_fraction()
                                          ? custom_event.start_fraction()
                                          : 0.0;
        lane_segment.end_fraction =
            custom_event.has_end_fraction() ? custom_event.end_fraction() : 1.0;
      }
    }
  }
  return avoid_lane_map;
}

}  // namespace qcraft::planner::route::noa
