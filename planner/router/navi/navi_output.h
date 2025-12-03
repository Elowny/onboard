#ifndef ONBOARD_PLANNER_ROUTER_NAVI_NAVI_OUTPUT_H_
#define ONBOARD_PLANNER_ROUTER_NAVI_NAVI_OUTPUT_H_

// IWYU pragma: no_include <ostream>

#include <cstdint>
#include <type_traits>
#include <utility>  // IWYU pragma: keep // for std::move
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"

#include "common/proto/map_geometry.pb.h"

#include "onboard/container/strong_int.h"
#include "onboard/lite/logging.h"
#include "onboard/maps/lane_point.h"
#include "onboard/maps/map_or_die_macros.h"
#include "onboard/maps/proto/semantic_map.pb.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/maps/v2/semantic_map_manager.h"
#include "onboard/math/vec.h"
#include "onboard/planner/composite_lane_path.h"
#include "onboard/planner/router/route_util.h"
#include "onboard/proto/route.pb.h"
#include "onboard/proto/route_navigation.pb.h"
#include "onboard/utils/source_location.h"

namespace qcraft::planner::route {

struct NaviActionInfo {
  NaviActionProto navi_action;
  double length = 0.0;
};

struct NaviOutput {
  NaviPreviewProto navi_preview;
  NaviInstructionProto current_navi_instruction;
};

absl::StatusOr<std::vector<NaviActionInfo>> CreateNaviActionInfoList(
    const mapping::v2::SemanticMapManager& semantic_map_manager,
    const std::vector<mapping::ElementId>& lane_ids, double start_fraction,
    double end_fraction, const Vec2d& dest_global);

NaviActionProto MakeNaviAction(NaviActionProto::NaviActionType navi_action_type,
                               const Vec2d& point, double distance,
                               mapping::SectionId section_id);

NaviInstructionProto CreateCurrentNaviInstruction(
    const std::vector<NaviActionInfo>& navi_infos, int64_t update_id);

NaviPreviewProto CreateNaviPreview(
    const std::vector<NaviActionInfo>& navi_infos, int64_t update_id);

template <typename CompositeLanePathType,
          std::enable_if_t<
              std::is_same_v<CompositeLanePathType, CompositeLanePath> ||
                  std::is_same_v<CompositeLanePathType, CompositeLanePathProto>,
              bool> = true>
absl::StatusOr<NaviOutput> CreateNaviOutputFromRoute(
    const mapping::v2::SemanticMapManager& semantic_map_manager,
    const CompositeLanePathType& composite_lane_path, int64_t update_id) {
  if (composite_lane_path.lane_paths().empty()) {
    return absl::InvalidArgumentError("composite_lane_path is empty.");
  }
  const auto& back_lane_path =
      composite_lane_path
          .lane_paths()[composite_lane_path.lane_paths().size() - 1];
  mapping::LanePoint dest_lane_pt(
      mapping::ElementId(
          back_lane_path.lane_ids()[back_lane_path.lane_ids().size() - 1]),
      back_lane_path.end_fraction());
  SMM_LANE_PROTO_OR_RETURN(lane_proto, semantic_map_manager,
                           dest_lane_pt.lane_id(),
                           absl::NotFoundError(absl::StrCat(
                               "Cannot find lane.", dest_lane_pt.lane_id())));
  const Vec3d dest3d = ComputePointOnLane(lane_proto->polyline().points(),
                                          dest_lane_pt.fraction());
  std::vector<mapping::ElementId> lane_ids;
  auto last_lane_id = mapping::kInvalidElementId;
  int max_lane_id_num = 0;
  for (const auto& lane_path : composite_lane_path.lane_paths()) {
    max_lane_id_num += lane_path.lane_ids().size();
  }
  lane_ids.reserve(max_lane_id_num);
  for (const auto& lane_path : composite_lane_path.lane_paths()) {
    for (const auto& lane_id : lane_path.lane_ids()) {
      if (last_lane_id != mapping::ElementId(lane_id)) {
        lane_ids.emplace_back(lane_id);
      }
      last_lane_id = mapping::ElementId(lane_id);
    }
  }

  const auto& front_lane_path = composite_lane_path.lane_paths()[0];
  auto navi_infos_or = CreateNaviActionInfoList(
      semantic_map_manager, lane_ids, front_lane_path.start_fraction(),
      back_lane_path.end_fraction(), {dest3d.x(), dest3d.y()});
  if (navi_infos_or.ok()) {
    auto current_navi_instruction =
        CreateCurrentNaviInstruction(*navi_infos_or, update_id);
    auto navi_preview = CreateNaviPreview(*navi_infos_or, update_id);
    return NaviOutput{
        .navi_preview = std::move(navi_preview),
        .current_navi_instruction = std::move(current_navi_instruction)};
  }
  QLOG(WARNING) << "UpdateNaviInfoFromRoute failed."
                << navi_infos_or.status().ToString()
                << " udpate_id: " << update_id;
  return navi_infos_or.status();
}

}  // namespace qcraft::planner::route

#endif  // ONBOARD_PLANNER_ROUTER_NAVI_NAVI_OUTPUT_H_
