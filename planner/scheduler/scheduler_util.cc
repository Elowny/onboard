#include "onboard/planner/scheduler/scheduler_util.h"

#include <algorithm>
#include <cmath>
#include <vector>

// IWYU pragma: no_include <google/protobuf/repeated_ptr_field.h>

#include "absl/container/flat_hash_map.h"
#include "absl/types/span.h"
#include "google/protobuf/repeated_ptr_field.h"

#include "onboard/autonomy_state/autonomy_state_util.h"
#include "onboard/container/strong_int.h"
#include "onboard/eval/qevent.h"
#include "onboard/eval/qevent_base.h"
#include "onboard/global/logging.h"
#include "onboard/global/trace.h"
#include "onboard/maps/lane_path.h"
#include "onboard/maps/lane_point.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/math/geometry/util.h"
#include "onboard/planner/assist/assist_util.h"
#include "onboard/planner/common/path_sl_boundary.h"
#include "onboard/planner/decision/proto/traffic_light_info.pb.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/planner/scheduler/path_boundary_builder_helper.h"
#include "onboard/utils/status_builder.h"
#include "onboard/utils/status_macros.h"

namespace qcraft::planner {
namespace {

LaneChangeStage DecideLaneChangeStage(
    const DrivePassage& drive_passage,
    const mapping::LanePath& prev_target_lane_path_from_start,
    const mapping::LanePath& prev_lane_path_before_lc_from_start,
    const LaneChangeStateProto& prev_lc_state) {
  const bool target_switched =
      prev_target_lane_path_from_start.front().lane_id() !=
      drive_passage.lane_path().front().lane_id();
  const auto stage_from_before_lc =
      prev_lane_path_before_lc_from_start.IsEmpty() ||
              prev_lane_path_before_lc_from_start.front().lane_id() !=
                  drive_passage.lane_path().front().lane_id()
          ? LaneChangeStage::LCS_EXECUTING
          : LaneChangeStage::LCS_RETURN;

  if (!target_switched) {
    if (prev_lc_state.stage() == LaneChangeStage::LCS_PAUSE) {
      // Treat lc pause as executing here since lc safety is checked later.
      return stage_from_before_lc;
    }
    return prev_lc_state.stage();
  }

  switch (prev_lc_state.stage()) {
    case LaneChangeStage::LCS_NONE:
    case LaneChangeStage::LCS_WAITING:
    case LaneChangeStage::LCS_RETURN:
      return LaneChangeStage::LCS_EXECUTING;
    case LaneChangeStage::LCS_EXECUTING:
    case LaneChangeStage::LCS_PAUSE:
      return stage_from_before_lc;
  }
}

mapping::ElementId FindWaitingZoneAhead(const TrafficLightInfoMap& tl_info_map,
                                        const mapping::LanePath& lane_path) {
  for (const auto& lane_id : lane_path.lane_ids()) {
    if (tl_info_map.count(lane_id) &&
        tl_info_map.at(lane_id).tl_control_type() ==
            TrafficLightControlType::LEFT_WAITING_AREA) {
      return lane_id;
    }
  }
  return mapping::kInvalidElementId;
}

}  // namespace

LaneChangeStateProto MakeNoneLaneChangeState() {
  LaneChangeStateProto proto;
  proto.set_stage(LaneChangeStage::LCS_NONE);
  // The rest fields remain unavailable.
  return proto;
}

bool ShouldSmoothRefLane(const TrafficLightInfoMap& tl_info_map,
                         const DrivePassage& dp, bool prev_smooth_state) {
  const auto& ego_station = dp.FindNearestStationAtS(0.0);
  if (ego_station.is_in_intersection()) {
    // Keep the previous choice once entered intersection.
    return prev_smooth_state;
  }

  const auto waiting_zone_lane_id =
      FindWaitingZoneAhead(tl_info_map, dp.lane_path());
  if (waiting_zone_lane_id == mapping::kInvalidElementId) {
    // No waiting zone, apply smooth regardless of ego pose or traffic light.
    return true;
  }
  // Has waiting zone, decide from traffic light state.
  return tl_info_map.at(waiting_zone_lane_id)
             .tls()
             .at(TrafficLightDirection::LEFT)
             .tl_state == TrafficLightState::TL_STATE_GREEN;
}

double CalcAvhRefCenterL(const PlannerSemanticMapManager& psmm,
                         const DrivePassage& drive_passage,
                         const FrenetBox& ego_frenet_box,
                         const SmoothedReferenceLineResultMap& smooth_res_map,
                         bool should_smooth) {
  if (!should_smooth) return 0.0;
  const auto center_l_vec =
      ComputeSmoothedReferenceLine(psmm, drive_passage, smooth_res_map);
  const auto ego_frenet_center = ego_frenet_box.center();
  return center_l_vec.at(
      drive_passage.FindNearestStationIndexAtS(ego_frenet_center.s).value());
}

absl::StatusOr<LaneChangeStateProto> MakeLaneChangeState(
    const DrivePassage& drive_passage, const Vec2d& ego_pos,
    const FrenetBox& ego_frenet_box,
    const mapping::LanePath& prev_target_lane_path_from_start,
    const mapping::LanePath& prev_lane_path_before_lc_from_start,
    const LaneChangeStateProto& prev_lc_state, double ref_center_l,
    AutonomyStateProto::State autonomy_state) {
  const auto ego_frenet_center = ego_frenet_box.center();
  if (std::abs(ego_frenet_center.l - ref_center_l) <
      kMaxLaneKeepLateralOffset) {
    // Close to target lane center, no lane change state.
    return MakeNoneLaneChangeState();
  }

  const double ref_l_min = ego_frenet_box.l_min - ref_center_l;
  const double ref_l_max = ego_frenet_box.l_max - ref_center_l;
  const auto boundaries =
      drive_passage.QueryEnclosingLaneBoundariesAtS(ego_frenet_center.s);
  // To deal with virtual lanes with no boundaries other than curbs.
  const double lane_boundary_right_offset =
      std::max(boundaries.right->lat_offset, -kDefaultHalfLaneWidth);
  const double lane_boundary_left_offset =
      std::min(boundaries.left->lat_offset, kDefaultHalfLaneWidth);
  if (lane_boundary_right_offset < ref_l_min &&
      ref_l_max < lane_boundary_left_offset) {
    // Completely within lane path, no lane change state.
    return MakeNoneLaneChangeState();
  }

  if ((lane_boundary_right_offset > ref_l_min &&
       ref_l_max > lane_boundary_right_offset) ||
      (lane_boundary_left_offset > ref_l_min &&
       ref_l_max > lane_boundary_left_offset)) {
    QEVENT_EVERY_N_SECONDS("zixuan", "cross_lane_boundary",
                           /*seconds=*/10.0, [](QEvent* /*qevent*/) {});
  }

  ASSIGN_OR_RETURN(
      const bool crossed_boundary, CrossedBoundary(drive_passage, ego_pos),
      _ << "Ego pos " << ego_pos.DebugString() << " is out of drive passage!");

  LaneChangeStateProto lc_state;
  lc_state.set_lc_left(ref_l_min < lane_boundary_right_offset);
  lc_state.set_crossed_boundary(crossed_boundary);
  lc_state.set_entered_target_lane(lc_state.lc_left() ? ref_l_max > 0.0
                                                      : ref_l_min < 0.0);
  if (IsAutoDrive(autonomy_state)) {
    lc_state.set_stage(DecideLaneChangeStage(
        drive_passage, prev_target_lane_path_from_start,
        prev_lane_path_before_lc_from_start, prev_lc_state));
  } else {
    lc_state.set_stage(lc_state.entered_target_lane()
                           ? LaneChangeStage::LCS_NONE
                           : LaneChangeStage::LCS_EXECUTING);
  }

  return lc_state;
}

void ToSchedulerOutputProto(const SchedulerOutput& output,
                            SchedulerOutputProto* proto) {
  SCOPED_QTRACE("ToSchedulerOutputProto");

  proto->Clear();

  // Target lane path
  output.drive_passage.lane_path().ToProto(proto->mutable_target_lane_path());
  const auto back_extend_lane_path =
      output.drive_passage.extend_lane_path().BeforeLastOccurrenceOfLanePoint(
          output.drive_passage.lane_path().back());
  back_extend_lane_path.ToProto(proto->mutable_backward_extended_lane_path());
  output.drive_passage.FindNearestStationAtS(0.0).GetLanePoint().ToProto(
      proto->mutable_anchor_point());

  *proto->mutable_lc_state() = output.lane_change_state;
  proto->set_length_along_route(output.length_along_route);
  proto->set_borrow_lane(output.borrow_lane);
  output.lane_path_before_lc.ToProto(proto->mutable_lane_path_before_lc());
  proto->set_max_reach_length(output.max_reach_length);
  proto->set_should_smooth(output.should_smooth);
  proto->set_request_help_lane_change_by_route(
      output.request_help_lane_change_by_route);
  proto->set_standard_congestion_factor(output.standard_congestion_factor);
  proto->set_traffic_congestion_factor(output.traffic_congestion_factor);
  proto->set_is_solid_lane_change(output.is_solid_lane_change);

  // Path boundary
  const int path_boundary_size = output.sl_boundary.size();
  auto* boundary = proto->mutable_path_boundary();
  boundary->mutable_reference_center()->Reserve(path_boundary_size);
  boundary->mutable_left_boundary()->Reserve(path_boundary_size);
  boundary->mutable_right_boundary()->Reserve(path_boundary_size);
  boundary->mutable_target_left_boundary()->Reserve(path_boundary_size);
  boundary->mutable_target_right_boundary()->Reserve(path_boundary_size);

  for (const auto& pt : output.sl_boundary.reference_center_xy_vector()) {
    Vec2dToProto(pt, boundary->add_reference_center());
  }
  for (const auto& pt : output.sl_boundary.left_xy_vector()) {
    Vec2dToProto(pt, boundary->add_left_boundary());
  }
  for (const auto& pt : output.sl_boundary.right_xy_vector()) {
    Vec2dToProto(pt, boundary->add_right_boundary());
  }
  for (const auto& pt : output.sl_boundary.target_left_xy_vector()) {
    Vec2dToProto(pt, boundary->add_target_left_boundary());
  }
  for (const auto& pt : output.sl_boundary.target_right_xy_vector()) {
    Vec2dToProto(pt, boundary->add_target_right_boundary());
  }
}

}  // namespace qcraft::planner
