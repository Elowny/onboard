#include "onboard/planner/decision/constraint_builder.h"

#include <optional>
#include <ostream>
#include <string>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "glog/logging.h"

#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/lite/logging.h"
#include "onboard/maps/lane_path.h"
#include "onboard/maps/lane_point.h"
#include "onboard/maps/maps_common.h"
#include "onboard/maps/proto/semantic_map.pb.h"
#include "onboard/math/frenet_common.h"
#include "onboard/math/geometry/halfplane.h"
#include "onboard/math/util.h"
#include "onboard/params/v2/proto/vehicle/common.pb.h"
#include "onboard/planner/common/speed_profile.h"
#include "onboard/planner/decision/beyond_length_along_route.h"
#include "onboard/planner/decision/cautious_brake_decider.h"
#include "onboard/planner/decision/constraint_manager.h"
#include "onboard/planner/decision/crosswalk_decider.h"
#include "onboard/planner/decision/decision_util.h"
#include "onboard/planner/decision/end_of_current_lane_path.h"
#include "onboard/planner/decision/end_of_path_boundary.h"
#include "onboard/planner/decision/inferred_object_decider.h"
#include "onboard/planner/decision/lc_end_of_current_lane_constraint.h"
#include "onboard/planner/decision/no_block.h"
#include "onboard/planner/decision/parking_brake_release.h"
#include "onboard/planner/decision/pedestrians_decider.h"
#include "onboard/planner/decision/proto/constraint.pb.h"
#include "onboard/planner/decision/proto/crosswalk_state.pb.h"
#include "onboard/planner/decision/proto/stop_sign_state.pb.h"
#include "onboard/planner/decision/solid_line_within_boundary.h"
#include "onboard/planner/decision/speed_bump.h"
#include "onboard/planner/decision/standstill_decider.h"
#include "onboard/planner/decision/stop_polyline_decider.h"
#include "onboard/planner/decision/stop_sign_decider.h"
#include "onboard/planner/decision/toll_decider.h"
#include "onboard/planner/decision/traffic_light/traffic_light_decider.h"
#include "onboard/planner/planner_flags.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/planner/scheduler/proto/lane_change.pb.h"
#include "onboard/planner/util/lane_point_util.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/proto/vehicle.pb.h"
#include "onboard/utils/status_macros.h"
#include "onboard/utils/time_util.h"

namespace qcraft {
namespace planner {

namespace {

absl::StatusOr<ConstraintProto::StopLineProto> BuildBrakeToStopConstraint(
    const std::string& type_str, const DrivePassage& passage,
    double front_to_ra, double ego_v, double brake) {
  constexpr double kStandStillDist = 0.3;  // m.

  // NOTE: We dont want to add more states. Recalculate stop s each frame is
  // acceptable as we do not require stop at a certain point.
  const double stop_dist =
      ego_v < 1.0 ? kStandStillDist : Sqr(ego_v) * 0.5 / brake;
  const double stop_s = passage.lane_path_start_s() + stop_dist + front_to_ra;

  ASSIGN_OR_RETURN(const auto curbs, passage.QueryCurbPointAtS(stop_s));
  ConstraintProto::StopLineProto stop_line;
  stop_line.set_s(stop_s);
  stop_line.set_standoff(0.0);
  stop_line.set_time(0.0);
  HalfPlane halfplane(curbs.first, curbs.second);
  halfplane.ToProto(stop_line.mutable_half_plane());
  stop_line.set_id(type_str);
  stop_line.mutable_source()->mutable_brake_to_stop()->set_reason(type_str);

  return stop_line;
}

bool ShouldConsiderLcEndOfCurrent(
    const PlannerSemanticMapManager& psmm, const DrivePassage& passage,
    const mapping::LanePath& lane_path_before_lc) {
  if (lane_path_before_lc.IsEmpty()) return false;
  if (passage.reach_destination()) return true;

  ASSIGN_OR_RETURN(const auto last_pos_sl,
                   passage.QueryLaterallyUnboundedFrenetCoordinateAt(
                       ComputeLanePointPos(psmm, lane_path_before_lc.back())),
                   false);

  constexpr double kLonProjErrorThres = 10.0;  // m.
  return last_pos_sl.s + kLonProjErrorThres <
         passage.lane_path().length() + passage.lane_path_start_s();
}

}  // namespace

// NOLINTNEXTLINE(readability-function-size,readability-function-cognitive-complexity)
absl::StatusOr<DeciderOutput> BuildConstraints(
    const DeciderInput& decider_input) {
  SCOPED_QTRACE("ConstraintBuilder");

  QCHECK_NOTNULL(decider_input.vehicle_geometry_params);
  QCHECK_NOTNULL(decider_input.motion_constraint_params);
  QCHECK_NOTNULL(decider_input.config);
  QCHECK_NOTNULL(decider_input.planner_semantic_map_manager);
  QCHECK_NOTNULL(decider_input.lc_state);
  QCHECK_NOTNULL(decider_input.plan_start_point);
  QCHECK_NOTNULL(decider_input.passage);
  QCHECK_NOTNULL(decider_input.obj_mgr);
  QCHECK_NOTNULL(decider_input.st_traj_mgr);
  QCHECK_NOTNULL(decider_input.pre_decider_state);
  QCHECK_NOTNULL(decider_input.ego_frenet_box);

  const auto& vehicle_geometry_params = *decider_input.vehicle_geometry_params;
  const auto& motion_constraint_params =
      *decider_input.motion_constraint_params;
  const auto& config = *decider_input.config;
  const auto& planner_semantic_map_manager =
      *decider_input.planner_semantic_map_manager;
  const auto& lc_state = *decider_input.lc_state;
  const auto& plan_start_point = *decider_input.plan_start_point;
  const auto& passage = *decider_input.passage;
  const auto& obj_mgr = *decider_input.obj_mgr;
  const auto& sl_boundary = *decider_input.sl_boundary;
  const auto& st_traj_mgr = *decider_input.st_traj_mgr;
  const auto& tl_info_map = *decider_input.tl_info_map;
  const auto& pre_decider_state = *decider_input.pre_decider_state;
  const auto& ego_frenet_box = *decider_input.ego_frenet_box;

  ConstraintManager constraint_manager;
  DeciderStateProto new_decider_state;

  if (passage.empty()) {
    return absl::UnavailableError(
        "Drive passage on target lane path not available.");
  }

  const double start_s_offset = decider_input.target_offset_from_start;
  const auto lane_path_from_start =
      start_s_offset == 0.0
          ? passage.lane_path()
          : passage.lane_path().AfterArclength(start_s_offset);

  // TODO(PNC-501): This is a hack. Remove this hack after refactor planner
  // params.
  const bool requires_parking_brake_release =
      decider_input.vehicle_model != VehicleModel::VEHICLE_ZHONGXING;
  if (config.enable_parking_brake_release() && requires_parking_brake_release) {
    auto parking_brake_release_constraint = BuildParkingBrakeReleaseConstraint(
        vehicle_geometry_params, passage,
        decider_input.parking_brake_release_time, decider_input.plan_time);
    if (parking_brake_release_constraint.ok()) {
      constraint_manager.AddStopLine(
          std::move(parking_brake_release_constraint).value());
    } else {
      VLOG(2) << "Build parking brake release constraint failed: "
              << parking_brake_release_constraint.status().ToString();
    }
  }

  if (config.enable_lc_end_of_current_lane() &&
      lc_state.stage() == LaneChangeStage::LCS_PAUSE &&
      decider_input.lane_path_before_lc != nullptr) {
    const auto& lane_path_before_lc = *decider_input.lane_path_before_lc;
    if (ShouldConsiderLcEndOfCurrent(planner_semantic_map_manager, passage,
                                     lane_path_before_lc)) {
      auto lcp_speed = BuildLcEndOfCurrentLaneConstraints(
          passage, lane_path_before_lc, plan_start_point.v());
      if (lcp_speed.ok()) {
        constraint_manager.AddSpeedRegion(std::move(lcp_speed).value());
      }
    }
  }

  const auto* lane_info_ptr = planner_semantic_map_manager.FindLaneInfoOrNull(
      lane_path_from_start.front().lane_id());
  if (config.enable_beyond_length_along_route() && lane_info_ptr != nullptr) {
    const auto* section_info_ptr =
        planner_semantic_map_manager.FindSectionInfoOrNull(
            lane_info_ptr->section_id);
    if (section_info_ptr != nullptr && section_info_ptr->proto != nullptr &&
        section_info_ptr->proto->road_class() ==
            mapping::SectionProto::NORMAL) {
      auto beyond_len_along_route_speed = BuildBeyondLengthAlongRouteConstraint(
          passage, motion_constraint_params, decider_input.max_reach_length,
          decider_input.borrow_lane_boundary, plan_start_point.v());
      if (beyond_len_along_route_speed.ok()) {
        constraint_manager.AddSpeedProfile(
            std::move(beyond_len_along_route_speed).value());
      }
    }
  }

  if (config.enable_crosswalk()) {
    // Crosswalk.
    ASSIGN_OR_RETURN(
        auto cw_decider_output,
        BuildCrosswalkConstraints(CrosswalkDeciderInput{
            .vehicle_geometry_params = &vehicle_geometry_params,
            .psmm = &planner_semantic_map_manager,
            .plan_start_point = &plan_start_point,
            .passage = &passage,
            .lane_path_from_start = &lane_path_from_start,
            .obj_mgr = &obj_mgr,
            .last_crosswalk_states = &pre_decider_state.crosswalk_state(),
            .valid_cw_types = {mapping::CrosswalkProto::MUST_YEILD},
            .now_in_seconds = ToUnixDoubleSeconds(decider_input.plan_time),
            .s_offset = start_s_offset,
        }));

    for (auto& cw_stop_line : cw_decider_output.stop_lines) {
      constraint_manager.AddStopLine(std::move(cw_stop_line));
    }
    for (auto& cw_speed_region : cw_decider_output.speed_regions) {
      constraint_manager.AddSpeedRegion(std::move(cw_speed_region));
    }
    for (auto& crosswalk_state : cw_decider_output.crosswalk_states) {
      *new_decider_state.add_crosswalk_state() = std::move(crosswalk_state);
    }
  }

  // Pedestrians.
  if (config.enable_pedestrians()) {
    // lane keep
    if (lc_state.stage() == LaneChangeStage::LCS_NONE) {
      ASSIGN_OR_RETURN(
          auto ped_speed_regions,
          BuildPedestriansConstraints(
              vehicle_geometry_params, planner_semantic_map_manager,
              plan_start_point, passage, lane_path_from_start, start_s_offset,
              sl_boundary, st_traj_mgr));

      for (auto& ped_speed_region : ped_speed_regions) {
        constraint_manager.AddSpeedRegion(std::move(ped_speed_region));
      }
    }
  }

  // No block constraints.
  if (config.enable_no_block()) {
    auto no_block_regions =
        BuildNoBlockConstraints(planner_semantic_map_manager, passage,
                                lane_path_from_start, start_s_offset);
    for (auto& no_block_region : no_block_regions) {
      QCHECK_LE(no_block_region.start_s(), no_block_region.end_s())
          << no_block_region.ShortDebugString();
      constraint_manager.AddSpeedRegion(std::move(no_block_region));
    }
  }

  if (FLAGS_planner_decision_enable_stop_sign && config.enable_stop_sign()) {
    ASSIGN_OR_RETURN(
        auto output,
        BuildStopSignConstraints(
            planner_semantic_map_manager, st_traj_mgr, vehicle_geometry_params,
            passage,
            /*now_in_seconds=*/ToUnixDoubleSeconds(decider_input.plan_time),
            plan_start_point, pre_decider_state.stop_sign_state()));
    for (auto& stop_line : output.stop_lines) {
      constraint_manager.AddStopLine(std::move(stop_line));
    }
    for (auto& stop_sign_state : output.stop_sign_states) {
      *new_decider_state.add_stop_sign_state() = std::move(stop_sign_state);
    }
  }

  // If we have end of current lane path constraint, add it.
  auto end_of_cur_lp_constraint = BuildEndOfCurrentLanePathConstraint(passage);
  if (end_of_cur_lp_constraint.ok()) {
    constraint_manager.AddStopLine(std::move(end_of_cur_lp_constraint).value());
  } else {
    // If we don't have end of current route constraint, add the end of drive
    // passage constraint.
    auto end_of_path_boundary_constraint =
        BuildEndOfPathBoundaryConstraint(passage, sl_boundary);
    if (end_of_path_boundary_constraint.ok()) {
      constraint_manager.AddStopLine(
          std::move(end_of_path_boundary_constraint).value());
    } else {
      QLOG(WARNING) << "Build end of path boundary constraint failed: "
                    << end_of_path_boundary_constraint.status().ToString();
    }
  }

  if (config.enable_speed_bump()) {
    // Speed bump.
    auto speed_bumps =
        BuildSpeedBumpConstraints(planner_semantic_map_manager, passage);
    for (auto& speed_bump : speed_bumps) {
      QCHECK_LE(speed_bump.start_s(), speed_bump.end_s())
          << speed_bump.ShortDebugString();
      constraint_manager.AddSpeedRegion(std::move(speed_bump));
    }
  }

  if (config.enable_cautious_brake()) {
    // Speed bump.
    auto cautious_brake_regions = BuildCautiousBrakeConstraints(
        planner_semantic_map_manager, passage, lane_path_from_start,
        start_s_offset, st_traj_mgr);
    for (auto& cautious_brake : cautious_brake_regions) {
      QCHECK_LE(cautious_brake.start_s(), cautious_brake.end_s())
          << cautious_brake.ShortDebugString();
      constraint_manager.AddSpeedRegion(std::move(cautious_brake));
    }
  }

  if (config.enable_toll()) {
    ASSIGN_OR_RETURN(
        auto toll_speed_regions,
        BuildTollConstraints(planner_semantic_map_manager, passage,
                             lane_path_from_start, start_s_offset));

    for (auto& toll_speed_region : toll_speed_regions) {
      constraint_manager.AddSpeedRegion(std::move(toll_speed_region));
    }
  }

  std::optional<double> distance_to_traffic_light_stop_line = std::nullopt;
  if (config.enable_traffic_light() &&
      decider_input.teleop_enable_traffic_light_stop) {
    // SpeedProfile need speed regions and stop line information from other
    // constraints, build this constraint at last.
    SpeedProfile preliminary_speed_profile = CreateSpeedProfile(
        plan_start_point.v(), passage, constraint_manager.SpeedRegion(),
        constraint_manager.StopLine());

    auto tl_decider_output = BuildTrafficLightConstraints(
        planner_semantic_map_manager, vehicle_geometry_params, plan_start_point,
        passage, lane_path_from_start, start_s_offset, tl_info_map,
        preliminary_speed_profile,
        pre_decider_state.traffic_light_decider_state());

    if (!tl_decider_output.ok()) {
      QLOG(WARNING) << "Build tl stop lines failed with message: "
                    << tl_decider_output.status().ToString();
    } else {
      for (auto& tl_stop_line : tl_decider_output.value().stop_lines) {
        if (!distance_to_traffic_light_stop_line.has_value()) {
          distance_to_traffic_light_stop_line = tl_stop_line.s();
        }
        constraint_manager.AddStopLine(std::move(tl_stop_line));
      }
      for (auto& tl_speed_profile : tl_decider_output.value().speed_profiles) {
        constraint_manager.AddSpeedProfile(std::move(tl_speed_profile));
      }
      *new_decider_state.mutable_traffic_light_decider_state() =
          tl_decider_output.value().traffic_light_decider_state;
    }
  }

  if (decider_input.enable_pull_over) {
    constexpr double kPullOverBrake = 1.0;  // m/s^2
    auto stop_line_or = BuildBrakeToStopConstraint(
        /*type_str=*/"pull_over", passage,
        vehicle_geometry_params.front_edge_to_center(), plan_start_point.v(),
        kPullOverBrake);

    if (stop_line_or.ok()) {
      constraint_manager.AddStopLine(std::move(stop_line_or).value());
    }
  }

  if (decider_input.brake_to_stop.has_value()) {
    auto stop_line_or = BuildBrakeToStopConstraint(
        /*type_str=*/"brake_to_stop", passage,
        vehicle_geometry_params.front_edge_to_center(), plan_start_point.v(),
        *decider_input.brake_to_stop);

    if (stop_line_or.ok()) {
      constraint_manager.AddStopLine(std::move(stop_line_or).value());
    }
  }

  if (config.enable_force_stop() && decider_input.enable_force_stop) {
    constexpr double kForceStopBrake = 2.0;  // m/s^2
    auto stop_line_or = BuildBrakeToStopConstraint(
        /*type_str=*/"force_stop", passage,
        vehicle_geometry_params.front_edge_to_center(), plan_start_point.v(),
        kForceStopBrake);

    if (stop_line_or.ok()) {
      constraint_manager.AddStopLine(std::move(stop_line_or).value());
    }
  }

  if (config.enable_standstill()) {
    ASSIGN_OR_RETURN(
        auto ss_stop_lines,
        BuildStandstillConstraints(vehicle_geometry_params, plan_start_point,
                                   passage, constraint_manager.StopLine()));
    for (auto& ss_stop_line : ss_stop_lines) {
      constraint_manager.AddStopLine(std::move(ss_stop_line));
    }
  }

  if (config.enable_solid_line_within_boundary()) {
    const auto solid_lines = BuildSolidLineWithinBoundaryConstraint(
        passage, sl_boundary, plan_start_point);
    if (solid_lines.ok()) {
      for (auto& solid_line : *solid_lines) {
        constraint_manager.AddAvoidLine(solid_line);
      }
    }
  }

  if (config.enable_inferred_object() &&
      decider_input.scene_reasoning != nullptr) {
    auto inferred_object_constraint_or = BuildInferredObjectConstraint(
        planner_semantic_map_manager, *decider_input.scene_reasoning,
        lane_path_from_start, plan_start_point.v());
    if (inferred_object_constraint_or.ok()) {
      constraint_manager.AddSpeedProfile(
          std::move(inferred_object_constraint_or).value());
    }
  }

  if (config.enable_stop_polyline() &&
      decider_input.enable_stop_polyline_stopping) {
    auto stop_res_or = BuildStopPolylineConstraints(
        planner_semantic_map_manager, passage, ego_frenet_box,
        pre_decider_state.stop_polyline_state(),
        decider_input.is_engage_steer_only);
    if (stop_res_or.ok()) {
      for (auto& stop_line : stop_res_or->stop_lines) {
        constraint_manager.AddStopLine(std::move(stop_line));
      }
      *new_decider_state.mutable_stop_polyline_state() =
          std::move(stop_res_or->stop_polyline_state);
    }
  }

  return DeciderOutput{
      .constraint_manager = std::move(constraint_manager),
      .decider_state = std::move(new_decider_state),
      .distance_to_traffic_light_stop_line =
          distance_to_traffic_light_stop_line,
  };
}

}  // namespace planner
}  // namespace qcraft
