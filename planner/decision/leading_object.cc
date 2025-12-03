#include "onboard/planner/decision/leading_object.h"

#include <float.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <string_view>
#include <utility>

#include "absl/container/flat_hash_set.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "absl/types/span.h"

#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/maps/lane_path.h"
#include "onboard/maps/maps_common.h"
#include "onboard/maps/proto/lane_path.pb.h"
#include "onboard/maps/proto/semantic_map.pb.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/math/frenet_frame.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/decision/decision_util.h"
#include "onboard/planner/object/planner_object.h"
#include "onboard/planner/object/spacetime_object_trajectory.h"
#include "onboard/planner/second_order_trajectory_point.h"
#include "onboard/utils/status_macros.h"

namespace qcraft {
namespace planner {

namespace {

using ObjectsOnLane =
    std::vector<std::pair<FrenetBox, const SpacetimeObjectTrajectory*>>;

// Filter on coming object by drive passage.
absl::StatusOr<bool> IsOncomingObjectJudgeByDrivePassage(
    const DrivePassage& passage, const SecondOrderTrajectoryPoint& obj_pose) {
  ASSIGN_OR_RETURN(const auto tangent, passage.QueryTangentAt(obj_pose.pos()));
  const double passage_angle = tangent.Angle();
  const double angle_diff =
      std::abs(NormalizeAngle(passage_angle - obj_pose.theta()));

  return angle_diff > M_PI_2;
}

// Filter on coming object by ego heading.
bool IsOncomingObjectJudgeByEgoHeading(
    const ApolloTrajectoryPointProto& plan_start_point,
    const SecondOrderTrajectoryPoint& obj_pose) {
  const double ego_heading_angle = plan_start_point.path_point().theta();
  const double object_heading_angle = obj_pose.theta();
  const double angle_diff =
      std::abs(NormalizeAngle(ego_heading_angle - object_heading_angle));

  // Object moving in the opposite direction of ego vehicle.
  return angle_diff > M_PI_2;
}

absl::StatusOr<FrenetBox> FilterObjectViaDrivePassage(
    const PlannerObject& object, const DrivePassage& passage,
    const PathSlBoundary& sl_boundary, const FrenetBox& ego_frenet_box) {
  FUNC_QTRACE();

  // Calculate object frenet coordinate.
  ASSIGN_OR_RETURN(const auto object_frenet_box,
                   passage.QueryFrenetBoxAtContour(object.contour()));

  // Filter objects behind ego front edge or beyond drive passage length.
  if (object_frenet_box.s_min < ego_frenet_box.s_max ||
      object_frenet_box.s_min > sl_boundary.end_s()) {
    return absl::OutOfRangeError(absl::StrFormat(
        "Object %s out of longitudinal boundary, s range: ( %.2f, %.2f)",
        object.id(), object_frenet_box.s_min, object_frenet_box.s_max));
  }

  constexpr double kLateralEnterThres = 0.5;  // m.
  const auto [boundary_l_max, boundary_l_min] =
      CalcSlBoundaries(sl_boundary, object_frenet_box);
  // Filter objects not on path boundary.
  if (object_frenet_box.l_min > boundary_l_max - kLateralEnterThres ||
      object_frenet_box.l_max < boundary_l_min + kLateralEnterThres) {
    return absl::OutOfRangeError(absl::StrFormat(
        "Object %s out of lateral boundary, l range: (%.2f, %.2f)", object.id(),
        object_frenet_box.l_min, object_frenet_box.l_max));
  }
  return object_frenet_box;
}

ObjectsOnLane FindFrontObjectsOnLane(
    const DrivePassage& passage, const PathSlBoundary& sl_boundary,
    absl::Span<const SpacetimeObjectTrajectory> st_trajs,
    const FrenetBox& ego_frenet_box) {
  ObjectsOnLane st_trajs_on_lane;
  st_trajs_on_lane.reserve(st_trajs.size());

  for (const auto& st_traj : st_trajs) {
    // Filter by object type.
    if (!IsLeadingObjectType(st_traj.planner_object().type())) {
      continue;
    }

    // Filter oncoming object.
    const auto res =
        IsOncomingObjectJudgeByDrivePassage(passage, st_traj.pose());
    if (!res.ok() || *res == true) {
      continue;
    }

    // Filter by drive passage and ego frenet box.
    ASSIGN_OR_CONTINUE(
        const auto obj_fbox,
        FilterObjectViaDrivePassage(st_traj.planner_object(), passage,
                                    sl_boundary, ego_frenet_box));
    st_trajs_on_lane.emplace_back(obj_fbox, &st_traj);
  }

  // Sort by objects arc length on lane path.
  std::stable_sort(st_trajs_on_lane.begin(), st_trajs_on_lane.end(),
                   [](const auto& a, const auto& b) {
                     return a.first.s_min < b.first.s_min;
                   });

  return st_trajs_on_lane;
}

// According the first lane id of traffic waiting queue's lane path to match
// current lane path.
absl::flat_hash_set<std::string_view> CollectTrafficWaitingObjectOnCurrentLane(
    const SceneOutputProto& scene_reasoning,
    const mapping::LanePath& lane_path) {
  absl::flat_hash_set<std::string_view> traffic_waiting_objects;
  for (const auto& traffic_waiting_queue :
       scene_reasoning.traffic_waiting_queue()) {
    const auto& lane_ids = lane_path.lane_ids();
    if (!traffic_waiting_queue.has_lane_path() ||
        traffic_waiting_queue.lane_path().lane_ids().empty()) {
      continue;
    }
    const mapping::ElementId first_lane_id(
        traffic_waiting_queue.lane_path().lane_ids(0));

    if (std::find(lane_ids.begin(), lane_ids.end(), first_lane_id) !=
        lane_ids.end()) {
      for (const auto& object_id : traffic_waiting_queue.object_id()) {
        traffic_waiting_objects.insert(object_id);
      }
    }
  }
  return traffic_waiting_objects;
}

bool IsObjectAvoidableWithinSlBoundary(const PathSlBoundary& sl_boundary,
                                       const FrenetBox& obj_frenet_box,
                                       double ego_width) {
  constexpr double kSampleStepAlongS = 1.0;  // m.
  double min_left_space = DBL_MAX, min_right_space = DBL_MAX;
  for (double sample_s = obj_frenet_box.s_min; sample_s <= obj_frenet_box.s_max;
       sample_s += kSampleStepAlongS) {
    const auto [right_l, left_l] = sl_boundary.QueryTargetBoundaryL(sample_s);
    min_left_space = std::clamp(
        left_l - std::max(right_l, obj_frenet_box.l_max), 0.0, min_left_space);
    min_right_space = std::clamp(
        std::min(left_l, obj_frenet_box.l_min) - right_l, 0.0, min_right_space);
  }

  return std::max(min_left_space, min_right_space) > ego_width;
}

// Check whether object within traffic light controlled intersection.
bool IsObjectWithinTlControlledIntersection(
    const PlannerSemanticMapManager& psmm, const FrenetBox& obj_frenet_box,
    const mapping::LanePath& lane_path, double s_offset) {
  for (const auto& seg : lane_path) {
    const auto* lane_info_ptr = psmm.FindLaneInfoOrNull(seg.lane_id);
    if (lane_info_ptr == nullptr) continue;
    if (lane_info_ptr->is_in_intersection == false) continue;
    if (!IsTrafficLightControlledLane(*QCHECK_NOTNULL(lane_info_ptr->proto))) {
      return false;
    }
    for (const auto& [id, frac] : lane_info_ptr->Intersections()) {
      const auto* intersection_ptr = psmm.FindIntersectionByIdOrNull(id);
      if (intersection_ptr == nullptr ||
          !intersection_ptr->proto->traffic_light_controlled()) {
        continue;
      }

      const double intersection_start_s =
          lane_path.LaneIndexPointToArclength(seg.lane_index, frac.x());
      const double intersection_end_s =
          lane_path.LaneIndexPointToArclength(seg.lane_index, frac.y());
      if (obj_frenet_box.s_min < intersection_end_s + s_offset &&
          obj_frenet_box.s_max > intersection_start_s + s_offset) {
        return true;
      }
    }
    if (seg.end_s + s_offset > obj_frenet_box.s_max) break;
  }
  return false;
}

bool IsObjectBlockingRefCenter(const PathSlBoundary& sl_boundary,
                               const FrenetBox& obj_frenet_box,
                               double ego_half_width) {
  const double obj_l_offset =
      sl_boundary.QueryReferenceCenterL(obj_frenet_box.center_s());

  return obj_frenet_box.l_max > obj_l_offset - ego_half_width &&
         obj_frenet_box.l_min < obj_l_offset + ego_half_width;
}

bool IsObjectBlockingEgoVehicle(const PathSlBoundary& sl_boundary,
                                const FrenetBox& obj_frenet_box,
                                const FrenetBox& ego_frenet_box) {
  const double ego_l_offset =
      sl_boundary.QueryReferenceCenterL(ego_frenet_box.center_s());
  const double obj_l_offset =
      sl_boundary.QueryReferenceCenterL(obj_frenet_box.center_s());

  // Check object lateral position.
  if (obj_frenet_box.l_max - obj_l_offset <
          ego_frenet_box.l_min - ego_l_offset ||
      obj_frenet_box.l_min - obj_l_offset >
          ego_frenet_box.l_max - ego_l_offset) {
    return false;
  }
  return true;
}

}  // namespace

std::vector<ConstraintProto::LeadingObjectProto> FindLeadingObjects(
    const PlannerSemanticMapManager& psmm, const DrivePassage& passage,
    const PathSlBoundary& sl_boundary, LaneChangeStage lc_stage,
    const SceneOutputProto& scene_reasoning,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const absl::flat_hash_set<std::string>& stalled_objects,
    const ApolloTrajectoryPointProto& plan_start_point,
    const qcraft::VehicleGeometryParamsProto& vehicle_geometry_params,
    const FrenetBox& ego_frenet_box, bool borrow_lane_boundary) {
  FUNC_QTRACE();

  std::vector<ConstraintProto::LeadingObjectProto> leading_objects;

  // Collect traffic waiting objects on current lane path.
  const auto traffic_waiting_objects = CollectTrafficWaitingObjectOnCurrentLane(
      scene_reasoning, passage.lane_path());

  // Find leading objects on current lane.
  const auto st_trajs_on_lane = FindFrontObjectsOnLane(
      passage, sl_boundary, st_traj_mgr.trajectories(), ego_frenet_box);
  leading_objects.reserve(st_trajs_on_lane.size());

  const double ego_width = vehicle_geometry_params.width();
  const double ego_half_width = 0.5 * ego_width;
  const bool lc_ongoing = (lc_stage == LaneChangeStage::LCS_EXECUTING ||
                           lc_stage == LaneChangeStage::LCS_RETURN);

  // Create forbidden to nudge leading objects.
  for (const auto& [obj_frenet_box, traj_ptr] : st_trajs_on_lane) {
    const auto& obj_id = traj_ptr->planner_object().id();

    // Filter objects by ego heading.
    if (IsOncomingObjectJudgeByEgoHeading(plan_start_point, traj_ptr->pose())) {
      continue;
    }

    // Stall object is not leading object.
    if (stalled_objects.contains(obj_id)) continue;

    // Traffic waiting object on current lane path is leading object.
    if (traffic_waiting_objects.contains(obj_id)) {
      leading_objects.push_back(CreateLeadingObject(
          *traj_ptr, passage,
          ConstraintProto::LeadingObjectProto::TRAFFIC_WAITING));
      continue;
    }

    if ((lc_ongoing && IsObjectBlockingRefCenter(sl_boundary, obj_frenet_box,
                                                 ego_half_width)) ||
        (!lc_ongoing && IsObjectBlockingEgoVehicle(sl_boundary, obj_frenet_box,
                                                   ego_frenet_box))) {
      // All blocking objects within borrow-lane sl boundary.
      if (borrow_lane_boundary) {
        leading_objects.push_back(CreateLeadingObject(
            *traj_ptr, passage,
            ConstraintProto::LeadingObjectProto::BORROW_BOUNDARY));
        continue;
      }

      // All blocking objects within a traffic light controlled intersection.
      if (IsObjectWithinTlControlledIntersection(psmm, obj_frenet_box,
                                                 passage.lane_path(),
                                                 passage.lane_path_start_s())) {
        leading_objects.push_back(CreateLeadingObject(
            *traj_ptr, passage,
            ConstraintProto::LeadingObjectProto::INTERSECTION));
        continue;
      }
    }

    // Unavoidable object is leading object.
    if (!IsObjectAvoidableWithinSlBoundary(sl_boundary, obj_frenet_box,
                                           ego_width)) {
      leading_objects.push_back(CreateLeadingObject(
          *traj_ptr, passage,
          ConstraintProto::LeadingObjectProto::UNABLE_TO_OVERTAKE));
      continue;
    }

    // TODO(jiayu): more strategy to make leading object decision.
  }
  return leading_objects;
}

std::vector<ConstraintProto::LeadingObjectProto>
DeriveLeadingObjectsFromCaptainNetTrajectory(
    const SpacetimeTrajectoryManager& st_traj_mgr, const DrivePassage& passage,
    const absl::flat_hash_set<std::string>& stalled_objects,
    const qcraft::VehicleGeometryParamsProto& vehicle_geometry_params,
    const std::vector<ApolloTrajectoryPointProto>& traj_points) {
  std::vector<ConstraintProto::LeadingObjectProto> leading_objects;

  // Build captainnet frenet frame
  std::vector<Vec2d> ref_points;
  ref_points.reserve(traj_points.size());
  for (const auto& traj_pt : traj_points) {
    ref_points.push_back(Vec2dFromApolloTrajectoryPointProto(traj_pt));
  }
  ASSIGN_OR_RETURN(const auto ref_frame,
                   BuildBruteForceFrenetFrame(ref_points,
                                              /*down_sample_raw_points=*/true),
                   leading_objects);

  for (const auto& st_traj : st_traj_mgr.trajectories()) {
    // Filter objects not leading type
    if (!IsLeadingObjectType(st_traj.planner_object().type())) {
      continue;
    }

    // Stall object is not leading object.
    if (stalled_objects.contains(st_traj.planner_object().id())) {
      continue;
    }

    // Calculate object frenet coordinate
    ASSIGN_OR_CONTINUE(
        const auto object_frenet_box,
        ref_frame.QueryFrenetBoxAtContour(st_traj.planner_object().contour()));

    // Filter objects behind ego front edge or beyond captain trajectory max
    // length
    const double beyond_s_buffer = vehicle_geometry_params.length();
    if (object_frenet_box.s_min > ref_frame.end_s() + beyond_s_buffer ||
        object_frenet_box.s_max < ref_frame.start_s()) {
      continue;
    }

    // Filter object far away from capnet trajectory
    const auto object_pos_in_frame = ref_frame.XYToSL(st_traj.pose().pos());
    const double k_max_l_from_captain_net_traj =
        vehicle_geometry_params.width();
    if (std::abs(object_pos_in_frame.l) > k_max_l_from_captain_net_traj) {
      continue;
    }

    // Filter object with large heading diff from capnet trajectory
    const auto angle_vec_ref =
        ref_frame.InterpolateTangentByS(object_pos_in_frame.s);
    const double angle_ref = NormalizeAngle2D(angle_vec_ref);
    const double angle_leading = st_traj.pose().theta();
    if (std::abs(NormalizeAngle(angle_leading - angle_ref)) < M_PI_2) {
      leading_objects.push_back(CreateLeadingObject(
          st_traj, passage,
          ConstraintProto::LeadingObjectProto::DERIVED_FROM_CAPTAINNET));
    }
  }
  return leading_objects;
}

}  // namespace planner
}  // namespace qcraft
