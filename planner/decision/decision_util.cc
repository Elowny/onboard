#include "onboard/planner/decision/decision_util.h"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <optional>
#include <ostream>
#include <string>
#include <utility>
#include <vector>

#include "absl/status/statusor.h"
#include "google/protobuf/repeated_ptr_field.h"

#include "onboard/container/strong_int.h"
#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/math/util.h"
#include "onboard/planner/decision/traffic_gap_result.h"
#include "onboard/planner/decision/traffic_gap_v2/proto/traffic_gap.pb.h"
#include "onboard/planner/object/spacetime_object_state.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/prediction/predicted_trajectory.h"
#include "onboard/utils/status_macros.h"

namespace qcraft {
namespace planner {

namespace {

constexpr double kEpsilon = 1e-3;

SpeedProfile IntegrateSpeedProfile(const DrivePassage& drive_passage,
                                   const std::vector<double>& v_s) {
  // Integrate the v-s profile in time.
  QCHECK_EQ(drive_passage.size(), v_s.size());
  std::vector<double> path_s(drive_passage.size());
  for (int i = 0; i < drive_passage.size(); ++i) {
    path_s[i] = drive_passage.station(StationIndex(i)).accumulated_s();
  }
  const PiecewiseSqrtFunction<double, double> v_s_plf(path_s, v_s);
  std::vector<double> t(kTrajectorySteps);
  for (int i = 0; i < t.size(); ++i) t[i] = i * kTrajectoryTimeStep;
  std::vector<double> s(t.size());
  double current_s = 0.0;
  for (int i = 0; i < t.size(); ++i) {
    s[i] = current_s;
    const double current_v = v_s_plf(current_s);
    current_s += std::max(current_v, kEpsilon) * kTrajectoryTimeStep;
    current_s = std::min(path_s.back(), current_s);
  }

  // Create the speed profile from the s-t function.
  return SpeedProfile(PiecewiseLinearFunction(std::move(t), std::move(s)));
}

void ApplySpeedConstraintForReferenceSpeedProfile(
    double s_start, double s_end, double v_max,
    const DrivePassage& drive_passage, std::vector<double>* v_s) {
  QCHECK_LE(s_start, s_end);
  QCHECK_EQ(drive_passage.size(), v_s->size());
  // Max speed dictated by this speed constraint as a function of s.
  // Between s_start and s_end, max speed is v_max. Before that the max
  // speed follows a constant deceleration profile, and after that a
  // constant acceleration profile.
  const double v_max_sqr = Sqr(v_max);
  const auto compute_v_constraint_sqr = [s_start, s_end, v_max_sqr](double s) {
    constexpr double kAccelLimit = 2.0;  // m/s^2.
    if (s < s_start) {
      // Deceleration segment.
      return v_max_sqr + (s_start - s) * (kAccelLimit * 2.0);
    }
    if (s > s_end) {
      // Acceleration segment.
      return v_max_sqr + (s - s_end) * (kAccelLimit * 2.0);
    }
    return v_max_sqr;
  };

  for (int i = 0; i < drive_passage.size(); ++i) {
    const double s = drive_passage.station(StationIndex(i)).accumulated_s();
    const double v_constraint_sqr = compute_v_constraint_sqr(s);
    if (v_constraint_sqr < Sqr((*v_s)[i])) {
      (*v_s)[i] = std::sqrt(v_constraint_sqr);
    }
  }
}

}  // namespace

SpeedProfile CreateSpeedProfile(
    double v_now, const DrivePassage& drive_passage,
    const absl::Span<const ConstraintProto::SpeedRegionProto>& speed_zones,
    const absl::Span<const ConstraintProto::StopLineProto>& stop_points) {
  const auto dp_size = drive_passage.size();

  std::vector<double> v_s;
  v_s.reserve(dp_size);
  for (int i = 0; i < dp_size; ++i) {
    const double speed_limit_mps =
        drive_passage.station(StationIndex(i)).speed_limit();  // m/s
    v_s.push_back(speed_limit_mps);
  }

  for (const auto& speed_zone : speed_zones) {
    ApplySpeedConstraintForReferenceSpeedProfile(
        speed_zone.start_s(), speed_zone.end_s(), speed_zone.max_speed(),
        drive_passage, &v_s);
  }
  for (const auto& stop_point : stop_points) {
    ApplySpeedConstraintForReferenceSpeedProfile(
        stop_point.s(), /*s_end=*/std::numeric_limits<double>::infinity(),
        /*v_max=*/0.0, drive_passage, &v_s);
  }

  for (int i = 0; i < dp_size; ++i) {
    const double s = drive_passage.station(StationIndex(i)).accumulated_s();
    // Note(jiayu): why? Refactor later.
    const double v_constraint =
        std::pow(std::abs(s) + pow(std::abs(v_now) / 2.3, 2.5), 0.4) * 2.3;
    (v_s)[i] = std::min((v_s)[i], v_constraint);
  }

  return IntegrateSpeedProfile(drive_passage, v_s);
}

ConstraintProto::SpeedRegionProto MergeSameElement(
    absl::Span<const ConstraintProto::SpeedRegionProto> elements) {
  QCHECK_GT(elements.size(), 0);
  ConstraintProto::SpeedRegionProto merged_ele = elements[0];
  int begin_idx = 0;
  double min_s = std::numeric_limits<double>::infinity();
  int end_idx = 0;
  double max_s = -std::numeric_limits<double>::infinity();
  for (int i = 0, n = elements.size(); i < n; ++i) {
    const auto& ele = elements[i];
    if (ele.start_s() < min_s) {
      min_s = ele.start_s();
      begin_idx = i;
    }
    if (ele.end_s() > max_s) {
      max_s = ele.end_s();
      end_idx = i;
    }
  }
  merged_ele.set_start_s(min_s);
  merged_ele.set_end_s(max_s);
  *merged_ele.mutable_start_point() = elements[begin_idx].start_point();
  *merged_ele.mutable_end_point() = elements[end_idx].end_point();
  return merged_ele;
}

void FillDecisionConstraintDebugInfo(const ConstraintManager& constraint_mgr,
                                     ConstraintProto* constraint) {
  QCHECK_NOTNULL(constraint);
  constraint->Clear();
  for (const auto& stop_line : constraint_mgr.StopLine()) {
    constraint->add_stop_line()->CopyFrom(stop_line);
  }
  for (const auto& speed_region : constraint_mgr.SpeedRegion()) {
    constraint->add_speed_region()->CopyFrom(speed_region);
  }
  for (const auto& path_speed_region : constraint_mgr.PathSpeedRegion()) {
    constraint->add_path_speed_region()->CopyFrom(path_speed_region);
  }
  for (const auto& path_stop_line : constraint_mgr.PathStopLine()) {
    constraint->add_path_stop_line()->CopyFrom(path_stop_line);
  }
  for (const auto& avoid_line : constraint_mgr.AvoidLine()) {
    constraint->add_avoid_line()->CopyFrom(avoid_line);
  }
  for (const auto& speed_profile : constraint_mgr.SpeedProfiles()) {
    constraint->add_speed_profile()->CopyFrom(speed_profile);
  }

  const auto& traffic_gap = constraint_mgr.TrafficGap();
  if (traffic_gap.leader_id.has_value()) {
    constraint->mutable_traffic_gap()->set_leader_id(*traffic_gap.leader_id);
  }
  if (traffic_gap.follower_id.has_value()) {
    constraint->mutable_traffic_gap()->set_follower_id(
        *traffic_gap.follower_id);
  }
  *constraint->mutable_traffic_gap_debug() = constraint_mgr.TrafficGapDebug();
}

bool IsLeadingObjectType(ObjectType type) {
  switch (type) {
    case OT_VEHICLE:
    case OT_LARGE_VEHICLE:
    case OT_UNKNOWN_MOVABLE:
    case OT_MOTORCYCLIST:
      return true;
    case OT_UNKNOWN_STATIC:
    case OT_PEDESTRIAN:
    case OT_CYCLIST:
    case OT_TRICYCLIST:
    case OT_FOD:
    case OT_VEGETATION:
    case OT_BARRIER:
    case OT_BARRIER_ANTI_COLLISION_BUCKET:
    case OT_BARRIER_ANTI_COLLISION_POST:
    case OT_CONE:
    case OT_WARNING_TRIANGLE:
      return false;
  }
}

std::pair<double, double> CalcSlBoundaries(const PathSlBoundary& sl_boundary,
                                           const FrenetBox& frenet_box) {
  if (frenet_box.s_max < sl_boundary.start_s() ||
      frenet_box.s_min > sl_boundary.end_s()) {
    // If the box is completely out of path boundary.
    return {0.0, 0.0};
  }

  const auto [boundary_front_right_l, boundary_front_left_l] =
      sl_boundary.QueryBoundaryL(frenet_box.s_max);
  const double boundary_front_center_l =
      sl_boundary.QueryReferenceCenterL(frenet_box.s_max);

  const auto [boundary_rear_right_l, boundary_rear_left_l] =
      sl_boundary.QueryBoundaryL(frenet_box.s_min);
  const double boundary_rear_center_l =
      sl_boundary.QueryReferenceCenterL(frenet_box.s_min);

  constexpr double kMinHalfBoundaryWidth = 1.1;  // m.
  const double boundary_l_min =
      std::min(std::min(boundary_front_right_l,
                        boundary_front_center_l - kMinHalfBoundaryWidth),
               std::min(boundary_rear_right_l,
                        boundary_rear_center_l - kMinHalfBoundaryWidth));
  const double boundary_l_max =
      std::max(std::max(boundary_front_left_l,
                        boundary_front_center_l + kMinHalfBoundaryWidth),
               std::max(boundary_rear_left_l,
                        boundary_rear_center_l + kMinHalfBoundaryWidth));

  return {boundary_l_max, boundary_l_min};
}

ConstraintProto::LeadingObjectProto CreateLeadingObject(
    const SpacetimeObjectTrajectory& traj, const DrivePassage& passage,
    ConstraintProto::LeadingObjectProto::Reason reason) {
  ConstraintProto::LeadingObjectProto leading_object;
  leading_object.set_traj_id(std::string(traj.traj_id()));
  leading_object.set_reason(reason);

  constexpr double kSampleTimeInterval = 1.0;  // Seconds.
  // Generate ST-constraints based on object current bounding box, for
  // stationary object.
  if (traj.is_stationary()) {
    ASSIGN_OR_RETURN(const auto obj_frenet_box,
                     passage.QueryFrenetBoxAt(traj.bounding_box()),
                     leading_object);
    for (double sample_time = 0.0; sample_time <= kTrajectoryTimeHorizon;
         sample_time += kSampleTimeInterval) {
      auto* constraint = leading_object.add_st_constraints();
      constraint->set_s(obj_frenet_box.s_min);
      constraint->set_t(sample_time);
    }
    if (kTrajectoryTimeHorizon - leading_object.st_constraints().rbegin()->t() >
        kEpsilon) {
      auto* constraint = leading_object.add_st_constraints();
      constraint->set_s(obj_frenet_box.s_min);
      constraint->set_t(kTrajectoryTimeHorizon);
    }

    return leading_object;
  }

  double sample_time = 0.0;
  const double traj_last_time = traj.states().back().traj_point->t();
  // Generate ST-constraints based on spacetime states, for moving object.
  for (const auto& state : traj.states()) {
    // Sample ST-constraints by 1.0s.
    const auto* traj_point = state.traj_point;
    const double t = traj_point->t();
    // Generate ST-constraints at sample time and trajectory last time.
    if (std::abs(t - sample_time) > kEpsilon &&
        std::abs(traj_last_time - t) > kEpsilon) {
      continue;
    }
    // Filter object state out of passage.
    ASSIGN_OR_CONTINUE(const auto obj_frenet_box,
                       passage.QueryFrenetBoxAt(state.box));
    // Filter object state by drive passage direction.
    ASSIGN_OR_CONTINUE(const auto passage_tangent,
                       passage.QueryTangentAngleAtS(obj_frenet_box.s_min));
    const double angle_diff =
        std::abs(NormalizeAngle(passage_tangent - traj_point->theta()));
    if (angle_diff > M_PI_2) continue;

    auto* constraint = leading_object.add_st_constraints();
    constraint->set_s(obj_frenet_box.s_min);
    constraint->set_t(t);

    sample_time += kSampleTimeInterval;
  }

  return leading_object;
}

bool IsTrafficLightControlledLane(const mapping::LaneProto& lane) {
  return !lane.startpoint_associated_traffic_lights().empty() ||
         !lane.multi_traffic_light_control_points().empty();
}

}  // namespace planner
}  // namespace qcraft
