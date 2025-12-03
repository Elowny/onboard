#ifndef ONBOARD_PATH_BOUNDARY_BUILDER_HELPER_H_
#define ONBOARD_PATH_BOUNDARY_BUILDER_HELPER_H_

#include <algorithm>
#include <string>  // for string
#include <utility>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "absl/types/span.h"

#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/math/frenet_common.h"
#include "onboard/math/vec.h"
#include "onboard/planner/common/path_sl_boundary.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/planner/scheduler/proto/lane_change.pb.h"
#include "onboard/planner/scheduler/smooth_reference_line_result.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft::planner {

class PathBoundary {
 public:
  PathBoundary(std::vector<double> right, std::vector<double> left)
      : right_(std::move(right)), left_(std::move(left)) {}

  const std::vector<double>& right_vec() const { return right_; }

  const std::vector<double>& left_vec() const { return left_; }

  // The following two function will move the member to the caller.
  std::vector<double>&& moved_left_vec() { return std::move(left_); }
  std::vector<double>&& moved_right_vec() { return std::move(right_); }

  double right(int i) const { return right_[i]; }

  double left(int i) const { return left_[i]; }

  void ShiftLeftByIndex(int index, double offset) { left_[index] += offset; }

  void ShiftRightByIndex(int index, double offset) { right_[index] += offset; }

  void OuterClampRightByIndex(int index, double right_l) {
    right_[index] = std::max(right_[index], right_l);
  }

  void OuterClampLeftByIndex(int index, double left_l) {
    left_[index] = std::min(left_[index], left_l);
  }

  void OuterClampBy(const PathBoundary& other) {
    const int n = static_cast<int>(left_.size());
    QCHECK_EQ(n, right_.size());
    QCHECK_LE(n, other.right_vec().size());
    for (int i = 0; i < n; ++i) {
      left_[i] = std::min(left_[i], other.left(i));
      right_[i] = std::max(right_[i], other.right(i));
    }
  }

  void InnerClampBy(const PathBoundary& other) {
    const int n = static_cast<int>(left_.size());
    QCHECK_EQ(n, right_.size());
    QCHECK_LE(n, other.size());
    for (int i = 0; i < n; ++i) {
      left_[i] = std::max(left_[i], other.left(i));
      right_[i] = std::min(right_[i], other.right(i));
    }
  }

  void EraseFrom(int index) {
    QCHECK_LT(index, size());
    left_.resize(index);
    right_.resize(index);
  }

  int size() const { return left_.size(); }

 private:
  std::vector<double> right_;
  std::vector<double> left_;
};

PathBoundary BuildPathBoundaryFromTargetLane(
    const PlannerSemanticMapManager& psmm, const DrivePassage& drive_passage,
    double min_half_lane_width, bool borrow_lane_boundary);

PathBoundary BuildPathBoundaryFromOnlineTargetLane(
    const PlannerSemanticMapManager& psmm, const DrivePassage& drive_passage,
    double min_half_lane_width);

PathBoundary BuildCurbPathBoundary(const DrivePassage& drive_passage);

PathBoundary BuildSolidPathBoundary(
    const DrivePassage& drive_passage, const FrenetCoordinate& cur_sl,
    const VehicleGeometryParamsProto& vehicle_geom, double target_lane_offset);

PathBoundary BuildPathBoundaryFromAvKinematics(
    const DrivePassage& drive_passage,
    const ApolloTrajectoryPointProto& plan_start_point,
    const VehicleGeometryParamsProto& vehicle_geom,
    const FrenetCoordinate& cur_sl, const FrenetBox& sl_box,
    absl::Span<const double> s_vec, double target_lane_offset,
    double max_lane_change_lat_accel, bool lane_change_pause);

PathBoundary BuildPathBoundaryFromEgoProtectBuffer(
    const DrivePassage& drive_passage,
    const ApolloTrajectoryPointProto& plan_start_point,
    const VehicleGeometryParamsProto& vehicle_geom);

PathBoundary ShrinkPathBoundaryForLaneChangePause(
    const VehicleGeometryParamsProto& vehicle_geom, const FrenetBox& sl_box,
    const LaneChangeStateProto& lc_state, PathBoundary boundary,
    double target_lane_offset);

PathBoundary ShrinkPathBoundaryForObject(
    const DrivePassage& drive_passage,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const ApolloTrajectoryPointProto& plan_start_point,
    absl::Span<const double> s_vec, absl::Span<const double> center_l,
    absl::Span<const Vec2d> center_xy, const PathBoundary& inner_boundary,
    const PathBoundary& curb_boundary, PathBoundary boundary);

double ComputeTargetLaneOffset(
    const DrivePassage& drive_passage, const FrenetCoordinate& cur_sl,
    const LaneChangeStateProto& lc_state,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const absl::flat_hash_set<std::string>& unsafe_object_ids,
    const ApolloTrajectoryPointProto& plan_start_point, double half_av_width);

PathSlBoundary BuildPathSlBoundary(const DrivePassage& drive_passage,
                                   std::vector<double> s_vec,
                                   std::vector<double> ref_center_l,
                                   PathBoundary inner_boundary,
                                   PathBoundary outer_boundary);

std::vector<double> ComputeSmoothedReferenceLine(
    const PlannerSemanticMapManager& psmm, const DrivePassage& drive_passage,
    const SmoothedReferenceLineResultMap& smooth_result_map);

}  // namespace qcraft::planner

#endif  // ONBOARD_PATH_BOUNDARY_BUILDER_HELPER_H_
