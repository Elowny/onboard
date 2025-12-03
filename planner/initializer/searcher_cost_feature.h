#ifndef ONBOARD_PLANNER_INITIALIZER_SEARCHER_COST_FEATURE_H_
#define ONBOARD_PLANNER_INITIALIZER_SEARCHER_COST_FEATURE_H_

#include <algorithm>
#include <limits>
#include <numeric>
#include <string>
#include <vector>

#include "absl/types/span.h"

#include "onboard/math/piecewise_linear_function.h"
#include "onboard/planner/common/path_sl_boundary.h"
#include "onboard/planner/initializer/collision_checker.h"
#include "onboard/planner/initializer/cost_feature.h"
#include "onboard/planner/initializer/dp_motion_searcher_defs.h"
#include "onboard/planner/initializer/ref_speed_table.h"
#include "onboard/planner/ml/captain_net/captain_net.h"
#include "onboard/planner/ml/captain_net/utils/cost_utils.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/proto/trajectory_point.pb.h"

namespace qcraft::planner {
class AccelerationFeatureCost : public FeatureCost {
 public:
  explicit AccelerationFeatureCost(
      const MotionConstraintParamsProto& motion_constraint_params)
      : FeatureCost("acceleration"),
        max_accel_constraint_(motion_constraint_params.max_acceleration()),
        max_decel_constraint_(motion_constraint_params.max_deceleration()) {}

  void ComputeCost(const MotionEdgeInfo& edge_info,
                   absl::Span<double> cost) const override;

 private:
  double max_accel_constraint_;
  double max_decel_constraint_;
};

class LaneBoundaryFeatureCost : public FeatureCost {
 public:
  explicit LaneBoundaryFeatureCost(const PathSlBoundary* path_sl,
                                   double sdc_half_width)
      : FeatureCost("lane_boundary"),
        path_sl_(path_sl),
        sdc_half_width_(sdc_half_width) {}

  void ComputeCost(const MotionEdgeInfo& edge_info,
                   absl::Span<double> cost) const override;

 private:
  // Not owned.
  const PathSlBoundary* path_sl_;
  double sdc_half_width_;
};

class CurvatureFeatureCost : public FeatureCost {
 public:
  CurvatureFeatureCost() : FeatureCost("curvature") {}

  void ComputeCost(const MotionEdgeInfo& edge_info,
                   absl::Span<double> cost) const override;
};

class LateralAccelerationFeatureCost : public FeatureCost {
 public:
  explicit LateralAccelerationFeatureCost(bool is_lane_change)
      : FeatureCost("lateral_acceleration"), is_lane_change_(is_lane_change) {}

  void ComputeCost(const MotionEdgeInfo& edge_info,
                   absl::Span<double> cost) const override;

 private:
  bool is_lane_change_;
};

class StopConstraintFeatureCost : public FeatureCost {
 public:
  explicit StopConstraintFeatureCost(const std::vector<double>& stop_s)
      : FeatureCost("stop_constraint"),
        nearest_stop_s_(stop_s.empty()
                            ? std::numeric_limits<double>::max()
                            : *std::min_element(stop_s.begin(), stop_s.end())) {
  }

  void ComputeCost(const MotionEdgeInfo& edge_info,
                   absl::Span<double> cost) const override;

 private:
  double nearest_stop_s_;
};

class RefSpeedFeatureCost : public FeatureCost {
 public:
  explicit RefSpeedFeatureCost(const RefSpeedTable* ref_speed_table)
      : FeatureCost("ref_speed"), ref_speed_table_(ref_speed_table) {}

  void ComputeCost(const MotionEdgeInfo& edge_info,
                   absl::Span<double> cost) const override;

 private:
  const RefSpeedTable* ref_speed_table_;
};

class DynamicCollisionFeatureCost : public FeatureCost {
 public:
  explicit DynamicCollisionFeatureCost(
      const CollisionChecker* collision_checker);

  void ComputeCost(const MotionEdgeInfo& edge_info,
                   absl::Span<double> cost) const override;

  // Provide & Update a set of ignorable trajectory ids.
  IgnoreTrajMap ComputeInteractiveCost(const MotionEdgeInfo& edge_info,
                                       const IgnoreTrajMap& ignored_trajs,
                                       absl::Span<double> cost) const;

 private:
  // Not owned.
  const CollisionChecker* cc_;
};

class LeadingObjectFeatureCost : public FeatureCost {
 public:
  // Construct the LeadingObjectFeatureCost without specifying the leading
  // objects.
  explicit LeadingObjectFeatureCost(
      double traj_horizon, const DrivePassage& drive_passage,
      const SpacetimeTrajectoryManager& st_traj_mgr,
      const std::vector<std::vector<std::string>>& leading_groups,
      const int leading_group_idx, double ego_front_to_ra,
      double ego_back_to_ra);

  void ComputeCost(const MotionEdgeInfo& edge_info,
                   absl::Span<double> cost) const override;

 private:
  double traj_horizon_;
  int leading_group_idx_;
  double ego_front_to_ra_;
  double ego_back_to_ra_;
  PiecewiseLinearFunction<double> max_s_t_;
  double min_s_;
};

class AstarLeadingObjectFeatureCost : public FeatureCost {
 public:
  // Construct the LeadingObjectFeatureCost without specifying the leading
  // objects.
  explicit AstarLeadingObjectFeatureCost(
      double traj_horizon, const DrivePassage& drive_passage,
      const SpacetimeTrajectoryManager& st_traj_mgr,
      const std::vector<std::vector<std::string>>& leading_groups,
      const int leading_group_idx, const bool able_to_overtake_leading_behind,
      double ego_front_to_ra, double ego_back_to_ra);

  void ComputeCost(const MotionEdgeInfo& edge_info,
                   absl::Span<double> cost) const override;

 private:
  double traj_horizon_;
  int leading_group_idx_;
  double ego_front_to_ra_;
  double ego_back_to_ra_;
  bool able_to_overtake_leading_behind_;
  PiecewiseLinearFunction<double> max_s_t_;
  double min_s_;
};

class RefTrajectoryFeatureCost : public FeatureCost {
 public:
  explicit RefTrajectoryFeatureCost(
      const ml::captain_net::CaptainNetOutput* captain_net_output)
      : FeatureCost("ref_traj"), captain_net_output_(captain_net_output) {
    if (!captain_net_output_->traj_points.empty()) {
      ref_traj_weight_ =
          std::vector<double>(captain_net_output_->traj_points.size(), 1.0);
      // TODO(jingqiao): Move gamma to planner default params after
      // testing.
      constexpr double kTimeDecayRate = 0.99;
      ml::CalculateTimeDecayWeight(kTimeDecayRate, &ref_traj_weight_);
      reg_factor_ = std::max(std::accumulate(ref_traj_weight_.begin(),
                                             ref_traj_weight_.end(), 0.0),
                             1e-6);
    }
  }

  void ComputeCost(const MotionEdgeInfo& edge_info,
                   absl::Span<double> cost) const override;

 private:
  const ml::captain_net::CaptainNetOutput* captain_net_output_;
  std::vector<double> ref_traj_weight_;
  double reg_factor_ = 1.0;
};

class FinalProgressFeatureCost : public FeatureCost {
 public:
  explicit FinalProgressFeatureCost(double traj_horizon,
                                    const PathSlBoundary* path_sl,
                                    double max_accumulated_s)
      : FeatureCost("final_progress"),
        traj_horizon_(traj_horizon),
        path_sl_(path_sl),
        max_accumulated_s_(max_accumulated_s) {}

  void ComputeCost(const MotionEdgeInfo& edge_info,
                   absl::Span<double> cost) const override;

 private:
  double traj_horizon_;
  const PathSlBoundary* path_sl_;
  double max_accumulated_s_;
};

}  // namespace qcraft::planner
#endif  // ONBOARD_PLANNER_INITIALIZER_SEARCHER_COST_FEATURE_H_
