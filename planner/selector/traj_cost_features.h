#ifndef ONBOARD_PLANNER_SELECTOR_TRAJ_COST_FEATURES_H_
#define ONBOARD_PLANNER_SELECTOR_TRAJ_COST_FEATURES_H_

#include <algorithm>
#include <iterator>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/hash/hash.h"
#include "absl/status/statusor.h"

#include "onboard/maps/lane_path.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/math/frenet_frame.h"
#include "onboard/math/geometry/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/est_planner_output.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/planner/scheduler/scheduler_output.h"
#include "onboard/planner/selector/candidate_stats.h"
#include "onboard/planner/selector/common_feature.h"
#include "onboard/planner/selector/cost_feature_base.h"
#include "onboard/planner/selector/proto/selector_state.pb.h"
#include "onboard/planner/selector/selector_util.h"
#include "onboard/proto/driving_style.pb.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {
namespace planner {

using CostVec = CostFeatureBase::CostVec;

class TrajProgressCost : public CostFeatureBase {
 public:
  TrajProgressCost(const SelectorCommonFeature* common_feature,
                   const LaneChangeStyle& lane_change_style,
                   const bool planner_enable_obstacle_lane_change,
                   ProgressStats stats)
      : CostFeatureBase("progress",
                        {"progress", "follow_slow", "front_non_block_car"},
                        /*is_common=*/true, common_feature),
        ego_v_(stats.ego_v),
        lane_speed_map_(std::move(stats.lane_speed_map)),
        max_lane_speed_(stats.max_lane_speed),
        min_leader_dist_(stats.min_leader_dist),
        lane_change_style_(lane_change_style),
        planner_enable_obstacle_lane_change_(
            planner_enable_obstacle_lane_change) {}

  absl::StatusOr<CostVec> ComputeCost(
      const EstPlannerOutput& planner_output,
      std::vector<std::string>* extra_info,
      TrajFeatureOutput* traj_feature_output) const override;

 private:
  double ego_v_;
  absl::flat_hash_map<SchedulerOutput::HashType, LaneSpeedInfo> lane_speed_map_;
  double max_lane_speed_;
  double min_leader_dist_;
  LaneChangeStyle lane_change_style_;
  bool planner_enable_obstacle_lane_change_;
};

class TrajMaxJerkCost : public CostFeatureBase {
 public:
  TrajMaxJerkCost(const SelectorCommonFeature* common_feature,
                  const MotionConstraintParamsProto& motion_constraints)
      : CostFeatureBase("max_jerk",
                        {"max_lon_jerk", "max_lat_jerk", "max_lon_deacc"},
                        /*is_common=*/true, common_feature),
        accel_jerk_constraint_(motion_constraints.max_accel_jerk()),
        decel_jerk_constraint_(motion_constraints.max_decel_jerk()),
        lat_jerk_constraint_(motion_constraints.max_lateral_jerk()) {
    for (int i = 0; i < kTrajectorySteps; ++i) {
      coeffs_[i] = ExpDecayCoeffAtStep(10, 0.6, i);
    }
  }

  absl::StatusOr<CostVec> ComputeCost(
      const EstPlannerOutput& planner_output,
      std::vector<std::string>* extra_info,
      TrajFeatureOutput* traj_feature_output) const override;

 private:
  double accel_jerk_constraint_, decel_jerk_constraint_;
  double lat_jerk_constraint_;
  double coeffs_[kTrajectorySteps];  // Decaying factor w.r.t. time step.
};

class TrajLaneChangeCost : public CostFeatureBase {
 public:
  TrajLaneChangeCost(const SelectorCommonFeature* common_feature,
                     const PlannerSemanticMapManager* psmm,
                     const mapping::LanePath* prev_lp_from_current,
                     const PlannerTrajectory* prev_traj,
                     const ApolloTrajectoryPointProto& plan_start_point,
                     const LastLcInfoProto& last_lc_info,
                     double time_since_last_red_light,
                     double time_since_last_lane_change,
                     const bool planner_enable_lane_change_in_intersection,
                     const bool already_turn_on_pre_turn_signal)
      : CostFeatureBase(
            "lane_change",
            {"lane_path_diff", "pose_to_target", "prev_lat_diff",
             "lc_in_curvy_road", "lc_in_intersection", "lc_in_redlight",
             "lc_opposite_direction", "lc_safety_effect",
             "lk_after_turn_signal", "lc_with_opposite_diverge"},
            /*is_common=*/true, common_feature),
        psmm_(psmm),
        prev_lp_from_current_(prev_lp_from_current),
        ego_pos_(Vec2dFromApolloTrajectoryPointProto(plan_start_point)),
        ego_v_(plan_start_point.v()),
        last_lc_info_(last_lc_info),
        time_since_last_red_light_(time_since_last_red_light),
        time_since_last_lane_change_(time_since_last_lane_change),
        planner_enable_lane_change_in_intersection_(
            planner_enable_lane_change_in_intersection),
        already_turn_on_pre_turn_signal_(already_turn_on_pre_turn_signal) {
    std::vector<Vec2d> prev_pts;
    if (prev_traj != nullptr && prev_traj->size() > 1) {
      prev_pts.reserve(prev_traj->size());
      std::transform(prev_traj->begin(), prev_traj->end(),
                     std::back_inserter(prev_pts), [](const auto& traj_pt) {
                       return Vec2dFromApolloTrajectoryPointProto(traj_pt);
                     });
    }
    prev_traj_ff_or_ =
        BuildBruteForceFrenetFrame(prev_pts, /*down_sample_raw_points=*/true);
  }

  absl::StatusOr<CostVec> ComputeCost(
      const EstPlannerOutput& planner_output,
      std::vector<std::string>* extra_info,
      TrajFeatureOutput* traj_feature_output) const override;

 private:
  const PlannerSemanticMapManager* psmm_;
  const mapping::LanePath* prev_lp_from_current_;
  Vec2d ego_pos_;
  double ego_v_;

  LastLcInfoProto last_lc_info_;
  double time_since_last_red_light_;
  double time_since_last_lane_change_;
  bool planner_enable_lane_change_in_intersection_;
  absl::StatusOr<BruteForceFrenetFrame> prev_traj_ff_or_;
  bool already_turn_on_pre_turn_signal_ = false;
};

class TrajCrossSolidBoundaryCost : public CostFeatureBase {
 public:
  explicit TrajCrossSolidBoundaryCost(
      const SelectorCommonFeature* common_feature,
      const VehicleGeometryParamsProto& vehicle_geom,
      const bool planner_enable_cross_solid_boundary)
      : CostFeatureBase(
            "cross_solid_boundary",
            {"solid_white", "solid_yellow", "solid_double_yellow", "curb"},
            /*is_common=*/true, common_feature),
        ego_length_(vehicle_geom.length()),
        ego_width_(vehicle_geom.width()),
        ego_front_to_ra_(vehicle_geom.front_edge_to_center()),
        planner_enable_cross_solid_boundary_(
            planner_enable_cross_solid_boundary) {}

  absl::StatusOr<CostVec> ComputeCost(
      const EstPlannerOutput& planner_output,
      std::vector<std::string>* extra_info,
      TrajFeatureOutput* traj_feature_output) const override;

 private:
  double ego_length_, ego_width_, ego_front_to_ra_;
  bool planner_enable_cross_solid_boundary_;
};

class TrajRouteLookAheadCost : public CostFeatureBase {
 public:
  explicit TrajRouteLookAheadCost(const SelectorCommonFeature* common_feature,
                                  RouteLookAheadStats stats,
                                  bool planner_is_bus_model,
                                  bool use_conservative_ttc)
      : CostFeatureBase("route_look_ahead",
                        {"length_along_route", "reach_destination",
                         "preview_beyond_horizon", "discourage_right_most_lane",
                         "behind_stalled_object", "merge_lane",
                         "encourage_right_most_lane"},
                        /*is_common=*/true, common_feature),
        driving_dist_map_(std::move(stats.driving_dist_map)),
        len_before_intersection_map_(
            std::move(stats.len_before_intersection_map)),
        len_before_merge_lane_map_(std::move(stats.len_before_merge_lane_map)),
        len_along_route_map_(std::move(stats.len_along_route_map)),
        raw_len_along_route_map_(std::move(stats.raw_len_along_route_map)),
        lc_num_to_targets_map_(std::move(stats.lc_num_to_targets_map)),
        lc_num_within_driving_dist_map_(
            std::move(stats.lc_num_within_driving_dist_map)),
        is_right_most_lane_map_(std::move(stats.is_right_most_lane_map)),
        is_valid_merge_lane_map_(std::move(stats.is_valid_merge_lane_map)),
        front_stalled_obj_map_(std::move(stats.front_stalled_obj_map)),
        enable_discourage_right_most_cost_(
            stats.enable_discourage_right_most_cost),
        enable_encourage_right_most_cost_(
            stats.enable_encourage_right_most_cost),
        max_len_along_route_(stats.max_length_along_route),
        min_len_along_route_(stats.min_length_along_route),
        is_non_occluding_leader_(stats.is_non_occluding_leader),
        min_lc_num_(stats.min_lc_num),
        max_lc_num_(stats.max_lc_num),
        is_left_turn_(stats.is_left_turn),
        is_right_turn_(stats.is_right_turn),
        ego_v_(stats.ego_v),
        planner_is_bus_model_(planner_is_bus_model),
        use_conservative_ttc_(use_conservative_ttc) {}

  absl::StatusOr<CostVec> ComputeCost(
      const EstPlannerOutput& planner_output,
      std::vector<std::string>* extra_info,
      TrajFeatureOutput* traj_feature_output) const override;

 private:
  absl::flat_hash_map<mapping::ElementId, double> driving_dist_map_,
      len_before_intersection_map_, len_before_merge_lane_map_;
  absl::flat_hash_map<SchedulerOutput::HashType, double> len_along_route_map_,
      raw_len_along_route_map_;
  absl::flat_hash_map<mapping::ElementId, int> lc_num_to_targets_map_,
      lc_num_within_driving_dist_map_;
  absl::flat_hash_map<mapping::ElementId, bool> is_right_most_lane_map_,
      is_valid_merge_lane_map_;
  absl::flat_hash_map<SchedulerOutput::HashType, std::optional<StalledObjInfo>>
      front_stalled_obj_map_;
  bool enable_discourage_right_most_cost_;
  bool enable_encourage_right_most_cost_;
  double max_len_along_route_;
  double min_len_along_route_;
  bool is_non_occluding_leader_ = false;
  int min_lc_num_;
  int max_lc_num_;
  bool is_left_turn_ = false;
  bool is_right_turn_ = false;

  double ego_v_;
  bool planner_is_bus_model_ = false;
  bool use_conservative_ttc_ = false;
};

// To make selector favor trajectories from non-expanded path boundary.
//
// || .   | ||       || |   . ||       || | ||       || .   |   . ||
// || .   | ||       || |   . ||       || | ||       || .   |   . ||
// || .   | ||       || |   . ||       || | ||       || .   |   . ||
// || .   | ||       || |   . ||       || | ||       || .   |   . ||
// || .   | ||       || |   . ||       || | ||       || .   |   . ||
// || .   | ||       || |   . ||       || | ||       || .   |   . ||
// || e   | ||       || e   . ||       || e ||       || .   e   . ||
//     low               high            low               high
class TrajBoundaryExpansionCost : public CostFeatureBase {
 public:
  explicit TrajBoundaryExpansionCost(
      const SelectorCommonFeature* common_feature,
      const ApolloTrajectoryPointProto& plan_start_point)
      : CostFeatureBase("boundary_expansion", {"boundary_expansion"},
                        /*is_common=*/false, common_feature),
        ego_pos_(Vec2dFromApolloTrajectoryPointProto(plan_start_point)) {}

  absl::StatusOr<CostVec> ComputeCost(
      const EstPlannerOutput& planner_output,
      std::vector<std::string>* extra_info,
      TrajFeatureOutput* traj_feature_output) const override;

 private:
  Vec2d ego_pos_;
};

// TODO(boqian): find more suitable cost features.

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_SELECTOR_TRAJ_COST_FEATURES_H_
