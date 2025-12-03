#ifndef ONBOARD_PLANNER_SELECTOR_COST_FEATURE_BASE_H_
#define ONBOARD_PLANNER_SELECTOR_COST_FEATURE_BASE_H_

#include <string>
#include <utility>
#include <vector>

#include "onboard/planner/est_planner_output.h"
#include "onboard/planner/selector/common_feature.h"

namespace qcraft {
namespace planner {

struct TrajFeatureOutput {
  bool is_blocked_by_stalled_obj = false;
  double progress_factor = 1.0;
  double driving_dist = 0.0;
  bool reach_need_confirmation_distance = false;
  bool in_route_target_lane = false;
  bool in_highway = false;
  bool has_obvious_route_cost = false;
  bool is_perform_lane_change = false;
  bool lane_change_left = false;
  bool lane_change_for_road_speed_limit = false;
  bool lane_change_for_right_most_lane = false;
  bool lane_change_for_route_cost = false;
  bool lane_change_for_moving_obj = false;
  bool lane_change_for_stationary_vehicle = false;
  bool lane_change_for_stalled_vehicle = false;
  bool lane_change_for_stationary_obj = false;
  bool cross_solid_boundary = false;
  std::optional<Vec2d> merge_point = std::nullopt;
};

class CostFeatureBase {
 public:
  using CostVec = std::vector<double>;

  explicit CostFeatureBase(std::string&& name,
                           std::vector<std::string>&& sub_names, bool is_common,
                           const SelectorCommonFeature* common_feature)
      : name_(std::move(name)),
        sub_names_(std::move(sub_names)),
        size_(sub_names_.size()),
        is_common_(is_common),
        common_feature_(common_feature) {}
  virtual ~CostFeatureBase() {}

  const std::string& name() const { return name_; }
  const std::vector<std::string>& sub_names() const { return sub_names_; }
  int size() const { return size_; }
  bool is_common() const { return is_common_; }
  const SelectorCommonFeature* common_feature() const {
    return common_feature_;
  }
  virtual absl::StatusOr<CostVec> ComputeCost(
      const EstPlannerOutput& planner_output,
      std::vector<std::string>* extra_info,
      TrajFeatureOutput* traj_feature_output) const = 0;

 private:
  std::string name_;
  std::vector<std::string> sub_names_;
  int size_;
  // True if the cost applies for all trajs;
  // false if only applies for comparing trajs from the same start lane.
  bool is_common_;
  const SelectorCommonFeature* common_feature_;
};

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_SELECTOR_COST_FEATURE_BASE_H_
