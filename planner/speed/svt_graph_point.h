#ifndef ONBOARD_PLANNER_SPEED_SVT_GRAPH_POINT_H_
#define ONBOARD_PLANNER_SPEED_SVT_GRAPH_POINT_H_

#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/types/span.h"

#include "onboard/planner/speed/proto/speed_finder.pb.h"
#include "onboard/planner/speed/svt_point.h"

namespace qcraft::planner {

// SvtGraphPoint is used for storing data, like
// svt_point, XX_cost, pre_point,...
// Corresponding to class::DpSvtCost, class::SvtPoint, class::GriddedSvtGraph.
class SvtGraphPoint {
 public:
  using DecisionType = StBoundaryProto::DecisionType;
  using StBoundaryDecision = std::pair<std::string, DecisionType>;

  struct BackupDecisionInfo {
    DecisionType backup_decision;
    double original_decision_cost = 0.0;
    double backup_decison_cost = 0.0;
  };
  using StBoundaryBackupDecision = std::pair<std::string, BackupDecisionInfo>;

  SvtGraphPoint(int grid_index_s, int grid_index_v, int index_t,
                const SvtPoint& point)
      : point_(point),
        grid_index_s_(grid_index_s),
        grid_index_v_(grid_index_v),
        index_t_(index_t) {
    // This is an experimental value.
    constexpr int kPointsNumPerGrid = 15;
    next_point_.reserve(kPointsNumPerGrid);
  }

  int grid_index_s() const { return grid_index_s_; }
  int grid_index_v() const { return grid_index_v_; }
  int index_t() const { return index_t_; }

  SvtPoint point() const { return point_; }
  SvtGraphPoint* pre_point() const { return pre_point_; }
  std::vector<const SvtGraphPoint*> next_point() const { return next_point_; }

  void set_pre_point(SvtGraphPoint* pre_point) { pre_point_ = pre_point; }
  void set_next_point(const SvtGraphPoint* next_point) {
    next_point_.push_back(next_point);
  }

  void set_acc_from_pre_point(double acc) { acc_from_pre_point_ = acc; }
  double acc_from_pre_point() const { return acc_from_pre_point_; }

  // cost
  double speed_limit_cost() const { return speed_limit_cost_; }
  double reference_speed_cost() const { return reference_speed_cost_; }
  double accel_cost() const { return accel_cost_; }
  double edge_cost() const { return edge_cost_; }

  double object_cost() const { return object_cost_; }
  double spatial_potential_cost() const { return spatial_potential_cost_; }
  double vertex_cost() const { return vertex_cost_; }

  double total_cost() const { return total_cost_; }
  void set_speed_limit_cost(double speed_limit_cost) {
    speed_limit_cost_ = speed_limit_cost;
  }
  void set_reference_speed_cost(double reference_speed_cost) {
    reference_speed_cost_ = reference_speed_cost;
  }
  void set_accel_cost(double accel_cost) { accel_cost_ = accel_cost; }
  void set_edge_cost(double edge_cost) { edge_cost_ = edge_cost; }

  void set_object_cost(double obs_cost) { object_cost_ = obs_cost; }
  void set_spatial_potential_cost(double spatial_potential_cost) {
    spatial_potential_cost_ = spatial_potential_cost;
  }
  void set_vertex_cost(double vertex_cost) { vertex_cost_ = vertex_cost; }

  void set_total_cost(double total_cost) { total_cost_ = total_cost; }

  std::optional<DecisionType> GetStBoundaryDecision(std::string_view id) const {
    const auto it = st_boundary_decisions_.find(id);
    if (it != st_boundary_decisions_.end()) {
      return it->second;
    } else {
      return std::nullopt;
    }
  }

  void UpdateStBoundaryDecisions(
      absl::Span<const StBoundaryDecision> st_boundary_decisions) {
    for (const auto& [id, decision] : st_boundary_decisions) {
      st_boundary_decisions_[id] = decision;
    }
  }

  std::optional<BackupDecisionInfo> GetBackupStBoundaryDecision(
      std::string_view id) const {
    const auto it = st_boundary_backup_decisions_.find(id);
    if (it != st_boundary_backup_decisions_.end()) {
      return it->second;
    } else {
      return std::nullopt;
    }
  }

  void UpdateBackupStBoundaryDecisions(
      absl::Span<const StBoundaryBackupDecision> st_boundary_backup_decisions) {
    for (const auto& [id, backup_decision_info] :
         st_boundary_backup_decisions) {
      st_boundary_backup_decisions_[id] = backup_decision_info;
    }
  }

 private:
  double total_cost_ = std::numeric_limits<double>::infinity();

  // Detailed cost params.
  double speed_limit_cost_ = 0.0;
  double reference_speed_cost_ = 0.0;
  double accel_cost_ = 0.0;
  double edge_cost_ = 0.0;
  double object_cost_ = 0.0;
  double spatial_potential_cost_ = 0.0;
  double vertex_cost_ = 0.0;

  SvtPoint point_;
  SvtGraphPoint* pre_point_ = nullptr;
  std::vector<const SvtGraphPoint*> next_point_;

  // It represents the index of GriddedSvtGraph::layers_.
  int grid_index_s_ = 0.0;
  int grid_index_v_ = 0.0;
  int index_t_ = 0.0;
  // Record decisions on st-boundaries without prior decision (decision is
  // unknown) and has t overlap with the point.
  absl::flat_hash_map<std::string, DecisionType> st_boundary_decisions_;
  // Record backup decision infos on st-boundaries with decisions. The costs in
  // backup decision infos are cumulative.
  absl::flat_hash_map<std::string, BackupDecisionInfo>
      st_boundary_backup_decisions_;

  double acc_from_pre_point_ = 0.0;
};

using SvtGraphPointRef = std::unique_ptr<SvtGraphPoint>;

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_SPEED_SVT_GRAPH_POINT_H_
