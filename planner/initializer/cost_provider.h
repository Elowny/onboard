#ifndef ONBOARD_PLANNER_INITIALIZER_COST_PROVIDER_H_
#define ONBOARD_PLANNER_INITIALIZER_COST_PROVIDER_H_

#include <memory>
#include <string>
#include <vector>

#include "absl/types/span.h"

#include "onboard/planner/common/path_sl_boundary.h"
#include "onboard/planner/initializer/collision_checker.h"
#include "onboard/planner/initializer/cost_feature.h"
#include "onboard/planner/initializer/dp_motion_searcher_defs.h"
#include "onboard/planner/initializer/geometry/geometry_form.h"
#include "onboard/planner/initializer/motion_form.h"
#include "onboard/planner/initializer/proto/initializer_config.pb.h"
#include "onboard/planner/initializer/ref_speed_table.h"
#include "onboard/planner/ml/captain_net/captain_net.h"
#include "onboard/planner/object/spacetime_planner_object_trajectories.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/proto/vehicle.pb.h"
namespace qcraft::planner {

class CostProviderBase {
 public:
  absl::Span<const std::string> cost_names() const { return cost_names_; }

  absl::Span<const double> weights() const { return weights_; }

  void ComputeDpCost(double start_t, const MotionForm* motion_form,
                     absl::Span<double> cost) const;
  IgnoreTrajMap ComputeInteractiveDpCost(double start_t,
                                         const MotionForm* motion_form,
                                         const IgnoreTrajMap& ignored_trajs,
                                         absl::Span<double> cost) const;
  void ComputeLeadingObjCost(double start_t, const MotionForm* motion_form,
                             absl::Span<double> cost) const;
  void ComputeRefLineCost(const GeometryForm* geometry_form, bool terminating,
                          absl::Span<double> cost) const;
  IgnoreTrajMap ComputeInteractiveAstarCost(double start_t,
                                            const MotionForm* motion_form,
                                            const IgnoreTrajMap& ignored_trajs,
                                            absl::Span<double> cost) const;

 protected:
  template <typename Config>
  void BuildWeightTable(const Config& cost_config);

  template <typename Config>
  void BuildDpWeightTable(const Config& cost_config);

  template <typename Config>
  void BuildAstarWeightTable(const Config& cost_config);

  std::vector<std::unique_ptr<FeatureCost>> features_;

 private:
  // The name of each feature cost.
  std::vector<std::string> cost_names_;

  // The weight of each cost feature.
  std::vector<double> weights_;

  std::vector<int> feature_size_;
};

class DpCostProvider : public CostProviderBase {
 public:
  // DP.
  DpCostProvider(const DrivePassage& drive_passage,
                 const InitializerConfig& initializer_params,
                 const MotionConstraintParamsProto& motion_constraint_params,
                 const std::vector<double>& stop_s_vec,
                 const SpacetimeTrajectoryManager& st_traj_mgr,
                 const std::vector<std::vector<std::string>>& leading_groups,
                 const int leading_group_idx,
                 const VehicleGeometryParamsProto& vehicle_geom,
                 const CollisionChecker* collision_checker,
                 const PathSlBoundary* path_sl,
                 const RefSpeedTable* ref_speed_table,
                 const ml::captain_net::CaptainNetOutput* captain_net_output,
                 bool is_lane_change, double max_accumulated_s,
                 bool is_post_evaluation = false);
};

class AstarCostProvider : public CostProviderBase {
 public:
  // Astar.
  AstarCostProvider(const DrivePassage& drive_passage,
                    const InitializerConfig& initializer_params,
                    const MotionConstraintParamsProto& motion_constraint_params,
                    const std::vector<double>& stop_s_vec,
                    const SpacetimeTrajectoryManager& st_traj_mgr,
                    const std::vector<std::vector<std::string>>& leading_groups,
                    const int leading_group_idx,
                    const bool able_to_overtake_leading_behind,
                    const VehicleGeometryParamsProto& vehicle_geom,
                    const CollisionChecker* collision_checker,
                    const PathSlBoundary* path_sl,
                    const RefSpeedTable* ref_speed_table,
                    const ml::captain_net::CaptainNetOutput* captain_net_output,
                    bool is_lane_change, double max_accumulated_s);
};

class RefLineCostProvider : public CostProviderBase {
 public:
  RefLineCostProvider(
      const SpacetimePlannerObjectTrajectories* st_planner_object_traj,
      const DrivePassage* drive_passage, const PathSlBoundary* path_sl,
      double geom_graph_mac_accum_s, double relaxed_center_max_curvature,
      const InitializerConfig& initializer_params);
};

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_INITIALIZER_COST_PROVIDER_H_
