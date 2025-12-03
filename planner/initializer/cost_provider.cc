#include "onboard/planner/initializer/cost_provider.h"

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type

#include <algorithm>
#include <string>
#include <utility>

#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "google/protobuf/descriptor.h"

#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/planner/initializer/geometry/geometry_state.h"
#include "onboard/planner/initializer/motion_state.h"
#include "onboard/planner/initializer/proto/initializer_config.pb.h"
#include "onboard/planner/initializer/ref_speed_table.h"
#include "onboard/planner/initializer/reference_line_cost_feature.h"
#include "onboard/planner/initializer/searcher_cost_feature.h"
#include "onboard/planner/planner_defs.h"

namespace qcraft::planner {

namespace {
constexpr double kGeometryEdgeSampleStep = 0.5;  // m.
}

template <typename Config>
void CostProviderBase::BuildWeightTable(const Config& cost_config) {
  const auto* reflection = cost_config.GetReflection();
  const auto* descriptor = cost_config.GetDescriptor();
  for (const auto& feature : features_) {
    auto* feature_desc =
        descriptor->FindFieldByName(std::string(feature->name()));
    const auto& feature_conf =
        reflection->GetMessage(cost_config, feature_desc);
    const auto* feature_ref = feature_conf.GetReflection();
    std::vector<const google::protobuf::FieldDescriptor*> feature_weights_desc;
    feature_ref->ListFields(feature_conf, &feature_weights_desc);
    int feature_size = 0;
    for (const auto* desc : feature_weights_desc) {
      if (desc->type() != google::protobuf::FieldDescriptor::TYPE_DOUBLE) {
        continue;
      }
      ++feature_size;
      cost_names_.push_back(absl::StrCat(feature->name(), ".", desc->name()));
      weights_.push_back(feature_ref->GetDouble(feature_conf, desc));
    }
    feature_size_.push_back(feature_size);
  }
}

template <typename Config>
void CostProviderBase::BuildDpWeightTable(const Config& cost_config) {
  const auto* reflection = cost_config.GetReflection();
  const auto* descriptor = cost_config.GetDescriptor();
  for (const auto& feature : features_) {
    auto* feature_desc =
        descriptor->FindFieldByName("dp_" + std::string(feature->name()));
    const auto& feature_conf =
        reflection->GetMessage(cost_config, feature_desc);
    const auto* feature_ref = feature_conf.GetReflection();
    std::vector<const google::protobuf::FieldDescriptor*> feature_weights_desc;
    feature_ref->ListFields(feature_conf, &feature_weights_desc);
    int feature_size = 0;
    for (const auto* desc : feature_weights_desc) {
      if (desc->type() != google::protobuf::FieldDescriptor::TYPE_DOUBLE) {
        continue;
      }
      ++feature_size;
      cost_names_.push_back(absl::StrCat(feature->name(), ".", desc->name()));
      weights_.push_back(feature_ref->GetDouble(feature_conf, desc));
    }
    feature_size_.push_back(feature_size);
  }
}

template <typename Config>
void CostProviderBase::BuildAstarWeightTable(const Config& cost_config) {
  const auto* reflection = cost_config.GetReflection();
  const auto* descriptor = cost_config.GetDescriptor();
  for (const auto& feature : features_) {
    auto* feature_desc =
        descriptor->FindFieldByName("astar_" + std::string(feature->name()));
    const auto& feature_conf =
        reflection->GetMessage(cost_config, feature_desc);
    const auto* feature_ref = feature_conf.GetReflection();
    std::vector<const google::protobuf::FieldDescriptor*> feature_weights_desc;
    feature_ref->ListFields(feature_conf, &feature_weights_desc);
    int feature_size = 0;
    for (const auto* desc : feature_weights_desc) {
      if (desc->type() != google::protobuf::FieldDescriptor::TYPE_DOUBLE) {
        continue;
      }
      ++feature_size;
      cost_names_.push_back(absl::StrCat(feature->name(), ".", desc->name()));
      weights_.push_back(feature_ref->GetDouble(feature_conf, desc));
    }
    feature_size_.push_back(feature_size);
  }
}

void CostProviderBase::ComputeDpCost(const double start_t,
                                     const MotionForm* motion_form,
                                     absl::Span<double> cost) const {
  QCHECK_EQ(cost.size(), weights_.size());
  auto sampled_states = motion_form->SampleStates();
  MotionEdgeInfo edge_info{
      .start_t = start_t,
      .motion_form = motion_form,
      .const_interval_states = std::move(sampled_states.const_interval_states),
      .equal_interval_states = std::move(sampled_states.equal_interval_states),
  };
  for (int i = 0, index = 0, n = features_.size(); i < n; ++i) {
    features_[i]->ComputeCost(edge_info, cost.subspan(index, feature_size_[i]));
    index += feature_size_[i];
  }
  for (int i = 0; i < cost.size(); ++i) {
    cost[i] = cost[i] * weights_[i];
  }
}

IgnoreTrajMap CostProviderBase::ComputeInteractiveDpCost(
    double start_t, const MotionForm* motion_form,
    const IgnoreTrajMap& ignored_trajs, absl::Span<double> cost) const {
  QCHECK_EQ(cost.size(), weights_.size());
  auto sampled_states = motion_form->SampleStates();
  MotionEdgeInfo edge_info{
      .start_t = start_t,
      .motion_form = motion_form,
      .const_interval_states = std::move(sampled_states.const_interval_states),
      .equal_interval_states = std::move(sampled_states.equal_interval_states),
  };
  IgnoreTrajMap new_ignored_trajs;
  for (int i = 0, index = 0, n = features_.size(); i < n; ++i) {
    if (features_[i]->name() == "dynamic_collision") {
      new_ignored_trajs =
          reinterpret_cast<const DynamicCollisionFeatureCost*>(
              features_[i].get())
              ->ComputeInteractiveCost(edge_info, ignored_trajs,
                                       cost.subspan(index, feature_size_[i]));
    } else {
      features_[i]->ComputeCost(edge_info,
                                cost.subspan(index, feature_size_[i]));
    }
    index += feature_size_[i];
  }
  for (int i = 0; i < cost.size(); ++i) {
    cost[i] = cost[i] * weights_[i];
  }
  return new_ignored_trajs;
}

IgnoreTrajMap CostProviderBase::ComputeInteractiveAstarCost(
    double start_t, const MotionForm* motion_form,
    const IgnoreTrajMap& ignored_trajs, absl::Span<double> cost) const {
  QCHECK_EQ(cost.size(), weights_.size());
  auto sampled_states = motion_form->SampleStates();
  MotionEdgeInfo edge_info{
      .start_t = start_t,
      .motion_form = motion_form,
      .const_interval_states = std::move(sampled_states.const_interval_states),
      .equal_interval_states = std::move(sampled_states.equal_interval_states),
  };
  IgnoreTrajMap new_ignored_trajs;
  for (int i = 0, index = 0, n = features_.size(); i < n; ++i) {
    if (features_[i]->name() == "dynamic_collision") {
      new_ignored_trajs =
          reinterpret_cast<const DynamicCollisionFeatureCost*>(
              features_[i].get())
              ->ComputeInteractiveCost(edge_info, ignored_trajs,
                                       cost.subspan(index, feature_size_[i]));
    } else {
      features_[i]->ComputeCost(edge_info,
                                cost.subspan(index, feature_size_[i]));
    }
    index += feature_size_[i];
  }
  for (int i = 0; i < cost.size(); ++i) {
    cost[i] = cost[i] * weights_[i];
  }
  return new_ignored_trajs;
}

void CostProviderBase::ComputeLeadingObjCost(double start_t,
                                             const MotionForm* motion_form,
                                             absl::Span<double> cost) const {
  QCHECK_EQ(cost.size(), weights_.size());
  MotionEdgeInfo edge_info{
      .start_t = start_t,
      .motion_form = motion_form,
      .equal_interval_states = motion_form->SampleEqualIntervalStates(),
  };
  for (int i = 0, index = 0, n = features_.size(); i < n; ++i) {
    if (features_[i]->name() == "leading_object") {
      features_[i]->ComputeCost(edge_info,
                                cost.subspan(index, feature_size_[i]));
      for (int j = 0; j < feature_size_[i]; ++j) {
        cost[index + j] = cost[index + j] * weights_[index + j];
      }
      break;
    }
    index += feature_size_[i];
  }
}

void CostProviderBase::ComputeRefLineCost(const GeometryForm* geometry_form,
                                          bool terminating,
                                          absl::Span<double> cost) const {
  QCHECK_EQ(cost.size(), weights_.size());
  GeometryEdgeInfo geom_edge_info;
  geom_edge_info.geometry_form = geometry_form;
  geom_edge_info.terminating = terminating;
  geom_edge_info.states = geometry_form->Sample(kGeometryEdgeSampleStep);
  for (int i = 0, index = 0, n = features_.size(); i < n; ++i) {
    features_[i]->ComputeCost(geom_edge_info,
                              cost.subspan(index, feature_size_[i]));
    index += feature_size_[i];
  }
  for (int i = 0; i < cost.size(); ++i) {
    cost[i] = cost[i] * weights_[i];
  }
}

// -------------- CostProvider -------------
// Dp
DpCostProvider::DpCostProvider(
    const DrivePassage& drive_passage,
    const InitializerConfig& initializer_params,
    const MotionConstraintParamsProto& motion_constraint_params,
    const std::vector<double>& stop_s_vec,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const std::vector<std::vector<std::string>>& leading_groups,
    const int leading_group_idx, const VehicleGeometryParamsProto& vehicle_geom,
    const CollisionChecker* collision_checker, const PathSlBoundary* path_sl,
    const RefSpeedTable* ref_speed_table,
    const ml::captain_net::CaptainNetOutput* captain_net_output,
    bool is_lane_change, double max_accumulated_s, bool is_post_evaluation) {
  const double traj_horizon =
      (initializer_params.traj_steps() - 1) * kTrajectoryTimeStep;
  const auto& cost_config = is_post_evaluation
                                ? initializer_params.dp_post_cost_config()
                                : initializer_params.dp_cost_config();
  if (cost_config.has_dp_acceleration()) {
    features_.emplace_back(
        std::make_unique<AccelerationFeatureCost>(motion_constraint_params));
  }

  if (cost_config.has_dp_lane_boundary()) {
    features_.emplace_back(std::make_unique<LaneBoundaryFeatureCost>(
        path_sl, vehicle_geom.width() * 0.5));
  }

  if (cost_config.has_dp_curvature()) {
    features_.emplace_back(std::make_unique<CurvatureFeatureCost>());
  }

  if (cost_config.has_dp_lateral_acceleration()) {
    features_.emplace_back(
        std::make_unique<LateralAccelerationFeatureCost>(is_lane_change));
  }

  if (cost_config.has_dp_stop_constraint()) {
    features_.emplace_back(
        std::make_unique<StopConstraintFeatureCost>(stop_s_vec));
  }

  if (cost_config.has_dp_ref_speed()) {
    features_.emplace_back(
        std::make_unique<RefSpeedFeatureCost>(ref_speed_table));
  }

  if (cost_config.has_dp_dynamic_collision()) {
    features_.emplace_back(
        std::make_unique<DynamicCollisionFeatureCost>(collision_checker));
  }

  if (cost_config.has_dp_leading_object()) {
    features_.emplace_back(std::make_unique<LeadingObjectFeatureCost>(
        traj_horizon, drive_passage, st_traj_mgr, leading_groups,
        leading_group_idx, vehicle_geom.front_edge_to_center(),
        vehicle_geom.back_edge_to_center()));
  }

  if (cost_config.has_dp_ref_traj()) {
    features_.emplace_back(
        std::make_unique<RefTrajectoryFeatureCost>(captain_net_output));
  }

  if (cost_config.has_dp_final_progress()) {
    features_.emplace_back(std::make_unique<FinalProgressFeatureCost>(
        traj_horizon, path_sl, max_accumulated_s));
  }

  BuildDpWeightTable<InitializerConfig::DpFeatureCostConfig>(cost_config);
}

// Astar
AstarCostProvider::AstarCostProvider(
    const DrivePassage& drive_passage,
    const InitializerConfig& initializer_params,
    const MotionConstraintParamsProto& motion_constraint_params,
    const std::vector<double>& stop_s_vec,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const std::vector<std::vector<std::string>>& leading_groups,
    const int leading_group_idx, const bool able_to_overtake_leading_behind,
    const VehicleGeometryParamsProto& vehicle_geom,
    const CollisionChecker* collision_checker, const PathSlBoundary* path_sl,
    const RefSpeedTable* ref_speed_table,
    const ml::captain_net::CaptainNetOutput* captain_net_output,
    bool is_lane_change, double max_accumulated_s) {
  const double traj_horizon =
      (initializer_params.traj_steps() - 1) * kTrajectoryTimeStep;
  const auto& cost_config = initializer_params.astar_cost_config();

  if (cost_config.has_astar_acceleration()) {
    features_.emplace_back(
        std::make_unique<AccelerationFeatureCost>(motion_constraint_params));
  }

  if (cost_config.has_astar_lane_boundary()) {
    features_.emplace_back(std::make_unique<LaneBoundaryFeatureCost>(
        path_sl, vehicle_geom.width() * 0.5));
  }

  if (cost_config.has_astar_curvature()) {
    features_.emplace_back(std::make_unique<CurvatureFeatureCost>());
  }

  if (cost_config.has_astar_lateral_acceleration()) {
    features_.emplace_back(
        std::make_unique<LateralAccelerationFeatureCost>(is_lane_change));
  }

  if (cost_config.has_astar_stop_constraint()) {
    features_.emplace_back(
        std::make_unique<StopConstraintFeatureCost>(stop_s_vec));
  }

  if (cost_config.has_astar_ref_speed()) {
    features_.emplace_back(
        std::make_unique<RefSpeedFeatureCost>(ref_speed_table));
  }

  if (cost_config.has_astar_dynamic_collision()) {
    features_.emplace_back(
        std::make_unique<DynamicCollisionFeatureCost>(collision_checker));
  }

  if (cost_config.has_astar_leading_object()) {
    features_.emplace_back(std::make_unique<AstarLeadingObjectFeatureCost>(
        traj_horizon, drive_passage, st_traj_mgr, leading_groups,
        leading_group_idx, able_to_overtake_leading_behind,
        vehicle_geom.front_edge_to_center(),
        vehicle_geom.back_edge_to_center()));
  }

  if (cost_config.has_astar_ref_traj()) {
    features_.emplace_back(
        std::make_unique<RefTrajectoryFeatureCost>(captain_net_output));
  }

  if (cost_config.has_astar_final_progress()) {
    features_.emplace_back(std::make_unique<FinalProgressFeatureCost>(
        traj_horizon, path_sl, max_accumulated_s));
  }

  BuildAstarWeightTable<InitializerConfig::AstarFeatureCostConfig>(cost_config);
}

// RefLineCostProvider.

RefLineCostProvider::RefLineCostProvider(
    const SpacetimePlannerObjectTrajectories* st_planner_object_traj,
    const DrivePassage* drive_passage, const PathSlBoundary* path_sl,
    double geom_graph_max_accum_s, double relaxed_center_max_curvature,
    const InitializerConfig& initializer_params) {
  const auto& cost_config = initializer_params.ref_line_cost_config();
  if (cost_config.has_ref_line_stationary_object()) {
    features_.emplace_back(std::make_unique<RefLineStationaryObjectFeatureCost>(
        st_planner_object_traj, drive_passage));
  }

  if (cost_config.has_ref_line_progress()) {
    features_.emplace_back(std::make_unique<RefLineProgressFeatureCost>(
        geom_graph_max_accum_s, path_sl));
  }

  if (cost_config.has_ref_line_path_boundary()) {
    features_.emplace_back(
        std::make_unique<RefLinePathBoundaryFeatureCost>(path_sl));
  }

  if (cost_config.has_ref_line_curvature()) {
    features_.emplace_back(std::make_unique<RefLineCurvatureFeatureCost>(
        relaxed_center_max_curvature));
  }

  BuildWeightTable<InitializerConfig::RefLineFeatureCostConfig>(cost_config);
}

}  // namespace qcraft::planner
