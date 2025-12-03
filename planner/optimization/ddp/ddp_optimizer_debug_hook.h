#ifndef ONBOARD_PLANNER_OPTIMIZATION_DDP_DDP_OPTIMIZER_DEBUG_HOOK_H_
#define ONBOARD_PLANNER_OPTIMIZATION_DDP_DDP_OPTIMIZER_DEBUG_HOOK_H_

#include <regex>
#include <string>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"

#include "onboard/math/eigen.h"
#include "onboard/math/geometry/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/object/planner_object_manager.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/optimization/ddp/ddp_optimizer_hook.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/util/trajectory_plot_util.h"
#include "onboard/proto/vehicle.pb.h"

#include "offboard/vis/vantage/vantage_server/vantage_canvas_client.h"

namespace qcraft {
namespace planner {

constexpr double kTrajVisZInc =
    kSpaceTimeVisualizationDefaultTimeScale * kTrajectoryTimeStep;

template <typename PROB>
class IterationVisualizerHook : public DdpOptimizerHook<PROB> {
 public:
  IterationVisualizerHook(int horizon, const std::string& base_name,
                          bool canvas_render_indices)
      : DdpOptimizerHook<PROB>(horizon),
        base_name_(base_name),
        canvas_render_indices_(canvas_render_indices) {}

  using OptimizerInspector =
      typename DdpOptimizerHook<PROB>::OptimizerInspector;
  using StatesType = typename PROB::StatesType;
  using ControlsType = typename PROB::ControlsType;

  void OnSolveStart(const StatesType& xs, const ControlsType& /*us*/) override {
    CanvasDrawTrajectory(
        VisIndexTrajToVector([&xs](int index) { return PROB::pos(xs, index); },
                             DdpOptimizerHook<PROB>::horizon(), kTrajVisZInc,
                             0.0),
        vis::Color(0.8, 0.4, 0.4),
        /*render_indices=*/canvas_render_indices_,
        base_name_ + "/iters/init_rollout");
  }

  void OnIterationEnd(int iteration, const StatesType& xs,
                      const ControlsType& /*us*/,
                      const OptimizerInspector& /*oi*/) override {
    CanvasDrawTrajectory(
        VisIndexTrajToVector([&xs](int index) { return PROB::pos(xs, index); },
                             DdpOptimizerHook<PROB>::horizon(), kTrajVisZInc,
                             0.0),
        vis::Color(0.4, 0.8, 0.8),
        /*render_indices=*/canvas_render_indices_,
        base_name_ + absl::StrFormat("/iters/iter_%03d", iteration));
  }

  void OnSolveEnd(const StatesType& xs, const ControlsType& /*us*/,
                  const OptimizerInspector& /*oi*/) override {
    CanvasDrawTrajectory(
        VisIndexTrajToVector([&xs](int index) { return PROB::pos(xs, index); },
                             DdpOptimizerHook<PROB>::horizon(), kTrajVisZInc,
                             0.0),
        vis::Color(0.4, 0.4, 0.8),
        /*render_indices=*/canvas_render_indices_, base_name_ + "/iters/final");
  }

 private:
  const std::string base_name_;
  const bool canvas_render_indices_;
};

template <typename PROB>
class IterationAvModelVisualizerHook : public DdpOptimizerHook<PROB> {
 public:
  struct Circle {
    // Distances from circle center to RAC.
    double l = 0.0;
    // Angle between RAC->circle_center and av heading.
    double theta = 0.0;
    double r = 0.0;
  };

  IterationAvModelVisualizerHook(
      int horizon, const std::string& base_name,
      const std::vector<Circle>& circles,
      const VehicleGeometryParamsProto* veh_geo_params)
      : DdpOptimizerHook<PROB>(horizon),
        base_name_(base_name),
        circles_(circles),
        veh_geo_params_(QCHECK_NOTNULL(veh_geo_params)) {}

  using OptimizerInspector =
      typename DdpOptimizerHook<PROB>::OptimizerInspector;
  using StatesType = typename PROB::StatesType;
  using ControlsType = typename PROB::ControlsType;

  void OnSolveStart(const StatesType& xs, const ControlsType& /*us*/) override {
    DrawAvModels(xs, "init_av_model", vis::Color(0.8, 0.4, 0.4));
  }

  void OnSolveEnd(const StatesType& xs, const ControlsType& /*us*/,
                  const OptimizerInspector& /*oi*/) override {
    DrawAvModels(xs, "final_av_model", vis::Color(0.4, 0.4, 0.8));
  }

 private:
  void DrawAvModels(const StatesType& xs, const std::string& channel_name,
                    vis::Color color) {
    const double rear_to_center = veh_geo_params_->front_edge_to_center() -
                                  veh_geo_params_->length() * 0.5;
    const Vec2d box_size(veh_geo_params_->length(), veh_geo_params_->width());
    for (int k = 0; k < DdpOptimizerHook<PROB>::horizon(); ++k) {
      vis::Canvas* canvas_circles = nullptr;
      canvas_circles = &vis::vantage::GetCanvasClient()->GetCanvas(
          base_name_ + absl::StrFormat("/iters/%s/%03d", channel_name, k));
      const Vec2d pos = PROB::pos(xs, k);
      const double theta = PROB::theta(xs, k);
      const Vec2d tangent = Vec2d::FastUnitFromAngle(theta);
      const double z =
          k * kSpaceTimeVisualizationDefaultTimeScale * kTrajectoryTimeStep;
      const Vec2d box_center = pos + tangent * rear_to_center;
      canvas_circles->DrawBox(Vec3d(box_center, z), theta, box_size, color);
      for (int i = 0; i < circles_.size(); ++i) {
        const Vec2d circle_center =
            PROB::pos(xs, k) +
            circles_[i].l * Vec2d::FastUnitFromAngle(theta + circles_[i].theta);
        canvas_circles->DrawCircle(Vec3d(circle_center, z), circles_[i].r,
                                   color);
      }
    }
  }

  const std::string base_name_;
  std::vector<Circle> circles_;
  const VehicleGeometryParamsProto* veh_geo_params_;
};

template <typename PROB>
struct OptimizerSolverDebugHook : public DdpOptimizerHook<PROB> {
 public:
  using ObjectResponseProto =
      TrajectoryOptimizerDebugProto::ObjectResponseProto;

  OptimizerSolverDebugHook(int horizon, const TrajectoryPoint& av_pose,
                           const SpacetimeTrajectoryManager& st_traj_mgr)
      : DdpOptimizerHook<PROB>(horizon), av_pose_(av_pose) {
    for (const auto& traj : st_traj_mgr.trajectories()) {
      object_map[traj.traj_id()] = &traj;
    }
  }

  using OptimizerInspector =
      typename DdpOptimizerHook<PROB>::OptimizerInspector;
  using StatesType = typename PROB::StatesType;
  using ControlsType = typename PROB::ControlsType;

  void OnSolveStart(const StatesType& xs, const ControlsType& us) override {
    solve_start_xs = xs;
    solve_start_us = us;
    iterations.clear();
    object_responses.clear();
    init_costs.ddp_costs.clear();
    final_costs.ddp_costs.clear();
  }

  void OnSolveEnd(const StatesType& xs, const ControlsType& us,
                  const OptimizerInspector& oi) override {
    solve_end_xs = xs;
    solve_end_us = us;
    // Example cost names:
    //  - Object (R): for Agent1-idx0
    //  - Object (F): for 12354-1
    //  - Emeraude Object (R): for Phantom 1-idx0
    //
    // Regex tokens:
    //  - object ID.
    //  - prediction ID.
    static const std::regex kObjResponseCostRegex =
        std::regex(".+for (.+)-(\\w+)");

    // Analyze the costs to deduce the objects we're responding to.
    for (const auto& [cost_name, cost, is_soft] : oi.named_costs) {
      std::smatch obj_response_cost_match;
      if (!std::regex_match(cost_name, obj_response_cost_match,
                            kObjResponseCostRegex)) {
        continue;
      }

      // Costs below this threshold are considered too weak to correspond to an
      // active "response" to an object.
      constexpr double kCostThresholdResponse = 1.0;

      if (cost < kCostThresholdResponse) continue;

      QCHECK_EQ(obj_response_cost_match.size(), 3);
      const std::string& obj_id = obj_response_cost_match[1];
      const std::string& obj_pred_index = obj_response_cost_match[2];
      VLOG(3) << "Object response: cost name [" << cost_name
              << "] cost: " << cost << " obj_id: " << obj_id
              << " obj_pred_index: " << obj_pred_index
              << ", is_soft: " << is_soft;
      std::string obj_traj_name = obj_id + "-";
      obj_traj_name += obj_pred_index;
      const auto it = object_map.find(obj_traj_name);
      if (it == object_map.end()) {
        QLOG(ERROR) << "Unknown object ID " << obj_id << " from cost name "
                    << cost_name;
        continue;
      }
      const SpacetimeObjectTrajectory& st_obj = *(it->second);
      ObjectResponseProto& response = object_responses.emplace_back();
      response.set_object_id(obj_traj_name);
      response.set_cost(cost);
      response.set_cost_name(cost_name);

      // TODO(Fang, Renjie) look for a way to find out the time of the peak of
      // the response, or a time range of the response. We may be responding to
      // an object (or its predicted trajectory) at a future time, and the
      // visualization should reflect that.
      response.set_peak_step(0);
      Vec2dToProto(st_obj.pose().pos(),
                   response.mutable_peak_object_position());
      const Vec2d force_dir =
          (st_obj.pose().pos() - av_pose_.pos()).normalized();
      Vec2dToProto(force_dir, response.mutable_peak_force_direction());
    }

    final_costs.cost = oi.cost;
    final_costs.ddp_costs.clear();
    for (const auto& named_cost : oi.named_costs) {
      // Don't log cost below this threshold.
      constexpr double kCostThresholdForLog = 1.0;
      if (named_cost.value < kCostThresholdForLog) continue;
      final_costs.ddp_costs.push_back(named_cost);
    }
  }

  void OnIterationStart(int iteration, const StatesType& /*xs*/,
                        const ControlsType& /*us*/,
                        const OptimizerInspector& oi) override {
    if (iteration >= iterations.size()) {
      iterations.emplace_back();
    }
    if (iteration == 0) {
      init_costs.cost = oi.cost;
      init_costs.ddp_costs.clear();
      for (const auto& named_cost : oi.named_costs) {
        // Don't log cost below this threshold.
        constexpr double kCostThresholdForLog = 1.0;
        if (named_cost.value < kCostThresholdForLog) continue;
        init_costs.ddp_costs.push_back(named_cost);
      }
    }
  }
  void OnIterationEnd(int iteration, const StatesType& /*xs*/,
                      const ControlsType& /*us*/,
                      const OptimizerInspector& oi) override {
    QCHECK_EQ(iteration + 1, iterations.size());
    iterations.back().final_cost = oi.cost;
    iterations.back().js0 = oi.js0;
  }
  void OnLineSearchIterationEnd(int iteration, double alpha,
                                double cost) override {
    QCHECK_EQ(iteration + 1, iterations.size());
    auto& iter = iterations.back();
    iter.alphas.push_back(alpha);
    iter.line_search_costs.push_back(cost);
  }

  void OnStepSizeAdjustmentIterationEnd(int iteration, int k_stepsize,
                                        double cost) override {
    QCHECK_EQ(iteration + 1, iterations.size());
    auto& iter = iterations.back();
    iter.k_s.push_back(k_stepsize);
    iter.stepsize_adjustment_costs.push_back(cost);
  }

  TrajectoryPoint av_pose_;
  absl::flat_hash_map<std::string, const SpacetimeObjectTrajectory*> object_map;

  StatesType solve_start_xs;
  ControlsType solve_start_us;

  StatesType solve_end_xs;
  ControlsType solve_end_us;

  struct CostDebugInfo {
    double cost;
    std::vector<NamedCostEntry> ddp_costs;
  };
  CostDebugInfo init_costs;
  CostDebugInfo final_costs;

  struct IterationDebugInfo {
    std::vector<double> alphas;
    std::vector<double> line_search_costs;
    std::vector<int> k_s;
    std::vector<double> stepsize_adjustment_costs;
    // The actual final cost.
    double final_cost;
    // The desired final cost.
    double js0;
  };
  std::vector<IterationDebugInfo> iterations;

  std::vector<ObjectResponseProto> object_responses;
};

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_OPTIMIZATION_DDP_DDP_OPTIMIZER_DEBUG_HOOK_H_
