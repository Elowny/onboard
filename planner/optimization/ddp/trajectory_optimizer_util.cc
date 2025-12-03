#include "onboard/planner/optimization/ddp/trajectory_optimizer_util.h"

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type

#include <algorithm>
#include <functional>
#include <limits>
#include <map>
#include <ostream>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "absl/container/btree_map.h"
#include "absl/container/flat_hash_map.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "gflags/gflags.h"
#include "glog/logging.h"

#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/math/vec.h"
#include "onboard/planner/optimization/ddp/ddp_optimizer.h"
#include "onboard/planner/optimization/ipopt/ipopt_adapter.h"
#include "onboard/planner/optimization/ipopt/ipopt_optimizer_debug_hook.h"
#include "onboard/planner/planner_util.h"
#include "onboard/planner/util/vehicle_geometry_util.h"
#include "onboard/utils/file_util.h"
#include "onboard/vis/common/color.h"

#include "offboard/vis/vantage/charts/chart_util.h"

DEFINE_string(
    traj_opt_compare_log_file_folder,
    "offboard/planner/optimizer/compare/data/",
    "Only used in comparison mode, it is the folder name of a log file."
    "Make sure the folder exists, see the default value as example.");
DEFINE_int32(traj_opt_ipopt_canvas_level, 0,
             "Traj opt ipopt solver canvas level.");

namespace qcraft {
namespace planner {
namespace optimizer {

namespace {

void ToIpoptDebugProto(const std::vector<TrajectoryPoint>& init_traj,
                       const std::vector<TrajectoryPoint>& smooth_init_traj,
                       const std::vector<TrajectoryPoint>& result_traj,
                       const IpoptOptimizerDebugHook<Mfob>& solver_debug_hook,
                       IpoptSolverDebugProto* ipopt_debug_proto) {
  // Write DdpDebugProto data.
  for (int k = 0; k < init_traj.size(); ++k) {
    init_traj[k].ToProto(ipopt_debug_proto->add_init_traj());
  }
  for (int k = 0; k < smooth_init_traj.size(); ++k) {
    smooth_init_traj[k].ToProto(ipopt_debug_proto->add_smooth_init_traj());
  }
  for (int k = 0; k < result_traj.size(); ++k) {
    result_traj[k].ToProto(ipopt_debug_proto->add_final_traj());
  }

  const auto& init_costs = solver_debug_hook.init_costs();
  ipopt_debug_proto->mutable_init_costs()->set_cost(init_costs.cost);
  for (int i = 0; i < init_costs.costs.size(); ++i) {
    TrajectoryOptimizerCost* cost_proto =
        ipopt_debug_proto->mutable_init_costs()->add_costs();
    cost_proto->set_name(init_costs.costs[i].first);
    cost_proto->set_cost(init_costs.costs[i].second);
  }
  const auto& final_costs = solver_debug_hook.final_costs();
  ipopt_debug_proto->mutable_final_costs()->set_cost(final_costs.cost);
  for (int i = 0; i < final_costs.costs.size(); ++i) {
    TrajectoryOptimizerCost* cost_proto =
        ipopt_debug_proto->mutable_final_costs()->add_costs();
    cost_proto->set_name(final_costs.costs[i].first);
    cost_proto->set_cost(final_costs.costs[i].second);
  }

  for (const auto& iteration : solver_debug_hook.iterations()) {
    IpoptSolverDebugProto::Iteration* iteration_proto =
        ipopt_debug_proto->add_iterations();
    for (int i = 0; i < iteration.final_xs.size(); ++i) {
      iteration_proto->add_final_xs(iteration.final_xs[i]);
    }
    for (int i = 0; i < iteration.final_us.size(); ++i) {
      iteration_proto->add_final_us(iteration.final_us[i]);
    }
    iteration_proto->set_final_cost(iteration.cost_info.cost);
    for (int i = 0; i < iteration.cost_info.costs.size(); ++i) {
      TrajectoryOptimizerCost* cost_proto = iteration_proto->add_costs();
      cost_proto->set_name(iteration.cost_info.costs[i].first);
      cost_proto->set_cost(iteration.cost_info.costs[i].second);
    }
  }
  ipopt_debug_proto->set_num_iters(solver_debug_hook.iterations().size());
}

void AddCompareTrajCharts(const std::string& base_name,
                          const TrajectoryPlotInfo& ddp_traj,
                          const TrajectoryPlotInfo& ipopt_traj,
                          vis::vantage::ChartDataBundleProto* charts_data) {
  QCHECK_EQ(ddp_traj.traj.size(), ipopt_traj.traj.size());

  const int size = ddp_traj.traj.size();
  const std::string& traj_name1 = ddp_traj.name;
  const std::string& traj_name2 = ipopt_traj.name;

  if (charts_data == nullptr) return;
  if (size == 0) return;
  std::vector<double> s1, s2;
  std::vector<double> t;
  std::vector<double> x1, x2, y1, y2;
  std::vector<std::string> names = {
      "v_" + traj_name1,         "v_" + traj_name2,
      "a_" + traj_name1,         "a_" + traj_name2,
      "j_" + traj_name1,         "j_" + traj_name2,
      "lon_jerk_" + traj_name1,  "lon_jerk_" + traj_name2,
      "theta_" + traj_name1,     "theta_" + traj_name2,
      "kappa_" + traj_name1,     "kappa_" + traj_name2,
      "psi_" + traj_name1,       "psi_" + traj_name2,
      "chi_" + traj_name1,       "chi_" + traj_name2,
      "lat_accel_" + traj_name1, "lat_accel_" + traj_name2,
      "lat_jerk_" + traj_name1,  "lat_jerk_" + traj_name2};
  std::vector<std::vector<double>> values(names.size());
  s1.reserve(size);
  s2.reserve(size);
  t.reserve(size);
  x1.reserve(size);
  x2.reserve(size);
  y1.reserve(size);
  y2.reserve(size);
  for (auto& v : values) v.reserve(size);
  const TrajectoryPoint traj_point_base1 = ddp_traj.traj[0];
  const TrajectoryPoint traj_point_base2 = ipopt_traj.traj[0];
  for (int i = 0; i < size; ++i) {
    const TrajectoryPoint traj_point1 = ddp_traj.traj[i];
    const TrajectoryPoint traj_point2 = ipopt_traj.traj[i];
    s1.push_back(traj_point1.s() - traj_point_base1.s());
    s2.push_back(traj_point2.s() - traj_point_base2.s());
    t.push_back(traj_point1.t() - traj_point_base2.t());
    int y_index = 0;
    values[y_index++].push_back(traj_point1.v());
    values[y_index++].push_back(traj_point2.v());
    values[y_index++].push_back(traj_point1.a());
    values[y_index++].push_back(traj_point2.a());
    values[y_index++].push_back(traj_point1.j());
    values[y_index++].push_back(traj_point2.j());
    values[y_index++].push_back(ComputeLongitudinalJerk(traj_point1));
    values[y_index++].push_back(ComputeLongitudinalJerk(traj_point2));
    values[y_index++].push_back(traj_point1.theta());
    values[y_index++].push_back(traj_point2.theta());
    values[y_index++].push_back(traj_point1.kappa());
    values[y_index++].push_back(traj_point2.kappa());
    values[y_index++].push_back(traj_point1.psi());
    values[y_index++].push_back(traj_point2.psi());
    values[y_index++].push_back(traj_point1.chi());
    values[y_index++].push_back(traj_point2.chi());
    values[y_index++].push_back(ComputeLateralAcceleration(traj_point1));
    values[y_index++].push_back(ComputeLateralAcceleration(traj_point2));
    values[y_index++].push_back(ComputeLateralJerk(traj_point1));
    values[y_index++].push_back(ComputeLateralJerk(traj_point2));

    x1.push_back(traj_point1.pos().x());
    y1.push_back(traj_point1.pos().y());
    x2.push_back(traj_point2.pos().x());
    y2.push_back(traj_point2.pos().y());
  }

  vis::vantage::ChartDataProto* time_chart = charts_data->add_charts();
  time_chart->set_title(base_name + "/compare/time");
  vis::vantage::GenerateSubchartFromData(t, {s1}, "t", {"s_" + traj_name1}, {},
                                         {{}}, time_chart->add_subcharts());
  vis::vantage::GenerateSubchartFromData(t, {s2}, "t", {"s_" + traj_name2}, {},
                                         {{}}, time_chart->add_subcharts());
  vis::vantage::GenerateSubchartFromData(
      t, values, "t", names, {},
      std::vector<std::vector<std::string>>(names.size()),
      time_chart->add_subcharts());
}

absl::Status ExportToFile(const DdpOptimizerDebugProto& ddp_debug_proto,
                          const IpoptSolverDebugProto& ipopt_debug_proto) {
  TrajectoryOptimizerCompareProto debug_proto;
  *debug_proto.mutable_ddp() = ddp_debug_proto;
  *debug_proto.mutable_ipopt() = ipopt_debug_proto;
  const auto time_current = absl::Now();
  file_util::ProtoToTextFile(
      debug_proto,
      absl::StrFormat("%s%s_%d.pb.txt", FLAGS_traj_opt_compare_log_file_folder,
                      "data", absl::ToUnixMillis(time_current)));
  return absl::OkStatus();
}

template <typename CostContainer>
void AddCostsToChart(
    const CostContainer& costs,
    absl::btree_map<std::string, vis::vantage::ChartSeriesDataProto*>*
        y_subcharts) {
  absl::btree_map<std::string, vis::vantage::ChartSeriesDataProto*> alt;
  for (int i = 0; i < costs.size(); ++i) {
    const auto& cost_name = costs[i].name();
    const double cost_value = costs[i].cost();
    auto it = y_subcharts->find(cost_name);
    QCHECK(it != y_subcharts->end())
        << "Cost name: " << cost_name << " not found in charts.";
    if (it != y_subcharts->end()) {
      it->second->add_values(cost_value);
      std::string tip =
          it->second->values_size() == 1
              ? absl::StrFormat("init\nname : %s\ncost : %f", it->first,
                                cost_value)
              : absl::StrFormat("iteration : %d\nname : %s\ncost : %f",
                                it->second->values_size() - 1, it->first,
                                cost_value);
      it->second->add_tips(std::move(tip));
      alt.insert(y_subcharts->extract(it));
    }
  }
  for (auto& [name, y] : *y_subcharts) {
    y->add_values(0.0f);
    std::string tip =
        y->values_size() == 1
            ? absl::StrFormat("init\nname : %s\ncost : %f", name, 0.0)
            : absl::StrFormat("iteration : %d\nname : %s\ncost : %f",
                              y->values_size() - 1, name, 0.0);
    y->add_tips(std::move(tip));
  }
  y_subcharts->merge(alt);
}

}  // namespace

void AddCompareTrajCanvas(const std::string& base_name,
                          const std::vector<TrajectoryPoint>& first_traj,
                          const std::string& first_name,
                          const std::vector<TrajectoryPoint>& second_traj,
                          const std::string& second_name) {
  CanvasDrawTrajectory(
      VisIndexTrajToVector(
          [&first_traj](int index) { return first_traj[index].pos(); },
          first_traj.size(), 0.1, 0.0),
      vis::Color(0.8, 0.4, 0.4),
      /*render_indices=*/true, base_name + "/" + first_name);
  CanvasDrawTrajectory(
      VisIndexTrajToVector(
          [&second_traj](int index) { return second_traj[index].pos(); },
          second_traj.size(), 0.1, 0.0),
      vis::Color(0.4, 0.8, 0.8),
      /*render_indices=*/true, base_name + "/" + second_name);
}

void AddTrajCharts(
    const std::string& base_name, const std::vector<TrajectoryPlotInfo>& trajs,
    google::protobuf::RepeatedPtrField<vis::vantage::ChartDataProto>*
        charts_data) {
  const auto add_charts_for_traj =
      [&base_name, charts_data](
          const std::function<const TrajectoryPoint(int)>& traj_func, int size,
          const std::string& traj_name) {
        if (charts_data == nullptr) return;
        if (size == 0) return;
        std::vector<std::string> names = {
            "s",     "v",   "a",   "j",         "lon_jerk", "theta",
            "kappa", "psi", "chi", "lat_accel", "lat_jerk"};
        std::vector<std::vector<double>> values(names.size());
        for (auto& v : values) v.reserve(size);

        const TrajectoryPoint traj_point0 = traj_func(0);
        for (int i = 0; i < size; ++i) {
          const TrajectoryPoint traj_point = traj_func(i);
          int y_index = 0;
          values[y_index++].push_back(traj_point.s() - traj_point0.s());
          values[y_index++].push_back(traj_point.v());
          values[y_index++].push_back(traj_point.a());
          values[y_index++].push_back(traj_point.j());
          values[y_index++].push_back(ComputeLongitudinalJerk(traj_point));
          values[y_index++].push_back(traj_point.theta());
          values[y_index++].push_back(traj_point.kappa());
          values[y_index++].push_back(traj_point.psi());
          values[y_index++].push_back(traj_point.chi());
          values[y_index++].push_back(ComputeLateralAcceleration(traj_point));
          values[y_index++].push_back(ComputeLateralJerk(traj_point));
        }

        vis::vantage::ChartDataProto* time_chart = charts_data->Add();
        time_chart->set_title(base_name + "/" + traj_name + "/time");
        vis::vantage::GenerateSubchartFromData(
            traj_point0.t(), size, traj_func(1).t() - traj_point0.t(), values,
            "t", names, {}, std::vector<std::vector<std::string>>(names.size()),
            time_chart->add_subcharts());
      };
  for (const auto& traj : trajs) {
    add_charts_for_traj([&traj](int index) { return traj.traj[index]; },
                        traj.traj.size(), traj.name);
  }
}

absl::Status CompareWithIpopt(
    const std::string& base_name, const std::string& canvas_base_name,
    const std::vector<TrajectoryPoint>& init_traj,
    const std::vector<TrajectoryPoint>& smooth_init_traj,
    const std::vector<TrajectoryPoint>& result_traj,
    bool enable_comparison_debug_info_output,
    const DdpOptimizerDebugProto& ddp_debug_proto,
    const TrajectoryPlotInfo& ddp_result_traj, const Mfob* problem,
    vis::vantage::ChartDataBundleProto* charts_data) {
  const int trajectory_steps = static_cast<int>(result_traj.size());

  IpoptAdapter<Mfob> solver(problem, trajectory_steps, "ipopt_traj_opt");
  solver.SetInitialPoints(smooth_init_traj);
  IpoptOptimizerDebugHook<Mfob> debug_hook;
  solver.AddHook(&debug_hook);
  std::string result_info;
  auto output = solver.Solve(&result_info);
  VLOG(2) << "Ipopt solver result info: " << result_info;
  if (!output.ok()) {
    return absl::InternalError(output.status().message());
  }
  std::vector<TrajectoryPoint> result_points = std::move(*output);
  IpoptSolverDebugProto ipopt_debug_proto;
  ToIpoptDebugProto(init_traj, smooth_init_traj, result_traj, debug_hook,
                    &ipopt_debug_proto);
  std::vector<TrajectoryPlotInfo> ipopt_trajs = {
      {.traj = result_points, .name = "ipopt_res", .color = vis::Color::kBlue}};
  if (charts_data != nullptr) {
    AddTrajCharts(base_name, ipopt_trajs, charts_data->mutable_charts());
  }
  if (enable_comparison_debug_info_output) {
    if (FLAGS_traj_opt_ipopt_canvas_level > 0) {
      AddCompareTrajCanvas(canvas_base_name, result_traj, "ddp_res",
                           result_points, "ipopt_res");
    }
    AddCompareTrajCharts(base_name, ddp_result_traj, ipopt_trajs.front(),
                         charts_data);
    return ExportToFile(ddp_debug_proto, ipopt_debug_proto);
  } else {
    return absl::OkStatus();
  }
}

absl::Status ValidateTrajectory(
    const std::vector<TrajectoryPoint>& trajectory_points,
    const TrajectoryOptimizerValidationParamsProto&
        trajectory_optimizer_validation_params,
    const TrajectoryOptimizerDebugProto& optimizer_debug) {
  // Fisrt: Check final cost, if cost too large, we think result is abnormal.
  const double final_cost = optimizer_debug.ddp().final_costs().cost();
  if (final_cost > trajectory_optimizer_validation_params.max_final_cost()) {
    QEVENT_EVERY_N_SECONDS("runbing",
                           "trajectory_optimization_final_cost_over_limit", 1.0,
                           [final_cost](QEvent* qevent) {
                             qevent->AddField("final_cost", final_cost);
                           });
    return absl::InternalError(
        absl::StrCat("Traj opt final cost too large, current cost is ",
                     final_cost, " max final cost is: ",
                     trajectory_optimizer_validation_params.max_final_cost()));
  }

  // Second: Check lateral acceleration.
  double max_abs_lateral_acc = 0.0;
  for (const auto& pt : trajectory_points) {
    max_abs_lateral_acc =
        std::max(std::abs(ComputeLateralAcceleration(pt)), max_abs_lateral_acc);
  }
  if (max_abs_lateral_acc >
      trajectory_optimizer_validation_params.max_lateral_acc()) {
    QEVENT_EVERY_N_SECONDS(
        "runbing", "trajectory_optimization_lateral_acc_over_limit", 1.0,
        [max_abs_lateral_acc](QEvent* qevent) {
          qevent->AddField("max_abs_lateral_acc", max_abs_lateral_acc);
        });
    return absl::InternalError(absl::StrCat(
        "Traj opt abs lateral acc to large, abs max lateral acc is ",
        max_abs_lateral_acc, " max abs lateral acc is: ",
        trajectory_optimizer_validation_params.max_lateral_acc()));
  }

  // Second: Check lateral jerk.
  double max_abs_lateral_jerk = 0.0;
  for (const auto& pt : trajectory_points) {
    max_abs_lateral_jerk =
        std::max(std::abs(ComputeLateralJerk(pt)), max_abs_lateral_jerk);
  }
  if (max_abs_lateral_jerk >
      trajectory_optimizer_validation_params.max_lateral_jerk()) {
    QEVENT_EVERY_N_SECONDS(
        "runbing", "trajectory_optimization_lateral_jerk_over_limit", 1.0,
        [max_abs_lateral_jerk](QEvent* qevent) {
          qevent->AddField("max_abs_lateral_jerk", max_abs_lateral_jerk);
        });
    return absl::InternalError(absl::StrCat(
        "Traj opt abs lateral jerk to large, abs max lateral jerk is ",
        max_abs_lateral_jerk, " max abs lateral jerk is: ",
        trajectory_optimizer_validation_params.max_lateral_jerk()));
  }

  // Fourth: If av brakes sharply, steer changes sharply and heading changes
  // large, we think trajectory is abnormal.
  bool possible_twist = false;
  for (const auto& pt : trajectory_points) {
    if (pt.a() < trajectory_optimizer_validation_params.max_deceleration() &&
        std::abs(pt.psi()) > trajectory_optimizer_validation_params.max_psi()) {
      possible_twist = true;
      break;
    }
  }
  if (possible_twist) {
    double max_theta = -std::numeric_limits<double>::infinity();
    double min_theta = std::numeric_limits<double>::infinity();
    double min_jerk = std::numeric_limits<double>::infinity();
    double abs_max_psi = -std::numeric_limits<double>::infinity();
    double min_acceleration = std::numeric_limits<double>::infinity();
    const double min_jerk_check_time =
        trajectory_optimizer_validation_params.min_jerk_check_time();
    for (const auto& pt : trajectory_points) {
      max_theta = std::max(max_theta, pt.theta());
      min_theta = std::min(min_theta, pt.theta());
      abs_max_psi = std::min(abs_max_psi, std::abs(pt.psi()));
      min_acceleration = std::min(min_acceleration, pt.a());
      if (pt.t() < min_jerk_check_time) {
        min_jerk = std::min(min_jerk, pt.j());
      }
    }
    const double theta_diff = NormalizeAngle(min_theta - max_theta);
    if (std::abs(NormalizeAngle(min_theta - max_theta)) >
            trajectory_optimizer_validation_params.theta_diff() &&
        min_jerk < trajectory_optimizer_validation_params.min_jerk()) {
      QEVENT_EVERY_N_SECONDS(
          "runbing", "trajectory_optimization_twist", 1.0, [&](QEvent* qevent) {
            qevent->AddField("theta_diff", theta_diff)
                .AddField("min_jerk", min_jerk)
                .AddField("abs_max_psi", abs_max_psi)
                .AddField("min_acceleration", min_acceleration);
          });
      return absl::InternalError(absl::StrCat("Traj opt result twist."));
    }
  }
  return absl::OkStatus();
}

std::optional<std::vector<TrajectoryPoint>>
AdaptTrajectoryToGivenPlanStartPoint(int trajectory_steps, const Mfob& problem,
                                     const DdpOptimizerParamsProto& params,
                                     double max_adaption_cost,
                                     const TrajectoryPoint& plan_start_point,
                                     std::vector<TrajectoryPoint> trajectory) {
  QCHECK_GE(trajectory.size(), trajectory_steps);
  // replace trajectory first point with plan start point.
  trajectory.front() = plan_start_point;

  // Make a solver for the problem.
  DdpOptimizer<Mfob> solver(&problem, trajectory_steps,
                            /*owner=*/"trajectory_optimizer_refit",
                            /*verbosity=*/0, params);

  // Solve for one iteration.
  constexpr int kAdaptTrajectorySolveIteration = 1;
  DdpOptimizer<Mfob>::SolveConfig config =
      DdpOptimizer<Mfob>::SolveConfig::Default();
  config.max_iteration = kAdaptTrajectorySolveIteration;
  absl::StatusOr<std::vector<TrajectoryPoint>> refitted_trajectory =
      solver.Solve(trajectory, config);

  // return the optimized trajectory.
  if (refitted_trajectory.ok()) {
    const double refit_cost =
        solver.EvaluateCostForTrajectory(refitted_trajectory.value());
    if (refit_cost <= max_adaption_cost) {
      return std::move(refitted_trajectory.value());
    }
  }

  return std::nullopt;
}

bool HasSameDecisionOverSpacetimeObject(
    const std::vector<TrajectoryPoint>& traj_1,
    const std::vector<TrajectoryPoint>& traj_2,
    const std::vector<SpacetimeObjectState>& spacetime_object_states) {
  // Check only the prediction exist part.
  // TODO(huaiyuan): if the end 90 degree check is too strict. We may only do
  // the end check for leading objects.
  const int n = static_cast<int>(
      std::min({traj_1.size(), traj_2.size(), spacetime_object_states.size()}));
  if (n < 1) {
    return true;
  }

  // Compute angles of each trajectory point in object co-moving coordinate.
  std::vector<double> traj_1_angles;
  std::vector<double> traj_2_angles;
  traj_1_angles.reserve(n);
  traj_2_angles.reserve(n);
  for (int i = 0; i < n; ++i) {
    const Vec2d traj_1_offset =
        traj_1[i].pos() - spacetime_object_states[i].box.center();
    const Vec2d traj_2_offset =
        traj_2[i].pos() - spacetime_object_states[i].box.center();

    constexpr double kEpsilon = 1e-9;
    traj_1_angles.push_back(
        (traj_1_offset.Sqr() < kEpsilon) ? 0.0 : traj_1_offset.FastAngle());
    traj_2_angles.push_back(
        (traj_2_offset.Sqr() < kEpsilon) ? 0.0 : traj_2_offset.FastAngle());
  }

  // Start and end trajectory points shall have close angle.
  const double start_angle_diff =
      NormalizeAngle(traj_1_angles.front() - traj_2_angles.front());
  if (std::abs(start_angle_diff) > M_PI_2) {
    return false;
  }
  const double end_angle_diff =
      NormalizeAngle(traj_1_angles.back() - traj_2_angles.back());
  if (std::abs(end_angle_diff) > M_PI_2) {
    return false;
  }

  // Go through traj_1.front -> traj_1.back -> traj_2.back-> traj_2.front ->
  // traj_1.front. Sum up the accumulated angle.
  double angle_sum = start_angle_diff - end_angle_diff;
  for (int i = 0; i < n - 1; ++i) {
    angle_sum += NormalizeAngle(traj_1_angles[i + 1] - traj_1_angles[i]);
    angle_sum += NormalizeAngle(traj_2_angles[i] - traj_2_angles[i + 1]);
  }

  constexpr double kNoCirclingThreshold = M_PI;
  return std::abs(angle_sum) < kNoCirclingThreshold;
}

absl::StatusOr<std::string_view> ExtractStationaryNudgeObjectId(
    const std::vector<TrajectoryPoint>& result_points,
    const std::map<std::string, ConstraintProto::LeadingObjectProto>&
        leading_trajs,
    const SpacetimePlannerObjectTrajectories& st_planner_object_traj,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const VehicleDriveParamsProto& vehicle_drive_params) {
  if (result_points.empty()) {
    return absl::InternalError("No trajectory");
  }
  // Condition 1: AV is full stop.
  constexpr double kFullStopSpeed = 0.2;  // m/s.
  if (std::abs(result_points.front().v()) > kFullStopSpeed) {
    return absl::InternalError("High speed");
  }
  // Condition 2: Path has large curvature.
  constexpr double kNudgePathLength = 6.0;  // m.
  constexpr double kNudgeKappaRatio = 0.5;
  const double kappa_threshold =
      kNudgeKappaRatio *
      ComputeCenterMaxCurvature(vehicle_geometry_params, vehicle_drive_params);
  bool has_large_curvature_path = false;
  for (const auto& pt : result_points) {
    if (pt.s() > kNudgePathLength) {
      break;
    }
    if (std::abs(pt.kappa()) > kappa_threshold) {
      has_large_curvature_path = true;
      break;
    }
  }
  if (!has_large_curvature_path) {
    return absl::InternalError("No large curvature path");
  }
  // Condition 3: Trajectory is close to object.
  constexpr double kMaxDist = 0.45;            // m.
  constexpr double kMaxDistAtPlanStart = 7.0;  // m.
  const Box2d av_plan_start_box =
      ComputeAvBox(result_points.front().pos(), result_points.front().theta(),
                   vehicle_geometry_params);
  for (const auto& traj : st_planner_object_traj.trajectories) {
    if ((IsStaticObjectType(traj.object_type()) || traj.is_stationary()) &&
        leading_trajs.find(std::string(traj.traj_id())) ==
            leading_trajs.end()) {
      if (traj.contour().DistanceTo(av_plan_start_box) > kMaxDistAtPlanStart) {
        continue;
      }
      for (const auto& pt : result_points) {
        const auto av_box =
            ComputeAvBox(pt.pos(), pt.theta(), vehicle_geometry_params);
        if (traj.contour().DistanceTo(av_box) < kMaxDist) {
          return traj.object_id();
        }
      }
    }
  }
  return absl::InternalError("No close object");
}

}  // namespace optimizer
}  // namespace planner
}  // namespace qcraft
