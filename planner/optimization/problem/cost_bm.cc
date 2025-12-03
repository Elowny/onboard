// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type

#include <memory>
#include <random>
#include <utility>
#include <vector>

#include "Eigen/Core"
#include "benchmark/benchmark.h"

#include "onboard/planner/optimization/problem/cost.h"
#include "onboard/planner/optimization/problem/reference_control_deviation_cost.h"
#include "onboard/planner/optimization/problem/reference_state_deviation_cost.h"
#include "onboard/planner/optimization/problem/third_order_bicycle.h"

namespace qcraft {
namespace planner {
namespace {

constexpr int kTrajectorySteps = 100;
using Tob = ThirdOrderBicycle;

std::vector<std::pair<Tob::StatesType, Tob::ControlsType>> GenerateRandomInput(
    int n) {
  std::vector<std::pair<Tob::StatesType, Tob::ControlsType>> v;
  v.reserve(n);
  std::mt19937 gen;
  std::uniform_real_distribution<> dis(0.0, 100.0);

  for (int i = 0; i < n; ++i) {
    Tob::StatesType xs(kTrajectorySteps * Tob::kStateSize);
    Tob::ControlsType us(kTrajectorySteps * Tob::kControlSize);
    for (int k = 0; k < kTrajectorySteps; ++k) {
      Tob::SetStateAtStep(Tob::MakeState(dis(gen), dis(gen), dis(gen), dis(gen),
                                         dis(gen), dis(gen), dis(gen)),
                          k, &xs);
      Tob::SetControlAtStep(Tob::MakeControl(dis(gen), dis(gen)), k, &us);
    }
    v.emplace_back(xs, us);
  }
  return v;
}

const std::vector<std::unique_ptr<Cost<Tob>>> GenerateCosts() {
  std::vector<std::unique_ptr<Cost<Tob>>> costs;
  const Tob::ControlsType tob_ref_us =
      Tob::ControlsType::Zero(kTrajectorySteps * Tob::kControlSize);
  costs.push_back(
      std::make_unique<ReferenceControlDeviationCost<Tob>>(tob_ref_us));
  const Tob::StatesType tob_ref_xs =
      Tob::StatesType::Zero(kTrajectorySteps * Tob::kStateSize);
  costs.push_back(
      std::make_unique<ReferenceStateDeviationCost<Tob>>(tob_ref_xs));

  return costs;
}

bool TestEvaluateBySum(const std::pair<Tob::StatesType, Tob::ControlsType>& v,
                       const std::vector<std::unique_ptr<Cost<Tob>>>& costs) {
  std::vector<double> g(costs.size(), 0.0);
  for (int i = 0; i < costs.size(); ++i) {
    g[i] +=
        costs[i]->SumGForAllSteps(v.first, v.second, kTrajectorySteps).sum();
  }
  return true;
}
void BM_SumAllSteps(benchmark::State& state) {  // NOLINT
  constexpr int kNum = 1000;
  const std::vector<std::pair<Tob::StatesType, Tob::ControlsType>> v =
      GenerateRandomInput(kNum);

  const std::vector<std::unique_ptr<Cost<Tob>>> costs = GenerateCosts();

  for (auto _ : state) {
    for (int i = 0; i < kNum; ++i) {
      benchmark::DoNotOptimize(TestEvaluateBySum(v[i], costs));
    }
  }
}
BENCHMARK(BM_SumAllSteps);

bool TestEvaluateByEvaluate(
    const std::pair<Tob::StatesType, Tob::ControlsType>& v,
    const std::vector<std::unique_ptr<Cost<Tob>>>& costs) {
  std::vector<double> g(costs.size(), 0.0);
  for (int i = 0; i < costs.size(); ++i) {
    for (int k = 0; k < kTrajectorySteps; ++k) {
      g[i] += costs[i]->EvaluateG(k, Tob::GetStateAtStep(v.first, k),
                                  Tob::GetControlAtStep(v.second, k));
    }
  }
  return true;
}

void BM_Evaluate(benchmark::State& state) {  // NOLINT
  constexpr int kNum = 1000;
  const std::vector<std::pair<Tob::StatesType, Tob::ControlsType>> v =
      GenerateRandomInput(kNum);

  const std::vector<std::unique_ptr<Cost<Tob>>> costs = GenerateCosts();
  for (auto _ : state) {
    for (int i = 0; i < kNum; ++i) {
      benchmark::DoNotOptimize(TestEvaluateByEvaluate(v[i], costs));
    }
  }
}
BENCHMARK(BM_Evaluate);

}  // namespace
}  // namespace planner
}  // namespace qcraft

BENCHMARK_MAIN();
