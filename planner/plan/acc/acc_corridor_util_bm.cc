#include <random>
#include <vector>

#include "benchmark/benchmark.h"

#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/plan/acc/acc_corridor_util.h"

namespace qcraft {
namespace planner {
namespace {

constexpr int kTimes = 100;

double RandomDouble(double start, double end, std::mt19937* gen) {
  std::uniform_real_distribution<> dis(start, end);
  return dis(*gen);
}

Vec2d RandomPointInUnitCircle(std::mt19937* gen) {
  constexpr int kTrialThreshold = 20;
  double x, y;
  for (int i = 0; i < kTrialThreshold; ++i) {
    x = RandomDouble(-1.0, 1.0, gen);
    y = RandomDouble(-1.0, 1.0, gen);

    if (Sqr(x) + Sqr(y) <= 1.0) {
      return Vec2d{x, y};
    }
  }
  // Probability of triggering this line is:
  // P_out^20, P_out = 1 - 3.14/4.0
  // Which evaluates to 5e-14.
  return Vec2d{0.0, 0.0};
}

std::vector<Vec2d> ComputeHeadingsForPath(const std::vector<Vec2d>& path) {
  std::vector<Vec2d> headings;
  headings.reserve(path.size());

  for (int i = 0; i < path.size() - 1; ++i) {
    const auto& cur = path[i];
    const auto& next = path[i + 1];
    headings.push_back((next - cur).Unit());
  }
  headings.push_back(headings[path.size() - 2]);
  QCHECK_EQ(headings.size(), path.size());
  return headings;
}

std::vector<double> ComputeArcLengthsForPath(const std::vector<Vec2d>& path) {
  std::vector<double> arc_lengths;
  arc_lengths.reserve(path.size());
  arc_lengths.push_back(0.0);
  for (int i = 1; i < path.size(); ++i) {
    const auto& cur = path[i];
    const auto& prev = path[i - 1];
    const auto prev_arc_length = arc_lengths.back();
    arc_lengths.push_back(prev_arc_length + cur.DistanceTo(prev));
  }
  QCHECK_EQ(arc_lengths.size(), path.size());
  return arc_lengths;
}

std::vector<Vec2d> GenerateRandomXYPointsAroundRefPathPoints(
    const std::vector<Vec2d>& path, double radius) {
  QCHECK(!path.empty());

  std::mt19937 gen;
  std::vector<Vec2d> points;
  const int n = path.size();
  points.reserve(n);
  for (int i = 0; i < n; ++i) {
    points.emplace_back(path[i] + radius * RandomPointInUnitCircle(&gen));
  }
  return points;
}

std::vector<Vec2d> GeneratePath(int n) {
  std::vector<Vec2d> points;
  points.reserve(n);
  for (int i = 0; i < n; ++i) {
    points.emplace_back(Vec2d(i * 1.0, 0.0));
  }
  return points;
}

void BM_ComputeExtendedSmoothPath(benchmark::State& state) {  // NOLINT
  // Generate randomly sampled paths.
  const auto ref_path = GeneratePath(100);

  std::vector<std::vector<Vec2d>> paths, headings;
  std::vector<std::vector<double>> arc_lengths;
  paths.reserve(kTimes);
  headings.reserve(kTimes);
  arc_lengths.reserve(kTimes);

  for (int i = 0; i < kTimes; ++i) {
    paths.push_back(
        GenerateRandomXYPointsAroundRefPathPoints(ref_path, /*radius=*/0.0));
    headings.push_back(ComputeHeadingsForPath(paths.back()));
    arc_lengths.push_back(ComputeArcLengthsForPath(paths.back()));
  }
  for (auto _ : state) {
    const auto idx = state.iterations() % kTimes;
    const auto& heading = headings[idx];
    const auto& arc_length = arc_lengths[idx];
    benchmark::DoNotOptimize(
        ComputeExtendedSmoothPath(paths[idx], arc_length, heading.back(),
                                  /*step_s=*/2.0, /*resample_step_s=*/0.2,
                                  /*extended_length=*/1.5 * arc_length.back()));
  }
}

BENCHMARK(BM_ComputeExtendedSmoothPath);

}  // namespace
}  // namespace planner
}  // namespace qcraft

BENCHMARK_MAIN();
