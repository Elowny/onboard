#include <random>
#include <vector>

#include "benchmark/benchmark.h"

#include "onboard/math/vec.h"
#include "onboard/planner/router/geometry/gfc.h"

namespace qcraft::planner::route {
namespace {
void BM_ProjectToLineString(benchmark::State& state) {  // NOLINT
  std::vector<Vec2d> points;
  points.reserve(state.range(0));

  std::uniform_real_distribution<double> unif_lon(2.1054, 2.1055);
  std::default_random_engine re;
  std::uniform_real_distribution<double> unif_lat(0.51054, 0.51055);
  for (int i = 0; i < state.range(0); ++i) {
    const double lon = unif_lon(re);
    const double lat = unif_lat(re);
    // Poly line can be self-intersected.
    points.emplace_back(lon, lat);
  }

  const Vec2d p = {2.1054, 0.51055};
  for (auto _ : state) {
    benchmark::DoNotOptimize(ProjectToLineString(points, p));
  }
}

BENCHMARK(BM_ProjectToLineString)
    ->Arg(5)
    ->Arg(10)
    ->Arg(20)
    ->Arg(50)
    ->Arg(100)
    ->Arg(200)
    ->Arg(500)
    ->Arg(1000);
}  // namespace
}  // namespace qcraft::planner::route

BENCHMARK_MAIN();
