#include <algorithm>
#include <cstdint>
#include <map>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/strings/str_format.h"
#include "benchmark/benchmark.h"

#include "onboard/planner/router/lane_graph/v2/dijkstra.h"
#include "onboard/planner/router/lane_graph/v2/lane_graph.h"
namespace qcraft::planner::route {

namespace {

void BM_StringFormatVertex(benchmark::State& state) {  // NOLINT
  std::vector<std::string> vertices;
  vertices.reserve(300);
  for (auto _ : state) {
    for (int i = 0; i < 100; i++) {
      for (int j = 0; j < 3; j++) {
        vertices.push_back(absl::StrFormat("%d-%d", i, j));
      }
    }
  }
}

void BM_StringVertex(benchmark::State& state) {  // NOLINT
  std::vector<std::string> vertices;
  vertices.reserve(300);
  for (auto _ : state) {
    for (int i = 0; i < 100; i++) {
      for (int j = 0; j < 3; j++) {
        vertices.push_back(std::to_string(i) + "-" + std::to_string(j));
      }
    }
  }
}

void BM_IntVertex(benchmark::State& state) {  // NOLINT
  std::vector<std::int64_t> vertices;
  vertices.reserve(300);
  for (auto _ : state) {
    for (int i = 0; i < 100; i++) {
      for (int j = 0; j < 3; j++) {
        vertices.push_back((j << 16) + i);
      }
    }
  }
}

void BM_BoostInt64GraphDijkstra(benchmark::State& state) {  // NOLINT
  v2::VertexGraph<std::pair<int, std::int64_t> > g;
  for (int i = 0; i < 100; i++) {
    for (int j = 0; j < 3; j++) {
      g.AddVertex(v2::ToVertId(i, j));
    }
  }
  std::random_device rd;
  std::mt19937 gen(rd());  // Standard mersenne_twister_engine seeded with rd()
  std::uniform_int_distribution<> distrib(1, 100);

  for (int i = 0; i < 100; i++) {
    for (int j = 0; j < 3; j++) {
      if (j - 1 > 0) {
        g.AddEdge(v2::ToVertId(i, j), v2::ToVertId(i, j - 1), distrib(gen));
      }
      if (j + 1 < 3) {
        g.AddEdge(v2::ToVertId(i, j), v2::ToVertId(i, j + 1), distrib(gen));
      }
      if (i + 1 < 100) {
        g.AddEdge(v2::ToVertId(i, j), v2::ToVertId(i + 1, j), distrib(gen));
        if (j - 1 > 0) {
          g.AddEdge(v2::ToVertId(i + 1, j), v2::ToVertId(i + 1, j - 1),
                    distrib(gen));
        }
        if (j + 1 < 3) {
          g.AddEdge(v2::ToVertId(i + 1, j), v2::ToVertId(i + 1, j + 1),
                    distrib(gen));
        }
      }
    }
  }
  for (auto _ : state) {
    // dijkstra
    benchmark::DoNotOptimize(
        v2::Dijkstra(g, v2::ToVertId(0, 0), v2::ToVertId(99, 2)));
  }
}

// 2 km / 10m = 100, if avg=3 lanes, about 100*3 vertex
BENCHMARK(BM_StringVertex)->Unit(benchmark::kMillisecond)->Arg(5);
BENCHMARK(BM_StringFormatVertex)->Unit(benchmark::kMillisecond)->Arg(5);
BENCHMARK(BM_IntVertex)->Unit(benchmark::kMillisecond)->Arg(5);
BENCHMARK(BM_BoostInt64GraphDijkstra)->Unit(benchmark::kMillisecond)->Arg(5);

}  // namespace

}  // namespace qcraft::planner::route

BENCHMARK_MAIN();
