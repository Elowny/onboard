#include <algorithm>
#include <vector>

#include "benchmark/benchmark.h"
#include "glog/logging.h"

#include "onboard/container/strong_vector.h"
#include "onboard/math/vec.h"
#include "onboard/planner/initializer/geometry/geometry_form.h"
#include "onboard/planner/initializer/geometry/geometry_graph.h"

// Cannot use __cpp_lib_hardware_interference_size MACRO
namespace qcraft::planner {
DECLARE_STRONG_VECTOR(GeometryNodeV2);
struct GeometryNodeV2 {
  Vec2d xy;
  double k;
  double lateral_offset = 0.0;
  double accumulated_s = 0.0;
  int station_index = 0;
  GeometryNodeV2Index index;
  // Flag to signify if the node is reachable from start state.
  bool reachable = false;
  bool resampled = false;
  // Flag turned off according to reference line.
  bool active = true;
};

struct GeometryEdgeV2 {
  int flag;
  int index;
  GeometryNodeV2Index start;
  GeometryNodeV2Index end;
  const GeometryForm* geometry;
  bool isTruncated() const { return flag & 0x2; }
  bool is_active() const { return flag & 0x01; }
};

void BM_GeometryNodeV2Size(benchmark::State& state) {  // NOLINT
  std::vector<GeometryNodeV2> v2;
  v2.reserve(1000);
  for (auto _ : state) {
    for (int i = 0; i < 1000; ++i) {
      v2.push_back(GeometryNodeV2());
    }
  }
}

void BM_GeometryNodeSize(benchmark::State& state) {  // NOLINT
  std::vector<GeometryNode> v;
  v.reserve(1000);
  for (auto _ : state) {
    for (int i = 0; i < 1000; ++i) {
      v.push_back(GeometryNode());
    }
  }
}

void BM_GeometryEdgeV2Size(benchmark::State& state) {  // NOLINT
  std::vector<GeometryEdgeV2> v2;
  v2.reserve(1000);
  for (auto _ : state) {
    for (int i = 0; i < 1000; ++i) {
      v2.push_back(GeometryEdgeV2());
    }
  }
}

void BM_GeometryEdgeSize(benchmark::State& state) {  // NOLINT
  std::vector<GeometryEdge> v;
  v.reserve(1000);
  for (auto _ : state) {
    for (int i = 0; i < 1000; ++i) {
      v.push_back(GeometryEdge());
    }
  }
}
namespace {

BENCHMARK(BM_GeometryNodeSize)->Unit(benchmark::kMillisecond)->Arg(5);
BENCHMARK(BM_GeometryNodeV2Size)->Unit(benchmark::kMillisecond)->Arg(5);
BENCHMARK(BM_GeometryEdgeSize)->Unit(benchmark::kMillisecond)->Arg(5);
BENCHMARK(BM_GeometryEdgeV2Size)->Unit(benchmark::kMillisecond)->Arg(5);

}  // namespace

}  // namespace qcraft::planner

BENCHMARK_MAIN();
