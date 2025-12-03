// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type
#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <vector>

#include "benchmark/benchmark.h"

#include "onboard/math/geometry/box2d.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/math/geometry/segment2d.h"
#include "onboard/math/vec.h"
#include "onboard/planner/common/vehicle_octagon_model.h"

namespace qcraft {
namespace planner {
namespace {
constexpr double kCornerLengthRatio = 0.25;
std::vector<Vec2d> GenerateRandomVector(int n) {
  std::vector<Vec2d> v;
  v.reserve(n);

  std::mt19937 gen;
  std::uniform_real_distribution<> dis(-5.0, 5.0);

  for (int i = 0; i < n; ++i) {
    v.push_back(Vec2d(dis(gen), dis(gen)));
  }
  return v;
}

std::vector<Box2d> GenerateRandomBox2d(int n) {
  const auto centers = GenerateRandomVector(n);
  auto tangents = GenerateRandomVector(n);
  for (auto& v : tangents) {
    v /= v.norm();
  }
  auto length_width = GenerateRandomVector(n);
  for (auto& lw : length_width) {
    lw /= 100.0;
  }
  std::vector<Box2d> boxes;
  boxes.reserve(n);
  for (int i = 0; i < n; ++i) {
    boxes.push_back(Box2d(centers[i], tangents[i],
                          std::max(0.1, length_width[i].x()),
                          std::max(0.1, length_width[i].y())));
  }
  return boxes;
}

std::vector<Segment2d> GenerateRandomSegment2d(int n) {
  const auto points = GenerateRandomVector(n + 1);
  std::vector<Segment2d> segments;
  segments.reserve(n);
  for (int i = 0; i < n; ++i) {
    segments.emplace_back(points[i], points[i + 1]);
  }
  return segments;
}

std::vector<Polygon2d> GenerateRandomPolygon2d(int n) {
  const auto centers = GenerateRandomVector(n);
  auto tangents = GenerateRandomVector(n);
  for (auto& v : tangents) {
    v /= v.norm();
  }
  const auto length_width = GenerateRandomVector(n);

  std::vector<Polygon2d> polys;
  polys.reserve(n);
  for (int i = 0; i < n; ++i) {
    polys.push_back(Polygon2d(Box2d(
        centers[i], tangents[i], std::max(0.1, std::fabs(length_width[i].x())),
        std::max(0.1, std::fabs(length_width[i].y())))));
  }
  return polys;
}

bool HasOverlap(const VehicleOctagonModel& octagon,
                const Segment2d& line_segment) {
  constexpr double kEpsilon = 1e-6;
  const auto& box = octagon.box();
  const Vec2d line_start = (line_segment.start() - box.center())
                               .Rotate(box.cos_heading(), -box.sin_heading());
  const Vec2d line_end = (line_segment.end() - box.center())
                             .Rotate(box.cos_heading(), -box.sin_heading());
  if (std::min(line_start.x(), line_end.x()) > box.half_length() ||
      std::max(line_start.x(), line_end.x()) < -box.half_length() ||
      std::min(line_start.y(), line_end.y()) > box.half_width() ||
      std::max(line_start.y(), line_end.y()) < -box.half_width()) {
    return false;
  }

  if (line_segment.length() <= kEpsilon) {
    return octagon.IsPointIn(line_segment.start());
  }

  if (octagon.IsPointIn(line_segment.start())) {
    return true;
  }
  if (octagon.IsPointIn(line_segment.end())) {
    return true;
  }

  for (const auto& poly_seg : octagon.line_segments()) {
    if (poly_seg.HasIntersect(line_segment)) return true;
  }
  return false;
}

double DistanceTo(const VehicleOctagonModel& octagon,
                  const Segment2d& line_segment) {
  constexpr double kEpsilon = 1e-6;
  if (line_segment.length() <= kEpsilon) {
    return octagon.DistanceTo(line_segment.start());
  }
  if (octagon.IsPointIn(line_segment.center())) {
    return 0.0;
  }
  if (std::any_of(octagon.line_segments().begin(),
                  octagon.line_segments().end(),
                  [&](const Segment2d& poly_seg) {
                    return poly_seg.HasIntersect(line_segment);
                  })) {
    return 0.0;
  }

  double res = std::numeric_limits<double>::max();
  for (const auto& poly_seg : octagon.line_segments()) {
    const double dist = poly_seg.DistanceTo(line_segment);
    if (dist == 0.0) return 0.0;
    res = std::min(res, dist);
  }
  return res;
}

void BM_OctagonConstructor(benchmark::State& state) {  // NOLINT
  const int size = state.range(0);
  const auto boxes = GenerateRandomBox2d(size);
  const auto points = GenerateRandomVector(size);
  for (auto _ : state) {
    for (int i = 0; i < size; ++i) {
      for (int j = 0; j < size; ++j) {
        benchmark::DoNotOptimize(VehicleOctagonModel(
            boxes[i], boxes[i].length() * kCornerLengthRatio,
            boxes[i].width() * kCornerLengthRatio));
      }
    }
  }
}
BENCHMARK(BM_OctagonConstructor)->Arg(10)->Arg(100)->Arg(1000);

void BM_GetCornersWithBufferCounterClockwise(
    benchmark::State& state) {  // NOLINT
  const int size = state.range(0);
  const auto boxes = GenerateRandomBox2d(size);
  std::vector<VehicleOctagonModel> octagons;
  octagons.reserve(size);
  for (int i = 0; i < size; ++i) {
    octagons.emplace_back(boxes[i], boxes[i].length() * kCornerLengthRatio,
                          boxes[i].width() * kCornerLengthRatio);
  }
  constexpr double kBuffer = 0.5;
  for (auto _ : state) {
    for (int i = 0; i < size; ++i) {
      for (int j = 0; j < size; ++j) {
        benchmark::DoNotOptimize(
            octagons[i].GetCornersWithBufferCounterClockwise(kBuffer, kBuffer));
      }
    }
  }
}
BENCHMARK(BM_GetCornersWithBufferCounterClockwise)
    ->Arg(10)
    ->Arg(100)
    ->Arg(1000);

void BM_OctagonIsPointIn(benchmark::State& state) {  // NOLINT
  const int size = state.range(0);
  const auto boxes = GenerateRandomBox2d(size);
  const auto points = GenerateRandomVector(size);
  std::vector<VehicleOctagonModel> octagons;
  octagons.reserve(size);
  for (int i = 0; i < size; ++i) {
    octagons.emplace_back(boxes[i], boxes[i].length() * kCornerLengthRatio,
                          boxes[i].width() * kCornerLengthRatio);
  }
  std::vector<Polygon2d> polygons;
  polygons.reserve(size);
  for (int i = 0; i < size; ++i) {
    polygons.emplace_back(
        VehicleOctagonModel(boxes[i], boxes[i].length() * kCornerLengthRatio,
                            boxes[i].width() * kCornerLengthRatio)
            .points(),
        true);
  }
  for (auto _ : state) {
    for (int i = 0; i < size; ++i) {
      for (int j = 0; j < size; ++j) {
        benchmark::DoNotOptimize(octagons[i].IsPointIn(points[j]));
      }
    }
  }
}
BENCHMARK(BM_OctagonIsPointIn)->Arg(10)->Arg(100)->Arg(1000);

void BM_PolygonIsPointIn(benchmark::State& state) {  // NOLINT
  const int size = state.range(0);
  const auto boxes = GenerateRandomBox2d(size);
  const auto points = GenerateRandomVector(size);
  std::vector<VehicleOctagonModel> octagons;
  octagons.reserve(size);
  for (int i = 0; i < size; ++i) {
    octagons.emplace_back(boxes[i], boxes[i].length() * kCornerLengthRatio,
                          boxes[i].width() * kCornerLengthRatio);
  }
  std::vector<Polygon2d> polygons;
  polygons.reserve(size);
  for (int i = 0; i < size; ++i) {
    polygons.emplace_back(
        VehicleOctagonModel(boxes[i], boxes[i].length() * kCornerLengthRatio,
                            boxes[i].width() * kCornerLengthRatio)
            .points(),
        true);
  }
  for (auto _ : state) {
    for (int i = 0; i < size; ++i) {
      for (int j = 0; j < size; ++j) {
        benchmark::DoNotOptimize(polygons[i].IsPointIn(points[j]));
      }
    }
  }
}
BENCHMARK(BM_PolygonIsPointIn)->Arg(10)->Arg(100)->Arg(1000);

void BM_HasOverlapSegment2dGjk(benchmark::State& state) {  // NOLINT
  const int size = state.range(0);
  const auto boxes = GenerateRandomBox2d(size);
  const auto line_segments = GenerateRandomSegment2d(size);
  std::vector<VehicleOctagonModel> octagons;
  octagons.reserve(size);
  for (int i = 0; i < size; ++i) {
    octagons.emplace_back(boxes[i], boxes[i].length() * kCornerLengthRatio,
                          boxes[i].width() * kCornerLengthRatio);
  }
  for (auto _ : state) {
    for (int i = 0; i < size; ++i) {
      for (int j = 0; j < size; ++j) {
        benchmark::DoNotOptimize(octagons[i].HasOverlap(line_segments[j]));
      }
    }
  }
}
BENCHMARK(BM_HasOverlapSegment2dGjk)->Arg(10)->Arg(100)->Arg(1000);

void BM_HasOverlapSegment2dBruteForce(benchmark::State& state) {  // NOLINT
  const int size = state.range(0);
  const auto boxes = GenerateRandomBox2d(size);
  const auto line_segments = GenerateRandomSegment2d(size);
  std::vector<VehicleOctagonModel> octagons;
  octagons.reserve(size);
  for (int i = 0; i < size; ++i) {
    octagons.emplace_back(boxes[i], boxes[i].length() * kCornerLengthRatio,
                          boxes[i].width() * kCornerLengthRatio);
  }
  for (auto _ : state) {
    for (int i = 0; i < size; ++i) {
      for (int j = 0; j < size; ++j) {
        benchmark::DoNotOptimize(HasOverlap(octagons[i], line_segments[j]));
      }
    }
  }
}
BENCHMARK(BM_HasOverlapSegment2dBruteForce)->Arg(10)->Arg(100)->Arg(1000);

void BM_HasOverlapWithBufferSegment2d(benchmark::State& state) {  // NOLINT
  const int size = state.range(0);
  const auto boxes = GenerateRandomBox2d(size);
  const auto line_segments = GenerateRandomSegment2d(size);
  constexpr double kBuffer = 0.0;
  std::vector<VehicleOctagonModel> octagons;
  octagons.reserve(size);
  for (int i = 0; i < size; ++i) {
    octagons.emplace_back(boxes[i], boxes[i].length() * kCornerLengthRatio,
                          boxes[i].width() * kCornerLengthRatio);
  }
  for (auto _ : state) {
    for (int i = 0; i < size; ++i) {
      for (int j = 0; j < size; ++j) {
        benchmark::DoNotOptimize(octagons[i].HasOverlapWithBuffer(
            line_segments[j], kBuffer, kBuffer));
      }
    }
  }
}
BENCHMARK(BM_HasOverlapWithBufferSegment2d)->Arg(10)->Arg(100)->Arg(1000);

void BM_DistanceToSegment2dGjk(benchmark::State& state) {  // NOLINT
  const int size = state.range(0);
  const auto boxes = GenerateRandomBox2d(size);
  const auto line_segments = GenerateRandomSegment2d(size);
  std::vector<VehicleOctagonModel> octagons;
  octagons.reserve(size);
  for (int i = 0; i < size; ++i) {
    octagons.emplace_back(boxes[i], boxes[i].length() * kCornerLengthRatio,
                          boxes[i].width() * kCornerLengthRatio);
  }
  for (auto _ : state) {
    for (int i = 0; i < size; ++i) {
      for (int j = 0; j < size; ++j) {
        benchmark::DoNotOptimize(octagons[i].DistanceTo(line_segments[j]));
      }
    }
  }
}
BENCHMARK(BM_DistanceToSegment2dGjk)->Arg(10)->Arg(100)->Arg(1000);

void BM_DistanceToSegment2dBruteForce(benchmark::State& state) {  // NOLINT
  const int size = state.range(0);
  const auto boxes = GenerateRandomBox2d(size);
  const auto line_segments = GenerateRandomSegment2d(size);
  std::vector<VehicleOctagonModel> octagons;
  octagons.reserve(size);
  for (int i = 0; i < size; ++i) {
    octagons.emplace_back(boxes[i], boxes[i].length() * kCornerLengthRatio,
                          boxes[i].width() * kCornerLengthRatio);
  }
  for (auto _ : state) {
    for (int i = 0; i < size; ++i) {
      for (int j = 0; j < size; ++j) {
        benchmark::DoNotOptimize(DistanceTo(octagons[i], line_segments[j]));
      }
    }
  }
}
BENCHMARK(BM_DistanceToSegment2dBruteForce)->Arg(10)->Arg(100)->Arg(1000);

void BM_HasOverlapPolygonGjk(benchmark::State& state) {  // NOLINT
  const int size = state.range(0);
  const auto boxes = GenerateRandomBox2d(size);
  const auto polygons = GenerateRandomPolygon2d(size);
  std::vector<VehicleOctagonModel> octagons;
  octagons.reserve(size);
  for (int i = 0; i < size; ++i) {
    octagons.emplace_back(boxes[i], boxes[i].length() * kCornerLengthRatio,
                          boxes[i].width() * kCornerLengthRatio);
  }
  for (auto _ : state) {
    for (int i = 0; i < size; ++i) {
      for (int j = 0; j < size; ++j) {
        benchmark::DoNotOptimize(octagons[i].HasOverlap(polygons[j]));
      }
    }
  }
}
BENCHMARK(BM_HasOverlapPolygonGjk)->Arg(10)->Arg(100)->Arg(1000);

void BM_HasOverlapPolygonBruteForce(benchmark::State& state) {  // NOLINT
  const int size = state.range(0);
  const auto boxes = GenerateRandomBox2d(size);
  const auto polygons = GenerateRandomPolygon2d(size);
  std::vector<VehicleOctagonModel> octagons;
  octagons.reserve(size);
  for (int i = 0; i < size; ++i) {
    octagons.emplace_back(boxes[i], boxes[i].length() * kCornerLengthRatio,
                          boxes[i].width() * kCornerLengthRatio);
  }
  std::vector<Polygon2d> polygons_octagons;
  polygons_octagons.reserve(size);
  for (int i = 0; i < size; ++i) {
    polygons_octagons.emplace_back(
        VehicleOctagonModel(boxes[i], boxes[i].length() * kCornerLengthRatio,
                            boxes[i].width() * kCornerLengthRatio)
            .points(),
        true);
  }
  for (auto _ : state) {
    for (int i = 0; i < size; ++i) {
      for (int j = 0; j < size; ++j) {
        benchmark::DoNotOptimize(polygons_octagons[i].HasOverlap(polygons[j]));
      }
    }
  }
}
BENCHMARK(BM_HasOverlapPolygonBruteForce)->Arg(10)->Arg(100)->Arg(1000);

void BM_HasOverlapWithBufferPolygon(benchmark::State& state) {  // NOLINT
  const int size = state.range(0);
  const auto boxes = GenerateRandomBox2d(size);
  const auto polygons = GenerateRandomPolygon2d(size);
  constexpr double kBuffer = 0.0;
  std::vector<VehicleOctagonModel> octagons;
  octagons.reserve(size);
  for (int i = 0; i < size; ++i) {
    octagons.emplace_back(boxes[i], boxes[i].length() * kCornerLengthRatio,
                          boxes[i].width() * kCornerLengthRatio);
  }
  for (auto _ : state) {
    for (int i = 0; i < size; ++i) {
      for (int j = 0; j < size; ++j) {
        benchmark::DoNotOptimize(
            octagons[i].HasOverlapWithBuffer(polygons[j], kBuffer, kBuffer));
      }
    }
  }
}
BENCHMARK(BM_HasOverlapWithBufferPolygon)->Arg(10)->Arg(100)->Arg(1000);

void BM_DistanceToPolygonGjk(benchmark::State& state) {  // NOLINT
  const int size = state.range(0);
  const auto boxes = GenerateRandomBox2d(size);
  const auto polygons = GenerateRandomPolygon2d(size);
  std::vector<VehicleOctagonModel> octagons;
  octagons.reserve(size);
  for (int i = 0; i < size; ++i) {
    octagons.emplace_back(boxes[i], boxes[i].length() * kCornerLengthRatio,
                          boxes[i].width() * kCornerLengthRatio);
  }
  for (auto _ : state) {
    for (int i = 0; i < size; ++i) {
      for (int j = 0; j < size; ++j) {
        benchmark::DoNotOptimize(octagons[i].DistanceTo(polygons[j]));
      }
    }
  }
}
BENCHMARK(BM_DistanceToPolygonGjk)->Arg(10)->Arg(100)->Arg(1000);

void BM_DistanceToPolygonForce(benchmark::State& state) {  // NOLINT
  const int size = state.range(0);
  const auto boxes = GenerateRandomBox2d(size);
  const auto polygons = GenerateRandomPolygon2d(size);
  std::vector<VehicleOctagonModel> octagons;
  octagons.reserve(size);
  for (int i = 0; i < size; ++i) {
    octagons.emplace_back(boxes[i], boxes[i].length() * kCornerLengthRatio,
                          boxes[i].width() * kCornerLengthRatio);
  }
  std::vector<Polygon2d> polygons_octagons;
  polygons_octagons.reserve(size);
  for (int i = 0; i < size; ++i) {
    polygons_octagons.emplace_back(
        VehicleOctagonModel(boxes[i], boxes[i].length() * kCornerLengthRatio,
                            boxes[i].width() * kCornerLengthRatio)
            .points(),
        true);
  }
  for (auto _ : state) {
    for (int i = 0; i < size; ++i) {
      for (int j = 0; j < size; ++j) {
        benchmark::DoNotOptimize(polygons_octagons[i].DistanceTo(polygons[j]));
      }
    }
  }
}
BENCHMARK(BM_DistanceToPolygonForce)->Arg(10)->Arg(100)->Arg(1000);

void BM_ExtremePoint(benchmark::State& state) {  // NOLINT
  const int size = state.range(0);
  const auto boxes = GenerateRandomBox2d(size);
  const auto vecs = GenerateRandomVector(size);
  std::vector<VehicleOctagonModel> octagons;
  octagons.reserve(size);
  for (int i = 0; i < size; ++i) {
    octagons.emplace_back(boxes[i], boxes[i].length() * kCornerLengthRatio,
                          boxes[i].width() * kCornerLengthRatio);
  }
  std::vector<Polygon2d> polygons_octagons;
  polygons_octagons.reserve(size);
  for (int i = 0; i < size; ++i) {
    polygons_octagons.emplace_back(
        VehicleOctagonModel(boxes[i], boxes[i].length() * kCornerLengthRatio,
                            boxes[i].width() * kCornerLengthRatio)
            .points(),
        true);
  }
  for (auto _ : state) {
    for (int i = 0; i < size; ++i) {
      for (int j = 0; j < size; ++j) {
        benchmark::DoNotOptimize(octagons[i].ExtremePoint(vecs[j]));
      }
    }
  }
}
BENCHMARK(BM_ExtremePoint)->Arg(10)->Arg(100)->Arg(1000);

void BM_ExtremePoint2(benchmark::State& state) {  // NOLINT
  const int size = state.range(0);
  const auto boxes = GenerateRandomBox2d(size);
  const auto vecs = GenerateRandomVector(size);
  std::vector<VehicleOctagonModel> octagons;
  octagons.reserve(size);
  for (int i = 0; i < size; ++i) {
    octagons.emplace_back(boxes[i], boxes[i].length() * kCornerLengthRatio,
                          boxes[i].width() * kCornerLengthRatio);
  }
  std::vector<Polygon2d> polygons_octagons;
  polygons_octagons.reserve(size);
  for (int i = 0; i < size; ++i) {
    polygons_octagons.emplace_back(
        VehicleOctagonModel(boxes[i], boxes[i].length() * kCornerLengthRatio,
                            boxes[i].width() * kCornerLengthRatio)
            .points(),
        true);
  }
  for (auto _ : state) {
    for (int i = 0; i < size; ++i) {
      for (int j = 0; j < size; ++j) {
        benchmark::DoNotOptimize(polygons_octagons[i].ExtremePoint(vecs[j]));
      }
    }
  }
}
BENCHMARK(BM_ExtremePoint2)->Arg(10)->Arg(100)->Arg(1000);

}  // namespace
}  // namespace planner
}  // namespace qcraft

BENCHMARK_MAIN();
