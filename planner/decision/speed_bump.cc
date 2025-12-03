#include "onboard/planner/decision/speed_bump.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <utility>

#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"

#include "onboard/maps/maps_helper.h"
#include "onboard/maps/proto/semantic_map.pb.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/math/frenet_common.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/math/geometry/segment2d.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"

namespace qcraft {
namespace planner {

namespace {
// TODO(yumeng): Maybe this function can be refactored as a utility function.
// Returns a speed region range. In the returned pair, the first value is min s,
// and the second value is max s.
bool CalculateSpeedRegionRange(const DrivePassage& passage,
                               const Polygon2d& polygon,
                               std::pair<double, double>* min_max_s,
                               Vec2d* start_point, Vec2d* end_point) {
  min_max_s->first = std::numeric_limits<double>::infinity();
  min_max_s->second = -std::numeric_limits<double>::infinity();

  // Find overlap points and calculate min max s.
  for (const auto index : passage.stations().index_from(1)) {
    const StationIndex prev_index(index.value() - 1);
    const Segment2d segment(passage.stations()[prev_index].xy(),
                            passage.stations()[index].xy());

    if (polygon.HasOverlap(segment)) {
      Vec2d first;
      Vec2d last;
      polygon.GetOverlap(segment, &first, &last);

      // TODO(weijun): Do not apply xy projection. It's very obvious that we can
      // compute s directly from line segment.

      const auto sl_first = passage.QueryFrenetCoordinateAt(first);
      if (!sl_first.ok()) {
        return false;
      }
      if (sl_first.value().s < min_max_s->first) {
        min_max_s->first = sl_first.value().s;
        *start_point = first;
      }
      if (sl_first.value().s > min_max_s->second) {
        min_max_s->second = sl_first.value().s;
        *end_point = first;
      }

      const auto sl_last = passage.QueryFrenetCoordinateAt(last);
      if (!sl_last.ok()) {
        return false;
      }
      if (sl_last.value().s < min_max_s->first) {
        min_max_s->first = sl_last.value().s;
        *start_point = last;
      }
      if (sl_last.value().s > min_max_s->second) {
        min_max_s->second = sl_last.value().s;
        *end_point = last;
      }
    }
  }

  // No overlap found.
  return !std::isinf(min_max_s->first) && !std::isinf(min_max_s->second);
}
}  // namespace

// This function returns speed bump constraints.
std::vector<ConstraintProto::SpeedRegionProto> BuildSpeedBumpConstraints(
    const PlannerSemanticMapManager& psmm, const DrivePassage& passage) {
  // TODO(yumeng) pre-load this info in SemanticMapManager.
  // Speed constraint: speed bumps.
  struct SpeedBumpInfo {
    mapping::ElementId id;
    Polygon2d polygon;
    double speed_limit;
  };
  std::vector<SpeedBumpInfo> speed_bumps;
  speed_bumps.reserve(psmm.semantic_map_proto().speed_bumps_size());
  for (const auto& speed_bump : psmm.semantic_map_proto().speed_bumps()) {
    Polygon2d polygon = mapping::GeoPolygonToSmoothPolygon2d(
        speed_bump.polygon(), psmm.coordinate_converter());
    const double v = Mph2Mps(speed_bump.speed_limit_mph());
    speed_bumps.push_back({.id = mapping::ElementId(speed_bump.id()),
                           .polygon = std::move(polygon),
                           .speed_limit = v});
  }

  // Construct speed bump constraints.
  std::vector<ConstraintProto::SpeedRegionProto> speed_bump_constraints;
  speed_bump_constraints.reserve(speed_bumps.size());
  for (const auto& speed_bump : speed_bumps) {
    Vec2d start_point;
    Vec2d end_point;
    std::pair<double, double> min_max_s;
    const bool success = CalculateSpeedRegionRange(
        passage, speed_bump.polygon, &min_max_s, &start_point, &end_point);
    if (success) {
      ConstraintProto::SpeedRegionProto speed_bump_constraint;
      start_point.ToProto(speed_bump_constraint.mutable_start_point());
      end_point.ToProto(speed_bump_constraint.mutable_end_point());
      speed_bump_constraint.set_start_s(min_max_s.first);
      speed_bump_constraint.set_end_s(min_max_s.second);
      speed_bump_constraint.set_max_speed(speed_bump.speed_limit);
      speed_bump_constraint.set_min_speed(0.0);
      speed_bump_constraint.mutable_source()->mutable_speed_bump()->set_id(
          speed_bump.id.value());
      speed_bump_constraint.set_id(
          absl::StrFormat("speed_bump_%d", speed_bump.id));
      speed_bump_constraints.push_back(std::move(speed_bump_constraint));
    }
  }

  return speed_bump_constraints;
}

}  // namespace planner
}  // namespace qcraft
