#include "onboard/planner/util/lane_point_util.h"

// IWYU pragma: no_include <google/protobuf/repeated_ptr_field.h>

#include <algorithm>
#include <limits>
#include <memory>
#include <vector>

#include "absl/status/status.h"
#include "absl/strings/str_cat.h"

#include "onboard/container/strong_int.h"
#include "onboard/global/logging.h"
#include "onboard/lite/check.h"
#include "onboard/maps/map_or_die_macros.h"
#include "onboard/maps/maps_common.h"
#include "onboard/maps/semantic_map_util.h"
#include "onboard/maps/v2/semantic_map_object.h"
#include "onboard/math/geometry/segment2d.h"
#include "onboard/math/util.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/utils/source_location.h"

namespace qcraft::planner {
namespace {
void GetNeighboringVertices(double fraction,
                            const std::vector<double>& cumulative_lengths,
                            const std::vector<Vec2d>& lane_vertices,
                            const std::vector<Vec2d>& lane_point_theta,
                            Vec2d* v0, Vec2d* v1, Vec2d* theta0, Vec2d* theta1,
                            double* alpha) {
  const int num_vertices = lane_vertices.size();

  *alpha = 0.0;

  const double s = fraction * cumulative_lengths.back();

  QCHECK_GE(s, cumulative_lengths.front());
  QCHECK_LE(s, cumulative_lengths.back());

  int i = std::upper_bound(cumulative_lengths.begin(), cumulative_lengths.end(),
                           s) -
          cumulative_lengths.begin();
  QCHECK_GT(i, 0);
  if (i == num_vertices) --i;
  *v0 = lane_vertices[i - 1];
  *v1 = lane_vertices[i];
  *theta0 = lane_point_theta[i - 1];
  *theta1 = lane_point_theta[i];
  const double s0 = cumulative_lengths[i - 1];
  const double s1 = cumulative_lengths[i];
  *alpha = (s - s0) / (s1 - s0);
  QCHECK_GE(*alpha, 0.0);
  QCHECK_LE(*alpha, 1.0);
}

void FindNeighboringVerticesWithSuccessorLane(
    const PlannerSemanticMapManager& psmm, mapping::ElementId lane_id,
    mapping::ElementId /*succ_lane_id*/, double fraction, Vec2d* v0, Vec2d* v1,
    Vec2d* theta0, Vec2d* theta1, double* alpha) {
  QCHECK(v0 != nullptr);
  QCHECK(v1 != nullptr);
  QCHECK(alpha != nullptr);

  SMM_ASSIGN_LANE_OR_RETURN(lane_info, psmm, lane_id, void());
  const std::vector<Vec2d>& lane_vertices = lane_info.points_smooth;

  SMM_ASSIGN_LANE_OR_RETURN(succ_lane_info, psmm, lane_id, void());
  const std::vector<Vec2d>& succ_lane_vertices = succ_lane_info.points_smooth;

  const int num_vertices = lane_vertices.size();
  const auto* lane_info_ptr = psmm.FindLaneInfoOrNull(lane_id);
  if (lane_info_ptr == nullptr) return;
  const auto& cumulative_lengths = lane_info_ptr->cumulative_lengths;

  std::vector<Vec2d> lane_point_theta(num_vertices, Vec2d(0.0, 0.0));
  for (int i = 0; i < num_vertices - 1; ++i) {
    lane_point_theta[i] = Vec2d(lane_vertices[i + 1] - lane_vertices[i]);
  }
  lane_point_theta[num_vertices - 1] =
      Vec2d(succ_lane_vertices[1] - lane_vertices[num_vertices - 1]);

  GetNeighboringVertices(fraction, cumulative_lengths, lane_vertices,
                         lane_point_theta, v0, v1, theta0, theta1, alpha);
}

void FindNeighboringVertices(const PlannerSemanticMapManager& psmm,
                             mapping::ElementId lane_id, double fraction,
                             Vec2d* v0, Vec2d* v1, Vec2d* theta0, Vec2d* theta1,
                             double* alpha) {
  QCHECK(v0 != nullptr);
  QCHECK(v1 != nullptr);
  QCHECK(alpha != nullptr);

  SMM_ASSIGN_LANE_OR_RETURN(lane_info, psmm, lane_id, void());
  const std::vector<Vec2d>& lane_vertices = lane_info.points_smooth;

  const int num_vertices = lane_vertices.size();
  const auto* lane_info_ptr = psmm.FindLaneInfoOrNull(lane_id);
  if (lane_info_ptr == nullptr) return;
  const auto& cumulative_lengths = lane_info_ptr->cumulative_lengths;

  std::vector<Vec2d> lane_point_theta(num_vertices, Vec2d(0.0, 0.0));
  for (int i = 0; i + 1 < num_vertices; ++i) {
    lane_point_theta[i] = Vec2d(lane_vertices[i + 1] - lane_vertices[i]);
    if (i == num_vertices - 2) {
      lane_point_theta[i + 1] = lane_point_theta[i];
    }
  }

  GetNeighboringVertices(fraction, cumulative_lengths, lane_vertices,
                         lane_point_theta, v0, v1, theta0, theta1, alpha);
}
}  // namespace

Vec2d ComputeLanePointPos(const PlannerSemanticMapManager& psmm,
                          const mapping::LanePoint& lane_point) {
  Vec2d v0, v1, theta0, theta1;
  double alpha = 0.0;
  FindNeighboringVertices(psmm, lane_point.lane_id(), lane_point.fraction(),
                          &v0, &v1, &theta0, &theta1, &alpha);
  return Lerp(v0, v1, alpha);
}

Vec2d ComputeLanePointTangent(const PlannerSemanticMapManager& psmm,
                              const mapping::LanePoint& lane_point) {
  Vec2d v0, v1, theta0, theta1;
  double alpha = 0.0;
  FindNeighboringVertices(psmm, lane_point.lane_id(), lane_point.fraction(),
                          &v0, &v1, &theta0, &theta1, &alpha);
  return (v1 - v0).normalized();
}

double ComputeLanePointLerpTheta(const PlannerSemanticMapManager& psmm,
                                 const mapping::LanePoint& lane_point) {
  Vec2d v0, v1, theta0, theta1;
  double alpha = 0.0;
  FindNeighboringVertices(psmm, lane_point.lane_id(), lane_point.fraction(),
                          &v0, &v1, &theta0, &theta1, &alpha);
  return LerpAngle(theta0.FastAngle(), theta1.FastAngle(), alpha);
}

double ComputeLanePointLerpThetaWithSuccessorLane(
    const PlannerSemanticMapManager& psmm, mapping::ElementId succ_lane_id,
    const mapping::LanePoint& lane_point) {
  Vec2d v0, v1, theta0, theta1;
  double alpha = 0.0;
  FindNeighboringVerticesWithSuccessorLane(psmm, lane_point.lane_id(),
                                           succ_lane_id, lane_point.fraction(),
                                           &v0, &v1, &theta0, &theta1, &alpha);
  return LerpAngle(theta0.FastAngle(), theta1.FastAngle(), alpha);
}

absl::StatusOr<Vec2d> ComputeLanePointGlobalPose(
    const mapping::v2::SemanticMapManager& smm,
    const mapping::LanePoint& lane_point) {
  const auto lane = smm.FindLane(lane_point.lane_id());
  if (lane == nullptr) {
    return absl::NotFoundError(
        absl::StrCat("Failed to find lane id, ", lane_point.lane_id()));
  }
  const auto [idx, alpha] =
      mapping::ComputePosition(lane->segment_points(), lane_point.fraction());
  return Lerp(lane->segment_points()[idx], lane->segment_points()[idx + 1],
              alpha);
}

absl::StatusOr<Vec2d> ComputeLanePointGlobalHeading(
    const mapping::v2::SemanticMapManager& smm,
    const mapping::LanePoint& lane_point) {
  const auto lane = smm.FindLane(lane_point.lane_id());
  if (lane == nullptr) {
    return absl::NotFoundError(
        absl::StrCat("Failed to find lane id, ", lane_point.lane_id()));
  }
  const auto [idx, alpha] =
      mapping::ComputePosition(lane->segment_points(), lane_point.fraction());
  return (lane->segment_points()[idx + 1] - lane->segment_points()[idx])
      .normalized();
}

absl::StatusOr<std::optional<mapping::LaneBoundaryProto::Type>>
QueryLanePointBoundaryType(const PlannerSemanticMapManager& psmm,
                           const mapping::LanePoint& lane_point, bool left) {
  SMM_ASSIGN_LANE_OR_RETURN(
      lane_info, psmm, lane_point.lane_id(),
      absl::NotFoundError(absl::StrCat("Input lane point is not found: ",
                                       lane_point.DebugString())));

  const auto& lane_neighbor = left ? lane_info.proto->lane_neighbors_on_left()
                                   : lane_info.proto->lane_neighbors_on_right();
  if (!lane_neighbor.empty()) {
    const auto& first_lane_neighbor = lane_neighbor[0];
    if (first_lane_neighbor.has_lane_boundary_id()) {
      const auto lane_boundary = psmm.semantic_map_manager()->FindLaneBoundary(
          mapping::ElementId(first_lane_neighbor.lane_boundary_id()));
      if (lane_boundary != nullptr) {
        return std::make_optional(lane_boundary->ComputeBoundaryTypeByFraction(
            lane_point.fraction()));
      }
    }
  }

  // If lane boundary type can not be provided by semantic map directly,
  // calculate it.
  const Vec2d pos = ComputeLanePointPos(psmm, lane_point);
  const Vec2d tangent = ComputeLanePointTangent(psmm, lane_point);
  const double search_radius = kMaxHalfLaneWidth;  // m.
  const Vec2d endpoint =
      pos + tangent.Perp() * (left ? search_radius : -search_radius);

  const auto candidate_boundaries =
      psmm.GetLaneBoundariesInfoAtLevel(psmm.GetLevel(), pos, search_radius);
  if (candidate_boundaries.empty()) {
    return std::nullopt;
  }
  const Segment2d normal_line(pos, endpoint);
  double min_sqr_dis = std::numeric_limits<double>::infinity();
  int closest_index;
  const mapping::LaneBoundaryInfo* closest_boundary_info = nullptr;
  for (const auto* boundary : candidate_boundaries) {
    for (int k = 0; k + 1 < static_cast<int>(boundary->points_smooth.size());
         ++k) {
      const Vec2d& p0 = boundary->points_smooth[k];
      const Vec2d& p1 = boundary->points_smooth[k + 1];
      const Segment2d boundary_segment(p0, p1);
      Vec2d intersection;
      if (!normal_line.GetIntersect(boundary_segment, &intersection)) continue;
      const double sqr_dis = pos.DistanceSquareTo(intersection);
      if (sqr_dis < min_sqr_dis) {
        min_sqr_dis = sqr_dis;
        closest_index = k;
        closest_boundary_info = boundary;
      }
    }
  }

  if (closest_boundary_info != nullptr &&
      closest_boundary_info->lane_boundary != nullptr) {
    return std::make_optional(
        closest_boundary_info->lane_boundary->ComputeLaneBoundaryType(
            mapping::SegmentId(closest_index)));
  }
  return std::nullopt;
}

}  // namespace qcraft::planner
