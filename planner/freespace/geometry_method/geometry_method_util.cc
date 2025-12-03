#include "onboard/planner/freespace/geometry_method/geometry_method_util.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "onboard/lite/check.h"
#include "onboard/lite/logging.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/common/geometry_path_util.h"
#include "onboard/planner/common/vehicle_octagon_model.h"
#include "onboard/planner/freespace/freespace_util.h"
#include "onboard/planner/freespace/geometry_method/geometry_path_collision_checker.h"
#include "onboard/vis/common/color.h"

namespace qcraft {
namespace planner {

GeometryMethodPoint ExtendPathByConstantKappa(const GeometryMethodPoint& start,
                                              double kappa, double length,
                                              GeometryPathType type) {
  switch (type) {
    case GeometryPathType::STRAIGHT:
      return {.pos = start.pos + length * start.tangent,
              .theta = start.theta,
              .tangent = start.tangent};
    case GeometryPathType::RIGHT:
      kappa = -kappa;
    case GeometryPathType::LEFT:
      const double radius = 1.0 / kappa;
      const double delta_theta = length * kappa;
      const Vec2d tangent_delta = Vec2d::FastUnitFromAngle(delta_theta);
      const Vec2d delta_pos =
          radius * Vec2d(tangent_delta.y(), 1.0 - tangent_delta.x());

      return {.pos = start.pos +
                     delta_pos.Rotate(start.tangent.x(), start.tangent.y()),
              .theta = NormalizeAngle(start.theta + delta_theta),
              .tangent =
                  start.tangent.Rotate(tangent_delta.x(), tangent_delta.y())};
  }
  return {};
}

std::vector<GeometryMethodPoint> ConvertLineCirclePathToDiscretePoints(
    const LineCirclePath& path, double step) {
  std::vector<GeometryMethodPoint> result;
  result.push_back(path.start);
  for (int i = 0; i < path.types.size(); ++i) {
    const auto cur_start = result.back();
    for (double s = step; s < std::abs(path.lengths[i]) + step; s += step) {
      const double len = std::min(s, std::abs(path.lengths[i]));
      result.push_back(ExtendPathByConstantKappa(
          cur_start, path.kappas[i], std::copysign(len, path.lengths[i]),
          path.types[i]));
    }
  }
  return result;
}

void ConnectPaths(const std::vector<LineCirclePath>& paths,
                  LineCirclePath* result) {
  QCHECK(!paths.empty());
  result->start = paths.front().start;
  result->types.clear();
  result->lengths.clear();
  result->kappas.clear();
  result->ends.clear();
  for (const auto& path : paths) {
    for (int i = 0; i < path.types.size(); ++i) {
      result->types.push_back(path.types[i]);
      result->lengths.push_back(path.lengths[i]);
      result->kappas.push_back(path.kappas[i]);
      result->ends.push_back(path.ends[i]);
    }
  }
}

void ReversePath(LineCirclePath* path) {
  std::reverse(path->types.begin(), path->types.end());
  std::reverse(path->lengths.begin(), path->lengths.end());
  std::reverse(path->kappas.begin(), path->kappas.end());
  std::reverse(path->ends.begin(), path->ends.end());

  path->ends.push_back(path->start);
  path->start = path->ends.front();
  path->ends.erase(path->ends.begin());

  for (auto& length : path->lengths) {
    length = -length;
  }
}

int ComputeGearChangeTimes(const LineCirclePath& path) {
  int res = 0;
  if (path.lengths.size() <= 1) {
    return res;
  }
  double prev_len = path.lengths[0];
  for (int i = 1; i < path.lengths.size(); ++i) {
    const double cur_len = path.lengths[i];
    if (prev_len * cur_len < 0.0) {
      res += 1;
    }
    prev_len = cur_len;
  }
  return res;
}

bool CheckPathValidityWithKDTreeAndVirtualBoundaries(
    const VehicleGeometryParamsProto& veh_geo_params,
    const FreespacePathFinderParamsProto& path_finder_params,
    const VehicleOctagonModelParamsProto& vehicle_model_params,
    const SegmentMatcherKdtree& segments_kd_tree,
    const absl::flat_hash_map<std::string, const FreespaceObject*>& objects_map,
    const absl::flat_hash_map<std::string, const FreespaceBoundary*>&
        boundaries_map,
    const std::vector<Segment2d>& virtual_boundaries,
    const LineCirclePath& path) {
  std::vector<std::unique_ptr<GeometryPathCollisonChecker>> path_checkers;
  path_checkers.reserve(path.types.size());
  auto cur_start = path.start;
  for (int i = 0; i < path.types.size(); ++i) {
    switch (path.types[i]) {
      case GeometryPathType::STRAIGHT:
        path_checkers.push_back(std::make_unique<LinePathCollisionChecker>(
            veh_geo_params, vehicle_model_params, cur_start, path.ends[i],
            path.types[i], path.lengths[i], path.kappas[i]));
        break;
      case GeometryPathType::LEFT:
      case GeometryPathType::RIGHT:
        path_checkers.push_back(std::make_unique<CirclePathCollisionChecker>(
            veh_geo_params, vehicle_model_params, cur_start, path.ends[i],
            path.types[i], path.lengths[i], path.kappas[i],
            /*use_fast_overlap=*/true));
    }
    cur_start = path.ends[i];
  }

  for (const auto& boundary : virtual_boundaries) {
    for (const auto& checker : path_checkers) {
      if (checker->HasOverlapWithBuffer(boundary, /*lateral_buffer=*/0.0,
                                        /*longitudinal_buffer=*/0.0,
                                        /*consider_mirror=*/false)) {
        return false;
      }
    }
  }

  for (const auto& checker : path_checkers) {
    const Vec2d center = 0.5 * (checker->start().pos + checker->end().pos);
    const double search_radius =
        std::abs(checker->length()) + veh_geo_params.length();
    const auto nearby_named_segments =
        segments_kd_tree.GetNamedSegmentsInRadius(center.x(), center.y(),
                                                  search_radius);
    for (const auto& named_segment : nearby_named_segments) {
      const auto iter = objects_map.find(named_segment.second);
      if (iter != objects_map.end()) {
        const bool consider_mirror =
            vehicle_model_params.consider_mirror() &&
            iter->second->height + path_finder_params.mirror_height_buffer() >
                vehicle_model_params.mirror_height();
        if (checker->HasOverlapWithBuffer(
                iter->second->contour,
                path_finder_params.object_lateral_buffer(),
                path_finder_params.object_longitudinal_buffer(),
                consider_mirror)) {
          return false;
        }
      } else {
        const auto boundary_iter = boundaries_map.find(named_segment.second);
        QCHECK(boundary_iter != boundaries_map.end());
        const auto buffers = GetVehicleBufferForBoundary(
            path_finder_params, *boundary_iter->second);
        const bool consider_mirror =
            vehicle_model_params.consider_mirror() &&
            boundary_iter->second->height +
                    path_finder_params.mirror_height_buffer() >
                vehicle_model_params.mirror_height();
        if (checker->HasOverlapWithBuffer(*named_segment.first, buffers.first,
                                          buffers.second, consider_mirror)) {
          return false;
        }
      }
    }
  }
  return true;
}

bool CheckPoseValidityWithKDTreeAndVirtualBoundaries(
    const VehicleGeometryParamsProto& veh_geo_params,
    const FreespacePathFinderParamsProto& path_finder_params,
    const VehicleOctagonModelParamsProto& vehicle_model_params,
    const SegmentMatcherKdtree& segments_kd_tree,
    const absl::flat_hash_map<std::string, const FreespaceObject*>& objects_map,
    const absl::flat_hash_map<std::string, const FreespaceBoundary*>&
        boundaries_map,
    const std::vector<Segment2d>& virtual_boundaries,
    const GeometryMethodPoint& pose) {
  const double offset = 0.5 * (veh_geo_params.front_edge_to_center() -
                               veh_geo_params.back_edge_to_center());
  const auto center = pose.pos + offset * pose.tangent;
  const VehicleOctagonModel av_octagon(
      0.5 * veh_geo_params.length(), 0.5 * veh_geo_params.width(), center,
      pose.theta, pose.tangent, vehicle_model_params.front_corner_side_length(),
      vehicle_model_params.rear_corner_side_length());
  for (const auto& boundary : virtual_boundaries) {
    if (av_octagon.HasOverlap(boundary)) return false;
  }
  return CheckPoseValidityWithKDTree(veh_geo_params, path_finder_params,
                                     vehicle_model_params, segments_kd_tree,
                                     objects_map, boundaries_map, pose.pos,
                                     pose.theta, pose.tangent);
}

std::optional<double> GetFirstOverlapDistanceWithKDTreeAndVirtualBoundaries(
    const VehicleGeometryParamsProto& veh_geo_params,
    const FreespacePathFinderParamsProto& path_finder_params,
    const VehicleOctagonModelParamsProto& vehicle_model_params,
    const SegmentMatcherKdtree& segments_kd_tree,
    const absl::flat_hash_map<std::string, const FreespaceObject*>& objects_map,
    const absl::flat_hash_map<std::string, const FreespaceBoundary*>&
        boundaries_map,
    const std::vector<Segment2d>& virtual_boundaries,
    const GeometryMethodPoint& pose, double kappa, bool forward,
    double expected_max_distance) {
  const double offset = 0.5 * (veh_geo_params.front_edge_to_center() -
                               veh_geo_params.back_edge_to_center());
  const auto center = pose.pos + offset * pose.tangent;
  const VehicleOctagonModel av_octagon(
      0.5 * veh_geo_params.length(), 0.5 * veh_geo_params.width(), center,
      pose.theta, pose.tangent, vehicle_model_params.front_corner_side_length(),
      vehicle_model_params.rear_corner_side_length());
  const Polygon2d ego_polygon{av_octagon.GetCornersWithBufferCounterClockwise(
      /*lat_buffer=*/0.0, /*lon_buffer=*/0.0)};
  const Polygon2d ego_polygon_with_object_buffer{
      av_octagon.GetCornersWithBufferCounterClockwise(
          path_finder_params.object_lateral_buffer(),
          path_finder_params.object_longitudinal_buffer())};
  const double mirror_half_length = vehicle_model_params.mirror_radius();
  const double mirror_half_width = vehicle_model_params.mirror_offset_y() +
                                   vehicle_model_params.mirror_radius();
  const Box2d mirror_box_with_object_buffer(
      mirror_half_length + path_finder_params.object_lateral_buffer(),
      mirror_half_width + path_finder_params.object_lateral_buffer(),
      pose.pos + vehicle_model_params.mirror_offset_x() * pose.tangent,
      pose.theta, pose.tangent);

  std::optional<double> res = std::nullopt;
  for (const auto& boundary : virtual_boundaries) {
    if (const auto dist = ComputeFirstOverlapDistanceWithSegment(
            pose.pos, pose.tangent, ego_polygon, forward, kappa, boundary);
        dist.has_value()) {
      res = res.has_value() ? std::min(*res, *dist) : *dist;
    }
  }

  const double search_radius = expected_max_distance + veh_geo_params.length();
  const auto nearby_named_segments = segments_kd_tree.GetNamedSegmentsInRadius(
      center.x(), center.y(), search_radius);
  for (const auto& named_segment : nearby_named_segments) {
    const auto iter = objects_map.find(named_segment.second);
    if (iter != objects_map.end()) {
      if (const auto dist = ComputeFirstOverlapDistanceWithPolygon(
              pose.pos, pose.tangent, ego_polygon_with_object_buffer, forward,
              kappa, iter->second->contour);
          dist.has_value()) {
        res = res.has_value() ? std::min(*res, *dist) : *dist;
      }
      const bool consider_mirror =
          vehicle_model_params.consider_mirror() &&
          iter->second->height + path_finder_params.mirror_height_buffer() >
              vehicle_model_params.mirror_height();
      if (consider_mirror) {
        if (const auto dist = ComputeFirstOverlapDistanceWithPolygon(
                pose.pos, mirror_box_with_object_buffer, forward, kappa,
                iter->second->contour);
            dist.has_value()) {
          res = res.has_value() ? std::min(*res, *dist) : *dist;
        }
      }
    } else {
      const auto boundary_iter = boundaries_map.find(named_segment.second);
      QCHECK(boundary_iter != boundaries_map.end());
      const auto buffers = GetVehicleBufferForBoundary(path_finder_params,
                                                       *boundary_iter->second);
      const Polygon2d ego_polygon_with_buffer{
          av_octagon.GetCornersWithBufferCounterClockwise(buffers.first,
                                                          buffers.second)};
      if (const auto dist = ComputeFirstOverlapDistanceWithSegment(
              pose.pos, pose.tangent, ego_polygon_with_buffer, forward, kappa,
              *named_segment.first);
          dist.has_value()) {
        res = res.has_value() ? std::min(*res, *dist) : *dist;
      }
      const bool consider_mirror =
          vehicle_model_params.consider_mirror() &&
          boundary_iter->second->height +
                  path_finder_params.mirror_height_buffer() >
              vehicle_model_params.mirror_height();
      if (consider_mirror) {
        const Box2d mirror_box_with_buffer(
            mirror_half_length + buffers.first,
            mirror_half_width + buffers.first,
            pose.pos + vehicle_model_params.mirror_offset_x() * pose.tangent,
            pose.theta, pose.tangent);
        if (const auto dist = ComputeFirstOverlapDistanceWithSegment(
                pose.pos, mirror_box_with_buffer, forward, kappa,
                *named_segment.first);
            dist.has_value()) {
          res = res.has_value() ? std::min(*res, *dist) : *dist;
        }
      }
    }
  }

  return res;
}

void SendLineCirclePathToCanvas(
    vis::Canvas* canvas, const VehicleGeometryParamsProto& veh_geo_params,
    const VehicleOctagonModelParamsProto& vehicle_model_params,
    const LineCirclePath& path, double step) {
  const double offset = 0.5 * (veh_geo_params.front_edge_to_center() -
                               veh_geo_params.back_edge_to_center());
  const Vec2d size(veh_geo_params.length(), veh_geo_params.width());
  for (int i = 0; i < path.types.size(); ++i) {
    LineCirclePath cur_path = {
        .start = (i == 0 ? path.start : path.ends[i - 1]),
        .types = {path.types[i]},
        .lengths = {path.lengths[i]},
        .kappas = {path.kappas[i]},
        .ends = {path.ends[i]}};
    const auto path_points =
        ConvertLineCirclePathToDiscretePoints(cur_path, step);
    vis::Color color;
    switch (path.types[i]) {
      case GeometryPathType::STRAIGHT:
        color = vis::Color::kGreen;
        break;
      case GeometryPathType::LEFT:
        color = vis::Color::kRed;
        break;
      case GeometryPathType::RIGHT:
        color = vis::Color::kYellow;
        break;
    }
    for (int j = 0; j + 1 < path_points.size(); ++j) {
      canvas->DrawLine(Vec3d(path_points[j].pos), Vec3d(path_points[j + 1].pos),
                       color);
    }
    for (const auto& pt : path_points) {
      const auto center = pt.pos + offset * pt.tangent;
      const VehicleOctagonModel av_octagon(
          0.5 * veh_geo_params.length(), 0.5 * veh_geo_params.width(), center,
          pt.theta, pt.tangent, vehicle_model_params.front_corner_side_length(),
          vehicle_model_params.rear_corner_side_length());
      for (const auto& line : av_octagon.line_segments()) {
        canvas->DrawLine(Vec3d(line.start()), Vec3d(line.end()),
                         vis::Color::kWhite);
      }
    }
  }
}

}  // namespace planner
}  // namespace qcraft
