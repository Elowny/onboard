#ifndef ONBOARD_PLANNER_FREESPACE_GEOMETRY_METHOD_UTIL_H
#define ONBOARD_PLANNER_FREESPACE_GEOMETRY_METHOD_UTIL_H

#include <optional>
#include <string>
#include <vector>

#include "absl/container/flat_hash_map.h"

#include "onboard/math/geometry/segment2d.h"
#include "onboard/math/segment_matcher/segment_matcher_kdtree.h"
#include "onboard/planner/freespace/freespace_planner_defs.h"
#include "onboard/planner/freespace/geometry_method/geometry_method_defs.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/proto/vehicle.pb.h"
#include "onboard/vis/canvas/canvas.h"

namespace qcraft {
namespace planner {

GeometryMethodPoint ExtendPathByConstantKappa(const GeometryMethodPoint& start,
                                              double kappa, double length,
                                              GeometryPathType type);

std::vector<GeometryMethodPoint> ConvertLineCirclePathToDiscretePoints(
    const LineCirclePath& path, double step);

void ConnectPaths(const std::vector<LineCirclePath>& paths,
                  LineCirclePath* result);

void ReversePath(LineCirclePath* path);

int ComputeGearChangeTimes(const LineCirclePath& path);

bool CheckPathValidityWithKDTreeAndVirtualBoundaries(
    const VehicleGeometryParamsProto& veh_geo_params,
    const FreespacePathFinderParamsProto& path_finder_params,
    const VehicleOctagonModelParamsProto& vehicle_model_params,
    const SegmentMatcherKdtree& segments_kd_tree,
    const absl::flat_hash_map<std::string, const FreespaceObject*>& objects_map,
    const absl::flat_hash_map<std::string, const FreespaceBoundary*>&
        boundaries_map,
    const std::vector<Segment2d>& virtual_boundaries,
    const LineCirclePath& path);

bool CheckPoseValidityWithKDTreeAndVirtualBoundaries(
    const VehicleGeometryParamsProto& veh_geo_params,
    const FreespacePathFinderParamsProto& path_finder_params,
    const VehicleOctagonModelParamsProto& vehicle_model_params,
    const SegmentMatcherKdtree& segments_kd_tree,
    const absl::flat_hash_map<std::string, const FreespaceObject*>& objects_map,
    const absl::flat_hash_map<std::string, const FreespaceBoundary*>&
        boundaries_map,
    const std::vector<Segment2d>& virtual_boundaries,
    const GeometryMethodPoint& pose);

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
    double expected_max_distance);

void SendLineCirclePathToCanvas(
    vis::Canvas* canvas, const VehicleGeometryParamsProto& veh_geo_params,
    const VehicleOctagonModelParamsProto& vehicle_model_params,
    const LineCirclePath& path, double step);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_FREESPACE_GEOMETRY_METHOD_UTIL_H
