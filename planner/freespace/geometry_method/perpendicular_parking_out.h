#ifndef ONBOARD_PLANNER_FREESPACE_GEOMETRY_METHOD_PERPENDICULAR_PARKING_OUT_H
#define ONBOARD_PLANNER_FREESPACE_GEOMETRY_METHOD_PERPENDICULAR_PARKING_OUT_H

#include <string>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/status/statusor.h"

#include "onboard/math/geometry/box2d.h"
#include "onboard/math/geometry/segment2d.h"
#include "onboard/math/segment_matcher/segment_matcher_kdtree.h"
#include "onboard/planner/freespace/freespace_planner_defs.h"
#include "onboard/planner/freespace/geometry_method/geometry_method_defs.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/proto/parking_spot_finder.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {
namespace planner {

absl::StatusOr<LineCirclePath> FindPerpendicularStraightParkingOutPath(
    const VehicleGeometryParamsProto& veh_geo_params, double max_kappa,
    const FreespacePathFinderParamsProto& path_finder_params,
    const VehicleOctagonModelParamsProto& vehicle_model_params,
    const SegmentMatcherKdtree& segments_kd_tree,
    const absl::flat_hash_map<std::string, const FreespaceObject*>& objects_map,
    const absl::flat_hash_map<std::string, const FreespaceBoundary*>&
        boundaries_map,
    const std::vector<Segment2d>& virtual_boundaries,
    const GeometryMethodPoint& start, double park_out_distance,
    ParkingOutDirection parking_out_direction);

absl::StatusOr<LineCirclePath> FindPerpendicularTurningParkingOutPath(
    const VehicleGeometryParamsProto& veh_geo_params, double max_kappa,
    const FreespacePathFinderParamsProto& path_finder_params,
    const VehicleOctagonModelParamsProto& vehicle_model_params,
    const SegmentMatcherKdtree& segments_kd_tree,
    const absl::flat_hash_map<std::string, const FreespaceObject*>& objects_map,
    const absl::flat_hash_map<std::string, const FreespaceBoundary*>&
        boundaries_map,
    const std::vector<Segment2d>& virtual_boundaries,
    const GeometryMethodPoint& start, const Box2d& target_box,
    ParkingOutDirection parking_out_direction, bool use_fast_method = false);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_FREESPACE_GEOMETRY_METHOD_PERPENDICULAR_PARKING_OUT_H
