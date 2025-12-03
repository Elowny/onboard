#ifndef ONBOARD_PLANNER_FREESPACE_FREESPACE_UTIL_H_
#define ONBOARD_PLANNER_FREESPACE_FREESPACE_UTIL_H_

#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/time/time.h"

#include "onboard/maps/maps_common.h"
#include "onboard/math/coordinate_converter.h"
#include "onboard/math/segment_matcher/segment_matcher_kdtree.h"
#include "onboard/math/vec.h"
#include "onboard/planner/common/discretized_path.h"
#include "onboard/planner/common/vehicle_shape.h"
#include "onboard/planner/freespace/directional_path.h"
#include "onboard/planner/freespace/freespace_planner_defs.h"
#include "onboard/planner/freespace/proto/freespace_planner.pb.h"
#include "onboard/planner/object/planner_object_manager.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/planner/speed/st_boundary_with_decision.h"
#include "onboard/proto/chassis.pb.h"
#include "onboard/proto/trajectory.pb.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft::planner {

struct MinStopSInfo {
  std::optional<double> min_stop_s;
  std::optional<double> min_stationary_object_stop_s;
  std::optional<std::string> nearest_stationary_object_id;
};

MinStopSInfo ComputeMinStopSInfo(
    const std::vector<StBoundaryWithDecision>& st_boundaries_wd);

bool HasGoal(const GlobalGoalReferenceProto& global_goal_ref);

TrajectoryProto CreateFreespaceTrajectoryProto(
    absl::Time plan_time,
    const std::vector<ApolloTrajectoryPointProto>& planned_trajectory,
    const std::vector<ApolloTrajectoryPointProto>& past_points,
    const Chassis::GearPosition& gear_position,
    const DrivingStateProto& driving_state, bool low_speed_freespace,
    bool enable_stationary_steering,
    const DirectionalPath& smooth_directional_path, double stop_s,
    const std::vector<PathPoint>& past_directional_path_points);

// Returns a pair of [lateral_buffer, longitudinal_buffer].
std::pair<double, double> GetVehicleBufferForBoundary(
    const FreespacePathFinderParamsProto& path_finder_params,
    const FreespaceBoundary& boundary);

// Adjust goal if it's invalid. For map boundaries, we only consider curb and
// yellow solid line.
// @param adjust_dir: F, B, L, R stands for forward, backward, leftward,
// rightward. Returns original goal if adjustment fails.
PathPoint MaybeAdjustGoal(
    const FreespacePathFinderParamsProto& path_finder_params,
    const VehicleGeometryParamsProto& veh_geo_params,
    const VehicleOctagonModelParamsProto& vehicle_model_params,
    const PlannerSemanticMapManager* psmm,
    const std::vector<FreespaceObject>& stationary_objects,
    const PathPoint& goal, bool is_parking_task, char adjust_dir,
    double max_adjust_dist, double adjust_step, double planner_buffer);

PathPoint MaybeAdjustGoal(
    const FreespacePathFinderParamsProto& path_finder_params,
    const VehicleGeometryParamsProto& veh_geo_params,
    const VehicleOctagonModelParamsProto& vehicle_model_params,
    const PlannerSemanticMapManager* psmm,
    const PlannerObjectManager* object_manager,
    const absl::flat_hash_set<std::string>& stalled_objects,
    const PathPoint& goal, bool is_parking_task, char adjust_dir,
    double max_adjust_dist, double adjust_step, double planner_buffer);

template <typename T>
bool MirrorHasOverlapWithBuffer(
    const VehicleGeometryParamsProto& veh_geo_params,
    const FreespacePathFinderParamsProto& path_finder_params,
    const VehicleOctagonModelParamsProto& vehicle_model_params,
    const T& object_or_boundary, double height, double buffer,
    const Vec2d& left_mirror, const Vec2d& right_mirror);

bool CheckPoseValidityWithKDTree(
    const VehicleGeometryParamsProto& veh_geo_params,
    const FreespacePathFinderParamsProto& path_finder_params,
    const VehicleOctagonModelParamsProto& vehicle_model_params,
    const SegmentMatcherKdtree& segments_kd_tree,
    const absl::flat_hash_map<std::string, const FreespaceObject*>& objects_map,
    const absl::flat_hash_map<std::string, const FreespaceBoundary*>&
        boundaries_map,
    const Vec2d& pos, double heading, const Vec2d& tangent);

double GetPoseDistanceToObstacle(
    const VehicleGeometryParamsProto& veh_geo_params,
    const FreespacePathFinderParamsProto& path_finder_params,
    const VehicleOctagonModelParamsProto& vehicle_model_params,
    const SegmentMatcherKdtree& segments_kd_tree,
    const absl::flat_hash_map<std::string, const FreespaceObject*>& objects_map,
    const absl::flat_hash_map<std::string, const FreespaceBoundary*>&
        boundaries_map,
    const Vec2d& pos, double heading, const Vec2d& tangent);

PathPoint RestoreSmoothGoalFromGlobalRef(
    const GlobalGoalReferenceProto& global_goal_ref,
    const CoordinateConverter* nullable_coordinate_converter,
    const mapping::ParkingSpotInfo* nullable_parking_spot_info);

std::vector<VehicleShapeBasePtr> BuildFreespaceAvShapes(
    const VehicleGeometryParamsProto& vehicle_geom,
    const DiscretizedPath& path_points, bool forward,
    const VehicleOctagonModelParamsProto& vehicle_model_params);

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_FREESPACE_FREESPACE_UTIL_H_
