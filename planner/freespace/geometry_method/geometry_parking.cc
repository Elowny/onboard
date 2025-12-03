#include "onboard/planner/freespace/geometry_method/geometry_parking.h"

#include <algorithm>
#include <cmath>
#include <deque>
#include <ostream>
#include <string>
#include <utility>

#include "absl/container/flat_hash_map.h"
#include "absl/status/status.h"
#include "absl/strings/str_format.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "gflags/gflags.h"
#include "glog/logging.h"

#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/math/geometry/segment2d.h"
#include "onboard/math/segment_matcher/segment_matcher_kdtree.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/common/discretized_path.h"
#include "onboard/planner/freespace/freespace_util.h"
#include "onboard/planner/freespace/geometry_method/geometry_method_defs.h"
#include "onboard/planner/freespace/geometry_method/geometry_method_util.h"
#include "onboard/planner/freespace/geometry_method/parallel_parking.h"
#include "onboard/planner/freespace/geometry_method/perpendicular_parking.h"
#include "onboard/planner/object/planner_object.h"
#include "onboard/planner/util/vehicle_geometry_util.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/vis/canvas/canvas.h"
#include "onboard/vis/common/color.h"

#include "offboard/vis/vantage/vantage_server/vantage_canvas_client.h"

DEFINE_bool(send_geometry_parking_path_to_canvas, false,
            "Whether to send geometry path to canvas.");
DEFINE_bool(send_geometry_parking_virtual_boundaries_to_canvas, false,
            "Whether to send virtual boundaries to canvas.");

namespace qcraft {
namespace planner {

absl::StatusOr<std::vector<DirectionalPath>> FindSinglePath(
    const FreespacePathFinderParamsProto& path_finder_params,
    const VehicleGeometryParamsProto& veh_geo_params,
    const VehicleDriveParamsProto& vehicle_drive_params,
    const VehicleOctagonModelParamsProto& vehicle_model_params,
    FreespaceTaskProto::TaskType task_type,
    FreespaceReplanReasonProto::ReplanReason replan_reason,
    const FreespaceMap& freespace_map,
    absl::Span<const SpacetimeObjectTrajectory* const> stalled_object_trajs,
    const mapping::ParkingSpotInfo* parking_spot_info, const PathPoint& start,
    const PathPoint& end, PathFinderDebugProto* debug_info) {
  if (task_type != FreespaceTaskProto::PARALLEL_PARKING &&
      task_type != FreespaceTaskProto::PERPENDICULAR_PARKING) {
    debug_info->set_geo_status(PathFinderDebugProto::TASK_TYPE_INVALID);
    return absl::InternalError(
        absl::StrFormat("Task type %s is invalid!",
                        FreespaceTaskProto::TaskType_Name(task_type)));
  }
  if (parking_spot_info == nullptr) {
    debug_info->set_geo_status(PathFinderDebugProto::NO_PARKING_SPOT);
    return absl::InternalError("No parking spot info!");
  }

  const auto start_time = absl::Now();
  // Construct boundary k-D tree and map.
  std::vector<std::pair<std::string, Segment2d>> named_segments;
  absl::flat_hash_map<std::string, const FreespaceObject*> objects_map;
  absl::flat_hash_map<std::string, const FreespaceBoundary*> boundaries_map;
  int boundary_index = 0;
  for (const auto& boundary : freespace_map.boundaries) {
    QCHECK_GE(boundary.points.size(), 2);
    for (int i = 0; i + 1 < boundary.points.size(); ++i) {
      std::string id = "b" + std::to_string(boundary_index);
      const Segment2d seg(boundary.points[i], boundary.points[i + 1]);
      named_segments.emplace_back(id, seg);
      boundaries_map.emplace(id, &boundary);
      boundary_index++;
    }
  }

  std::vector<std::pair<std::string, FreespaceObject>> stationary_objects;
  stationary_objects.reserve(stalled_object_trajs.size());
  for (const auto& traj_ptr : stalled_object_trajs) {
    std::string id = std::string(traj_ptr->object_id());
    const auto& object_proto = traj_ptr->planner_object().object_proto();
    FreespaceObject obj = {
        .contour = traj_ptr->contour(),
        .height = object_proto.max_z() - object_proto.ground_z()};
    stationary_objects.emplace_back(std::move(id), std::move(obj));
  }
  for (const auto& named_obj : stationary_objects) {
    named_segments.push_back(std::make_pair(
        named_obj.first, Segment2d(Vec2d(named_obj.second.contour.min_x(),
                                         named_obj.second.contour.min_y()),
                                   Vec2d(named_obj.second.contour.max_x(),
                                         named_obj.second.contour.max_y()))));
    objects_map.emplace(named_obj.first, &named_obj.second);
  }
  SegmentMatcherKdtree segments_kd_tree(named_segments);
  VLOG(2) << "Construct k-D tree time(ms): "
          << absl::ToDoubleMilliseconds(absl::Now() - start_time);

  // Construct start and goal.
  GeometryMethodPoint start_pose = {
      .pos = Vec2d(start.x(), start.y()),
      .theta = NormalizeAngle(start.theta()),
      .tangent = Vec2d::FastUnitFromAngle(start.theta())};
  GeometryMethodPoint goal_pose = {
      .pos = Vec2d(end.x(), end.y()),
      .theta = NormalizeAngle(end.theta()),
      .tangent = Vec2d::FastUnitFromAngle(end.theta())};
  const double max_kappa =
      path_finder_params.geometry_method_params().kappa_slack_ratio() *
      ComputeCenterMaxCurvature(veh_geo_params, vehicle_drive_params);
  if (replan_reason != FreespaceReplanReasonProto::NONE &&
      !CheckPoseValidityWithKDTree(veh_geo_params, path_finder_params,
                                   vehicle_model_params, segments_kd_tree,
                                   objects_map, boundaries_map, start_pose.pos,
                                   start_pose.theta, start_pose.tangent)) {
    debug_info->set_geo_status(PathFinderDebugProto::START_INVALID);
    return absl::InternalError(
        absl::StrFormat("Start pose %s theta: %f invalid!",
                        start_pose.pos.DebugString(), start_pose.theta));
  }
  if (!CheckPoseValidityWithKDTree(veh_geo_params, path_finder_params,
                                   vehicle_model_params, segments_kd_tree,
                                   objects_map, boundaries_map, goal_pose.pos,
                                   goal_pose.theta, goal_pose.tangent)) {
    debug_info->set_geo_status(PathFinderDebugProto::GOAL_INVALID);
    return absl::InternalError(
        absl::StrFormat("Goal pose %s theta: %f invalid!",
                        goal_pose.pos.DebugString(), goal_pose.theta));
  }

  // TODO(Zhuang): Construct virtual boundaries.
  const std::vector<Segment2d> virtual_boundaries;
  if (FLAGS_send_geometry_parking_virtual_boundaries_to_canvas) {
    vis::Canvas& canvas = vis::vantage::GetCanvasClient()->GetCanvas(
        "freespace/geometry/virtual_boundaries");
    for (const auto& boundary : virtual_boundaries) {
      canvas.DrawLine(Vec3d(boundary.start()), Vec3d(boundary.end()),
                      vis::Color::kRed);
    }
  }

  // Compute path.
  absl::StatusOr<LineCirclePath> geometry_path_status;
  if (task_type == FreespaceTaskProto::PARALLEL_PARKING) {
    if (replan_reason ==
        FreespaceReplanReasonProto::PARALLEL_PARKING_FINAL_PATH) {
      geometry_path_status = FindParallelParkingReplanPath(
          veh_geo_params, max_kappa, path_finder_params, vehicle_model_params,
          segments_kd_tree, objects_map, boundaries_map, virtual_boundaries,
          replan_reason, start_pose, goal_pose);
    } else {
      geometry_path_status = FindParallelParkingPath(
          veh_geo_params, max_kappa, path_finder_params, vehicle_model_params,
          segments_kd_tree, objects_map, boundaries_map, virtual_boundaries,
          start_pose, goal_pose);
    }
  } else {
    geometry_path_status = FindPerpendicularParkingPath(
        veh_geo_params, max_kappa, path_finder_params, vehicle_model_params,
        segments_kd_tree, objects_map, boundaries_map, virtual_boundaries,
        start_pose, goal_pose);
  }

  if (!geometry_path_status.ok()) {
    VLOG(1) << "Total time(ms): "
            << absl::ToDoubleMilliseconds(absl::Now() - start_time);
    debug_info->set_geo_status(PathFinderDebugProto::SEARCH_FAIL);
    return absl::InternalError("Find geometry parking path fails.");
  }
  VLOG(1) << "Total time(ms): "
          << absl::ToDoubleMilliseconds(absl::Now() - start_time);
  const auto& geometry_path = geometry_path_status.value();

  if (FLAGS_send_geometry_parking_path_to_canvas) {
    vis::Canvas& canvas =
        vis::vantage::GetCanvasClient()->GetCanvas("freespace/geometry/path");
    SendLineCirclePathToCanvas(&canvas, veh_geo_params, vehicle_model_params,
                               geometry_path, /*step=*/0.2);
  }

  // Convert geometry path to discrete points.
  std::vector<std::vector<PathPoint>> paths;
  std::deque<bool> gears;
  constexpr double kStep = 0.3;  // m.
  double cur_s = 0.0;
  auto cur_start = start_pose;
  for (int i = 0; i < geometry_path.lengths.size(); ++i) {
    std::vector<PathPoint> path;
    const bool forward = geometry_path.lengths[i] > 0.0;
    double kappa = geometry_path.types[i] == GeometryPathType::LEFT
                       ? geometry_path.kappas[i]
                       : -geometry_path.kappas[i];
    kappa = forward ? kappa : -kappa;
    for (double s = 0.0; s < std::abs(geometry_path.lengths[i]) + kStep;
         s += kStep) {
      const double len = std::min(s, std::abs(geometry_path.lengths[i]));
      const auto path_point = ExtendPathByConstantKappa(
          cur_start, geometry_path.kappas[i],
          std::copysign(len, geometry_path.lengths[i]), geometry_path.types[i]);
      PathPoint pt;
      pt.set_x(path_point.pos.x());
      pt.set_y(path_point.pos.y());
      pt.set_z(0.0);
      pt.set_theta(forward ? path_point.theta
                           : NormalizeAngle(path_point.theta + M_PI));
      pt.set_kappa(kappa);
      pt.set_s(cur_s + len);
      path.push_back(std::move(pt));
    }

    // Check if append a new path.
    if (i == 0 ||
        geometry_path.lengths[i] * geometry_path.lengths[i - 1] < 0.0) {
      paths.push_back(std::move(path));
      gears.push_back(forward);
    } else {
      paths.back().insert(paths.back().end(), path.begin() + 1, path.end());
    }

    cur_s += std::abs(geometry_path.lengths[i]);
    cur_start = geometry_path.ends[i];
  }
  // Make sure the path s starts from zero.
  for (int i = 0; i < paths.size(); ++i) {
    const double start_s = paths[i][0].s();
    for (int j = 0; j < paths[i].size(); ++j) {
      paths[i][j].set_s(paths[i][j].s() - start_s);
    }
  }
  // Fill results.
  std::vector<DirectionalPath> res;
  res.reserve(paths.size());
  for (int i = 0; i < paths.size(); ++i) {
    res.push_back({DiscretizedPath(std::move(paths[i])), gears[i]});
  }

  debug_info->set_geo_status(PathFinderDebugProto::SUCCESS);
  return res;
}

}  // namespace planner
}  // namespace qcraft
