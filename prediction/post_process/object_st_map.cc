#include "onboard/prediction/post_process/object_st_map.h"

// IWYU pragma: no_include <google/protobuf/repeated_ptr_field.h>
// IWYU pragma: no_include <stddef.h>
// IWYU pragma: no_include <ext/alloc_traits.h>

#include <algorithm>
#include <cmath>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "absl/strings/str_join.h"
#include "glog/logging.h"

#include "onboard/async/parallel_for.h"
#include "onboard/container/strong_int.h"
#include "onboard/global/trace.h"
#include "onboard/maps/maps_common.h"
#include "onboard/maps/proto/semantic_map.pb.h"
#include "onboard/math/geometry/segment2d.h"
#include "onboard/math/vec.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/speed/overlap_info.h"
#include "onboard/planner/speed/proto/speed_finder.pb.h"
#include "onboard/planner/speed/st_point.h"
#include "onboard/planner/speed/vt_point.h"
#include "onboard/proto/perception/fusion/object.pb.h"
#include "onboard/proto/prediction.pb.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/utils/status_macros.h"

namespace qcraft::prediction {
namespace {

// Reuse planner's stboundary related classes.
using StBoundaryRef = planner::StBoundaryRef;  // unique_ptr.
using StBoundary = planner::StBoundary;
using StBoundaryPoints = planner::StBoundaryPoints;

// If the path is behind the half plane by this radius, still consider the half
// plane can be mapped to the path.
constexpr double kRedTLHalfPlaneSearchRadius = 2.0;  // m.

StBoundaryProto::ObjectType ToStBoundaryObjectType(ObjectType type) {
  switch (type) {
    case ObjectType::OT_VEHICLE:
    case ObjectType::OT_LARGE_VEHICLE:
    case ObjectType::OT_UNKNOWN_MOVABLE:
      return StBoundaryProto::VEHICLE;
    case ObjectType::OT_MOTORCYCLIST:
    case ObjectType::OT_TRICYCLIST:
    case ObjectType::OT_CYCLIST:
      return StBoundaryProto::CYCLIST;
    case ObjectType::OT_PEDESTRIAN:
      return StBoundaryProto::PEDESTRIAN;
    case ObjectType::OT_UNKNOWN_STATIC:
    case ObjectType::OT_BARRIER:
    case ObjectType::OT_BARRIER_ANTI_COLLISION_BUCKET:
    case ObjectType::OT_BARRIER_ANTI_COLLISION_POST:
    case ObjectType::OT_CONE:
    case ObjectType::OT_WARNING_TRIANGLE:
      return StBoundaryProto::STATIC;
    case ObjectType::OT_FOD:
    case ObjectType::OT_VEGETATION:
      return StBoundaryProto::IGNORABLE;
  }
}

bool CheckOverlap(const Box2d& av_box, const Box2d& object_box,
                  const ConflictResolutionConfigProto::ConflictResolverConfig&
                      general_config) {
  const double shrink_ratio = general_config.stationary_object_shrink_ratio();
  auto new_object_box = object_box;
  new_object_box.LateralExtendByRatio(shrink_ratio);
  new_object_box.LongitudinalExtendByRatio(shrink_ratio);
  const double distance = new_object_box.center().DistanceTo(av_box.center());
  // Av_box circle diameter = box width, object_box is shrinked so radius
  // using half diagonal of the shrinked box.
  return (distance <= 0.5 * (av_box.width() + new_object_box.diagonal()));
}

std::unique_ptr<SegmentMatcherKdtree> BuildKdTree(
    const planner::DiscretizedPath& path) {
  FUNC_QTRACE();
  std::vector<Vec2d> points;
  points.reserve(path.size());
  for (const auto& point : path) {
    points.emplace_back(point.x(), point.y());
  }
  return std::make_unique<SegmentMatcherKdtree>(std::move(points));
}

std::vector<Box2d> BuildEgoBoxes(const Box2d& av_box, double buffer,
                                 const planner::DiscretizedPath& path) {
  const double half_length = av_box.length() * 0.5 + buffer;
  const double half_width = av_box.width() * 0.5 + buffer;

  // Use path point pos as center of the box.
  std::vector<Box2d> boxes;
  boxes.reserve(path.size());
  for (size_t i = 0; i < path.size(); ++i) {
    const auto& pt = path[i];
    const Vec2d center(pt.x(), pt.y());
    boxes.push_back(Box2d(half_length, half_width, center, pt.theta()));
  }
  return boxes;
}

absl::Status FindOverLapRange(
    const SegmentMatcherKdtree& path_kd_tree,
    const std::vector<Box2d>& av_boxes_on_path,
    const ConflictResolutionConfigProto::ConflictResolverConfig& general_config,
    const Vec2d& search_point, double radius, const Box2d& object_box,
    double required_lateral_gap, int* low_idx, int* /*high_idx*/) {
  auto indices = path_kd_tree.GetSegmentIndexInRadius(search_point.x(),
                                                      search_point.y(), radius);
  if (indices.empty()) {
    return absl::NotFoundError(
        absl::StrFormat("No point found within %.2f meter radius.", radius));
  }
  std::sort(indices.begin(), indices.end());
  bool updated = false;
  for (auto it = indices.begin(); it != indices.end(); ++it) {
    const auto& raw_av_box = av_boxes_on_path[*it];
    Box2d av_box = raw_av_box;
    av_box.LongitudinalExtend(required_lateral_gap * 2.0);
    av_box.LateralExtend(required_lateral_gap * 2.0);
    if (CheckOverlap(av_box, object_box, general_config)) {
      *low_idx = *it;
      updated = true;
      break;
    }
  }
  if (!updated) {
    return absl::InternalError("search_point has no overlap with path.");
  }
  return absl::OkStatus();
}

struct StationaryObjectInfo {
  std::string traj_id;
  const PredictedTrajectory* predicted_trajectory = nullptr;
  const ObjectProto* object_proto = nullptr;
  const PredictedTrajectoryPoint* predicted_point = nullptr;
};

absl::StatusOr<StBoundaryPoints> MapStationaryObject(
    const SegmentMatcherKdtree& path_kd_tree,
    const planner::DiscretizedPath& path,
    const std::vector<Box2d>& av_boxes_on_path,
    const ConflictResolutionConfigProto::ConflictResolverConfig& general_config,
    const StationaryObjectInfo& object_info, double av_radius,
    double max_plan_time) {
  if (path.empty()) {
    return absl::InternalError("Path is empty for mapping object.");
  }
  const auto& object_proto = *object_info.object_proto;
  const auto& predicted_point = *object_info.predicted_point;
  StBoundaryPoints st_boundary_points;
  const Box2d obj_box(object_proto.bounding_box());
  const double search_radius = obj_box.diagonal() * 0.5 + av_radius;
  int low_idx = 0;
  int high_idx = path.size() - 1;
  // TODO(changqing): Set required lateral gap for different object type.
  RETURN_IF_ERROR(
      FindOverLapRange(path_kd_tree, av_boxes_on_path, general_config,
                       predicted_point.pos(), search_radius, obj_box,
                       /*required_lateral_gap=*/0.0, &low_idx, &high_idx));
  const double low_s = path[low_idx].s();
  const double high_s = path.back().s();
  st_boundary_points.lower_points = {{low_s, 0.0}, {low_s, max_plan_time}};
  st_boundary_points.upper_points = {{high_s, 0.0}, {high_s, max_plan_time}};
  st_boundary_points.speed_points = {{0.0, 0.0}, {0.0, max_plan_time}};
  st_boundary_points.overlap_infos = {
      planner::OverlapInfo{.time = 0.0,
                           .obj_idx = 0,
                           .av_start_idx = low_idx,
                           .av_end_idx = high_idx}};
  return st_boundary_points;
}

absl::StatusOr<StBoundaryPoints> MapTlRedLane(
    const SegmentMatcherKdtree& path_kd_tree,
    const planner::PlannerSemanticMapManager& smm,
    const planner::DiscretizedPath& path,
    const std::vector<Box2d>& av_boxes_on_path, double av_radius,
    double max_plan_time, const mapping::ElementId& lane_id, double control_s) {
  // Try mapping a half plane onto the path. Generate a st boundary.
  // Might have unsuccessful mapping due to path length.

  // Get lane point position to create a half plane.
  const auto* lane_info = smm.FindLaneInfoOrNull(lane_id);
  if (lane_info == nullptr) {
    return absl::NotFoundError(
        absl::StrFormat("Cannot find Lane %d info. So we can not map this red "
                        "light controlled lane onto st map.",
                        lane_id));
  }
  const auto lane_control_point = lane_info->LerpPointFromFraction(control_s);
  Vec2d lane_control_point_tangent;
  if (!lane_info->GetTangent(0.0, &lane_control_point_tangent)) {
    return absl::InternalError(absl::StrFormat(
        "Cannot find Lane %d start point tangent info.", lane_id));
  }
  // Make sure the path/ego proceeds towards the stopline.
  auto indices = path_kd_tree.GetSegmentIndexInRadiusWithHeading(
      lane_control_point.x(), lane_control_point.y(),
      lane_control_point_tangent.FastAngle(),
      kRedTLHalfPlaneSearchRadius + av_radius, M_PI_2);
  if (indices.empty()) {
    return absl::InternalError(absl::StrFormat(
        "No close segments around Lane %d red light controlled start point.",
        lane_id));
  }
  std::sort(indices.begin(), indices.end());

  // Create segment2d and check real overlapping range.
  const auto angle = -lane_control_point_tangent.Perp().Angle();
  const auto unit_vec = Vec2d::FastUnitFromAngle(angle);
  const auto seg =
      Segment2d(lane_control_point - kRedTLHalfPlaneSearchRadius * unit_vec,
                lane_control_point + kRedTLHalfPlaneSearchRadius * unit_vec);
  double s_min = path.back().s();
  bool updated = false;
  for (auto it = indices.begin(); it != indices.end(); ++it) {
    const auto& av_box = av_boxes_on_path[*it];
    if (av_box.HasOverlap(seg)) {
      s_min = path[*it].s();
      updated = true;
      break;
    }
  }

  if (!updated) {
    return absl::InternalError(
        absl::StrFormat("Cannot find overlapping s on path with tl red "
                        "controlled point on Lane %d",
                        lane_id));
  }
  s_min = std::max(0.0, s_min);
  const double s_max = path.back().s();
  StBoundaryPoints st_boundary_points;
  st_boundary_points.lower_points = {planner::StPoint(s_min, 0.0),
                                     planner::StPoint(s_min, max_plan_time)};
  st_boundary_points.upper_points = {planner::StPoint(s_max, 0.0),
                                     planner::StPoint(s_max, max_plan_time)};
  st_boundary_points.speed_points = {planner::VtPoint(0.0, 0.0),
                                     planner::VtPoint(0.0, max_plan_time)};

  return st_boundary_points;
}
}  // namespace

ObjectStMap::ObjectStMap(const ObjectConflictManager& conflict_mgr,
                         const ObjectProto& av_proto,
                         const planner::DiscretizedPath& path,
                         const planner::SpeedVector& ref_speed,
                         const PredictedTrajectory& predicted_trajectory,
                         const ConflictResolverParams* params,
                         ThreadPool* thread_pool) {
  SCOPED_QTRACE("ObjectStMap::ObjectStMap");
  thread_pool_ = thread_pool;
  conflict_mgr_ = &conflict_mgr;
  av_proto_ = &av_proto;
  path_ = &path;
  ref_speed_ = &ref_speed;
  predicted_traj_ = &predicted_trajectory;
  max_length_ = path.length();
  // TODO(changqing): KdTree or BF?
  path_kd_tree_ = BuildKdTree(path);
  const Box2d av_box((*av_proto_).bounding_box());
  general_config_ = &(params->GetGeneralConfig());
  object_config_ = &(params->GetConfigByObjectType(av_proto.type()));
  av_boxes_on_path_ =
      BuildEgoBoxes(av_box, object_config_->ego_box_buffer(), path);
  av_radius_ = 0.5 * av_box.diagonal();
}

absl::Status ObjectStMap::MapStationaryObjects(
    absl::Span<const std::string> traj_ids) {
  SCOPED_QTRACE("ObjectStMap::MapStationaryObjects");
  if (traj_ids.empty()) {
    return absl::OkStatus();
  }
  std::vector<StationaryObjectInfo> infos;
  for (const auto& traj_id : traj_ids) {
    ASSIGN_OR_CONTINUE(const PredictedTrajectory* ptr_traj,
                       conflict_mgr_->GetPredictedTrajectoryByTrajId(traj_id));
    const auto& predicted_trajectory = *ptr_traj;
    if (predicted_trajectory.type() != PT_STATIONARY ||
        predicted_trajectory.points().empty()) {
      continue;
    }
    const auto& predicted_point = predicted_trajectory.points().front();
    ASSIGN_OR_CONTINUE(const ObjectProto* ptr_object_proto,
                       conflict_mgr_->GetObjectProtoByTrajId(traj_id));
    infos.push_back(StationaryObjectInfo({
        .traj_id = traj_id,
        .predicted_trajectory = ptr_traj,
        .object_proto = ptr_object_proto,
        .predicted_point = &predicted_point,
    }));
  }
  std::vector<planner::StBoundaryRef> st_bounds;
  st_bounds.resize(infos.size());
  ParallelFor(0, infos.size(), thread_pool_, [&](int i) {
    const auto& object_info = infos[i];
    const auto st_boundary_points_or = MapStationaryObject(
        *path_kd_tree_, *path_, av_boxes_on_path_, *general_config_,
        object_info, av_radius_, max_plan_time_);
    if (st_boundary_points_or.ok()) {
      // TODO(yuhang): Determine is_large_vehicle.
      st_bounds[i] = planner::StBoundary::CreateInstance(
          *st_boundary_points_or,
          ToStBoundaryObjectType(object_info.object_proto->type()),
          object_info.traj_id, object_info.predicted_trajectory->probability(),
          /*is_stationary=*/true, StBoundaryProto::NON_PROTECTIVE,
          /*is_large_vehicle=*/false);
    }
  });
  for (auto&& st_bound_ref : st_bounds) {
    if (st_bound_ref == nullptr) continue;
    stationary_objects_.push_back(std::move(st_bound_ref));
  }
  return absl::OkStatus();
}

void ObjectStMap::MapTLRedLanes(
    const std::set<mapping::ElementId>& red_tls,
    absl::Span<const mapping::ElementId> possible_lane_path) {
  FUNC_QTRACE();
  // Only map traffic light for vehicles.
  if (av_proto_->type() != ObjectType::OT_VEHICLE) return;
  const auto& psmm = *conflict_mgr_->semantic_map_manager();
  for (const auto& lane_id : possible_lane_path) {
    const auto* lane_proto = psmm.FindLaneByIdOrNull(lane_id);
    if (lane_proto == nullptr) continue;

    std::optional<double> red_light_control_lane_fraction = std::nullopt;
    // Query start point associated traffic lights.
    for (const auto& id : lane_proto->startpoint_associated_traffic_lights()) {
      if (red_tls.find(mapping::ElementId(id)) != red_tls.end()) {
        // Since start point is controlled.
        red_light_control_lane_fraction = 0.0;
      }
    }

    // Query multiple control points for this lane (mainly to deal with left
    // turn wait zone).
    for (const auto& control_point :
         lane_proto->multi_traffic_light_control_points()) {
      for (const auto& left_tl_id : control_point.left_tls()) {
        if (red_tls.find(mapping::ElementId(left_tl_id)) == red_tls.end()) {
          continue;
        }
        if (red_light_control_lane_fraction.has_value()) {
          red_light_control_lane_fraction = std::max<double>(
              *red_light_control_lane_fraction, control_point.lane_fraction());
        } else {
          red_light_control_lane_fraction = control_point.lane_fraction();
        }
      }
    }

    if (!red_light_control_lane_fraction.has_value()) continue;
    const auto st_boundary_points_or = MapTlRedLane(
        *path_kd_tree_, psmm, *path_, av_boxes_on_path_, av_radius_,
        max_plan_time_, lane_id, *red_light_control_lane_fraction);

    if (!st_boundary_points_or.ok()) {
      VLOG(3) << absl::StrFormat(
          "Fail to map Lane %d red light controlled point onto path for "
          "object %s traj id %d.",
          lane_id, av_proto_->id(), predicted_traj_->index());
      continue;
    }
    // Add stopline instance onto map.
    stop_lines_.push_back(StBoundary::CreateInstance(
        *st_boundary_points_or, StBoundaryProto::VIRTUAL,
        absl::StrFormat("RED_TL_LANE_%d_%.3f", lane_id,
                        *red_light_control_lane_fraction),
        /*probability=*/1.0, /*is_stationary=*/true,
        StBoundaryProto::NON_PROTECTIVE, /*is_large_vehicle=*/false));
    VLOG(3) << absl::StrFormat(
        "Map Lane %d red light controlled point onto path for "
        "object %s traj id %d.",
        lane_id, av_proto_->id(), predicted_traj_->index());
  }
}

std::vector<double> ObjectStMap::QueryStoplines() const {
  std::vector<double> ret;
  ret.reserve(stop_lines_.size());
  for (const auto& st_bound : stop_lines_) {
    ret.push_back(st_bound->min_s());
  }
  std::sort(ret.begin(), ret.end());
  return ret;
}

std::vector<double> ObjectStMap::QueryStationaryObjects() const {
  std::vector<double> ret;
  ret.reserve(stationary_objects_.size());
  for (const auto& st_bound : stationary_objects_) {
    ret.push_back(st_bound->min_s());
  }
  std::sort(ret.begin(), ret.end());
  return ret;
}

std::string ObjectStMap::Annotation() const {
  std::vector<std::string> mapped;
  mapped.push_back("S");
  for (auto&& stationary_object : stationary_objects_) {
    mapped.push_back(stationary_object->id());
  }
  mapped.push_back("SL");
  for (auto&& tl : stop_lines_) {
    mapped.push_back(tl->id());
  }
  return absl::StrJoin(mapped, ",");
}
}  // namespace qcraft::prediction
