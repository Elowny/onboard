#include "onboard/prediction/prediction_util.h"

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type
#include <algorithm>
#include <limits>
#include <memory>
#include <ostream>
#include <string>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "absl/strings/str_format.h"

#include "onboard/container/strong_int.h"
#include "onboard/maps/proto/lane_path.pb.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/math/geometry/util.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/prediction/container/object_history_span.h"
#include "onboard/prediction/prediction_defs.h"
#include "onboard/prediction/prediction_flags.h"
#include "onboard/prediction/util/perception_util.h"
#include "onboard/proto/assist_state.pb.h"
#include "onboard/proto/trajectory_point.pb.h"

namespace qcraft {
namespace prediction {

namespace {
// Guess object type.
constexpr double kPedMaxLen = 1.0;
constexpr double kBikeMaxLen = 2.0;
constexpr double kBikeMaxSpeed = 10.0;
constexpr double kPedMaxSpeed = 3.0;

PredictedTrajectoryPoint
CreateStationaryPredictedTrajectoryPointFromStationaryObject(
    const ObjectProto& object) {
  PredictedTrajectoryPoint point;
  point.set_pos(Vec2dFromProto(object.pos()));
  point.set_s(0.0);
  point.set_theta(object.yaw());
  point.set_kappa(0.0);

  point.set_t(0.0);
  point.set_v(0.0);
  point.set_a(0.0);

  return point;
}

PredictedTrajectoryPointProto CreateTrajectoryPointFromStationaryObject(
    const ObjectProto& object) {
  PredictedTrajectoryPointProto point;
  *point.mutable_pos() = object.pos();
  point.set_s(0.0);
  point.set_theta(object.yaw());
  point.set_kappa(0.0);

  point.set_t(0.0);
  point.set_v(0.0);
  point.set_a(0.0);

  return point;
}

std::vector<PredictedTrajectoryPoint> DevelopPredictedTrajectoryPointsFromState(
    const Vec2d& pos, const Vec2d& init_v, double init_heading, double yaw_rate,
    int time_step_horizon) {
  constexpr double kZeroLinearAcceleration =
      0.0;  // Linear acceleration is zero!
  std::vector<PredictedTrajectoryPoint> points;
  points.reserve(time_step_horizon);

  double s = 0.0;
  auto cur_pos = pos;
  double cur_speed = init_v.norm();
  double heading = init_heading;

  for (int i = 0; i < time_step_horizon; ++i) {
    PredictedTrajectoryPoint point;
    point.set_pos(cur_pos);
    point.set_t(i * kPredictionTimeStep);
    point.set_s(s);
    point.set_theta(NormalizeAngle(heading));
    point.set_kappa(cur_speed > 0.0 ? yaw_rate / cur_speed : 0.0);
    point.set_a(kZeroLinearAcceleration);
    point.set_v(cur_speed);
    points.push_back(point);

    // Update.
    const auto cur_v = cur_speed * Vec2d::FastUnitFromAngle(heading);
    cur_pos += cur_v * kPredictionTimeStep;
    s += cur_speed * kPredictionTimeStep;
    heading += yaw_rate * kPredictionTimeStep;
    cur_speed += kZeroLinearAcceleration * kPredictionTimeStep;
  }
  return points;
}

bool IsStationaryType(ObjectType type) {
  switch (type) {
    case OT_UNKNOWN_STATIC:
    case OT_FOD:
    case OT_VEGETATION:
    case OT_BARRIER:
    case OT_BARRIER_ANTI_COLLISION_BUCKET:
    case OT_BARRIER_ANTI_COLLISION_POST:
    case OT_CONE:
    case OT_WARNING_TRIANGLE:
      return true;
    case OT_VEHICLE:
    case OT_LARGE_VEHICLE:
    case OT_UNKNOWN_MOVABLE:
    case OT_MOTORCYCLIST:
    case OT_PEDESTRIAN:
    case OT_CYCLIST:
    case OT_TRICYCLIST:
      return false;
  }
}

// Determine object type by heuristic, only for unknown objs.
ObjectType DetermineTypeByHeuristicForUnknown(double len, double cur_v) {
  if (cur_v > kBikeMaxSpeed) {
    return OT_VEHICLE;
  }
  if (len > kBikeMaxLen) {
    return OT_VEHICLE;
  }
  if (len > kPedMaxLen || cur_v > kPedMaxSpeed) {
    return OT_CYCLIST;
  }
  return OT_PEDESTRIAN;
}

struct TrajCurbCollisionInfo {
  int collision_index;
  bool can_pass;

  std::string DebugString() const {
    return absl::StrFormat("Traj curb collision info index: %d, can pass? %d",
                           collision_index, can_pass);
  }
};

}  // namespace

bool AllLanesFoundInSemanticMap(
    const mapping::LanePathProto& lane_path_proto,
    const planner::PlannerSemanticMapManager& semantic_map_manager) {
  if (lane_path_proto.lane_ids().empty()) {
    return false;
  }
  for (const auto& id : lane_path_proto.lane_ids()) {
    const auto* lane_info_ptr =
        semantic_map_manager.FindLaneInfoOrNull(mapping::ElementId(id));
    if (lane_info_ptr == nullptr) {
      return false;
    }
  }
  return true;
}

ObjectProto LerpObjectProto(const ObjectProto& a, const ObjectProto& b,
                            double alpha) {
  ObjectProto object = b;
  const double lerped_ts = Lerp(a.timestamp(), b.timestamp(), alpha);
  const Vec2d lerped_pos =
      Lerp(Vec2dFromProto(a.pos()), Vec2dFromProto(b.pos()), alpha);
  const Vec2d lerped_vel =
      Lerp(Vec2dFromProto(a.vel()), Vec2dFromProto(b.vel()), alpha);
  const Vec2d lerped_accel =
      Lerp(Vec2dFromProto(a.accel()), Vec2dFromProto(b.accel()), alpha);
  const double lerped_yaw = NormalizeAngle(LerpAngle(a.yaw(), b.yaw(), alpha));
  const double lerped_yaw_rate =
      NormalizeAngle(LerpAngle(a.yaw_rate(), b.yaw_rate(), alpha));
  object.set_timestamp(lerped_ts);
  lerped_pos.ToProto(object.mutable_pos());
  lerped_vel.ToProto(object.mutable_vel());
  lerped_accel.ToProto(object.mutable_accel());
  object.set_yaw(lerped_yaw);
  object.set_yaw_rate(lerped_yaw_rate);
  return object;
}

ObjectType GuessType(const ObjectHistory& obj) {
  const auto hist = obj.GetHistory();
  if (obj.type() == OT_PEDESTRIAN) {
    return OT_PEDESTRIAN;
  }
  if (obj.type() == OT_CYCLIST || obj.type() == OT_MOTORCYCLIST ||
      obj.type() == OT_TRICYCLIST) {
    return OT_CYCLIST;
  }
  if (obj.type() == OT_VEHICLE || obj.type() == OT_LARGE_VEHICLE) {
    return OT_VEHICLE;
  }
  if (obj.type() == OT_UNKNOWN_MOVABLE) {
    return DetermineTypeByHeuristicForUnknown(hist.bounding_box().length(),
                                              hist.v());
  }
  return obj.type();
}

std::vector<ObjectProto> ResampleObjectProtos(
    absl::Span<const ObjectProto> objs, double current_ts, double time_step,
    int max_steps) {
  if (objs.empty()) {
    return {};
  }
  if (objs.size() == 1) {
    ObjectProto object = objs.back();
    const auto align_status = AlignPerceptionObjectTime(current_ts, &object);
    return {object};
  }
  std::vector<double> vect;
  std::vector<ObjectProto> vec_objs;
  vect.reserve(objs.size());
  vec_objs.reserve(objs.size());
  for (const auto& obj : objs) {
    vect.push_back(obj.timestamp() - current_ts);
    vec_objs.push_back(obj);
  }
  class ObjectProtoLerper {
   public:
    ObjectProto operator()(const ObjectProto& a, const ObjectProto& b,
                           double alpha) const {
      return LerpObjectProto(a, b, alpha);
    }
  };
  PiecewiseLinearFunction<ObjectProto, double, ObjectProtoLerper>
      object_proto_plf(vect, vec_objs);
  std::vector<ObjectProto> resampled_objects;
  resampled_objects.reserve(max_steps);
  for (int i = 0; i < max_steps; ++i) {
    const double ts = i * time_step;
    if (ts > vect.back()) {
      break;
    }
    auto resampled_obj = object_proto_plf.EvaluateWithExtrapolation(ts);
    resampled_objects.push_back(std::move(resampled_obj));
  }
  return resampled_objects;
}

absl::StatusOr<ObjectPredictionProto> InstantPredictionForNewObject(
    const ObjectProto& object, double prediction_time) {
  ObjectPredictionProto proto;
  if (!object.has_id() || !object.has_pos() || !object.has_vel()) {
    return absl::NotFoundError(
        absl::StrFormat("Input object proto missing one of the fields: "
                        "id[exist=%d], pos[exist=%d], val[exist=%d]",
                        object.has_id(), object.has_pos(), object.has_vel()));
  }

  proto.set_id(object.id());
  *proto.mutable_perception_object() = object;

  if (IsStationaryType(object.type())) {
    auto* traj = proto.add_trajectories();
    traj->set_probability(1.0);
    traj->set_type(PredictionType::PT_STATIONARY);
    *traj->add_points() = CreateTrajectoryPointFromStationaryObject(object);
    return proto;
  }

  const double linear_a = 0.0;

  double s = 0.0;  // m.
  Vec2d pos = Vec2dFromProto(object.pos());
  const Vec2d initial_v = Vec2dFromProto(object.vel());
  double speed = initial_v.norm();
  constexpr double kZeroSpeedEpsilon = 1e-8;
  double heading =
      speed < kZeroSpeedEpsilon ? object.yaw() : NormalizeAngle2D(initial_v);
  const double yaw_rate = speed < kZeroSpeedEpsilon ? 0.0 : object.yaw_rate();
  const int prediction_horizon =
      FloorToInt(prediction_time / kPredictionTimeStep);

  auto* traj_ptr = proto.add_trajectories();
  traj_ptr->mutable_points()->Reserve(prediction_horizon);
  for (int j = 0; j < prediction_horizon; ++j) {
    PredictedTrajectoryPoint point;
    const double t = j * kPredictionTimeStep;
    point.set_t(t);
    point.set_pos(pos);
    const Vec2d v = speed * Vec2d::FastUnitFromAngle(heading);
    pos += v * kPredictionTimeStep;
    point.set_s(s);
    s += speed * kPredictionTimeStep;
    point.set_theta(NormalizeAngle(heading));
    point.set_kappa(speed > 0.0 ? yaw_rate / speed : 0.0);
    point.set_a(linear_a);
    point.set_v(speed);
    heading += yaw_rate * kPredictionTimeStep;
    speed += linear_a * kPredictionTimeStep;
    auto* traj_point_proto_ptr = traj_ptr->add_points();
    point.ToProto(traj_point_proto_ptr);
  }
  traj_ptr->set_probability(1.0);
  if (proto.trajectories_size() == 1) {
    return proto;
  }
  return absl::UnknownError(absl::StrFormat(
      "Unknown problem causing result to have trajectories_size != 1."));
}

absl::StatusOr<ObjectPrediction> InstantObjectPredictionForNewObject(
    const ObjectProto& object, double prediction_time) {
  if (!object.has_id() || !object.has_pos() || !object.has_vel()) {
    return absl::NotFoundError(
        absl::StrFormat("Input object proto missing one of the fields: "
                        "id[exist=%d], pos[exist=%d], val[exist=%d]",
                        object.has_id(), object.has_pos(), object.has_vel()));
  }
  if (IsStationaryType(object.type())) {
    std::vector<PredictedTrajectoryPoint> points = {
        CreateStationaryPredictedTrajectoryPointFromStationaryObject(object)};
    PredictedTrajectory stationary_traj(
        /*probability=*/1.0,
        /*annotation=*/"instant prediction for stationary object",
        PredictionType::PT_STATIONARY, /*index=*/0, std::move(points),
        /*is_reversed=*/false,
        /*is_hard_braking=*/false);
    return ObjectPrediction(
        {std::move(stationary_traj)}, object,
        /*road_status=*/ObjectRoadStatus::ORS_NONE,
        /*intersection_status=*/ObjectIntersectionStatus::OIS_NONE);
  }
  const auto init_pos = Vec2dFromProto(object.pos());
  const Vec2d init_v = Vec2dFromProto(object.vel());
  const double init_speed = init_v.norm();
  constexpr double kZeroSpeedEpsilon = 1e-8;
  const double init_heading =
      init_speed < kZeroSpeedEpsilon ? object.yaw() : NormalizeAngle2D(init_v);
  const double yaw_rate =
      init_speed < kZeroSpeedEpsilon ? 0.0 : object.yaw_rate();
  auto traj_points = DevelopPredictedTrajectoryPointsFromState(
      init_pos, init_v, init_heading, yaw_rate,
      FloorToInt(prediction_time / kPredictionTimeStep));
  PredictedTrajectory pred_traj(
      /*probability=*/1.0,
      /*annotation=*/"instant prediction for moving object",
      PredictionType::PT_CYCV, /*index=*/0, std::move(traj_points),
      /*is_reversed=*/false,
      /*is_hard_braking=*/false);
  return ObjectPrediction(
      {std::move(pred_traj)}, object,
      /*road_status=*/ObjectRoadStatus::ORS_NONE,
      /*intersection_status=*/ObjectIntersectionStatus::OIS_NONE);
}

std::unique_ptr<SegmentMatcherKdtree> TrajectoryProtoToSegmentMatcherKdtree(
    const TrajectoryProto& trajectory) {
  constexpr double kClosePointDistanceThreshold = 1e-3;
  const auto& traj_points = trajectory.trajectory_point();
  std::vector<Vec2d> points;
  for (int i = 0; i < traj_points.size(); ++i) {
    const auto& pt = traj_points[i];
    const Vec2d pt_vec2d(pt.path_point().x(), pt.path_point().y());
    if (points.empty()) {
      points.push_back(pt_vec2d);
      continue;
    }
    if (points.back().DistanceTo(pt_vec2d) > kClosePointDistanceThreshold) {
      points.push_back(pt_vec2d);
    }
  }
  if (points.size() <= 1) {
    return nullptr;
  }
  return std::make_unique<SegmentMatcherKdtree>(points);
}

bool MustReceiveHDMapForPrediction(const AutonomyStateProto& autonomy_state) {
  if (!autonomy_state.has_assist_state()) {
    return true;
  }
  if (!autonomy_state.assist_state().has_assist_drive_system_state()) {
    return true;
  }
  switch (autonomy_state.assist_state().assist_drive_system_state()) {
    case AssistStateProto::ASSIST_OFF:
    case AssistStateProto::ASSIST_NOT_READY:
    case AssistStateProto::ASSIST_ACC_READY:
    case AssistStateProto::ASSIST_LCC_READY:
    case AssistStateProto::ASSIST_NOA_READY:
    case AssistStateProto::ASSIST_LCC_ACTIVE:
    case AssistStateProto::ASSIST_ACC_ACTIVE:
    case AssistStateProto::ASSIST_APA_ACTIVE:
      return false;
    case AssistStateProto::ASSIST_NOA_ACTIVE: {
      return true;
    }
  }
}

std::pair<double, double> QueryDistanceToLeftAndRightAvLaneBoundary(
    const planner::DrivePassage& drive_passage, const double& s,
    const double& l) {
  constexpr double kMaxLaneBoundaryOffset = 1.875;  // 3.75/2.0 m
  const auto& boundaries_at_s =
      drive_passage.QueryEnclosingLaneBoundariesAtS(s);
  double dist_to_left_lane_boundary = std::numeric_limits<double>::max();
  double dist_to_right_lane_boundary = std::numeric_limits<double>::max();

  if (boundaries_at_s.left.has_value()) {
    dist_to_left_lane_boundary =
        l - std::min(boundaries_at_s.left->lat_offset, kMaxLaneBoundaryOffset);
  } else {
    dist_to_left_lane_boundary = l - kMaxLaneBoundaryOffset;
  }
  if (boundaries_at_s.right.has_value()) {
    dist_to_right_lane_boundary =
        l -
        std::max(boundaries_at_s.right->lat_offset, -kMaxLaneBoundaryOffset);
  } else {
    dist_to_right_lane_boundary = l + kMaxLaneBoundaryOffset;
  }
  return std::pair<double, double>(dist_to_left_lane_boundary,
                                   dist_to_right_lane_boundary);
}

bool IsOnlineMapMode(const AutonomyStateProto* autonomy_state) {
  bool is_dynamic_online_map_mode = false;
  if (autonomy_state != nullptr &&
      !MustReceiveHDMapForPrediction(*autonomy_state)) {
    is_dynamic_online_map_mode = true;
  }
  return FLAGS_prediction_enable_debug_perception_map ||
         is_dynamic_online_map_mode;
}

Box2d GetRegionBox(const Vec2d& pos, const double heading,
                   double detection_region_front,
                   double detection_region_behind,
                   double detection_half_width) {
  const double half_length =
      (detection_region_front + detection_region_behind) * 0.5;
  const double center_ahead_dis = half_length - detection_region_behind;
  const Vec2d tangent = Vec2d::FastUnitFromAngle(heading);
  const Vec2d center = pos + tangent * center_ahead_dis;
  return Box2d(half_length, detection_half_width, center, heading, tangent);
}

std::vector<double> GetActNetScanBoxInfo(ObjectType type, double speed) {
  if (type == OT_PEDESTRIAN) {
    return {kPedestrianScanBoxFront, kPedestrianScanBoxBack,
            kPedestrianScanBoxFrontHalfWidth};
  } else {
    return {kScanBoxFrontPlf(speed), kScanBoxBack,
            kScanBoxFrontHalfWidthPlf(speed)};
  }
}

Box2d GetDynamicRegionBox(const Vec2d& ref_position,
                          const ObjectMotionHistory& agent_history,
                          double heading) {
  const double speed = agent_history.states.back().vel.Length();
  const auto scan_box_info = GetActNetScanBoxInfo(agent_history.type, speed);
  const double scan_box_front = scan_box_info.at(0);
  const double scan_box_back = scan_box_info.at(1);
  const double scan_box_front_half_width = scan_box_info.at(2);
  return GetRegionBox(ref_position, heading, scan_box_front, scan_box_back,
                      scan_box_front_half_width);
}

std::map<std::string, FeatureScaleConfig> BuildModelScaleParamMap(
    const ModelScaleParam& scale_param) {
  std::map<std::string, FeatureScaleConfig> map;
  for (const auto& feature_scale_proto : scale_param.feature_scales()) {
    FeatureScaleConfig scale_config;
    scale_config.FromProto(feature_scale_proto);
    map[feature_scale_proto.feature_name()] = std::move(scale_config);
  }
  return map;
}
}  // namespace prediction
}  // namespace qcraft
