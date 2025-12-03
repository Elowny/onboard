#include "onboard/planner/speed/decider/close_object_slowdown_decider.h"

#include <algorithm>
#include <cmath>
#include <optional>
#include <ostream>
#include <string>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/status/statusor.h"
#include "glog/logging.h"

#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/math/frenet_common.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/speed/proto/speed_finder.pb.h"
#include "onboard/planner/util/path_util.h"
#include "onboard/utils/map_util.h"

namespace qcraft {
namespace planner {

namespace {

// TODO(jingqiao): Move to params file.
constexpr double kStationarySStartLength = 10.0;  // m
constexpr double kStationarySEndLength = 3.0;     // m

constexpr double kMaxDecelByCloseObject = -0.8;  // m/s^2.

const std::vector<double> kCloseObjectDistanceRange = {0.0, 0.5, 1.0, 1.5,
                                                       2.0};  // m

const std::vector<double> kStationaryUnknownObjectMaxSpeed = {
    5.0, 8.0, 12.0, 14.0, 18.0};  // m/s
const std::vector<double> kStationaryVehicleMaxSpeed = {5.0, 8.0, 12.0, 14.0,
                                                        18.0};  // m/s
const std::vector<double> kStationaryCyclistMaxSpeed = {2.0, 3.0, 4.0, 6.0,
                                                        8.0};  // m/s
const std::vector<double> kStationaryPedestrianMaxSpeed = {2.0, 3.0, 4.0, 6.0,
                                                           8.0};  // m/s
const std::vector<double> kStationaryStaticMaxSpeed = {10.0, 15.0, 20.0, 25.0,
                                                       30.0};  // m/s

// NOTE: Consider the case when change line and the sl boundary is wide.
const std::vector<double> kStationaryInsideSlBoundaryUnknownObjectMaxSpeed = {
    4.0, 6.0, 8.0, 10.0, 15.0};  // m/s
const std::vector<double> kStationaryInsideSlBoundaryVehicleMaxSpeed = {
    4.0, 6.0, 8.0, 10.0, 15.0};  // m/s
const std::vector<double> kStationaryInsideSlBoundaryCyclistMaxSpeed = {
    2.0, 3.0, 4.0, 6.0, 8.0};  // m/s
const std::vector<double> kStationaryInsideSlBoundaryPedestrianMaxSpeed = {
    2.0, 3.0, 4.0, 6.0, 8.0};  // m/s
// Copy to AddAggregateStaticObjectCost in trajectory_optizmier, please modify
// at the same time.
const std::vector<double> kStationaryInsideSlBoundaryStaticMaxSpeed = {
    10.0, 15.0, 20.0, 25.0, 30.0};  // m/s

std::optional<double> DecideStationaryMaxSpeed(
    StBoundaryProto::ObjectType object_type,
    const StDistancePoint& st_distance_point) {
  static const auto kMap =
      absl::flat_hash_map<StBoundaryProto::ObjectType,
                          PiecewiseLinearFunction<double, double>>{
          {StBoundaryProto::UNKNOWN_OBJECT,
           PiecewiseLinearFunction(kCloseObjectDistanceRange,
                                   kStationaryUnknownObjectMaxSpeed)},
          {StBoundaryProto::VEHICLE,
           PiecewiseLinearFunction(kCloseObjectDistanceRange,
                                   kStationaryVehicleMaxSpeed)},
          {StBoundaryProto::CYCLIST,
           PiecewiseLinearFunction(kCloseObjectDistanceRange,
                                   kStationaryCyclistMaxSpeed)},
          {StBoundaryProto::PEDESTRIAN,
           PiecewiseLinearFunction(kCloseObjectDistanceRange,
                                   kStationaryPedestrianMaxSpeed)},
          {StBoundaryProto::STATIC,
           PiecewiseLinearFunction(kCloseObjectDistanceRange,
                                   kStationaryStaticMaxSpeed)}};
  const auto* res = FindOrNull(kMap, object_type);
  if (res != nullptr) {
    return res->Evaluate(st_distance_point.distance);
  } else {
    return std::nullopt;
  }
}

std::optional<double> DecideStationaryInsideSlBoundaryMaxSpeed(
    StBoundaryProto::ObjectType object_type,
    const StDistancePoint& st_distance_point) {
  static const auto kMap = absl::flat_hash_map<
      StBoundaryProto::ObjectType, PiecewiseLinearFunction<double, double>>{
      {StBoundaryProto::UNKNOWN_OBJECT,
       PiecewiseLinearFunction(
           kCloseObjectDistanceRange,
           kStationaryInsideSlBoundaryUnknownObjectMaxSpeed)},
      {StBoundaryProto::VEHICLE,
       PiecewiseLinearFunction(kCloseObjectDistanceRange,
                               kStationaryInsideSlBoundaryVehicleMaxSpeed)},
      {StBoundaryProto::CYCLIST,
       PiecewiseLinearFunction(kCloseObjectDistanceRange,
                               kStationaryInsideSlBoundaryCyclistMaxSpeed)},
      {StBoundaryProto::PEDESTRIAN,
       PiecewiseLinearFunction(kCloseObjectDistanceRange,
                               kStationaryInsideSlBoundaryPedestrianMaxSpeed)},
      {StBoundaryProto::STATIC,
       PiecewiseLinearFunction(kCloseObjectDistanceRange,
                               kStationaryInsideSlBoundaryStaticMaxSpeed)}};
  const auto* res = FindOrNull(kMap, object_type);
  if (res != nullptr) {
    return res->Evaluate(st_distance_point.distance);
  } else {
    return std::nullopt;
  }
}

void ComputeStationaryPathS(const StDistancePoint& st_distance_point,
                            double max_s, double* start_path_s,
                            double* end_path_s) {
  *start_path_s =
      std::max(st_distance_point.path_s - kStationarySStartLength, 0.0);
  *end_path_s =
      std::min(st_distance_point.path_s + kStationarySEndLength, max_s);
}

bool CheckInsidePathSlBoundary(const Box2d& box,
                               const PathSlBoundary& path_sl_boundary,
                               const DrivePassage& drive_passage) {
  const auto fbox = drive_passage.QueryFrenetBoxAt(box);
  if (fbox.ok()) {
    const auto [right_l_s_min, left_l_s_min] =
        path_sl_boundary.QueryBoundaryL(fbox->s_min);
    VLOG(2) << "fbox_l_min: " << fbox->l_min << ", fbox_l_max: " << fbox->l_max
            << ", right_l_s_min: " << right_l_s_min
            << ", left_l_s_min: " << left_l_s_min;
    if ((right_l_s_min <= fbox->l_min && fbox->l_min <= left_l_s_min) ||
        (right_l_s_min <= fbox->l_max && fbox->l_max <= left_l_s_min)) {
      return true;
    }
    const auto [right_l_s_max, left_l_s_max] =
        path_sl_boundary.QueryBoundaryL(fbox->s_max);
    VLOG(2) << "right_l_s_max: " << right_l_s_max
            << ", left_l_s_max: " << left_l_s_max;
    return (right_l_s_max <= fbox->l_min && fbox->l_min <= left_l_s_max) ||
           (right_l_s_max <= fbox->l_max && fbox->l_max <= left_l_s_max);
  } else {
    VLOG(2) << "Failed to project to frenet for box: " << box.DebugString();
    return false;
  }
  // Should never be here.
  return false;
}

}  // namespace

std::vector<ConstraintProto::PathSpeedRegionProto>
MakeCloseObjectSlowdownDecision(
    const std::vector<CloseSpaceTimeObject>& close_space_time_objects,
    const DrivePassage& drive_passage, const DiscretizedPath& path_points,
    double av_speed, const PathSlBoundary& path_sl_boundary) {
  std::vector<ConstraintProto::PathSpeedRegionProto> res;

  for (auto close_space_time_object = close_space_time_objects.begin();
       close_space_time_object != close_space_time_objects.end();
       ++close_space_time_object) {
    QCHECK_EQ(close_space_time_object->st_distance_points.size(), 1);
    const auto& st_distance_point =
        close_space_time_object->st_distance_points[0];

    VLOG(2) << "------- ID: " << close_space_time_object->id << " ---------";

    std::optional<double> max_speed;
    double start_path_s = 0.0;
    double end_path_s = 0.0;

    const bool inside_path_sl_boundary = CheckInsidePathSlBoundary(
        close_space_time_object->box, path_sl_boundary, drive_passage);
    if (inside_path_sl_boundary) {
      max_speed = DecideStationaryInsideSlBoundaryMaxSpeed(
          close_space_time_object->object_type, st_distance_point);
    } else {
      max_speed = DecideStationaryMaxSpeed(close_space_time_object->object_type,
                                           st_distance_point);
    }
    VLOG(2) << "Inside_sl_boundary: "
            << (inside_path_sl_boundary ? "true" : "false");
    if (!max_speed.has_value()) continue;
    ComputeStationaryPathS(st_distance_point, path_points.length(),
                           &start_path_s, &end_path_s);

    VLOG(2) << "start_path_s: " << start_path_s
            << ", end_path_s: " << end_path_s
            << ", path_s: " << st_distance_point.path_s
            << ", relative_v: " << st_distance_point.relative_v
            << ", max_speed: " << *max_speed
            << ", distance: " << st_distance_point.distance
            << ", type: " << close_space_time_object->object_type;

    const auto start_point = ToVec2d(path_points.Evaluate(start_path_s));
    const auto end_point = ToVec2d(path_points.Evaluate(end_path_s));

    // Avoid hard brake caused by close object speed limit.
    const double decel =
        (Sqr(*max_speed) - Sqr(av_speed)) / ((2.0 * start_path_s) + 1e-3);
    if (decel < kMaxDecelByCloseObject) {
      max_speed = std::sqrt(std::max(
          0.0, 2.0 * kMaxDecelByCloseObject * start_path_s + Sqr(av_speed)));
    }

    ConstraintProto::PathSpeedRegionProto close_object_speed_region;
    close_object_speed_region.set_start_s(start_path_s);
    close_object_speed_region.set_end_s(end_path_s);
    start_point.ToProto(close_object_speed_region.mutable_start_point());
    end_point.ToProto(close_object_speed_region.mutable_end_point());
    close_object_speed_region.set_max_speed(*max_speed);
    close_object_speed_region.set_id(close_space_time_object->id);
    close_object_speed_region.mutable_source()->mutable_close_object()->set_id(
        close_space_time_object->id);
    res.emplace_back(std::move(close_object_speed_region));
  }

  return res;
}

}  // namespace planner
}  // namespace qcraft
