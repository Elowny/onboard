#include "onboard/prediction/long_term/slow_front_vehicle_recognizer.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "boost/circular_buffer.hpp"

#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/math/geometry/util.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/util/perception_util.h"
#include "onboard/prediction/container/object_history_span.h"
#include "onboard/prediction/container/prediction_object.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/proto/perception/fusion/object.pb.h"
#include "onboard/proto/positioning.pb.h"
#include "onboard/utils/elements_history.h"

namespace qcraft {
namespace prediction {
namespace {

PoseProto FindPoseAtNearestTimestamp(
    const boost::circular_buffer<PoseProto>& av_pose_cache, double timestamp) {
  QCHECK(!av_pose_cache.empty());
  const int cache_size = av_pose_cache.size();
  int min_index = cache_size - 1;
  double min_abs_diff =
      std::abs(av_pose_cache[min_index].timestamp() - timestamp);
  for (int i = cache_size - 2; i >= 0; --i) {
    const double timestamp_diff =
        std::abs(av_pose_cache[i].timestamp() - timestamp);
    if (timestamp_diff < min_abs_diff) {
      min_index = i;
      min_abs_diff = timestamp_diff;
    } else {
      break;
    }
  }
  return av_pose_cache[min_index];
}

bool IsObjectHistoryInFrontOfAv(
    const AvContext& av_context, const ObjectHistorySpan& object_hist,
    const VehicleGeometryParamsProto& vehicle_geometry_params) {
  constexpr double kLateralDistThres = 3.0;        // m.
  constexpr double kLongitudinalDistThres = 20.0;  // m.
  if (!av_context.GetAvPoseCache().full()) return false;
  for (const auto& [time, obj] : object_hist) {
    if (time < av_context.GetAvPoseCache()[0].timestamp()) continue;
    const auto av_pose_proto =
        FindPoseAtNearestTimestamp(av_context.GetAvPoseCache(), time);
    if (std::abs(NormalizeAngle(av_pose_proto.yaw() - av_pose_proto.yaw())) >
        M_PI_2) {
      return false;
    }
    const Vec2d av_tan = Vec2d::FastUnitFromAngle(av_pose_proto.yaw());
    const Vec2d av_pos(av_pose_proto.pos_smooth().x(),
                       av_pose_proto.pos_smooth().y());
    const Polygon2d& obj_contour =
        planner::ComputeObjectContour(obj.object_proto());
    Vec2d front_most, back_most;
    obj_contour.ExtremePoints(av_tan, &back_most, &front_most);
    const double back_dis = (back_most - av_pos).Dot(av_tan);
    const Vec2d av_to_obj = obj.bounding_box().center() - av_pos;
    const double l = av_to_obj.Dot(av_tan.Perp());
    const double dist_to_av =
        back_dis - vehicle_geometry_params.front_edge_to_center();
    if (std::abs(l) > kLateralDistThres || dist_to_av < 0.0 ||
        dist_to_av > kLongitudinalDistThres) {
      return false;
    }
  }
  return true;
}

}  // namespace

bool RecognizeSlowFrontVehicle(
    const AvContext& av_context,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const ObjectPredictionScenario& object_scenario,
    const ObjectHistory& object_history) {
  constexpr double kMaxAverageSpeedThres = 7.0;     // m/s.->25kph
  constexpr double kConsiderHistoryTime = 10.0;     // s.
  constexpr double kStopDurationThres = 5.0;        // s.
  constexpr double kLastMovingDurationThres = 3.0;  // s.
  constexpr double kMaxAccelThres = 1.0;            // s.
  constexpr double kMaxHistAccelThres = 0.2;        // m/s^2;
  constexpr double kMaxHistDecelThres = -0.5;       // m/s^2;

  if (object_history.type() != OT_VEHICLE ||
      object_scenario.road_status() != ObjectRoadStatus::ORS_ON_ROAD) {
    return false;
  }

  // Ensure that the historical cache of object is greater than 10 seconds.
  const auto& hist = object_history.GetHistory();
  if (hist.back().time - hist.front().time < kConsiderHistoryTime) {
    return false;
  }

  const auto& stop_time_info = object_history.GetStopTimeInfo();
  // If the stopping time is greater than 5 seconds.
  if (stop_time_info.time_duration_since_stop() > kStopDurationThres) {
    return false;
  }

  // If the vehicle has just started and the duration is less than 3 seconds,
  // and the last stopping time was greater than 5 seconds. We consider this to
  // be a normal starting scenario, not a traffic jam situation.
  if (stop_time_info.last_move_time_duration() < kLastMovingDurationThres &&
      stop_time_info.previous_stop_time_duration() > kStopDurationThres) {
    return false;
  }

  // Ignore object with current acceleration greater than 1.0.
  const double curr_accel = Vec2dFromProto(hist.object_proto().accel())
                                .Dot(Vec2d::FastUnitFromAngle(hist.heading()));
  if (curr_accel > kMaxAccelThres) return false;

  // Ignore the object that has not been in front of AV for a long time.
  const auto& consider_obj_hist =
      hist.GetHistoryFrom(hist.timestamp() - kConsiderHistoryTime);
  if (!IsObjectHistoryInFrontOfAv(av_context, consider_obj_hist,
                                  vehicle_geometry_params)) {
    return false;
  }

  // Ignore object with high average speed.
  double avg_speed = 0.0;
  for (const auto& [_, val] : consider_obj_hist) {
    avg_speed += val.v();
  }
  avg_speed /= consider_obj_hist.size();
  if (avg_speed > kMaxAverageSpeedThres) return false;

  double max_accel = std::numeric_limits<double>::lowest();
  double min_accel = std::numeric_limits<double>::max();
  for (const auto& [_, val] : consider_obj_hist) {
    const double accel = Vec2dFromProto(val.object_proto().accel())
                             .Dot(Vec2d::FastUnitFromAngle(val.heading()));
    max_accel = std::max(max_accel, accel);
    min_accel = std::min(min_accel, accel);
  }

  return max_accel > kMaxHistAccelThres && min_accel < -kMaxHistDecelThres;
}

}  // namespace prediction
}  // namespace qcraft
