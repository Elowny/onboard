#include "onboard/prediction/object_prediction.h"

// IWYU pragma: no_include <google/protobuf/repeated_ptr_field.h>
#include <optional>
#include <utility>

#include "absl/status/status.h"
#include "absl/strings/str_format.h"

#include "onboard/lite/check.h"
#include "onboard/lite/logging.h"
#include "onboard/prediction/util/perception_util.h"

namespace qcraft {
namespace prediction {

namespace {

Polygon2d GetPolygonFromPerception(const ObjectProto& object) {
  std::optional<Polygon2d> maybe_contour =
      Polygon2d::FromPoints(object.contour(),
                            /*is_convex=*/true);
  QCHECK(maybe_contour.has_value());
  return std::move(maybe_contour.value());
}

}  // namespace

ObjectPrediction::ObjectPrediction(
    std::vector<PredictedTrajectory> trajectories, const ObjectProto& object,
    ObjectRoadStatus road_status, ObjectIntersectionStatus intersection_status)
    : trajectories_(std::move(trajectories)),
      contour_(GetPolygonFromPerception(object)),
      raw_contour_(GetPolygonFromPerception(object)),
      perception_object_(object),
      road_status_(road_status),
      intersection_status_(intersection_status) {}

ObjectPrediction::ObjectPrediction(const ObjectPredictionProto& proto,
                                   double prediction_shift_time,
                                   const ObjectProto& object,
                                   double object_shift_time) {
  raw_contour_ = GetPolygonFromPerception(object);
  FromProto(proto, prediction_shift_time, object, object_shift_time);
  contour_ = GetPolygonFromPerception(object);
}

ObjectPrediction::ObjectPrediction(const ObjectPredictionProto& proto)
    : ObjectPrediction(proto, /*prediction_shift_time=*/0.0,
                       proto.perception_object(),
                       /*object_shift_time=*/0.0) {}

void ObjectPrediction::PrintDebugInfo() const {
  QLOG(INFO) << absl::StrFormat(
      "object %s (%7.3f) prediction result (total prob = %6.5f, max prob = "
      "%7.6f, min prob = %7.6f):",
      perception_object_.id(), timestamp(), trajectory_prob_sum(),
      trajectory_max_prob(), trajectory_min_prob());
  for (int i = 0; i < trajectories_.size(); ++i) {
    QLOG(INFO) << absl::StrFormat("\ttraj %d / %d:", i + 1,
                                  trajectories_.size());
    trajectories_.at(i).PrintDebugInfo();
  }
}

Polygon2d ObjectPrediction::CreateContourForPoint(
    int predicted_trajectory_index, int predicted_point_index) const {
  return trajectories()[predicted_trajectory_index].CreateContourForPoint(
      contour(), predicted_point_index);
}

void ObjectPrediction::FromProto(const ObjectPredictionProto& proto) {
  FromProto(proto, /*prediction_shift_time=*/0.0, proto.perception_object(),
            /*object_shift_time=*/0.0);
}

// TODO(lidong): Change to a builder function as it may construct a prediction
// with empty trajectory.
void ObjectPrediction::FromProto(const ObjectPredictionProto& proto,
                                 double prediction_shift_time,
                                 const ObjectProto& object,
                                 double object_shift_time) {
  perception_object_ = object;
  constexpr double kTimeShiftThreshold = 1e-6;
  if (object_shift_time > kTimeShiftThreshold) {
    AlignPerceptionObjectTime(object_shift_time + object.timestamp(),
                              &perception_object_)
        .IgnoreError();
  }
  // trajectories
  trajectories_.reserve(proto.trajectories_size());
  for (const auto& trajectory_proto : proto.trajectories()) {
    trajectories_.emplace_back(prediction_shift_time, trajectory_proto);
    if (trajectories_.back().points().empty()) {
      trajectories_.pop_back();
    }
  }

  // stop time
  stop_time_ = proto.stop_time();

  // Long-term behavior.
  long_term_behavior_.FromProto(proto.long_term_behavior());

  road_status_ = proto.road_status();

  intersection_status_ = proto.intersection_status();
}

void ObjectPrediction::ToProto(ObjectPredictionProto* proto) const {
  proto->Clear();
  proto->set_id(perception_object_.id());
  // perception object
  proto->mutable_perception_object()->CopyFrom(perception_object_);

  // trajectories
  for (const auto& trajectory : trajectories_) {
    trajectory.ToProto(proto->add_trajectories(), /*compress_traj=*/false);
  }

  proto->mutable_stop_time()->CopyFrom(stop_time_);

  // Long-term behavior.
  long_term_behavior_.ToProto(proto->mutable_long_term_behavior());

  proto->set_road_status(road_status_);

  proto->set_intersection_status(intersection_status_);
}

void ObjectPrediction::ToCompressedProto(ObjectPredictionProto* proto) const {
  proto->Clear();
  proto->set_id(perception_object_.id());
  // perception object
  proto->mutable_perception_object()->CopyFrom(perception_object_);

  // trajectories
  for (const auto& trajectory : trajectories_) {
    trajectory.ToProto(proto->add_trajectories(), /*compress_traj=*/true);
  }

  proto->mutable_stop_time()->CopyFrom(stop_time_);

  // Long-term behavior.
  long_term_behavior_.ToProto(proto->mutable_long_term_behavior());

  proto->set_road_status(road_status_);

  proto->set_intersection_status(intersection_status_);
}

bool ObjectPrediction::ShiftTimeWithObjectProto(double prediction_shift_time,
                                                double object_shift_time,
                                                const ObjectProto& object) {
  perception_object_ = object;
  raw_contour_ = GetPolygonFromPerception(object);
  constexpr double kTimeShiftThreshold = 1e-6;
  if (object_shift_time > kTimeShiftThreshold) {
    AlignPerceptionObjectTime(object_shift_time + object.timestamp(),
                              &perception_object_)
        .IgnoreError();
  }
  contour_ = GetPolygonFromPerception(perception_object_);
  trajectories_.erase(
      std::remove_if(trajectories_.begin(), trajectories_.end(),
                     [&prediction_shift_time](PredictedTrajectory& traj) {
                       return !traj.shift_by_time(prediction_shift_time);
                     }),
      trajectories_.end());
  if (trajectories_.empty()) {
    return false;
  }
  return true;
}

}  // namespace prediction
}  // namespace qcraft
