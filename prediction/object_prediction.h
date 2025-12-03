#ifndef ONBOARD_PREDICTION_OBJECT_PREDICTION_H_
#define ONBOARD_PREDICTION_OBJECT_PREDICTION_H_

#include <algorithm>
#include <numeric>
#include <string>
#include <vector>

#include "onboard/math/geometry/polygon2d.h"
#include "onboard/prediction/predicted_trajectory.h"
#include "onboard/prediction/proto/prediction_common.pb.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/proto/prediction.pb.h"

namespace qcraft {
namespace prediction {

struct ObjectLongTermBehavior {
  double avg_speed;
  double obs_duration = 0.0;
  std::vector<double> accel_history;
  bool is_slow_front_vehicle = false;

  void FromProto(const ObjectLongTermBehaviorProto& proto) {
    avg_speed = proto.average_speed();
    obs_duration = proto.observation_duration();

    accel_history.reserve(proto.accel_history_size());
    for (double accel : proto.accel_history()) {
      accel_history.push_back(accel);
    }
    is_slow_front_vehicle = proto.is_slow_front_vehicle();
  }

  void ToProto(ObjectLongTermBehaviorProto* proto) const {
    proto->set_average_speed(avg_speed);
    proto->set_observation_duration(obs_duration);

    auto* accel_hist = proto->mutable_accel_history();
    accel_hist->Reserve(accel_history.size());
    for (double accel : accel_history) accel_hist->Add(accel);
    proto->set_is_slow_front_vehicle(is_slow_front_vehicle);
  }
};

class ObjectPrediction {
 public:
  ObjectPrediction() = default;

  explicit ObjectPrediction(const ObjectProto& object_proto)
      : perception_object_(object_proto) {}

  ObjectPrediction(std::vector<PredictedTrajectory> trajectories,
                   const ObjectProto& object, ObjectRoadStatus road_status,
                   ObjectIntersectionStatus intersection_status);
  // Construct without semantic_map, no rebuilding lane_path.
  explicit ObjectPrediction(const ObjectPredictionProto& proto);

  // Construct from given object & prediction time shifts and a given perception
  // object.
  ObjectPrediction(const ObjectPredictionProto& proto,
                   double prediction_shift_time, const ObjectProto& object,
                   double object_shift_time);

  const ObjectProto& perception_object() const { return perception_object_; }

  double timestamp() const { return perception_object_.timestamp(); }

  const std::string& id() const { return perception_object_.id(); }
  const std::vector<PredictedTrajectory>& trajectories() const {
    return trajectories_;
  }
  double trajectory_prob_sum() const {
    return std::accumulate(trajectories_.begin(), trajectories_.end(), 0.0,
                           [](const double sum, const auto& traj) {
                             return sum + traj.probability();
                           });
  }
  double trajectory_max_prob() const {
    return std::accumulate(trajectories_.begin(), trajectories_.end(), 0.0,
                           [](const double max, const auto& traj) {
                             return std::max(max, traj.probability());
                           });
  }
  double trajectory_min_prob() const {
    return std::accumulate(trajectories_.begin(), trajectories_.end(), 1.0,
                           [](const double min, const auto& traj) {
                             return std::min(min, traj.probability());
                           });
  }

  const Polygon2d& contour() const { return contour_; }
  // Contour without time shift.
  const Polygon2d& raw_contour() const { return raw_contour_; }

  void set_id(std::string id) { perception_object_.set_id(id); }

  // Shift all PredictedTrajectory by assigned prediction_shift_time and sync
  // perception_object_ with a given object proto by assigned object_shift_time.
  // If all of the trajectories become empty after shifting, return False.
  bool ShiftTimeWithObjectProto(double prediction_shift_time,
                                double object_shift_time,
                                const ObjectProto& object);

  std::vector<PredictedTrajectory>* mutable_trajectories() {
    return &trajectories_;
  }

  void PrintDebugInfo() const;

  Polygon2d CreateContourForPoint(int predicted_trajectory_index,
                                  int predicted_point_index) const;
  const ObjectStopTimeProto& stop_time() const { return stop_time_; }
  const ObjectLongTermBehavior& long_term_behavior() const {
    return long_term_behavior_;
  }
  const ObjectRoadStatus& road_status() const { return road_status_; }
  const ObjectIntersectionStatus& intersection_status() {
    return intersection_status_;
  }

  // Serialization to proto.
  void FromProto(const ObjectPredictionProto& proto);
  void ToProto(ObjectPredictionProto* proto) const;
  void ToCompressedProto(ObjectPredictionProto* proto) const;

 private:
  void FromProto(const ObjectPredictionProto& proto,
                 double prediction_shift_time, const ObjectProto& object,
                 double object_shift_time);

  std::vector<PredictedTrajectory> trajectories_;
  Polygon2d contour_;
  Polygon2d raw_contour_;

  ObjectProto perception_object_;
  // stop time
  ObjectStopTimeProto stop_time_;

  // Some statictics for object's long-term behavior.
  ObjectLongTermBehavior long_term_behavior_;

  ObjectRoadStatus road_status_ = ObjectRoadStatus::ORS_NONE;

  ObjectIntersectionStatus intersection_status_ =
      ObjectIntersectionStatus::OIS_NONE;
};

using ObjectsPrediction = std::vector<ObjectPrediction>;

}  // namespace prediction
}  // namespace qcraft

#endif  // ONBOARD_PREDICTION_OBJECT_PREDICTION_H_
