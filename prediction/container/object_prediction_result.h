#ifndef ONBOARD_PREDICTION_CONTAINER_OBJECT_PREDICTION_RESULT_H_
#define ONBOARD_PREDICTION_CONTAINER_OBJECT_PREDICTION_RESULT_H_

// IWYU pragma: no_include <google/protobuf/repeated_ptr_field.h>

#include <optional>
#include <string>
#include <vector>

#include "onboard/prediction/container/object_history.h"
#include "onboard/prediction/predicted_trajectory.h"
#include "onboard/prediction/prediction_defs.h"
#include "onboard/prediction/proto/prediction_common.pb.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/proto/prediction.pb.h"

namespace qcraft {
namespace prediction {
struct ObjectPredictionResult {
  ObjectIDType id;                    // object id.
  ObjectPredictionPriority priority;  // Priority of object.
  std::string priority_annotation;    // Priority annotation of object.
  ObjectPredictionScenario scenario;  // Scenario of object.
  ObjectProto perception_object;      // Perception object.
  StopTimeInfo stop_time_info;        // How long did this object stop.
  ObjectLongTermBehaviorProto
      long_term_behavior;  // Long term behavior of objects.
  std::vector<PredictedTrajectory> trajectories;  // predicted trajectories.
  std::vector<PredictedTrajectory>
      startup_trajs;  // predicted startup trajectories.
  std::optional<double> startup_prob =
      std::nullopt;  // Startup probability for the static vehicle in 3s.

  inline void ToCompressedProto(ObjectPredictionProto* obj_pred) const {
    obj_pred->set_id(id);
    for (const auto& traj : trajectories) {
      traj.ToProto(obj_pred->add_trajectories(), /*compress_traj=*/true);
    }
    for (const auto& traj : startup_trajs) {
      traj.ToProto(obj_pred->add_startup_trajs(), /*compress_traj=*/true);
    }
    auto& stop_time = *obj_pred->mutable_stop_time();
    stop_time.set_time_duration_since_stop(
        stop_time_info.time_duration_since_stop());
    stop_time.set_previous_stop_time_duration(
        stop_time_info.previous_stop_time_duration());
    stop_time.set_last_move_time_duration(
        stop_time_info.last_move_time_duration());
    *obj_pred->mutable_perception_object() = perception_object;
    // Do not keep the tracker history to reduce message size.
    obj_pred->mutable_perception_object()->mutable_trajectory()->Clear();
    *obj_pred->mutable_long_term_behavior() = long_term_behavior;
    obj_pred->set_road_status(scenario.road_status());
    obj_pred->set_intersection_status(scenario.intersection_status());
    obj_pred->set_priority(priority);
    obj_pred->set_priority_annotation(priority_annotation);
    if (startup_prob.has_value()) {
      obj_pred->set_static_to_moving_prob(*startup_prob);
    }
  }

  inline std::vector<PredictedTrajectory>* mutable_trajectories() {
    return &trajectories;
  }
};

}  // namespace prediction
}  // namespace qcraft

#endif  // ONBOARD_PREDICTION_CONTAINER_OBJECT_PREDICTION_RESULT_H_
