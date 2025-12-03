#ifndef ONBOARD_PREDICTION_CONTAINER_PREDICTION_INPUT_H_
#define ONBOARD_PREDICTION_CONTAINER_PREDICTION_INPUT_H_

#include <memory>

#include "absl/time/time.h"

#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/router/proto/route_manager_output.pb.h"
#include "onboard/prediction/container/av_context.h"
#include "onboard/prediction/container/objects_history.h"
#include "onboard/prediction/post_process/conflict_resolver_params.h"
#include "onboard/prediction/prediction_flags.h"
#include "onboard/proto/autonomy_state.pb.h"
#include "onboard/proto/localization.pb.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/proto/positioning.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft::prediction {

struct PredictionInput {
  explicit PredictionInput(int history_size, double history_len, double min_dt,
                           int long_term_history_size,
                           double long_term_history_len,
                           double long_term_min_dt) {
    if (FLAGS_prediction_use_tracker_history) {
      // In order to calculate the stop time, at least 2 frames of history are
      // re!quired.
      constexpr int kObjectHistoryCapacity = 2;
      objects_history = std::make_unique<ObjectsHistory>(kObjectHistoryCapacity,
                                                         history_len, min_dt);
    } else {
      objects_history =
          std::make_unique<ObjectsHistory>(history_size, history_len, min_dt);
    }
    av_context = std::make_unique<AvContext>(history_size, history_len);
    long_term_objects_history = std::make_unique<ObjectsHistory>(
        long_term_history_size, long_term_history_len, long_term_min_dt);
  }
  // Same as planner input.
  absl::Time prediction_init_time;
  std::shared_ptr<const ObjectsProto> real_objects;
  std::shared_ptr<const ObjectsProto> virtual_objects;
  std::shared_ptr<const PoseProto> pose;
  std::shared_ptr<const TrafficLightStatesProto> traffic_light_states;
  std::shared_ptr<const LocalizationTransformProto> localization_transform;
  std::shared_ptr<const AutonomyStateProto> autonomy_state;
  std::shared_ptr<const planner::RouteManagerOutputProto> route_manager_output;
  const planner::PlannerSemanticMapManager* semantic_map_manager = nullptr;
  const VehicleGeometryParamsProto* veh_geom_params;

  // cross-iteration [IN & OUT]
  std::unique_ptr<ObjectsHistory> objects_history;
  std::unique_ptr<AvContext> av_context;
  std::unique_ptr<ObjectsHistory> long_term_objects_history;

  // Conflict resolver config.
  const ConflictResolverParams* conflict_resolver_params = nullptr;
};

}  // namespace qcraft::prediction

#endif  // ONBOARD_PREDICTION_CONTAINER_PREDICTION_INPUT_H_
