#ifndef ONBOARD_PLANNER_SCENE_TRAFFIC_FLOW_REASONING_H_
#define ONBOARD_PLANNER_SCENE_TRAFFIC_FLOW_REASONING_H_

#include <vector>

#include "absl/status/statusor.h"

#include "onboard/async/thread_pool.h"
#include "onboard/maps/lane_path.h"
#include "onboard/planner/decision/traffic_light/tl_info.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/scene/scene_understanding_output.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/proto/prediction.pb.h"

namespace qcraft::planner {

struct TrafficFlowReasoningInput {
  const PlannerSemanticMapManager* psmm = nullptr;
  const ObjectsPredictionProto* prediction = nullptr;
  const std::vector<mapping::LanePath>* lane_paths = nullptr;
  const TrafficLightInfoMap* tl_info_map = nullptr;
  const SensorFovsProto* sensor_fovs = nullptr;
  const ApolloTrajectoryPointProto* plan_start_point = nullptr;
};

absl::StatusOr<TrafficFlowReasoningOutput> RunTrafficFlowReasoning(
    const TrafficFlowReasoningInput& input, ThreadPool* thread_pool);
}  // namespace qcraft::planner

#endif
