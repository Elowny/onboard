#ifndef ONBOARD_PREDICTION_CONTAINER_TRAFFIC_LIGHT_MANAGER_H_
#define ONBOARD_PREDICTION_CONTAINER_TRAFFIC_LIGHT_MANAGER_H_

#include "absl/container/flat_hash_map.h"
#include "absl/hash/hash.h"

#include "onboard/maps/semantic_map_defs.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/proto/perception.pb.h"

namespace qcraft {
namespace prediction {
class TrafficLightManager {
 public:
  using TLStateHashMap =
      absl::flat_hash_map<mapping::ElementId, TrafficLightStateProto>;
  void UpdateTlStateMap(
      const planner::PlannerSemanticMapManager& semantic_map_manager,
      const TrafficLightStatesProto& tl_states);
  const TLStateHashMap& GetInferedTlStateMap() const {
    return inferred_tl_state_map_;
  }
  const TLStateHashMap& GetOriginalTlStateMap() const {
    return orig_tl_state_map_;
  }

 private:
  TLStateHashMap inferred_tl_state_map_;
  TLStateHashMap orig_tl_state_map_;
};
}  // namespace prediction
}  // namespace qcraft
#endif  // ONBOARD_PREDICTION_CONTAINER_TRAFFIC_LIGHT_MANAGER_H_
