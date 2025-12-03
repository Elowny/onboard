#include "onboard/prediction/container/traffic_light_manager.h"

#include <memory>
#include <ostream>

#include "onboard/container/strong_int.h"
#include "onboard/eval/qevent.h"
#include "onboard/eval/qevent_base.h"
#include "onboard/lite/logging.h"
#include "onboard/maps/proto/semantic_map.pb.h"
#include "onboard/utils/map_util.h"

namespace qcraft {
namespace prediction {
void TrafficLightManager::UpdateTlStateMap(
    const planner::PlannerSemanticMapManager& semantic_map_manager,
    const TrafficLightStatesProto& tl_states) {
  inferred_tl_state_map_.clear();
  orig_tl_state_map_.clear();
  /*
    Set the observable TL states.

    Note that, in the current setting, a TL may show up multiple times in the
    original TL state proto since it can be observed by multiple cameras at the
    same time.

    TODO(weixin):
     - Need to implement a principled TL state inference logic given multiple
    observations that the same time.
   */
  for (const auto& tl_state : tl_states.states()) {
    const mapping::ElementId id(tl_state.traffic_light_id());
    if (auto* ptr = FindOrNull(inferred_tl_state_map_, id);
        ptr != nullptr && ptr->color() != TrafficLightColor::TL_UNKNOWN) {
      continue;
    } else {
      inferred_tl_state_map_[id] = tl_state;
    }
  }

  // Inferring the hidden TL states via deterministic logic
  for (const auto& tl_state : tl_states.states()) {
    const mapping::ElementId tl_id(tl_state.traffic_light_id());
    const auto tl_color = tl_state.color();
    orig_tl_state_map_[tl_id].set_color(tl_color);
    if (tl_color == TrafficLightColor::TL_GREEN ||
        tl_color == TrafficLightColor::TL_YELLOW) {
      const auto* tl_ptr =
          semantic_map_manager.FindTrafficLightByIdOrNull(tl_id);
      if (tl_ptr == nullptr) {
        QLOG_EVERY_N_SEC(ERROR, 5.0)
            << "Cannot found, may load faild with map patch, "
               "traffic_light_id: "
            << tl_id;
        QEVENT_EVERY_N_SECONDS(
            "xiang", "prediction_tl_found_fail", 5, [&](QEvent* qevent) {
              qevent->AddField("traffic_light_id", tl_id.value());
            });
        continue;
      }
      const auto& tl = *tl_ptr;
      for (const auto id : tl.blocking_tl_ids()) {
        const mapping::ElementId blocking_tl_id(id);
        if (inferred_tl_state_map_.find(blocking_tl_id) !=
            inferred_tl_state_map_.end()) {
          const auto blocking_tl_color =
              inferred_tl_state_map_[blocking_tl_id].color();
          if (blocking_tl_color == TrafficLightColor::TL_GREEN ||
              blocking_tl_color == TrafficLightColor::TL_YELLOW) {
            QEVENT_EVERY_N_SECONDS(
                "runlin", "tl_state_conflict", /*seconds=*/1.0,
                ([tl_id = tl_id, tl_color, blocking_tl_id = blocking_tl_id,
                  blocking_tl_color = blocking_tl_color](QEvent* qevent) {
                  qevent->AddField("tl_id", tl_id.value())
                      .AddField("tl_color", TrafficLightColor_Name(tl_color))
                      .AddField("blocking_tl", blocking_tl_id.value())
                      .AddField("blocking_tl_color",
                                TrafficLightColor_Name(blocking_tl_color));
                }));
            continue;
          }
          inferred_tl_state_map_[blocking_tl_id].set_color(
              TrafficLightColor::TL_RED);
        } else {
          auto& blocking_tl_state = inferred_tl_state_map_[blocking_tl_id];
          blocking_tl_state.set_traffic_light_id(id);
          blocking_tl_state.set_camera_trigger_timestamp(
              tl_state.camera_trigger_timestamp());
          blocking_tl_state.set_color(TrafficLightColor::TL_RED);
          blocking_tl_state.set_score(1.0);
          blocking_tl_state.set_occluded(true);
          blocking_tl_state.set_occlusion_ratio(1.0);
          blocking_tl_state.set_state_source(TL_SS_DETERMINISTIC_LOGIC);
        }
      }
    }
  }
}

}  // namespace prediction
}  // namespace qcraft
