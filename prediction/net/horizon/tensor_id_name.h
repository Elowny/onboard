#ifndef ONBOARD_PREDICTION_NET_HORIZON_TENSOR_ID_NAME_H_
#define ONBOARD_PREDICTION_NET_HORIZON_TENSOR_ID_NAME_H_

#include <string_view>
#include <unordered_map>

#include "onboard/utils/map_util.h"

namespace qcraft {
namespace prediction {

enum class J5QNNTensorId {
  kActorCtrs,
  kActorInfoAttr,
  kActorInfoHist,
  kAgentAttr,
  kAgentCtrs,
  kAgentInfo,
  kAgentInfoHist,
  kChannelProbs,
  kLanePathProb,
  kLbAttrInfo,
  kLbPos,
  kLbSegInfo,
  kLcAttrInfo,
  kLcPos,
  kLcSegInfo,
  kObjAttr,
  kObjCtrs,
  kObjInfo,
  kObjInfoHist,
  kStartupProb,
  kStopTimeInfo,
  kTrajProbs,
};

// Get the name of a given tensor id.
inline std::string_view J5QNNTensorIdToName(J5QNNTensorId tensor_id) {
  switch (tensor_id) {
    case J5QNNTensorId::kActorCtrs:
      return "actor_ctrs";
    case J5QNNTensorId::kActorInfoAttr:
      return "actor_info_attr";
    case J5QNNTensorId::kActorInfoHist:
      return "actor_info_hist";
    case J5QNNTensorId::kAgentAttr:
      return "agent_attr";
    case J5QNNTensorId::kAgentCtrs:
      return "agent_ctrs";
    case J5QNNTensorId::kAgentInfo:
      return "agent_info";
    case J5QNNTensorId::kAgentInfoHist:
      return "agent_info_hist";
    case J5QNNTensorId::kChannelProbs:
      return "channel_probs";
    case J5QNNTensorId::kLanePathProb:
      return "lane_path_prob";
    case J5QNNTensorId::kLbAttrInfo:
      return "lb_attr_info";
    case J5QNNTensorId::kLbPos:
      return "lb_pos";
    case J5QNNTensorId::kLbSegInfo:
      return "lb_seg_info";
    case J5QNNTensorId::kLcAttrInfo:
      return "lc_attr_info";
    case J5QNNTensorId::kLcPos:
      return "lc_pos";
    case J5QNNTensorId::kLcSegInfo:
      return "lc_seg_info";
    case J5QNNTensorId::kObjAttr:
      return "obj_attr";
    case J5QNNTensorId::kObjCtrs:
      return "obj_ctrs";
    case J5QNNTensorId::kObjInfo:
      return "obj_info";
    case J5QNNTensorId::kObjInfoHist:
      return "obj_info_hist";
    case J5QNNTensorId::kStartupProb:
      return "startup_prob";
    case J5QNNTensorId::kStopTimeInfo:
      return "stop_time_info";
    case J5QNNTensorId::kTrajProbs:
      return "traj_probs";
  }
}

// Get the tensor id of a tensor name.
inline J5QNNTensorId J5QNNTensorNameToId(std::string_view tensor_name) {
  static const std::unordered_map<std::string_view, J5QNNTensorId>*
      tensor_name_map =
          new std::unordered_map<std::string_view, J5QNNTensorId>({
              {"actor_ctrs", J5QNNTensorId::kActorCtrs},
              {"actor_info_attr", J5QNNTensorId::kActorInfoAttr},
              {"actor_info_hist", J5QNNTensorId::kActorInfoHist},
              {"agent_attr", J5QNNTensorId::kAgentAttr},
              {"agent_ctrs", J5QNNTensorId::kAgentCtrs},
              {"agent_info", J5QNNTensorId::kAgentInfo},
              {"agent_info_hist", J5QNNTensorId::kAgentInfoHist},
              {"channel_probs", J5QNNTensorId::kChannelProbs},
              {"lane_path_prob", J5QNNTensorId::kLanePathProb},
              {"lb_attr_info", J5QNNTensorId::kLbAttrInfo},
              {"lb_pos", J5QNNTensorId::kLbPos},
              {"lb_seg_info", J5QNNTensorId::kLbSegInfo},
              {"lc_attr_info", J5QNNTensorId::kLcAttrInfo},
              {"lc_pos", J5QNNTensorId::kLcPos},
              {"lc_seg_info", J5QNNTensorId::kLcSegInfo},
              {"obj_attr", J5QNNTensorId::kObjAttr},
              {"obj_ctrs", J5QNNTensorId::kObjCtrs},
              {"obj_info", J5QNNTensorId::kObjInfo},
              {"obj_info_hist", J5QNNTensorId::kObjInfoHist},
              {"startup_prob", J5QNNTensorId::kStartupProb},
              {"stop_time_info", J5QNNTensorId::kStopTimeInfo},
              {"traj_probs", J5QNNTensorId::kTrajProbs},
          });

  return FindOrDie(*tensor_name_map, tensor_name);
}

}  // namespace prediction
}  // namespace qcraft

#endif  // ONBOARD_PREDICTION_NET_HORIZON_TENSOR_ID_NAME_H_
