#ifndef ONBOARD_PLANNER_DECISION_TRAFFIC_GAP_RESULT_H_
#define ONBOARD_PLANNER_DECISION_TRAFFIC_GAP_RESULT_H_

#include <optional>
#include <string>

#include "onboard/planner/decision/traffic_gap_v2/proto/traffic_gap.pb.h"

namespace qcraft::planner {

struct TrafficGapResult {
  std::optional<std::string> leader_id = std::nullopt;
  std::optional<std::string> follower_id = std::nullopt;

  void ToProto(TrafficGapProto* proto) {
    proto->set_leader_id(leader_id.has_value() ? *leader_id : "");
    proto->set_follower_id(follower_id.has_value() ? *follower_id : "");
  }
};

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_DECISION_TRAFFIC_GAP_RESULT_H_
