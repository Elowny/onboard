#ifndef ONBOARD_PLANNER_DECISION_TL_INFO_H_
#define ONBOARD_PLANNER_DECISION_TL_INFO_H_

#include <map>
#include <optional>
#include <string>
#include <vector>

#include "absl/container/flat_hash_map.h"

#include "onboard/maps/proto/semantic_map.pb.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/planner/decision/proto/traffic_light_info.pb.h"

namespace qcraft {
namespace planner {

struct SingleTlInfo {
  mapping::ElementId tl_id;
  TrafficLightState tl_state;
  double estimated_turn_red_time_left;
  std::optional<double> countdown;
};

class TlInfo {
 public:
  using Direction = mapping::LaneProto::Direction;
  TlInfo(mapping::ElementId lane_id,
         std::vector<double> control_point_relative_s, bool can_go_on_red,
         absl::flat_hash_map<TrafficLightDirection, SingleTlInfo> tls,
         bool is_fresh, std::string last_error_msg, Direction direction);

  mapping::ElementId lane_id() const { return lane_id_; }
  bool can_go_on_red() const { return can_go_on_red_; }

  const std::vector<double>& control_point_relative_s() const {
    return control_point_relative_s_;
  }

  const absl::flat_hash_map<TrafficLightDirection, SingleTlInfo>& tls() const {
    return tls_;
  }
  TrafficLightControlType tl_control_type() const { return tl_control_type_; }

  bool is_fresh() const { return is_fresh_; }
  const std::string& last_error_msg() const { return last_error_msg_; }
  bool empty() const { return is_empty_; }

  std::string DebugString() const;

  Direction direction() const { return direction_; }

  void ToProto(TrafficLightInfoProto* proto) const;

 private:
  // lane properties
  mapping::ElementId lane_id_;
  std::vector<double> control_point_relative_s_;
  bool can_go_on_red_;
  // associated tl properties
  absl::flat_hash_map<TrafficLightDirection, SingleTlInfo> tls_;
  TrafficLightControlType tl_control_type_ =
      TrafficLightControlType::SINGLE_DIRECTION;
  // freshness properties
  bool is_fresh_ = false;
  std::string last_error_msg_;
  Direction direction_ = mapping::LaneProto::STRAIGHT;
  bool is_empty_ = true;
};

using TrafficLightInfoMap = std::map<mapping::ElementId, TlInfo>;

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_DECISION_TL_INFO_H_
