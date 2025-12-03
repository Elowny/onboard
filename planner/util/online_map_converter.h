#ifndef ONBOARD_PLANNER_UTIL_ONLINE_MAP_CONVERTER_H_
#define ONBOARD_PLANNER_UTIL_ONLINE_MAP_CONVERTER_H_

#include "absl/status/statusor.h"

#include "onboard/maps/proto/online_semantic_map.pb.h"
#include "onboard/planner/planner_semantic_map_manager.h"

namespace qcraft::planner {

struct OnlineSemanticMapConverterOption {
  double timestamp_s;
  double smooth_x;
  double smooth_y;
  double smooth_yaw;
  double look_ahead_distance;
  double look_back_distance;
  double lane_sample_interval = 1.0;      // Recommend value: 1.0 meters.
  double boundary_sample_interval = 1.0;  // Recommend value: 1.0 meters.
  double perception_lateral_range = 12.5;
};

absl::StatusOr<mapping::OnlineSemanticMapProto> RunOnlineSemanticMapConverter(
    const PlannerSemanticMapManager& psmm,
    const OnlineSemanticMapConverterOption& option);

absl::StatusOr<mapping::OnlineSemanticMapProto>
RunOnlineSemanticMapPredictionConverter(
    const PlannerSemanticMapManager& psmm,
    const OnlineSemanticMapConverterOption& option);
}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_UTIL_ONLINE_MAP_CONVERTER_H_
