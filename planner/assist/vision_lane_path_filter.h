#ifndef ONBOARD_PLANNER_ASSIST_VISION_LANE_PATH_FILTER_H_
#define ONBOARD_PLANNER_ASSIST_VISION_LANE_PATH_FILTER_H_

#include <string>

#include "absl/status/statusor.h"

#include "onboard/async/thread_pool.h"
#include "onboard/maps/lane_path.h"
#include "onboard/maps/proto/online_semantic_map.pb.h"
#include "onboard/math/frenet_frame.h"
#include "onboard/math/vec.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/proto/positioning.pb.h"

namespace qcraft::planner {

absl::StatusOr<mapping::LanePath> ProjectLaneFrenetFrameCurrentOnlineMap(
    const PlannerSemanticMapManager& psmm,
    const mapping::OnlineSemanticMapProto& curr_online_map,
    const KdTreeFrenetFrame& prev_frenet_frame, const Vec2d& ego_pos,
    const std::string& prev_lane_string, double ego_v,
    double check_preview_length, ThreadPool* thread_pool);

absl::StatusOr<mapping::LanePath> ProjectLanePathToCurrentOnlineMap(
    const PlannerSemanticMapManager& psmm,
    const mapping::OnlineSemanticMapProto& curr_online_map,
    const PlannerSemanticMapManager& prev_psmm,
    const mapping::LanePath& prev_lane_path, const Vec2d& ego_pos, double ego_v,
    double check_preview_length, ThreadPool* thread_pool);

absl::StatusOr<mapping::LanePath> FindInitialLanePathFromEgoPose(
    const PlannerSemanticMapManager& psmm, const PoseProto& pose,
    ThreadPool* thread_pool);

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_ASSIST_VISION_LANE_PATH_FILTER_H_
