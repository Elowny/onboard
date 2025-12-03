#ifndef ONBOARD_PLANNER_UTIL_HMI_CONTENT_UTIL_H_
#define ONBOARD_PLANNER_UTIL_HMI_CONTENT_UTIL_H_

#include <limits>
#include <optional>
#include <string>
#include <vector>

#include "onboard/maps/lane_path.h"
#include "onboard/math/vec.h"
#include "onboard/planner/common/path_sl_boundary.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/planner/scheduler/proto/lane_change.pb.h"
#include "onboard/proto/hmi_content.pb.h"
#include "onboard/proto/lane_change_type.pb.h"
#include "onboard/proto/trajectory_point.pb.h"

namespace qcraft {
namespace planner {

struct NudgeOjbectInfo {
  enum Type {
    kNormal = 0,
    kLargeVehicle = 1,
  };
  std::string id;
  int direction;
  double arc_dist_to_object;
  Type type;
};

// Export data from planner to HMI content.
struct HmiContentInput {
  const PlannerSemanticMapManager* psmm = nullptr;
  const mapping::LanePath* lane_path = nullptr;
  const DrivePassage* drive_passage = nullptr;
  const std::vector<ApolloTrajectoryPointProto>* traj_points = nullptr;
  const std::string* alerted_front_vehicle = nullptr;
  bool collision_warning_request = false;
  LaneChangeType lane_change_type;
  LaneChangeGeneralType lane_change_general_type;
  LaneChangeStage lane_change_stage;
  bool borrow_lane = false;
  bool request_help_lane_change_by_route = false;
  std::optional<double> distance_to_traffic_light_stop_line = std::nullopt;
  std::optional<double> distance_to_roadblock = std::nullopt;
  const std::vector<std::string>* unsafe_object_ids = nullptr;
  const mapping::LanePath* plc_target_lane_path = nullptr;
  const NudgeOjbectInfo* nudge_object_info = nullptr;
  std::optional<bool> all_trajectories_blocked = std::nullopt;
  bool lc_left = false;
  bool lane_change_for_obstacle_fail = false;
  double av_half_width = std::numeric_limits<double>::max();
};

HmiContentProto ReportHmiContent(const HmiContentInput& input);

HmiPathBoundaryProto ReportBoundaryPointsToHmiContent(
    const std::vector<Vec2d>& points, bool is_left,
    HmiPathBoundaryProto::BoundaryRenderStyle style);

HmiPathBoundaryProto ReportPathBoundaryToHmiContent(
    const PathSlBoundary* sl_boundary,
    HmiPathBoundaryProto::BoundaryRenderStyle left_style =
        HmiPathBoundaryProto::STYLE_NORMAL,
    HmiPathBoundaryProto::BoundaryRenderStyle right_style =
        HmiPathBoundaryProto::STYLE_NORMAL);
}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_UTIL_HMI_CONTENT_UTIL_H_
