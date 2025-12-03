#ifndef ONBOARD_PLANNER_UTIL_LANE_PATH_UTIL_H_
#define ONBOARD_PLANNER_UTIL_LANE_PATH_UTIL_H_

#include <functional>
#include <vector>

#include "absl/status/statusor.h"
#include "absl/types/span.h"

#include "onboard/maps/lane_path.h"
#include "onboard/maps/lane_path_data.h"
#include "onboard/maps/lane_point.h"
#include "onboard/maps/maps_common.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/math/vec.h"
#include "onboard/planner/common/driving_map_topo.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/proto/positioning.pb.h"

namespace qcraft::planner {

absl::StatusOr<mapping::LanePath> BuildLanePathFromData(
    const mapping::LanePathData& data, const PlannerSemanticMapManager& psmm);

bool IsLanePathConnectedTo(const mapping::LanePath& lane_path,
                           const mapping::LanePath& other,
                           const PlannerSemanticMapManager& psmm,
                           double distance_threshold = 0.01);

absl::StatusOr<mapping::LanePath> ConnectLanePath(
    const mapping::LanePath& lane_path, const mapping::LanePath& other,
    const PlannerSemanticMapManager& psmm, double distance_threshold = 0.01);

mapping::LanePath BackwardExtendTargetAlignedRouteLanePath(
    const PlannerSemanticMapManager& psmm, bool left,
    const mapping::LanePoint& start_point, const mapping::LanePath& target);

mapping::LanePath BackwardExtendLanePath(
    const PlannerSemanticMapManager& psmm,
    const mapping::LanePath& raw_lane_path, double extend_len,
    const std::function<bool(const mapping::LaneInfo&)>*
        nullable_should_stop_and_avoid_extend = nullptr);

absl::StatusOr<mapping::LanePath> ForwardExtendLanePathWithMinimumHeadingDiff(
    const PlannerSemanticMapManager& psmm,
    const mapping::LanePath& raw_lane_path, double extend_len,
    bool allow_virtual);

mapping::LanePath ForwardExtendLanePathWithoutFork(
    const PlannerSemanticMapManager& psmm,
    const mapping::LanePath& raw_lane_path, double extend_len);

absl::StatusOr<mapping::LanePath> FindNearestLanePathFromEgoPose(
    const PoseProto& pose, const PlannerSemanticMapManager& psmm,
    double required_min_length);

Vec2d ArclengthToPos(const PlannerSemanticMapManager& psmm,
                     const mapping::LanePath& lane_path, double s);

double ArclengthToLerpTheta(const PlannerSemanticMapManager& psmm,
                            const mapping::LanePath& lane_path, double s);

// TODO(zuowei): Merge with ArclengthToLerpTheta.
double LaneIndexPointToLerpTheta(
    const PlannerSemanticMapManager& psmm, const mapping::LanePath& lane_path,
    const mapping::LanePath::LaneIndexPoint& lane_index_point);

/**
 * @brief return all lanes that found before immediately when not found the lane
 * in the lane_path
 */
std::vector<const mapping::LaneInfo*> GetLanesInfoBreakIfNotFound(
    const PlannerSemanticMapManager& psmm, const mapping::LanePath& lane_path);

std::vector<const mapping::LaneInfo*> GetLanesInfoContinueIfNotFound(
    const PlannerSemanticMapManager& psmm, const mapping::LanePath& lane_path);

absl::StatusOr<mapping::LanePath> TrimTrailingNotFoundLanes(
    const PlannerSemanticMapManager& psmm, const mapping::LanePath& lane_path);

mapping::LanePath ForwardExtendLanePath(const PlannerSemanticMapManager& psmm,
                                        const mapping::LanePath& raw_lane_path,
                                        double extend_len);

std::vector<mapping::LanePath> CollectAllLanePathFromStartLane(
    const PlannerSemanticMapManager& psmm, const mapping::LanePath& start_lane,
    double max_search_len);

absl::StatusOr<std::vector<Vec2d>> SampleLanePathByStep(
    const PlannerSemanticMapManager& psmm, const mapping::LanePath& lane_path,
    double step);

absl::StatusOr<std::vector<mapping::LanePath>> FindNearLanePathsFromEgoPose(
    const PlannerSemanticMapManager& psmm, const Vec2d& pos, double heading,
    double required_min_length, double heading_penalty_weight,
    double distance_threshold, double angle_error_threshold);

std::vector<mapping::LanePath> BuildLanePathsFromDrivingMapTopo(
    const DrivingMapTopo& dm, const PlannerSemanticMapManager& psmm);

std::vector<std::vector<mapping::ElementId>>
FindLanePathSequencesFromStartIdInDrivingMapTopo(const DrivingMapTopo& dm,
                                                 mapping::ElementId start_id);

std::vector<mapping::LanePath>
BuildAllLanePathsFromStartIdInDrivingMapTopoWithDesireLength(
    const DrivingMapTopo& dm, const PlannerSemanticMapManager& psmm,
    mapping::ElementId start_id, double desire_length, double min_length);

absl::StatusOr<mapping::LanePath> BuildLanePathFromLaneIdSeqInDrivingMap(
    const DrivingMapTopo& dm, absl::Span<const mapping::ElementId> seq,
    const PlannerSemanticMapManager& psmm, double desire_length,
    double min_length);

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_UTIL_LANE_PATH_UTIL_H_
