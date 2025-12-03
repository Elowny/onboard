#ifndef ONBOARD_PLANNER_SELECTOR_SELECTOR_UTIL_H_
#define ONBOARD_PLANNER_SELECTOR_SELECTOR_UTIL_H_

#include <algorithm>
#include <cmath>
#include <deque>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/status/statusor.h"
#include "absl/time/time.h"

#include "onboard/maps/semantic_map_defs.h"
#include "onboard/math/frenet_common.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/common/planner_status.h"
#include "onboard/planner/est_planner_output.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/planner/scheduler/proto/lane_change.pb.h"
#include "onboard/planner/scheduler/scheduler_output.h"
#include "onboard/planner/selector/common_feature.h"
#include "onboard/planner/selector/cost_feature_base.h"
#include "onboard/planner/selector/proto/selector_debug.pb.h"
#include "onboard/planner/selector/proto/selector_state.pb.h"
#include "onboard/planner/selector/selector_defs.h"
#include "onboard/planner/selector/selector_input.h"
#include "onboard/planner/selector/selector_output.h"
#include "onboard/planner/selector/selector_state.h"
#include "onboard/proto/lane_change_type.pb.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {
namespace planner {
using IndexedStationBoundary = std::pair<StationIndex, StationBoundary>;
using PlannerTrajectory = std::vector<ApolloTrajectoryPointProto>;

struct BoundaryInterval {
  std::vector<Vec2d> points;
  StationBoundaryType type;
};

inline double CalcPsi(const ApolloTrajectoryPointProto& prev_pt,
                      const ApolloTrajectoryPointProto& succ_pt) {
  return (succ_pt.path_point().kappa() - prev_pt.path_point().kappa()) /
         (succ_pt.relative_time() - prev_pt.relative_time());
}

inline double CalcLatJerk(const ApolloTrajectoryPointProto& pt, double psi) {
  // j_lat = 3 * v * a * kappa + v^2 * psi
  return std::abs(3.0 * pt.v() * pt.a() * pt.path_point().kappa() +
                  Sqr(pt.v()) * psi);
}

// Value range (0, 1], `base` for shape and `reg` for scaling along x axis.
inline double ExpDecayCoeffAtStep(double base, double reg, int i) {
  return std::pow(base, -i / (reg * kTrajectorySteps));
}

/**
 * Performs linear interpolation between two points (x0, t0) and (x1, t1)
 * to compute the interpolated value at the point t.
 */
double LinearInterpolate(double x0, double x1, double t0, double t1, double t);

inline double GetLatBoundaryToleranceError(double dist) {
  constexpr double kMaxLatToleranceError = 0.2;  // m.
  constexpr double kDistErrorRate = 0.005;
  return std::min(kMaxLatToleranceError, std::fabs(dist) * kDistErrorRate);
}

std::vector<double> MultiplyVector(const std::vector<double>& vec1,
                                   const std::vector<double>& vec2);

LaneChangeGeneralType ConvertLaneChangeTypeToGeneralType(
    LaneChangeType lane_change_type);

void UpdateSelectorStateBeforeSelection(absl::Time plan_time,
                                        SelectorState* selector_state);

bool IsSameTargetLane(const TargetLaneStateProto& prev,
                      const TargetLaneStateProto& curr);

void UpdateSelectorStateAfterSelection(
    absl::Time plan_time, const SelectorFlags& selector_flags,
    const std::vector<EstPlannerOutput>& results,
    const std::vector<PlannerStatus>& est_status,
    const absl::flat_hash_map<int, TrajFeatureOutput>&
        idx_traj_feature_output_map,
    int final_selected_idx, int best_traj_idx, bool is_paddle_lane_change,
    SelectorState* selector_state);

void UpdateSelectorOutput(const std::vector<EstPlannerOutput>& results,
                          const std::vector<PlannerStatus>& est_status,
                          const SelectorState& selector_state,
                          const absl::flat_hash_map<int, TrajFeatureOutput>&
                              idx_traj_feature_output_map,
                          bool in_high_way, int final_selected_idx,
                          int last_selected_idx,
                          SelectorOutput* selector_output);

void ProcessAutoLaneChangeRequest(
    absl::Time plan_time, int best_traj_idx,
    const SelectorFlags& selector_flags,
    const absl::flat_hash_map<int, TrajFeatureOutput>&
        idx_traj_feature_output_map,
    SelectorState* selector_state);

void UpdateTrajectoryCostForEachLane(
    const std::vector<PlannerStatus>& est_status,
    const std::vector<EstPlannerOutput>& results,
    const absl::flat_hash_map<int, FeatureCostSum>& all_trajectory_cost,
    absl::flat_hash_map<mapping::ElementId, FeatureCostSum>* lane_id_cost_map,
    absl::flat_hash_map<mapping::ElementId, int>* lane_id_idx_map,
    absl::flat_hash_map<int, int>* idx_selector_debug_map);

void UpdateRouteTtcSettingInHighway(
    std::optional<bool> alc_confirmation, int last_selected_idx,
    const SelectorFlags& selector_flags,
    const absl::flat_hash_map<int, TrajFeatureOutput>&
        idx_traj_feature_output_map,
    SelectorState* selector_state);

void HandleAlcConfirmation(absl::Time plan_time,
                           std::optional<bool> alc_confirmation,
                           SelectorState* selector_state);

int DecideBeginLaneChangeFrame(
    const SelectorFlags& selector_flags,
    const absl::flat_hash_map<int, TrajFeatureOutput>&
        idx_traj_feature_output_map);

std::vector<PlannerStatus> PreFilterEstResults(
    absl::Time plan_time, const PlannerSemanticMapManager& psmm,
    const SelectorFlags& selector_flags,
    const absl::flat_hash_set<std::string>& stalled_object_ids,
    const std::vector<PlannerStatus>& est_status,
    const std::vector<EstPlannerOutput>& results,
    const SelectorCommonFeature& common_feature, int last_selected_idx,
    SelectorDebugProto* selector_debug, SelectorState* selector_state);

bool IsInTlControlledIntersection(const PlannerSemanticMapManager& psmm,
                                  const DrivePassage& drive_passage, double s);

void AddBoundariesToIntervals(const DrivePassage& drive_passage,
                              std::vector<IndexedStationBoundary> boundaries,
                              std::vector<BoundaryInterval>* intervals);

std::vector<BoundaryInterval> FindSolidBoundaryIntervals(
    const DrivePassage& drive_passage, const FrenetCoordinate& first_point_sl,
    double cutoff_s);

double CalculateCrossingBoundary(
    const DrivePassage& drive_passage,
    const std::vector<BoundaryInterval>& solid_boundaries,
    const std::vector<Box2d>& ego_boxes, LaneChangeStage stage,
    const absl::flat_hash_set<StationBoundaryType>& type_set,
    const absl::StatusOr<double>& start_l_or,
    const absl::StatusOr<double>& end_l_or, const double ego_half_width);

absl::flat_hash_set<std::string> FindBlockObjectIds(
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const EstPlannerOutput& planner_output,
    const VehicleGeometryParamsProto& vehicle_geom);

LeaderInfo FindNearestLeader(
    const absl::flat_hash_set<std::string>& block_ids,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const absl::flat_hash_set<std::string>& stalled_objects,
    const EstPlannerOutput& planner_output);

/**
 * Find front non-block object ids in target lane.
 */
absl::flat_hash_set<std::string> FindFrontNonBlockObjectIds(
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const EstPlannerOutput& planner_output,
    const VehicleGeometryParamsProto& vehicle_geom,
    const ApolloTrajectoryPointProto& plan_start_point,
    const absl::flat_hash_set<std::string>& block_obj_ids);

LaneChangeType AnalyzeLaneChangeType(
    const absl::flat_hash_map<int, TrajFeatureOutput>&
        idx_traj_feature_output_map,
    bool is_paddle_lane_change, int final_chosen_idx,
    LaneChangeType last_lane_change_type);

int CalculateSuccessiveChooseCount(
    const std::deque<TargetLaneStateProto>& target_lane_states);

void SendAutoLaneChangeRequestEvent(
    const SelectorLaneChangeRequestProto& selector_lane_change_request);

bool IsPerformLaneChange(const LaneChangeStage& lc_stage);

TargetLaneStateProto GenerateTargetLaneState(
    const PlannerSemanticMapManager& psmm, const SchedulerOutput& output);

int FindLastSelectedTrjectory(const PlannerSemanticMapManager& psmm,
                              const std::vector<PlannerStatus>& est_status,
                              const std::vector<EstPlannerOutput>& results,
                              const SelectorState& selector_state);

int ChooseLaneKeepTrajDirectly(
    bool planner_is_l4_mode, absl::Time plan_time, int last_selected_idx,
    const SelectorState& selector_state, const SelectorFlags& selector_flags,
    const absl::flat_hash_map<mapping::ElementId, int>& lane_id_idx_map,
    const absl::flat_hash_map<mapping::ElementId, FeatureCostSum>&
        lane_id_cost_map,
    const absl::flat_hash_map<int, TrajFeatureOutput>&
        idx_traj_feature_output_map,
    const std::vector<EstPlannerOutput>& results);

void FillSelectorOutputToDebug(const SelectorOutput& selector_output,
                               SelectorDebugProto* selector_debug);
}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_SELECTOR_SELECTOR_UTIL_H_
