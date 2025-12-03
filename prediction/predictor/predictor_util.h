#ifndef ONBOARD_PREDICTION_PREDICTOR_PREDICTOR_UTIL_H_
#define ONBOARD_PREDICTION_PREDICTOR_PREDICTOR_UTIL_H_

#include <algorithm>
#include <map>
#include <memory>
#include <numeric>
#include <unordered_map>
#include <vector>

#include "absl/status/statusor.h"
#include "absl/types/span.h"

#include "onboard/math/geometry/box2d.h"
#include "onboard/math/vec.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/prediction/container/object_history.h"
#include "onboard/prediction/container/object_history_span.h"
#include "onboard/prediction/feature_extractor/object_history_sampler.h"
#include "onboard/prediction/predicted_trajectory.h"
#include "onboard/prediction/prediction_defs.h"
#include "onboard/proto/perception/fusion/object.pb.h"  // for ObjectType

namespace qcraft {
namespace prediction {

std::vector<const ObjectHistory*> ScreenPredictObjectsByDistance(
    const Box2d& ego_box,
    absl::Span<const ObjectHistory* const> objs_to_predict,
    int max_objects_num);

double GetCurrentTimeStamp(double av_latest_timestamp,
                           absl::Span<const ObjectHistory* const> objs);
std::vector<const ObjectHistory*> SelectPredictedObjectsByTypePriority(
    const Box2d& ego_box, const int max_objects_num,
    absl::Span<const ObjectHistory* const> objs_to_predict,
    const TypePrioMap& type_prio_map,
    const std::map<PredictTypePrio, int>& type_max_num_map);

std::vector<PredictedTrajectoryPoint> Vec2dPointsToPredTrajPoints(
    absl::Span<const Vec2d> vec2d_points, absl::Span<const double> vec_t,
    int traj_points_size, bool use_pos_fitter, int polyfit_downsample_step);

std::vector<PredictedTrajectoryPoint> PostProcessModelOutputTrajPts(
    std::vector<PredictedTrajectoryPoint> raw_traj_pts,
    const ObjectMotionState& obj_state, double time_step, double duration,
    double perception_acc, bool rectify_speed);

// Hack (xiangjun): this is a hack util to filter out all objects in the range.
inline bool IsObjectInRange(const Vec2d& obj_pos, const Vec2d& center,
                            double radius) {
  return obj_pos.DistanceSquareTo(center) < radius * radius;
}

inline bool IsBicycleModelLike(ObjectType obj_type) {
  switch (obj_type) {
    case OT_VEHICLE:
    case OT_LARGE_VEHICLE:
    case OT_CYCLIST:
    case OT_TRICYCLIST:
    case OT_MOTORCYCLIST:
    case OT_UNKNOWN_MOVABLE:
      return true;
    case OT_UNKNOWN_STATIC:
    case OT_FOD:
    case OT_VEGETATION:
    case OT_BARRIER:
    case OT_BARRIER_ANTI_COLLISION_BUCKET:
    case OT_BARRIER_ANTI_COLLISION_POST:
    case OT_CONE:
    case OT_WARNING_TRIANGLE:
    case OT_PEDESTRIAN:
      return false;
  }
}

inline void NormalizeAndDescSortTrajProbs(
    absl::Span<PredictedTrajectory> trajs) {
  const double sum_prob = std::accumulate(trajs.begin(), trajs.end(), 0.0,
                                          [](double prob, const auto& traj2) {
                                            return prob + traj2.probability();
                                          });
  if (sum_prob <= 0.0) {
    return;
  }
  for (auto& traj : trajs) {
    traj.set_probability(traj.probability() / sum_prob);
  }
  std::sort(trajs.begin(), trajs.end(),
            [](const auto& traj1, const auto& traj2) {
              return traj1.probability() > traj2.probability();
            });
}
std::vector<AgentCentricObjectProbTraj> FilterTrajsByProb(
    absl::Span<const AgentCentricObjectProbTraj> prob_trajs,
    double prob_threshold);

absl::StatusOr<std::vector<PredictedTrajectoryPoint>>
GeneratePredictedTrajectoryPoints(const ObjectMotionState& cur_state,
                                  const planner::DrivePassage& dp, double accel,
                                  double pred_horizon, double target_l);
enum class ParallelRiskType {
  kNone = 0,
  kNormal = 1,
  kNearLargeVehicle = 2,
  kNearNormalVehicle = 3,
};

std::unordered_map<ObjectIDType, ParallelRiskType>
SelectParallelHighRiskObjects(
    const ObjectHistorySpan& av_history_span,
    absl::Span<const ObjectHistory* const> objs_to_predict,
    const planner::DrivePassage& drive_passage,
    const ObjectHistorySampler& obj_sampler);

std::vector<ObjectIDType> SelectCutinSLNetPredictedObjects(
    const ObjectHistorySpan& av_history_span, int max_objects_num,
    absl::Span<const ObjectHistory* const> objs_to_predict,
    const planner::DrivePassage& drive_passage,
    const ObjectHistorySampler& obj_sampler);

const planner::DrivePassage* FindBestMatchingDrivePassage(
    absl::Span<const std::unique_ptr<planner::DrivePassage>> dps,
    const Vec2d& pos, const double heading);

std::vector<const planner::DrivePassage*> FindNearbyDrivePassages(
    absl::Span<const std::unique_ptr<planner::DrivePassage>> dps,
    const Vec2d& pos, const double heading, double max_offset,
    double max_heading_diff, int max_num);

std::vector<const planner::DrivePassage*> FilterDrivePassagesByCTRA(
    absl::Span<const planner::DrivePassage* const> dps,
    const ObjectMotionState& cur_state, int max_num);

}  // namespace prediction
}  // namespace qcraft
#endif  // ONBOARD_PREDICTION_PREDICTOR_PREDICTOR_UTIL_H_
