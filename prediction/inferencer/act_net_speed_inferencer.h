#ifndef ONBOARD_PREDICTION_INFERENCER_ACT_NET_SPEED_INFERENCER_H_
#define ONBOARD_PREDICTION_INFERENCER_ACT_NET_SPEED_INFERENCER_H_

#include <algorithm>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "absl/strings/str_format.h"
#include "absl/strings/str_join.h"
#include "absl/types/span.h"

#include "onboard/nets/proto/net_param.pb.h"
#include "onboard/prediction/feature_extractor/act_net_feature.h"
#include "onboard/prediction/feature_extractor/act_net_speed_feature.h"
#include "onboard/prediction/inferencer/prediction_qnn.h"
#include "onboard/prediction/prediction_defs.h"

namespace qcraft {
namespace actnetspeed {

inline constexpr int kMinBatch = 1;
inline constexpr int kOptBatch = 8;
inline constexpr int kMaxBatch = 16;

inline constexpr int kOutCoords = 1;  // s(t).
inline constexpr int kPathNum = 2;
inline constexpr int kTrajectoryNum =
    prediction::kActNetSpeedConfig.mode_num;  // yield / pass
inline constexpr int kObjectTypeDim = 16;
inline constexpr int kLaneTypeDim = 16;
inline constexpr int kLaneLightDim = 4;
inline constexpr int kProbDim = 2;  // yield / pass.

inline constexpr int kPathPointNum =
    prediction::kActNetSpeedConfig.path_point_num;
inline constexpr int kPathPointDim =
    prediction::kActNetSpeedConfig.path_coord_num;
inline constexpr int kPathPointPreIntersectionNum =
    prediction::kActNetSpeedConfig.path_point_num_pre_intersection;
inline constexpr int kSegDim = 4;
inline constexpr int kLaneSegsNum = prediction::kActNetConfig.max_lane_seg_num;
inline constexpr int kCoords = prediction::kActNetSpeedConfig.coord_num;
inline constexpr int kMaxOtherObjsNum =
    prediction::kActNetSpeedConfig.max_other_objects_num;
inline constexpr int kMaxLaneCenterNum = prediction::kActNetConfig.max_lc_num;
inline constexpr int kMaxLaneBoundaryNum =
    prediction::kActNetConfig.max_solid_lb_num;
inline constexpr int kMaxCrossWalkNum =
    prediction::kActNetConfig.max_crosswalk_num;
inline constexpr int kHistoryNum = 10;  // to confirm
inline constexpr int kFutureNum = prediction::kActNetConfig.future_num;
struct SpeedTraj {
  std::vector<double> yield_speed;
  std::vector<double> pass_speed;
  std::string DebugString(bool yield) const {
    return yield ? absl::StrFormat("speed: %s, \n",
                                   absl::StrJoin(yield_speed, ","))
                 : absl::StrFormat("speed: %s, \n",
                                   absl::StrJoin(pass_speed, ","));
  }
};
struct SpeedTrajPair {
  int traj_index;  // Target trajectory of which the speed need modifying.
  prediction::AVObjectRelation relation_probs;
  SpeedTraj av_speeds;
  SpeedTraj agent_speeds;

  std::string RelationDebugString() const {
    return absl::StrFormat("Traj index: %d, relation: %s.", traj_index,
                           relation_probs.DebugString());
  }
  std::string AgentSpeedDebugString() const {
    return agent_speeds.DebugString(relation_probs.yield > relation_probs.pass);
  }
};
using SpeedTrajPairs = std::vector<SpeedTrajPair>;
using ObjectsSpeedTrajPairs = std::map<std::string, SpeedTrajPairs>;

class ActNetSpeedInferencer {
 public:
  explicit ActNetSpeedInferencer(const NetParam& net_param);

  ObjectsSpeedTrajPairs PredictForObjects(
      absl::Span<const prediction::ActNetSpeedFeature> input_features) const;

 private:
  const NetParam net_param_;
  std::unique_ptr<prediction::PredictionQNN> act_net_speed_;
};
}  // namespace actnetspeed
}  // namespace qcraft

#endif  // ONBOARD_PREDICTION_INFERENCER_ACT_NET_SPEED_INFERENCER_H_
