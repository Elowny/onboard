#ifndef ONBOARD_NETS_TRT_CAPTAIN_NET_H_
#define ONBOARD_NETS_TRT_CAPTAIN_NET_H_

#include <memory>
#include <vector>

#include "onboard/planner/trajectory_point.h"

namespace qcraft::planner::ml::captain_net {

inline constexpr int kBatch = 1;          // Batch Size
inline constexpr int kModes = 1;          // Multimodal modes
inline constexpr int kPredictSec = 10;    // Duration seconds
inline constexpr double kTimeStep = 0.1;  // Step seconds
inline constexpr int kOutputPointDim = 5;
// NOLINTNEXTLINE
inline constexpr double kOutputAccScale =
    10;  // Divide output acc by this value.
// NOLINTNEXTLINE
inline constexpr double kOutputOmegaScale =
    3;  // Divide output acc by this value.

inline constexpr int kCoords = 2;
inline constexpr int kMaxObjectsNum = 32;  // Max other objects.
// NOLINTNEXTLINE
inline constexpr int kFeatureV2HistoryStepNum =
    21;  // Num for full history trajectory points.
inline constexpr int kHistoryNum = 10;    // Num for history trajectory points.
inline constexpr int kLanePointsNum = 5;  // Max points num in a lane.
inline constexpr int kMaxLaneCenterNum = 160;   // Max lanes center num.
inline constexpr int kMaxLaneBoundaryNum = 80;  // Max lanes boundary num.
inline constexpr int kMaxCrosswalkNum = 16;     // Max crosswalk num.
inline constexpr int kActorTypeDim = 16;
inline constexpr int kLaneTypeDim = 16;
inline constexpr int kMaxLightNum = 4;
inline constexpr int kMaxTargetLanesNum = 3;
inline constexpr int kMaxTargetLanePointNum = 20;
inline constexpr int kMaxTargetLaneSegNum = 5;

// scale
inline constexpr float kSpeedScale = 10.0f;
inline constexpr float kCoordScale = 100.0f;
// Inv scale, do not modify here
inline constexpr float kInvSpeedScale = 1.0f / kSpeedScale;
inline constexpr float kInvCoordScale = 1.0f / kCoordScale;

inline constexpr int kLaneControls = 2;  // Intersection and speed-limit.
inline constexpr double kLaneSearchRadius = 100.0;  // Attend radius in meters.
inline constexpr int kLaneLightsFeatNum = 4;  // Num for lane light feature.

inline constexpr int kMaxOtherObjsNum = 32;  // j5_add
inline constexpr int kLaneSegsNum = 5;
inline constexpr int kSegDim = 4;
inline constexpr int kFutureNum = 100;

// NOLINTNEXTLINE(readability-identifier-naming)
static std::vector<std::vector<int>> kInitInputDims{
    {kBatch, kCoords},                                     // ego_pos
    {kBatch, kCoords, kCoords},                            // rot_mat
    {kBatch, kMaxObjectsNum, kHistoryNum, kCoords},        // trajs
    {kBatch, kMaxObjectsNum, kHistoryNum},                 // speeds
    {kBatch, kMaxObjectsNum, kHistoryNum, kCoords},        // headings
    {kBatch, kMaxObjectsNum},                              // types
    {kBatch, kMaxObjectsNum, kCoords},                     // cur_poses
    {kBatch, kMaxLaneCenterNum, kLanePointsNum, kCoords},  // lane_centers
    {kBatch, kMaxObjectsNum + kMaxLaneCenterNum},          // masks
};

// NOLINTNEXTLINE(readability-identifier-naming)
static std::vector<std::vector<int>> kOutputDims{
    {kBatch, kMaxObjectsNum, kModes},  // prob_out
    {kBatch, kMaxObjectsNum, kModes, static_cast<int>(kPredictSec / kTimeStep),
     2},  // traj_out
};

struct AgentFeature {
  int agent_type;
  std::vector<float> agent_lw;
  std::vector<float> agent_pos;
  std::vector<float> agent_pos_diff;
  std::vector<float> agent_speed;
  std::vector<float> agent_heading;
};

struct ActorsFeature {
  std::vector<int64_t> obj_type;
  std::vector<float> obj_lw;
  std::vector<float> obj_pos;
  std::vector<float> obj_pos_diff;
  std::vector<float> obj_speed;
  std::vector<float> obj_heading;
  std::vector<float> rel_dist;
  std::vector<float> rel_pos;
  std::vector<float> rel_yaw;
  std::vector<float> rel_speed;
  std::vector<int> valid_mask;
};

struct LanesFeature {
  std::vector<float> seg;
  std::vector<float> unit_seg_vec;
  std::vector<float> seg_len;
  std::vector<float> unit_tangent;
  std::vector<float> dist;
  std::vector<float> unit_dist_vec;
  std::vector<float> nearest_to_end_dist;
  std::vector<float> speed_limits;
  std::vector<float> yaw_diff;
  std::vector<float> light;
  std::vector<int64_t> type;
  std::vector<int> valid_mask;
};

struct TargetLanesFeature {
  std::vector<std::vector<float>> target_lanes;
  std::vector<int> valid_mask;
  std::vector<int> flattened_mask;
};

struct CaptainNetFeature {
  AgentFeature agent_feature;
  ActorsFeature actors_feature;
  LanesFeature lane_center_feature;
  LanesFeature lane_boundary_feature;
  LanesFeature crosswalk_feature;
  TargetLanesFeature target_lane_feature;
};

class CaptainNet {
 public:
  virtual bool GetOutputs(
      const CaptainNetFeature& input_features,
      std::vector<std::vector<std::vector<float>>>* prob_out,
      std::vector<std::vector<std::vector<float>>>* traj_out) = 0;
  virtual void UnitTest() = 0;

  virtual ~CaptainNet() = default;
};

struct TrajectoryValidation {
  bool is_kinematic_infeasible = false;
  bool is_curb_collsion = false;
  bool is_object_collision = false;
  bool is_path_boundary_collision = false;
  bool is_reverse_driving = false;
  bool is_speed_limit_violation = false;
  // Ignore kinematic feasibility for ref validation check for now.
  bool IsValidAsRef() const {
    return !is_curb_collsion && !is_object_collision &&
           !is_path_boundary_collision && !is_reverse_driving &&
           !is_speed_limit_violation;
  }
  bool IsValidAsE2E() const {
    return !is_curb_collsion && !is_object_collision;
  }
};

struct CaptainNetOutput {
  std::vector<ApolloTrajectoryPointProto> original_traj_points;
  std::vector<ApolloTrajectoryPointProto> traj_points;
  std::vector<double> sdxs;
  std::vector<double> sdys;
  std::vector<double> covariances;
  double mode_prob = -1.0;
  bool is_post_processed = false;
  TrajectoryValidation validation;
};

}  // namespace qcraft::planner::ml::captain_net
#endif  // ONBOARD_NETS_TRT_CAPTAIN_NET_H_
