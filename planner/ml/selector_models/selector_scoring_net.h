#ifndef ONBOARD_PLANNER_ML_SELECTOR_MODELS_SELECTOR_SCORING_NET_H_
#define ONBOARD_PLANNER_ML_SELECTOR_MODELS_SELECTOR_SCORING_NET_H_

#include <memory>
#include <vector>

namespace qcraft {

namespace selector_scoring_net {

inline constexpr int kBatch = 1;  // Batch Size
inline constexpr int kModes = 3;  // Multimodal modes.
inline constexpr int kCoords = 2;
inline constexpr int kMaxObjectsNum = 33;  // Max objects in the scene.
inline constexpr int kHistoryNum = 21;     // Num for history trajectory points.
inline constexpr int kFutureNum = 100;     // Num for future trajectory points.
inline constexpr int kMaxLanesNum = 160;   // Max lanes num.
inline constexpr int kMaxBoundsNum = 80;   // Max lane boundary num.
inline constexpr int kMaxCWNum = 16;       // Max crosswalk num.
inline constexpr int kLanePointsNum = 6;   // Max points num in a lane.
inline constexpr int kLaneLightsNum = 4;   // Type num of traffic light.
inline constexpr int kLaneTypesNum = 10;   // Type num of lane.
inline constexpr int kBoundTypesNum = 6;   // Type num of bound.
inline constexpr int kTargetLanePointsNum =
    120;  // Max points num in a target lane.

// TODO(Jinyun): update it when onnx or trt is ready.
// NOLINTNEXTLINE(readability-identifier-naming)
static std::vector<std::vector<int>> kInitInputDims{
    {kBatch, kCoords},                                // ego_pos
    {kBatch, kCoords, kCoords},                       // rot_mat
    {kBatch, kMaxObjectsNum, kHistoryNum, kCoords},   // trajs
    {kBatch, kMaxObjectsNum, kHistoryNum},            // speeds
    {kBatch, kMaxObjectsNum, kHistoryNum, kCoords},   // headings
    {kBatch, kMaxObjectsNum},                         // types
    {kBatch, kMaxObjectsNum, kCoords},                // cur_poses
    {kBatch, kMaxLanesNum, kLanePointsNum, kCoords},  // lane_centers
};

// NOLINTNEXTLINE(readability-identifier-naming)
static std::vector<std::vector<int>> kOutputDims{{kBatch, kModes}};

struct ActorsFeature {
  std::vector<float> trajs;
  std::vector<float> speeds;
  std::vector<float> headings;
  std::vector<float> types;
  std::vector<float> cur_poses;
  std::vector<int> mask;
};

struct LanesFeature {
  std::vector<float> lane_centers;
  std::vector<float> lane_lights;
  std::vector<int64_t> lane_types;
  std::vector<int> mask;
};

struct LaneBoundaryFeature {
  std::vector<float> boundaries;
  std::vector<int64_t> boundary_types;
  std::vector<int> mask;
};

struct CrossWalkFeature {
  std::vector<float> encirclingline;
  std::vector<int> mask;
};

struct TargetLanesFeature {
  std::vector<float> lane_centers;
  std::vector<float> stats;
};

struct SelectorScoringNetFeature {
  ActorsFeature actors_feature;
  LanesFeature lanes_feature;
  LaneBoundaryFeature bounds_feature;
  CrossWalkFeature cws_feature;
  std::vector<ActorsFeature> c_trajs_feature;
  std::vector<TargetLanesFeature> tl_feature;
};

class SelectorScoringNet {
 public:
  virtual bool GetOutputs(const SelectorScoringNetFeature& input_features,
                          std::vector<std::vector<float>>* scores) = 0;
  virtual void UnitTest() = 0;

  virtual ~SelectorScoringNet() = default;
};

}  // namespace selector_scoring_net
}  // namespace qcraft

// NOLINTNEXTLINE
// NOLINTNEXTLINE
#endif  // ONBOARD_PLANNER_ML_SELECTOR_MODELS_SELECTOR_SCORING_NET_H_
