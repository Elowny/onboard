#ifndef ONBOARD_PLANNER_COMMON_FEATURE_EXTRACTION_UTILS_H_
#define ONBOARD_PLANNER_COMMON_FEATURE_EXTRACTION_UTILS_H_

#include <algorithm>
#include <vector>

#include "absl/status/status.h"

#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/maps/lane_path.h"
#include "onboard/planner/common/path_sl_boundary.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/prediction/proto/prophnet.pb.h"
#include "onboard/proto/prediction.pb.h"
#include "onboard/proto/trajectory_point.pb.h"

namespace qcraft::planner {

inline constexpr double kPathBoundarySampleLen = 5.0;  // m.

absl::Status ExtractTrajectoryFeature(
    const PredictionDebugProto& prediction_debug,
    const std::vector<ApolloTrajectoryPointProto>& traj,
    ProphnetDumpedFeatureProto::ObjectsDumpedFeature* traj_model_feature);

absl::Status ExtractLanePathFeature(
    const PredictionDebugProto& prediction_debug,
    const PlannerSemanticMapManager& psmm, const mapping::LanePath& lane_path,
    ProphnetDumpedFeatureProto::LanesDumpedFeature* lane_feature);

absl::Status ExtractPathBoundaryFeature(
    const PredictionDebugProto& prediction_debug,
    const PathSlBoundary& path_sl_boundary,
    ProphnetDumpedFeatureProto::LanesDumpedFeature* left_path_boundary,
    ProphnetDumpedFeatureProto::LanesDumpedFeature* right_path_boundary,
    ProphnetDumpedFeatureProto::LanesDumpedFeature* smooth_ref_center_line);

absl::Status ExtractRefPathFeature(
    const PredictionDebugProto& prediction_debug,
    const PathSlBoundary& path_sl_boundary,
    ProphnetDumpedFeatureProto::LanesDumpedFeature* smooth_ref_center_line);

template <typename T>
std::vector<float> OneHotObjectType(T type, int num_class) {
  // Put type into range
  QCHECK_GT(num_class, 0);
  int type_idx =
      std::max(std::min<int>(static_cast<int>(type), num_class - 1), 0);
  std::vector<float> one_hot(num_class, 0.0);
  one_hot[type_idx] = 1.0;
  return one_hot;
}

// Create flattened one hots vector.
template <typename T>
std::vector<float> OneHotObjectsType(std::vector<T> types, int num_class) {
  QCHECK_GT(num_class, 0);
  const int flat_size = types.size();
  std::vector<float> one_hots(flat_size * num_class, 0.0);
  for (int i = 0; i < flat_size; ++i) {
    const auto& type = types[i];
    const auto type_idx =
        std::max(std::min<int>(static_cast<int>(type), num_class - 1), 0);
    one_hots[i * num_class + type_idx] = 1.0;
  }
  return one_hots;
}

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_COMMON_FEATURE_EXTRACTION_UTILS_H_
