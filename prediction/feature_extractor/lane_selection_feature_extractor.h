#ifndef ONBOARD_PREDICTION_FEATURE_EXTRACTOR_LANE_SELECTION_FEATURE_EXTRACTOR_H_  // NOLINT
#define ONBOARD_PREDICTION_FEATURE_EXTRACTOR_LANE_SELECTION_FEATURE_EXTRACTOR_H_  // NOLINT

#include <optional>  // for optional
#include <vector>    // for vector

#include "onboard/math/frenet_common.h"            // for FrenetCoordinate
#include "onboard/planner/router/drive_passage.h"  // for DrivePassage
#include "onboard/prediction/feature_extractor/lane_selection_feature.h"  // for LaneSelectionFeature
#include "onboard/prediction/feature_extractor/map_sampler.h"  // for MapSampler
#include "onboard/prediction/feature_extractor/object_history_sampler.h"  // for ObjectHistorySampler
#include "onboard/prediction/prediction_defs.h"  // for ObjectIDType
#include "onboard/prediction/proto/prediction_lane_selection.pb.h"  // for LaneSelectionDumpedFeatureProto

namespace qcraft {
namespace prediction {
std::optional<LaneSelectionFeature> ExtractLaneSelectionFeature(
    const ObjectIDType& agent_id, const ObjectHistorySampler& obj_sampler,
    const planner::DrivePassage& drive_passage,
    MapSampler* const map_sampler_ptr);

LaneSelectionDumpedFeatureProto ToLaneSelectionDumpedFeatureProto(
    const LaneSelectionFeature& lane_selection_feature,
    const std::vector<FrenetCoordinate>& agent_gt, double ts);

std::optional<LaneSelectionObjectCommonFeatures>
GetLaneSelectionObjectCommonFeatures(const ObjectMotionHistory& obj_history,
                                     const planner::DrivePassage& drive_passage,
                                     int history_len);

}  // namespace prediction
}  // namespace qcraft

#endif
// ONBOARD_PREDICTION_FEATURE_EXTRACTOR_LANE_SELECTION_FEATURE_EXTRACTOR_H_
