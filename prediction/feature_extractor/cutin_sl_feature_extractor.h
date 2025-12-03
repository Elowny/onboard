#ifndef ONBOARD_PREDICTION_FEATURE_EXTRACTOR_CUTIN_SL_FEATURE_EXTRACTOR_H_  // NOLINT
#define ONBOARD_PREDICTION_FEATURE_EXTRACTOR_CUTIN_SL_FEATURE_EXTRACTOR_H_  // NOLINT

#include <optional>
#include <vector>

#include "absl/types/span.h"

#include "onboard/maps/maps_common.h"
#include "onboard/math/frenet_common.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/prediction/feature_extractor/cutin_sl_feature.h"
#include "onboard/prediction/feature_extractor/map_sampler.h"
#include "onboard/prediction/feature_extractor/object_history_sampler.h"
#include "onboard/prediction/prediction_defs.h"
#include "onboard/prediction/proto/prediction_common.pb.h"
#include "onboard/prediction/proto/prediction_cutin_sl.pb.h"

namespace qcraft {
namespace prediction {
// TODO(zifeng): move TransformXYGroundTruthToSL to feature_extraction_util.
std::optional<std::vector<FrenetCoordinate>> TransformXYGroundTruthToSL(
    const TrajectoryGroundTruth& gt, const planner::DrivePassage& drive_passage,
    int gt_size);

std::optional<CutinSLFeature> ExtractCutinSLFeature(
    const ObjectIDType& agent_id, const ObjectHistorySampler& obj_sampler,
    const planner::DrivePassage& drive_passage,
    const std::vector<const mapping::LaneInfo*>& candi_lane_centers,
    const std::vector<const mapping::LaneBoundaryInfo*>& candi_lane_boundaries,
    const std::vector<const mapping::CrosswalkInfo*>& candi_crosswalks,
    MapSampler* const map_sampler_ptr);

CutinSLDumpedFeatureProto ToCutinSLDumpedFeatureProto(
    const CutinSLFeature& cutin_sl_feature,
    const absl::Span<const FrenetCoordinate> agent_gt,
    const absl::Span<const CrossType> if_cross_gt, double ts);

}  // namespace prediction
}  // namespace qcraft

#endif  // ONBOARD_PREDICTION_FEATURE_EXTRACTOR_CUTIN_FEATURE_EXTRACTOR_H_
