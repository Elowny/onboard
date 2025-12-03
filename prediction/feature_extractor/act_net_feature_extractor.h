#ifndef ONBOARD_PREDICTION_FEATURE_EXTRACTOR_ACT_NET_FEATURE_EXTRACTOR_H_  // NOLINT
#define ONBOARD_PREDICTION_FEATURE_EXTRACTOR_ACT_NET_FEATURE_EXTRACTOR_H_  // NOLINT

#include <vector>

#include "absl/types/span.h"

#include "onboard/math/geometry/box2d.h"
#include "onboard/math/vec.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/prediction/container/object_history.h"
#include "onboard/prediction/feature_extractor/act_net_feature.h"
#include "onboard/prediction/feature_extractor/map_sampler.h"
#include "onboard/prediction/feature_extractor/object_history_sampler.h"
#include "onboard/prediction/prediction_defs.h"
#include "onboard/prediction/proto/act_net.pb.h"

namespace qcraft {
namespace prediction {

std::vector<ActNetPolylineFeature> ExtractMapFeatureFromPolylines(
    absl::Span<const SampledPolyline* const> polylines, const Box2d& cur_box,
    const AgentCoordTransform& agent_coord_transform, int max_lane_seg_num,
    int max_feat_num);

// Map sampler use a cache mechanism, thus it cannot be called with const
// reference.
ActNetFeature ExtractActNetFeature(
    const ObjectIDType& agent_id, const ObjectHistorySampler& obj_sampler,
    const StopTimeInfo& stop_time_info,
    const planner::DrivePassage* drive_passage_ptr,
    MapSampler* const map_sampler_ptr, const AvMapCache& av_map_cache,
    const Box2d& region_box);

ActNetDumpedFeatureProto ToActNetDumpedFeatureProto(
    const ActNetFeature& act_feature, double ts, const Vec2d& ref_pos,
    double rot_rad, const Vec2d& av_pos, double av_angle);

ActNetObjectAbsoluteFeature GetActNetObjectAbsoluteFeature(
    const ObjectMotionHistory& obj_history,
    const AgentCoordTransform& agent_coord_transform, int coord_num,
    double time_step, int max_steps);

ActNetObjectRelFeature GetActNetObjectRelFeature(
    const ActNetObjectAbsoluteFeature& agent_feat,
    const ActNetObjectAbsoluteFeature& obj_feat, int coord_num, int max_steps);

}  // namespace prediction
}  // namespace qcraft

#endif  // ONBOARD_PREDICTION_FEATURE_EXTRACTOR_ACT_NET_FEATURE_EXTRACTOR_H_
