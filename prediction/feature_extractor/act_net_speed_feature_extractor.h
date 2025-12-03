#ifndef ONBOARD_PREDICTION_FEATURE_EXTRACTOR_ACT_NET_SPEED_FEATURE_EXTRACTOR_H_
#define ONBOARD_PREDICTION_FEATURE_EXTRACTOR_ACT_NET_SPEED_FEATURE_EXTRACTOR_H_

#include <string>
#include <string_view>

#include "absl/strings/str_format.h"

#include "onboard/planner/common/discretized_path.h"
#include "onboard/planner/object/spacetime_object_trajectory.h"
#include "onboard/prediction/feature_extractor/act_net_feature.h"
#include "onboard/prediction/feature_extractor/act_net_speed_feature.h"
#include "onboard/prediction/prediction_defs.h"

namespace qcraft {
namespace prediction {

// Planner module.

struct ActNetSpeedPlannerTarget {
  std::string object_id;
  const planner::SpacetimeObjectTrajectory* st_traj;
  double av_s;
  double object_s;

  std::string DebugString() const {
    return absl::StrFormat(
        "Act net speed model target: object %s, traj id: %s, av_s: %f, "
        "object_s: %f.",
        object_id, st_traj->traj_id(), av_s, object_s);
  }
};

ActNetSpeedFeature ExtractActNetSpeedFeaturesForPlannerTarget(
    const ActNetFeature& act_net_feature,
    const planner::DiscretizedPath& av_path,
    const ActNetSpeedPlannerTarget& target);

ActNetFeature ExtractActNetFeatureWithAVOnly(
    const ObjectIDType& agent_id, const ObjectMotionHistory& agent_history,
    const ObjectMotionHistory& av_history);

}  // namespace prediction
}  // namespace qcraft

#endif
