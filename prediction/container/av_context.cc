#include "onboard/prediction/container/av_context.h"

// IWYU pragma: no_include <boost/move/utility_core.hpp>  // for move
#include <algorithm>  // for max
#include <numeric>
#include <utility>
#include <vector>

#include "boost/circular_buffer.hpp"

#include "onboard/math/coordinate_converter.h"
#include "onboard/planner/util/perception_util.h"
#include "onboard/prediction/container/prediction_object.h"
#include "onboard/prediction/prediction_defs.h"
#include "onboard/prediction/util/transform_util.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/utils/elements_history.h"
namespace qcraft {
namespace prediction {
void AvContext::Update(const PoseProto& pose,
                       const LocalizationTransformProto& loc_transform,
                       const VehicleGeometryParamsProto& veh_geom_params) {
  std::vector<const LocalizationTransformProto*> loc_transforms;
  loc_transforms.reserve(av_object_history_.size() + 1);
  for (auto& node : *av_object_history_.mutable_buffer()) {
    loc_transforms.push_back(
        &node.val.origin_coordinate_converter().localization_transform());
  }
  loc_transforms.push_back(&loc_transform);
  const bool has_invalid_loc = HasInvalidLocTransformInHistory(loc_transforms);

  CoordinateConverter target(loc_transform);
  // We only perform localization transform if they are valid.
  if (!has_invalid_loc) {
    for (auto& node : *av_object_history_.mutable_buffer()) {
      node.val.TransformCoordinate(target);
    }
  }
  av_object_history_.Push(
      pose.timestamp(), PredictionObject(planner::AvPoseProtoToObjectProto(
                                             kAvObjectId, veh_geom_params, pose,
                                             /*offroad=*/false),
                                         std::move(target)));

  av_object_history_.PopBegin(pose.timestamp() - len_);
  av_kappa_cache_.push_back(pose.curvature());
  av_pose_cache_.push_back(pose);
}

const ObjectHistory& AvContext::GetAvObjectHistory() const {
  return av_object_history_;
}

double AvContext::GetAvKappaCacheAverage() const {
  return av_kappa_cache_.full() ? std::accumulate(av_kappa_cache_.begin(),
                                                  av_kappa_cache_.end(), 0.0) /
                                      kCacheSize
                                : 0.0;
}

double AvContext::GetAvCurrentSpeed() const {
  return av_object_history_.back().val.v();
}

}  // namespace prediction
}  // namespace qcraft
