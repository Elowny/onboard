#ifndef ONBOARD_PREDICTION_CONTAINER_AV_CONTEXT_H_
#define ONBOARD_PREDICTION_CONTAINER_AV_CONTEXT_H_

#include <stddef.h>

#include "boost/circular_buffer.hpp"

#include "onboard/prediction/container/object_history.h"
#include "onboard/proto/localization.pb.h"
#include "onboard/proto/positioning.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {
namespace prediction {
class AvContext {
 public:
  AvContext(size_t capacity, double len)
      : av_object_history_(capacity), len_(len) {}
  void Update(const PoseProto& pose,
              const LocalizationTransformProto& loc_transform,
              const VehicleGeometryParamsProto& veh_geom_params);
  const ObjectHistory& GetAvObjectHistory() const;
  double GetAvKappaCacheAverage() const;
  double GetAvCurrentSpeed() const;
  const boost::circular_buffer<PoseProto>& GetAvPoseCache() const {
    return av_pose_cache_;
  }

 private:
  ObjectHistory av_object_history_;
  double len_ = 0.0;
  static constexpr int kCacheSize = 20;  // ~2s.
  boost::circular_buffer<double> av_kappa_cache_{kCacheSize};

  static constexpr int kPoseCacheSize = 100;  // ~10s.
  boost::circular_buffer<PoseProto> av_pose_cache_{kPoseCacheSize};
};

}  // namespace prediction
}  // namespace qcraft

#endif  // ONBOARD_PREDICTION_CONTAINER_AV_CONTEXT_H_
