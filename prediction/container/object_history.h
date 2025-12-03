#ifndef ONBOARD_PREDICTION_CONTAINER_OBJECT_HISTORY_H_
#define ONBOARD_PREDICTION_CONTAINER_OBJECT_HISTORY_H_

#include "onboard/prediction/container/object_history_span.h"
#include "onboard/prediction/container/prediction_object.h"
#include "onboard/prediction/prediction_defs.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/utils/elements_history.h"

struct StopTimeInfo {
  // move : 0  stop:  how long time the object has stoped.
  double time_duration_since_stop() const { return last_time - stopped_since; }

  // the most recent moving duration
  double last_move_time_duration() const {
    return stopped_since - last_moved_since;
  }

  // the most recent stop duration before the most rencent moving duration.
  double previous_stop_time_duration() const {
    return last_moved_since - prev_stopped_since;
  }
  double last_time = 0.0;
  double stopped_since = 0.0;
  double prev_stopped_since = 0.0;
  double last_moved_since = 0.0;
};

namespace qcraft {
namespace prediction {
class ObjectHistory
    : public elements_history::ElementHistory<double, PredictionObject,
                                              ObjectHistorySpan> {
 public:
  using ElementHistory<double, PredictionObject,
                       ObjectHistorySpan>::ElementHistory;

  const ObjectIDType& id() const { return GetHistory().id(); }
  const ObjectProto& object_proto() const {
    return GetHistory().object_proto();
  }
  ObjectType type() const { return GetHistory().type(); }
  double timestamp() const { return GetHistory().timestamp(); }

  bool UpdateStopTimeInfo();
  const StopTimeInfo& GetStopTimeInfo() const { return stop_time_info_; }

 private:
  StopTimeInfo stop_time_info_;
};
}  // namespace prediction
}  // namespace qcraft

#endif  // ONBOARD_PREDICTION_CONTAINER_OBJECT_HISTORY_H_
