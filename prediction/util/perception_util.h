#ifndef ONBOARD_PREDICTION_UTIL_PERCEPTION_UTIL_H_
#define ONBOARD_PREDICTION_UTIL_PERCEPTION_UTIL_H_

#include "absl/status/status.h"

#include "onboard/proto/perception.pb.h"

namespace qcraft {
namespace prediction {

// Align an object's timestamp to current time. It assumes the object moves
// along its current direction with constant acceleration. If the provided
// current time is too different than perception's time, an error message will
// be returned.
absl::Status AlignPerceptionObjectTime(double current_time,
                                       ObjectProto* object);

}  // namespace prediction
}  // namespace qcraft

#endif  // ONBOARD_PREDICTION_UTIL_PERCEPTION_UTIL_H_
