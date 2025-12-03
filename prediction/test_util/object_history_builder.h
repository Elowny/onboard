#ifndef ONBOARD_PREDICTION_TEST_UTIL_OBJECT_HISTORY_BUILDER_H_
#define ONBOARD_PREDICTION_TEST_UTIL_OBJECT_HISTORY_BUILDER_H_

#include <vector>

#include "absl/strings/string_view.h"

#include "onboard/math/vec.h"
#include "onboard/prediction/container/object_history.h"
#include "onboard/prediction/prediction_defs.h"
#include "onboard/proto/perception/fusion/object.pb.h"

namespace qcraft {
namespace prediction {
ObjectHistory BuildVehicleHistoryByConstVel(absl::string_view id,
                                            int history_num,
                                            const Vec2d& init_pos, double vel);
ObjectHistory BuildHistoryByConstVel(absl::string_view id, int history_num,
                                     const Vec2d& init_pos, double vel,
                                     ObjectType type);
ObjectMotionHistory BuildVehicleMotionHistoryByConstVel(absl::string_view id,
                                                        int history_num,
                                                        const Vec2d& init_pos,
                                                        double vel);

std::vector<ObjectMotionState> BuildRawTrackerHistoryCTRA(
    const Vec2d& earliest_pos, double vel, double acc, double heading,
    double yaw_rate, double length, double width, int history_num,
    double start_ts, double dt);

}  // namespace prediction
}  // namespace qcraft

#endif  // ONBOARD_PREDICTION_TEST_UTIL_OBJECT_HISTORY_BUILDER_H_
