#ifndef ONBOARD_PLANNER_TEST_UTIL_OBJECT_PREDICTION_BUILDER_H_
#define ONBOARD_PLANNER_TEST_UTIL_OBJECT_PREDICTION_BUILDER_H_

#include <memory>
#include <optional>
#include <vector>

#include "onboard/planner/test_util/predicted_trajectory_builder.h"
#include "onboard/prediction/object_prediction.h"
#include "onboard/prediction/predicted_trajectory.h"
#include "onboard/prediction/proto/prediction_common.pb.h"
#include "onboard/proto/perception.pb.h"

namespace qcraft {
namespace planner {

class ObjectPredictionBuilder {
 public:
  ObjectPredictionBuilder() {}

  ObjectPredictionBuilder& set_object(const ObjectProto& object);

  PredictedTrajectoryBuilder* add_predicted_trajectory();

  prediction::ObjectPrediction Build();

 private:
  ObjectRoadStatus road_status_ = ObjectRoadStatus::ORS_NONE;
  ObjectIntersectionStatus intersection_status_ =
      ObjectIntersectionStatus::OIS_NONE;
  std::vector<prediction::PredictedTrajectory> trajs_;
  std::optional<ObjectProto> perception_object_;
  std::vector<std::unique_ptr<PredictedTrajectoryBuilder>>
      predicted_trajectory_builders_;
};

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_TEST_UTIL_OBJECT_PREDICTION_BUILDER_H_
