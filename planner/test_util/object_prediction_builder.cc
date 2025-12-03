#include "onboard/planner/test_util/object_prediction_builder.h"

#include <utility>

#include "absl/memory/memory.h"
#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/math/geometry/util.h"

namespace qcraft {
namespace planner {

ObjectPredictionBuilder& ObjectPredictionBuilder::set_object(
    const ObjectProto& object) {
  perception_object_.emplace(object);
  return *this;
}

PredictedTrajectoryBuilder*
ObjectPredictionBuilder::add_predicted_trajectory() {
  auto builder = absl::make_unique<PredictedTrajectoryBuilder>();
  auto* ptr = builder.get();
  predicted_trajectory_builders_.push_back(std::move(builder));
  return ptr;
}

prediction::ObjectPrediction ObjectPredictionBuilder::Build() {
  QCHECK(perception_object_.has_value());
  if (predicted_trajectory_builders_.empty()) {
    auto builder = absl::make_unique<PredictedTrajectoryBuilder>();
    builder->set_probability(1.0).set_stationary_traj(
        Vec2dFromProto(perception_object_->pos()), perception_object_->yaw());
    predicted_trajectory_builders_.push_back(std::move(builder));
  }
  trajs_.clear();
  trajs_.reserve(predicted_trajectory_builders_.size());
  int index = 0;
  for (auto& builder : predicted_trajectory_builders_) {
    builder->set_annotation(perception_object_->id());
    builder->set_index(index++);
    trajs_.push_back(builder->Build());
  }

  double prob = 0.0;
  for (const auto& traj : trajs_) {
    prob += traj.probability();
  }
  QCHECK_LE(prob, 1.0);

  // TODO(lidong): Add set intention_.
  // TODO(lidong): Add set prob_.

  prediction::ObjectPrediction prediction(std::move(trajs_),
                                          perception_object_.value(),
                                          road_status_, intersection_status_);
  return prediction;
}
}  // namespace planner
}  // namespace qcraft
