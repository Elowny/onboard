#include "onboard/planner/object/predicted_motion_filter.h"

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type

#include <algorithm>
#include <iterator>

#include "onboard/prediction/object_prediction.h"
#include "onboard/prediction/prediction_util.h"
#include "onboard/proto/perception/fusion/object.pb.h"

namespace qcraft {
namespace planner {

namespace {
// The time duration to check for object prediction collision.
constexpr double kConfidentDuration = 1.0;  // Seconds

// Check if an object is a confident object based on its type. When collisions
// happen, we trust the predictions of confident objects.
bool IsConfidentObjectType(ObjectType type) {
  switch (type) {
    case OT_VEHICLE:
    case OT_LARGE_VEHICLE:
    case OT_BARRIER:
    case OT_BARRIER_ANTI_COLLISION_BUCKET:
    case OT_BARRIER_ANTI_COLLISION_POST:
    case OT_UNKNOWN_STATIC:
      return true;
    case OT_MOTORCYCLIST:
    case OT_PEDESTRIAN:
    case OT_CYCLIST:
    case OT_TRICYCLIST:
    case OT_FOD:
    case OT_UNKNOWN_MOVABLE:
    case OT_VEGETATION:
    case OT_CONE:
    case OT_WARNING_TRIANGLE:
      return false;
  }
}

bool IsConfidentPrediction(const prediction::PredictedTrajectory& pred) {
  constexpr double kConfidentTrajectoryProbability = 0.6;
  if (pred.probability() > kConfidentTrajectoryProbability) {
    return true;
  }
  return false;
}

Box2d ComputeConfidentStationaryObjectBox(const PlannerObject& object) {
  constexpr double kShrinkFactor = 0.6;
  auto box = object.bounding_box();
  box.LongitudinalExtendByRatio(kShrinkFactor);
  box.LateralExtendByRatio(kShrinkFactor);
  return box;
}

Box2d ComputeConfidentMovingObjectBox(
    const PlannerObject& object,
    const prediction::PredictedTrajectoryPoint& traj_point) {
  constexpr double kShrinkFactor = 0.6;
  const auto& box = object.bounding_box();
  return Box2d(box.half_length() * kShrinkFactor,
               box.half_width() * kShrinkFactor, traj_point.pos(),
               traj_point.theta());
}

std::vector<PredictedMotionFilter::ObjectCollisionCheckingState>
ExtractConfidentStates(const PlannerObject& object,
                       const prediction::PredictedTrajectory& traj) {
  std::vector<PredictedMotionFilter::ObjectCollisionCheckingState> states;
  // Returns one state if the object is a stationary object.
  if (prediction::IsStationaryTrajectory(traj)) {
    states.push_back({.id = object.id(),
                      .box = ComputeConfidentStationaryObjectBox(object)});
    return states;
  }

  if (traj.points().empty()) return states;

  // If the object is a moving object, return the first few states.
  for (const auto& point : traj.points()) {
    states.push_back({
        .id = object.id(),
        .box = ComputeConfidentMovingObjectBox(object, point),
    });
    if (point.t() > kConfidentDuration) break;
  }
  return states;
}

}  // namespace
PredictedMotionFilter::PredictedMotionFilter(
    absl::Span<const PlannerObject> planner_objects) {
  std::vector<ObjectCollisionCheckingState> confident_states;

  for (const auto& object : planner_objects) {
    if (!IsConfidentObjectType(object.type())) continue;

    for (const auto& traj : object.prediction().trajectories()) {
      if (!IsConfidentPrediction(traj)) continue;
      auto states = ExtractConfidentStates(object, traj);
      confident_object_states_.reserve(confident_object_states_.size() +
                                       states.size());
      for (auto it = std::make_move_iterator(states.begin()),
                end = std::make_move_iterator(states.end());
           it != end; ++it) {
        confident_object_states_.push_back(*it);
      }
    }
  }

  // Build KdTree from the confident object states.
  AABoxKDTreeParams params{
      .max_depth = 4,
      .max_leaf_size = 8,
  };
  kdtree_ = std::make_unique<AABoxKDTree2d<ObjectCollisionCheckingState>>(
      confident_object_states_, params);
}

FilterReason::Type PredictedMotionFilter::Filter(
    const PlannerObject& object,
    const prediction::PredictedTrajectory& traj) const {
  if (IsConfidentObjectType(object.type())) return FilterReason::NONE;
  if (IsStationaryTrajectory(traj)) return FilterReason::NONE;
  if (object.type() == OT_PEDESTRIAN) return FilterReason::NONE;

  const auto& points = traj.points();
  for (int i = 0, n = points.size(); i < n; ++i) {
    if (points[i].t() > kConfidentDuration) break;
    const auto box = ComputeConfidentMovingObjectBox(object, points[i]);
    auto* nearest = kdtree_->GetNearestObject(box.center());
    if (nearest != nullptr && nearest->box.HasOverlap(box)) {
      return FilterReason::TRAJECTORY_COLLIDES_WITH_CONFIDENT_OTHERS;
    }
  }
  return FilterReason::NONE;
}

}  // namespace planner
}  // namespace qcraft
