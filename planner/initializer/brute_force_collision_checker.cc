#include "onboard/planner/initializer/brute_force_collision_checker.h"

#include <algorithm>
#include <vector>

#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/object/planner_object.h"
#include "onboard/prediction/prediction_defs.h"

namespace qcraft {
namespace planner {

namespace {
template <typename StateType>
std::vector<Box2d> StatesToBoxes(absl::Span<const StateType> states,
                                 const VehicleGeometryParamsProto* params) {
  std::vector<Box2d> boxes;
  boxes.reserve(states.size());
  const double ra_to_center =
      params->front_edge_to_center() - 0.5 * params->length();
  for (const auto& state : states) {
    const Vec2d tangent = Vec2d::FastUnitFromAngle(state.h);
    const Vec2d center = state.xy + tangent * ra_to_center;
    boxes.push_back(Box2d(center, tangent, params->length(), params->width()));
  }
  return boxes;
}

void CheckStationaryObjectCollision(
    int sample_step, absl::Span<const GeometryState> states,
    absl::Span<const SpacetimeObjectTrajectory* const> stationary_trajs,
    const VehicleGeometryParamsProto* vehicle_geom, double buffer,
    CollisionInfo* info) {
  const auto av_boxes = StatesToBoxes(states, vehicle_geom);
  bool has_collision = false;
  for (const auto* traj : stationary_trajs) {
    auto obj_contour = traj->planner_object().contour();
    obj_contour = obj_contour.ExpandByDistance(buffer);
    for (int i = 0; i < av_boxes.size(); ++i) {
      if (obj_contour.HasOverlap(av_boxes[i])) {
        info->collision_objects.push_back(
            {.time = i * sample_step, .traj = traj});
        has_collision = true;
      }
      if (has_collision) break;
    }
  }
}
}  // namespace

BruteForceCollisionChecker::BruteForceCollisionChecker(
    absl::Span<const SpacetimeObjectTrajectory> trajectories,
    const VehicleGeometryParamsProto* vehicle_geom, int sample_step,
    double stationary_buffer, double moving_buffer)
    : vehicle_geom_(vehicle_geom),
      sample_step_(sample_step),
      stationary_buffer_(stationary_buffer),
      moving_buffer_(moving_buffer) {
  QCHECK_GE(sample_step, 1);
  for (const auto& traj : trajectories) {
    if (traj.is_stationary()) {
      considered_stationary_trajs_.push_back(&traj);
    } else {
      considered_moving_trajs_.push_back(&traj);
    }
  }
  // Sample the prediction trajectories.
  sampled_trajs_ = SampleTrajectory(considered_moving_trajs_, sample_step_);
}
void BruteForceCollisionChecker::UpdateStationaryObjectBuffer(
    double stationary_buffer) {
  stationary_buffer_ = stationary_buffer;
}
void BruteForceCollisionChecker::UpdateMovingObjectBuffer(
    double moving_buffer) {
  moving_buffer_ = moving_buffer;
}

void BruteForceCollisionChecker::CheckCollisionWithTrajectories(
    double init_t, absl::Span<const MotionState> states,
    const IgnoreTrajMap& ignored_trajs, CollisionInfo* info) const {
  const double extend_len = moving_buffer_ * 2.0;
  const auto av_boxes = StatesToBoxes(states, vehicle_geom_);
  const int start_ix =
      RoundToInt(init_t / (sample_step_ * prediction::kPredictionTimeStep));
  for (const auto& sampled_traj : sampled_trajs_) {
    // Do not perform collision checking on ignored trajs.
    if (ignored_trajs.contains(sampled_traj.traj->traj_id())) {
      continue;
    }
    const auto& obj_states = sampled_traj.states;
    for (int i = 0, t = start_ix; i < states.size() && t < obj_states.size();
         ++i, ++t) {
      auto obj_box = obj_states[t].box;
      obj_box.LongitudinalExtend(extend_len);
      obj_box.LateralExtend(extend_len);
      if (av_boxes[i].HasOverlap(obj_box)) {
        std::optional<CollisionConfiguration> config = std::nullopt;
        if (t > 0) {
          config = DetermineConfiguration(av_boxes[i], obj_states[t - 1].box);
        }
        info->collision_objects.push_back({
            .time = t * sample_step_,
            .traj = sampled_traj.traj,
            .collision_configuration = config,
        });
        break;
      }
    }
  }
}

void BruteForceCollisionChecker::CheckCollisionWithStationaryObjects(
    absl::Span<const GeometryState> states, CollisionInfo* info) const {
  CheckStationaryObjectCollision(/*sample_step=*/1, states,
                                 considered_stationary_trajs_, vehicle_geom_,
                                 stationary_buffer_, info);
}

}  // namespace planner
}  // namespace qcraft
