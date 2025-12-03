#include "onboard/prediction/test_util/object_history_builder.h"

// IWYU pragma: no_include <boost/move/utility_core.hpp>  // for move

#include <algorithm>
#include <cmath>
#include <optional>
#include <utility>

#include "onboard/math/coordinate_converter.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/planner/test_util/perception_object_builder.h"
#include "onboard/prediction/container/prediction_object.h"
#include "onboard/prediction/util/kinematic_model.h"

namespace qcraft {
namespace prediction {
namespace {
constexpr double kHistoryTimeStep = 0.1;

ObjectMotionState UniCycleStateToObjectMotionState(const UniCycleState& state,
                                                   double length, double width,
                                                   double ts) {
  const Vec2d pos(state.x, state.y);
  return ObjectMotionState{
      .timestamp = ts,
      .pos = pos,
      .heading = state.heading,
      .vel = Vec2d::FastUnitFromAngle(state.heading) * state.v,
      .bbox = Box2d(pos, state.heading, length, width)};
}

}  // namespace
ObjectHistory BuildVehicleHistoryByConstVel(absl::string_view id,
                                            int history_num,
                                            const Vec2d& init_pos, double vel) {
  CoordinateConverter coord_converter;
  ObjectHistory obj_history(history_num * 2);
  planner::PerceptionObjectBuilder perception_builder;
  for (int i = 0; i < history_num; ++i) {
    const double ts = kHistoryTimeStep * i;
    auto perception_obj = perception_builder.set_id(id)
                              .set_pos(Vec2d(vel * ts, 0.0) + init_pos)
                              .set_box_center(Vec2d(vel * ts, 0.0) + init_pos)
                              .set_timestamp(ts)
                              .set_velocity(vel)
                              .set_accel(Vec2d::Zero())
                              .set_trajectory(10)
                              .Build();
    obj_history.Push(ts, PredictionObject(perception_obj, coord_converter));
  }
  return obj_history;
}

ObjectHistory BuildHistoryByConstVel(absl::string_view id, int history_num,
                                     const Vec2d& init_pos, double vel,
                                     ObjectType type) {
  CoordinateConverter coord_converter;
  ObjectHistory obj_history(history_num * 2);
  planner::PerceptionObjectBuilder perception_builder;
  for (int i = 0; i < history_num; ++i) {
    const double ts = kHistoryTimeStep * i;
    auto perception_obj = perception_builder.set_id(id)
                              .set_pos(Vec2d(vel * ts, 0.0) + init_pos)
                              .set_box_center(Vec2d(vel * ts, 0.0) + init_pos)
                              .set_timestamp(ts)
                              .set_velocity(vel)
                              .set_accel(Vec2d::Zero())
                              .set_type(type)
                              .Build();
    obj_history.Push(ts, PredictionObject(perception_obj, coord_converter));
  }
  return obj_history;
}

ObjectMotionHistory BuildVehicleMotionHistoryByConstVel(absl::string_view id,
                                                        int history_num,
                                                        const Vec2d& init_pos,
                                                        double vel) {
  ObjectMotionHistory obj_motion_hist;
  obj_motion_hist.id = id;
  obj_motion_hist.type = OT_VEHICLE;
  std::vector<ObjectMotionState> motion_states = BuildRawTrackerHistoryCTRA(
      init_pos, vel, /*acc=*/0.0,
      /*heading=*/0.0, /*yaw_rate=*/0.0, /*length=*/4.0, /*width=*/2.0,
      history_num, /*start_ts=*/0.0, kHistoryTimeStep);
  obj_motion_hist.states = std::move(motion_states);
  return obj_motion_hist;
}

std::vector<ObjectMotionState> BuildRawTrackerHistoryCTRA(
    const Vec2d& earliest_pos, double vel, double acc, double heading,
    double yaw_rate, double length, double width, int history_num,
    double start_ts, double dt) {
  std::vector<ObjectMotionState> trajectory;
  trajectory.reserve(history_num);
  UniCycleState cur_state{
      .x = earliest_pos.x(),
      .y = earliest_pos.y(),
      .v = vel,
      .heading = heading,
      .yaw_rate = yaw_rate,
      .acc = acc,
  };
  trajectory.push_back(
      UniCycleStateToObjectMotionState(cur_state, length, width, start_ts));

  // If decel, calculate set acc and yaw rate to zero idx.
  std::optional<int> stop_idx = std::nullopt;
  if (acc < 0.0) {
    stop_idx = static_cast<int>(vel / std::fabs(acc) / dt);
  }

  for (int i = 1; i < history_num; ++i) {
    if (stop_idx.has_value() && i >= *stop_idx) {
      cur_state.yaw_rate = 0.0;
      cur_state.acc = 0.0;
    }
    const auto next_state = SimulateUniCycleModel(cur_state, dt);
    trajectory.push_back(UniCycleStateToObjectMotionState(
        next_state, length, width, start_ts + i * dt));
    cur_state = next_state;
  }

  return trajectory;
}

}  // namespace prediction
}  // namespace qcraft
