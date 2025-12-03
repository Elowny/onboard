
#include "onboard/prediction/predictor/heuristic_pedestrian_predictor.h"

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

#include "absl/status/statusor.h"

#include "onboard/global/trace.h"
#include "onboard/math/line2d.h"
#include "onboard/math/line_fitter.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/prediction/util/kinematic_model.h"
#include "onboard/prediction/util/trajectory_developer.h"
#include "onboard/proto/prediction.pb.h"

namespace qcraft {
namespace prediction {
namespace {
constexpr double kFittingExpDecayRate = 0.5;
// Get the speed and std along the line path.
constexpr double kMaxPedestrianSpeed = 12.42;  // m/s.  World record held by
                                               // Usain Bolt in 2009.

}  // namespace
PredictedTrajectory MakeHeuristicPedestrianPrediction(
    const ObjectMotionHistory& obj_hist) {
  SCOPED_QTRACE("HeuristicPedestrianPrediction");
  const auto& cur_state = obj_hist.states.back();
  std::vector<ObjectMotionState> states;
  states.reserve(obj_hist.states.size());
  for (const auto& state : obj_hist.states) {
    if (state.timestamp >= cur_state.timestamp - kHistoryTime) {
      states.push_back(state);
    }
  }

  PredictedTrajectory predicted_traj;
  std::vector<PredictedTrajectoryPoint> traj_pts;
  if (states.size() == 1) {
    traj_pts =
        DevelopCYCVTrajectory(ObjectMotionStateToUniCycleState(cur_state),
                              kPredictionTimeStep, kEmergencyGuardHorizon,
                              /*is_reversed=*/false);
    predicted_traj = PredictedTrajectory(
        /*probability=*/1.0, "heuristic ped prediction: cvch",
        PredictionType::PT_CYCV, 1, std::move(traj_pts),
        /*is_reversed=*/false);
  } else {
    std::vector<Vec2d> vec_pos;
    vec_pos.reserve(states.size());
    std::vector<double> weights;
    weights.reserve(states.size());
    for (const auto& ele : states) {
      const double dt = cur_state.timestamp - ele.timestamp;
      vec_pos.push_back(ele.pos);
      weights.push_back(std::pow(kFittingExpDecayRate, dt));
    }
    const auto line_or = FitLine2dToData(vec_pos, weights);
    const Vec2d line_tangent = line_or->tangent();
    const double line_angle = line_tangent.FastAngle();
    const Vec2d& vec_vel = states.back().vel;
    double fitted_direction = (line_tangent.dot(vec_vel)) >= 0
                                  ? line_angle
                                  : NormalizeAngle(M_PI + line_angle);
    const double clamped_speed =
        std::clamp(vec_vel.norm(), 0.0, kMaxPedestrianSpeed);

    UniCycleState uni_state{.x = cur_state.pos.x(),
                            .y = cur_state.pos.y(),
                            .v = clamped_speed,
                            .heading = fitted_direction,
                            .yaw_rate = 0.0,
                            .acc = 0.0};
    traj_pts =
        DevelopCYCVTrajectory(uni_state, kPredictionTimeStep,
                              kComfortableHorizon, /*is_reversed=*/false);
    predicted_traj = PredictedTrajectory(
        /*probability=*/1.0, "heuristic ped prediction",
        PredictionType::PT_PED_KINEMATIC, 1, std::move(traj_pts),
        /*is_reversed=*/false);
  }
  // Check if termination is needed. This assumes that pedestrians are rational
  // and will not behave in aggressive manners such that collision with the AV
  // is possible.

  return predicted_traj;
}
}  // namespace prediction
}  // namespace qcraft
