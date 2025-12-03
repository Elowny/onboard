#ifndef ONBOARD_PLANNER_TEST_UTIL_PREDICTED_TRAJECTORY_BUILDER_H_
#define ONBOARD_PLANNER_TEST_UTIL_PREDICTED_TRAJECTORY_BUILDER_H_

#include <memory>
#include <string>
#include <vector>

#include "onboard/math/vec.h"
#include "onboard/prediction/predicted_trajectory.h"
#include "onboard/proto/prediction.pb.h"

namespace qcraft {
namespace planner {

class PredictedTrajectoryBuilder {
 public:
  PredictedTrajectoryBuilder();

  PredictedTrajectoryBuilder& set_probability(double probability);

  PredictedTrajectoryBuilder& set_annotation(std::string annotation);

  PredictedTrajectoryBuilder& set_type(PredictionType type);

  PredictedTrajectoryBuilder& set_index(int index);

  PredictedTrajectoryBuilder& set_points(
      std::vector<prediction::PredictedTrajectoryPoint> points);

  // Specify a straight line prediction by start point, end point, start
  // velocity and end velocity. Using constant acceleration.
  PredictedTrajectoryBuilder& set_straight_line(const Vec2d& start,
                                                const Vec2d& end, double init_v,
                                                double last_v);
  // Specify a straight line prediction by start point, theta, trajectory
  // duration, initial velocity, and acceleration. velocity and const
  // acceleration.
  PredictedTrajectoryBuilder& set_straight_line(const Vec2d& start,
                                                double theta, double duration,
                                                double init_v, double acc);

  PredictedTrajectoryBuilder& set_stationary_traj(const Vec2d& pos,
                                                  double theta);

  prediction::PredictedTrajectory Build();

 private:
  double probability_ = 1.0;
  std::string annotation_ = "obj";
  int index_ = 0;
  PredictionType type_ = PredictionType::PT_CYCV;
  std::vector<prediction::PredictedTrajectoryPoint> points_;
  std::unique_ptr<prediction::PredictedTrajectory> traj_;
};

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_TEST_UTIL_PREDICTED_TRAJECTORY_BUILDER_H_
