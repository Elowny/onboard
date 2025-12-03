#ifndef ONBOARD_PLANNER_UTIL_MOTION_UTIL_H_
#define ONBOARD_PLANNER_UTIL_MOTION_UTIL_H_

#include <optional>

namespace qcraft::planner {

class ConstJerkBrakingMotion {
 public:
  static constexpr double kSixInv = 1.0 / 6.0;
  // Constant jerk motion starting from s = 0.0, v = v0, a = a0.
  // With min acc limit and min v = 0.0;
  // If a0 < min_acc, a will raise to min_acc immediately at t=0.
  ConstJerkBrakingMotion(double v0, double a0, double min_acc, double jerk);

  // Get s at time t.
  double GetS(double t) const;

  // Get v at time t.
  double GetV(double t) const;

  // Get a at time t.
  double GetA(double t) const;

  double t_stop() const { return t_stop_; }

 private:
  double v0_ = 0.0;
  double a0_ = 0.0;
  double min_acc_ = 0.0;
  double jerk_ = 0.0;

  double t_stop_ = 0.0;

  bool v_reach_zero_first_ = false;
  // Valid only when v reach zero after acc hit min_acc.
  double min_acc_time_ = 0.0;
  double s_min_acc_ = 0.0;
  double v_min_acc_ = 0.0;
};

class ConstJerkAcceleratingMotion {
 public:
  static constexpr double kSixInv = 1.0 / 6.0;
  static constexpr double kJerkEpsilon = 1e-5;
  // Constant jerk motion starting from s = 0.0, v = v0, a = a0.
  // With max acc limit;
  // If a0 > max_acc, a will drop to max_acc immediately at t=0.
  ConstJerkAcceleratingMotion(double v0, double a0, double max_acc,
                              double jerk);

  // Get s at time t.
  double GetS(double t) const;

  // Get v at time t.
  double GetV(double t) const;

  // Get a at time t.
  double GetA(double t) const;

 private:
  double v0_ = 0.0;
  double a0_ = 0.0;
  double max_acc_ = 0.0;
  double jerk_ = 0.0;

  double t_stop_ = 0.0;
  double v_reach_zero_first_ = false;

  double max_acc_time_ = 0.0;

  // Valid only when max_acc_time_ is finite.
  double s_max_acc_ = 0.0;
  double v_max_acc_ = 0.0;

  // Valid only when v_reach_zero_first_.
  double s_stop_ = 0.0;
  double t_restart_ = 0.0;
};

class ConstJerkMotion {
 public:
  // Constant jerk motion starting from s = 0.0, v = v0, a = a0.
  // With min & max acc limit and min v = 0.0;
  // If a0 < min_acc, a will raise to min_acc immediately at t=0.
  // If a0 > max_acc, a will drop to max_acc immediately at t=0.
  ConstJerkMotion(double v0, double a0, double min_acc, double max_acc,
                  double jerk) {
    if (jerk >= 0.0) {
      accelerating_motion_.emplace(v0, a0, max_acc, jerk);
    } else {
      braking_motion_.emplace(v0, a0, min_acc, jerk);
    }
  }

  // Get s at time t.
  constexpr double GetS(double t) const {
    return accelerating_motion_ ? (accelerating_motion_->GetS(t))
                                : (braking_motion_->GetS(t));
  }

  // Get v at time t.
  constexpr double GetV(double t) const {
    return accelerating_motion_ ? (accelerating_motion_->GetV(t))
                                : (braking_motion_->GetV(t));
  }

  // Get a at time t.
  constexpr double GetA(double t) const {
    return accelerating_motion_ ? (accelerating_motion_->GetA(t))
                                : (braking_motion_->GetA(t));
  }

 private:
  std::optional<ConstJerkBrakingMotion> braking_motion_;
  std::optional<ConstJerkAcceleratingMotion> accelerating_motion_;
};

}  // namespace qcraft::planner
#endif  // ONBOARD_PLANNER_UTIL_MOTION_UTIL_H_
