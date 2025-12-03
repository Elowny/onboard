#include "onboard/planner/util/motion_util.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "glog/logging.h"

#include "onboard/math/util.h"

namespace qcraft::planner {

ConstJerkBrakingMotion::ConstJerkBrakingMotion(double v0, double a0,
                                               double min_acc, double jerk)
    : v0_(v0), a0_(std::max(a0, min_acc)), min_acc_(min_acc), jerk_(jerk) {
  DCHECK_LT(min_acc_, 0.0);
  DCHECK_LT(jerk_, 0.0);
  DCHECK_GE(v0_, 0.0);

  min_acc_time_ = (min_acc_ - a0_) / jerk_;
  v_min_acc_ = v0_ + a0_ * min_acc_time_ + 0.5 * jerk_ * Sqr(min_acc_time_);
  s_min_acc_ = v0_ * min_acc_time_ + 0.5 * a0_ * Sqr(min_acc_time_) +
               kSixInv * jerk_ * Cube(min_acc_time_);

  v_reach_zero_first_ = v_min_acc_ < 0.0;

  if (v_reach_zero_first_) {
    // v reach zero before acc hit min_acc.
    // v0 + a0 *t + 0.5 * jerk * t^2 = 0;
    const double delta = std::sqrt(Sqr(a0_) - 2 * v0_ * jerk_);
    t_stop_ = (-delta - a0_) / jerk_;
  } else {
    // v reach zero after acc hit min_acc.
    t_stop_ = min_acc_time_ - v_min_acc_ / min_acc_;
  }
  t_stop_ = std::max(0.0, t_stop_);
}

double ConstJerkBrakingMotion::GetS(double t) const {
  DCHECK_GE(t, 0.0);
  // s won't change after t_stop_.
  t = std::min(t, t_stop_);
  if (v_reach_zero_first_) {
    return v0_ * t + 0.5 * a0_ * Sqr(t) + kSixInv * jerk_ * Cube(t);
  } else {
    if (t > min_acc_time_) {
      t -= min_acc_time_;
      return s_min_acc_ + v_min_acc_ * t + 0.5 * min_acc_ * Sqr(t);
    } else {
      return v0_ * t + 0.5 * a0_ * Sqr(t) + kSixInv * jerk_ * Cube(t);
    }
  }
}

double ConstJerkBrakingMotion::GetV(double t) const {
  DCHECK_GE(t, 0.0);
  // v = 0  after t_stop_.
  if (t > t_stop_) {
    return 0.0;
  }
  if (v_reach_zero_first_) {
    return v0_ + a0_ * t + 0.5 * jerk_ * Sqr(t);
  } else {
    if (t > min_acc_time_) {
      t -= min_acc_time_;
      return v_min_acc_ + min_acc_ * t;
    } else {
      return v0_ + a0_ * t + 0.5 * jerk_ * Sqr(t);
    }
  }
}

double ConstJerkBrakingMotion::GetA(double t) const {
  DCHECK_GE(t, 0.0);
  if (t > min_acc_time_) {
    return min_acc_;
  } else {
    return a0_ + jerk_ * t;
  }
}

ConstJerkAcceleratingMotion::ConstJerkAcceleratingMotion(double v0, double a0,
                                                         double max_acc,
                                                         double jerk)
    : v0_(v0), a0_(std::min(a0, max_acc)), max_acc_(max_acc), jerk_(jerk) {
  DCHECK_GT(max_acc_, 0.0);
  DCHECK_GE(jerk_, 0.0);
  DCHECK_GE(v0_, 0.0);

  if (jerk_ < kJerkEpsilon) {
    max_acc_time_ = std::numeric_limits<double>::infinity();

    if (a0_ < 0.0) {
      t_stop_ = -v0_ / a0_;
      v_reach_zero_first_ = true;
      s_stop_ = v0_ * t_stop_ + 0.5 * a0_ * Sqr(t_stop_);
      t_restart_ = std::numeric_limits<double>::infinity();
    } else {
      t_stop_ = std::numeric_limits<double>::infinity();
      v_reach_zero_first_ = false;
    }

  } else {
    max_acc_time_ = (max_acc_ - a0_) / jerk_;

    // v0 + a0 *t + 0.5 * jerk * t^2 = 0;
    const double delta_sqr = Sqr(a0_) - 2.0 * v0_ * jerk_;
    if (delta_sqr > 0.0 && a0_ < 0.0) {
      // Vehicle stopped before max acc time.
      const double delta = std::sqrt(delta_sqr);
      t_stop_ = std::max(0.0, (-delta - a0_) / jerk_);
      v_reach_zero_first_ = true;

      s_stop_ = v0_ * t_stop_ + 0.5 * a0_ * Sqr(t_stop_) +
                kSixInv * jerk_ * Cube(t_stop_);
      t_restart_ = (0.0 - a0_) / jerk_;

      const double dt = max_acc_time_ - t_restart_;
      s_max_acc_ = s_stop_ + kSixInv * jerk_ * Cube(dt);
      v_max_acc_ = 0.5 * jerk_ * Sqr(dt);
    } else {
      // Vehicle never stopped.
      s_max_acc_ = v0_ * max_acc_time_ + 0.5 * a0_ * Sqr(max_acc_time_) +
                   kSixInv * jerk_ * Cube(max_acc_time_);
      v_max_acc_ = v0_ + a0_ * max_acc_time_ + 0.5 * jerk_ * Sqr(max_acc_time_);
      t_stop_ = std::numeric_limits<double>::infinity();
      v_reach_zero_first_ = false;
    }
  }
}

double ConstJerkAcceleratingMotion::GetS(double t) const {
  DCHECK_GE(t, 0.0);

  if (v_reach_zero_first_) {
    if (t < t_stop_) {
      return v0_ * t + 0.5 * a0_ * Sqr(t) + kSixInv * jerk_ * Cube(t);
    } else if (t < t_restart_) {
      return s_stop_;
    } else {
      if (t < max_acc_time_) {
        t -= t_restart_;
        return s_stop_ + kSixInv * jerk_ * Cube(t);
      } else {
        t -= max_acc_time_;
        // Const acc motion after max acc time.
        return s_max_acc_ + v_max_acc_ * t + 0.5 * max_acc_ * Sqr(t);
      }
    }
  } else {
    if (t < max_acc_time_) {
      return v0_ * t + 0.5 * a0_ * Sqr(t) + kSixInv * jerk_ * Cube(t);
    } else {
      t -= max_acc_time_;
      // Const acc motion after max acc time.
      return s_max_acc_ + v_max_acc_ * t + 0.5 * max_acc_ * Sqr(t);
    }
  }
}

double ConstJerkAcceleratingMotion::GetV(double t) const {
  DCHECK_GE(t, 0.0);
  if (v_reach_zero_first_) {
    if (t < t_stop_) {
      return v0_ + a0_ * t + 0.5 * jerk_ * Sqr(t);
    } else if (t < t_restart_) {
      return 0.0;
    } else {
      if (t < max_acc_time_) {
        t -= t_restart_;
        return 0.5 * jerk_ * Sqr(t);
      } else {
        t -= max_acc_time_;
        // Const acc motion after max acc time.
        return v_max_acc_ + max_acc_ * t;
      }
    }
  } else {
    if (t < max_acc_time_) {
      return v0_ + a0_ * t + 0.5 * jerk_ * Sqr(t);
    } else {
      t -= max_acc_time_;
      // Const acc motion after max acc time.
      return v_max_acc_ + max_acc_ * t;
    }
  }
}

double ConstJerkAcceleratingMotion::GetA(double t) const {
  DCHECK_GE(t, 0.0);
  if (t < max_acc_time_) {
    return a0_ + jerk_ * t;
  } else {
    return max_acc_;
  }
}

}  // namespace qcraft::planner
