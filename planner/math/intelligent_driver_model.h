
#ifndef ONBOARD_PLANNER_MATH_INTELLIGENT_DRIVER_MODEL_H_
#define ONBOARD_PLANNER_MATH_INTELLIGENT_DRIVER_MODEL_H_

// IDM https://en.wikipedia.org/wiki/Intelligent_driver_model
// IDM params reference:

namespace qcraft {
namespace planner {

namespace idm {
struct Parameters {
  double v_desire;           // m/s.
  double s_min;              // m.
  double t_desire;           // t.
  double acc_max;            // m/s^2.
  double comfortable_brake;  // (absolute value) m/s^2.
  double brake_max;          // maximal brake.
  double delta;              // exponent.
};
// ds = s_front - s;
// dv = v_front - v;
double ComputeIDMAcceleration(double v, double ds, double dv,
                              const Parameters& params);
}  // namespace idm
}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_MATH_INTELLIGENT_DRIVER_MODEL_H_
