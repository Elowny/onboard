#ifndef ONBOARD_PLANNER_INITIALIZER_GEOMETRY_GEOMETRY_STATE_H_
#define ONBOARD_PLANNER_INITIALIZER_GEOMETRY_GEOMETRY_STATE_H_

#include <string>

#include "absl/strings/str_cat.h"

#include "onboard/math/vec.h"

namespace qcraft {
namespace planner {

struct GeometryState {
  Vec2d xy;
  double h = 0.0;      // heading.
  double k = 0.0;      // curvature.
  double ref_k = 0.0;  // reference curvature on centerline.
  // Approximate frenet variable for cost computation later.
  double accumulated_s = 0.0;  // station s.
  double l = 0.0;              // lateral offset in drive passage.

  double dk = 0.0;  // derivative of curvature (for quintic spiral computation).
  double ddk = 0.0;  // second derivative of curvature.
};

}  // namespace planner
}  // namespace qcraft
#endif  // ONBOARD_PLANNER_INITIALIZER_GEOMETRY_GEOMETRY_STATE_H_
