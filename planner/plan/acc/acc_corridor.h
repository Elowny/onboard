#ifndef ONBOARD_PLANNER_PLAN_ACC_CORRIDOR_H_
#define ONBOARD_PLANNER_PLAN_ACC_CORRIDOR_H_

#include <optional>

#include "common/proto/qacc.pb.h"

#include "onboard/math/frenet_frame.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/planner/common/discretized_path.h"
#include "onboard/planner/common/path_sl_boundary.h"

namespace qcraft {
namespace planner {
struct AccCorridor {
  PathSlBoundary boundary;
  QtfmEnhancedKdTreeFrenetFrame frenet_frame;
  DiscretizedPath path;
  double credible_length = 0.0;
  // Curvature profile with respect to arc length.
  PiecewiseLinearFunction<double, double> kappa_s;
  void ToProto(AccCorridorProto* proto) const {
    boundary.ToProto(proto->mutable_path_boundary());
    proto->set_credible_length(credible_length);
  }
};
}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_PLAN_ACC_CORRIDOR_H_
