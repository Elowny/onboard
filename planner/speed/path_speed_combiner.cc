#include "onboard/planner/speed/path_speed_combiner.h"

#include <algorithm>
#include <cmath>
#include <optional>
#include <ostream>
#include <string>

#include "absl/strings/str_format.h"
#include "glog/logging.h"

#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/lite/logging.h"
#include "onboard/math/util.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/speed/speed_point.h"

namespace qcraft::planner {
namespace {
constexpr double kEpsilon = 1e-6;
}

absl::Status CombinePathAndSpeed(
    const DiscretizedPath& path_data, bool forward,
    const SpeedVector& speed_data,
    std::vector<ApolloTrajectoryPointProto>* trajectory) {
  FUNC_QTRACE();

  QCHECK_NOTNULL(trajectory);

  for (int i = 0; i < speed_data.size(); ++i) {
    VLOG(3) << "speed_data:[" << i << "] = " << speed_data[i].DebugString();
  }
  QCHECK_GT(path_data.size(), 1);
  QCHECK_GT(speed_data.size(), 1);

  trajectory->clear();
  double t = 0.0;
  while (t < speed_data.TotalTime()) {
    const auto speed_point = speed_data.EvaluateByTime(t);
    if (!speed_point.has_value()) {
      const auto error_msg =
          absl::StrFormat("Fail to evaluate speed vector at time %.2f", t);
      QLOG(WARNING) << error_msg;
      return absl::InternalError(error_msg);
    }

    PathPoint path_point;
    if (path_data.length() < kEpsilon) {
      path_point = path_data.front();
    } else {
      path_point = path_data.Evaluate(speed_point->s());
    }

    ApolloTrajectoryPointProto& traj_point = trajectory->emplace_back();
    if (!forward) {
      path_point.set_s(-path_point.s());
      path_point.set_theta(NormalizeAngle(path_point.theta() + M_PI));
      path_point.set_kappa(-path_point.kappa());
    }
    *(traj_point.mutable_path_point()) = path_point;
    traj_point.set_v(forward ? speed_point->v() : -speed_point->v());
    traj_point.set_a(forward ? speed_point->a() : -speed_point->a());
    traj_point.set_j(forward ? speed_point->j() : -speed_point->j());
    traj_point.set_relative_time(t);

    t += kTrajectoryTimeStep;
  }
  return absl::OkStatus();
}

}  // namespace qcraft::planner
