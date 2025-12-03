#include "onboard/planner/min_length_path_extension.h"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "absl/strings/str_format.h"
#include "gflags/gflags.h"

#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/util/path_util.h"

DEFINE_double(zigzag_start_end_point_dist, 1.0,
              "If we find a zigzag path in whole path, we need to remove the "
              "zigzag range. The start and end points of the path should not "
              "be too far, if so, path after disposing will be acceptable.");

namespace qcraft {
namespace planner {

absl::StatusOr<std::vector<PathPoint>> ExtendPathAndDeleteUnreasonablePart(
    absl::Span<const ApolloTrajectoryPointProto> trajectory_points,
    double required_min_length, double max_curvature) {
  std::vector<PathPoint> raw_path_points;
  raw_path_points.reserve(trajectory_points.size());
  for (const auto& pt : trajectory_points) {
    raw_path_points.push_back(pt.path_point());
  }
  // Delete Zigzag path.
  // Traversal path, if we find that next point's arc length is less than
  // current, we start to find the point pt_next whose s is larger than current
  // point and has a minimum positive projection along path, then remove points
  // between current point and pt_next. Repeat above steps.
  int index = 1;
  constexpr double kEpsilon = 1e-6;
  while (index < raw_path_points.size()) {
    if (raw_path_points[index].s() < raw_path_points[index - 1].s()) {
      // Find next s > current_s
      const PathPoint& current_path_point = raw_path_points[index - 1];
      // Add epsilon to avoid problem cause by numerical fault when integration.
      const double current_s = current_path_point.s() + kEpsilon;
      const Vec2d current_tangent =
          Vec2d::UnitFromAngle(current_path_point.theta());
      for (int i = index; i < raw_path_points.size(); ++i) {
        const Vec2d delta_vec =
            ToVec2d(raw_path_points[i]) - ToVec2d(current_path_point);
        const double projection = delta_vec.dot(current_tangent);
        if (raw_path_points[i].s() > current_s && projection > kEpsilon) {
          const double dist =
              DistanceTo(current_path_point, raw_path_points[i]);
          if (dist < FLAGS_zigzag_start_end_point_dist) {
            // Delete points between index and i;
            raw_path_points.erase(raw_path_points.begin() + index,
                                  raw_path_points.begin() + i);
            // Recompute s.
            const double delta_s =
                dist - (raw_path_points[index].s() - current_path_point.s());
            for (int k = index; k < raw_path_points.size(); ++k) {
              raw_path_points[k].set_s(raw_path_points[k].s() + delta_s);
            }
            break;
          } else {
            return absl::InternalError(absl::StrFormat(
                "Zigzag point dist too large: dist(%fm), "
                "start_index(%d), end_index(%d), projection(%f)",
                dist, index - 1, i, projection));
          }
        } else if (i == (raw_path_points.size() - 1)) {
          raw_path_points.erase(raw_path_points.begin() + index,
                                raw_path_points.end());
          break;
        }
      }
    }
    ++index;
  }
  // Recompute s.
  raw_path_points.begin()->set_s(0.0);
  for (int index = 1; index < raw_path_points.size(); ++index) {
    const double d =
        DistanceTo(raw_path_points[index - 1], raw_path_points[index]);
    raw_path_points[index].set_s(raw_path_points[index - 1].s() + d);
  }

  // Delete the part that exceeds curvature limit.
  std::optional<double> end_kappa = std::nullopt;
  auto init_point = raw_path_points.front();
  for (auto iter = raw_path_points.begin(); iter < raw_path_points.end();
       ++iter) {
    if (std::abs(iter->kappa()) > max_curvature) {
      raw_path_points.erase(iter, raw_path_points.end());
      // This is a hack to compute sign of end_kappa, because input path may
      // exceed -max_curvature firstly and exceed max_curvature later.
      // TODO(runbing): Solve such problem in optimization.
      double sum_kappa = 0.0;
      for (int i = std::distance(raw_path_points.begin(), iter);
           i < raw_path_points.size(); ++i) {
        if (std::abs(raw_path_points[i].kappa()) > max_curvature) {
          sum_kappa += raw_path_points[i].kappa();
        }
      }
      end_kappa = std::copysign(max_curvature, sum_kappa);
      break;
    }
  }
  if (raw_path_points.empty()) {
    raw_path_points.push_back(std::move(init_point));
  }
  if (end_kappa.has_value()) {
    // Add one more point with max kappa to the end of the path.
    auto last_pt = raw_path_points.back();
    last_pt.set_kappa((last_pt.kappa() + *end_kappa) * 0.5);
    raw_path_points.push_back(
        GetPathPointAlongCircle(last_pt, kPathSampleInterval));
    // Set end point kappa to the max.
    raw_path_points.back().set_kappa(*end_kappa);
  }

  // Extend raw path if it is too short.
  const double v_now = trajectory_points.front().v();
  constexpr double kDecel = 2.0;  // m/s^2.
  const double min_length =
      std::max(required_min_length, 0.5 * Sqr(v_now) / kDecel);
  while (raw_path_points.back().s() < min_length) {
    PathPoint p =
        GetPathPointAlongCircle(raw_path_points.back(), kPathSampleInterval);
    raw_path_points.push_back(std::move(p));
  }

  return raw_path_points;
}

}  // namespace planner
}  // namespace qcraft
