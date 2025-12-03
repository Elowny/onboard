#include "onboard/planner/freespace/hybrid_a_star/fast_reeds_shepp.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "absl/status/statusor.h"
#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/math/util.h"
#include "onboard/planner/freespace/hybrid_a_star/fast_reeds_shepp_table.h"
#include "onboard/planner/freespace/hybrid_a_star/reeds_shepp.h"

namespace qcraft {
namespace planner {

FastReedSheppPath GetShortestReedsSheppFromTable(const Node3d& start,
                                                 const Node3d& end,
                                                 double max_kappa) {
  QCHECK_GT(max_kappa, 0.0);
  // Normalize the initial point to (0,0,0).
  const double dx = end.x() - start.x();
  const double dy = end.y() - start.y();
  double normalized_theta = NormalizeAngle(end.theta() - start.theta());
  const double normalized_x =
      (start.cos_theta() * dx + start.sin_theta() * dy) * max_kappa;
  double normalized_y =
      (-start.sin_theta() * dx + start.cos_theta() * dy) * max_kappa;

  // Normalize the end point to y >= 0.
  if (normalized_y < 0) {
    normalized_y = -normalized_y;
    normalized_theta = NormalizeAngle(-normalized_theta);
  }

  const int x =
      FloorToInt((normalized_x - kRSTableMinX) / kRSTableXYResolution);
  const int y =
      FloorToInt((normalized_y - kRSTableMinY) / kRSTableXYResolution);
  // Theta maybe out of [0, 19] because of float error.
  const int theta = std::clamp(
      FloorToInt((normalized_theta + M_PI) / kRSTableThetaResolution), 0,
      kRSTableTheta - 1);

  const auto get_gear_change_num = [](const std::vector<double>& segs_lengths) {
    int res = 0;
    for (int i = 0; i + 1 < segs_lengths.size(); ++i) {
      if (segs_lengths[i] * segs_lengths[i + 1] < 0.0) {
        res++;
      }
    }
    return res;
  };

  FastReedSheppPath res;
  if (x < 0 || x >= kRSTableX || y < 0 || y >= kRSTableY) {
    const auto result_status = GetShortestReedsShepp(start, end, max_kappa);
    if (result_status.ok()) {
      res.total_length = result_status->total_length;
      res.gear_change_num = get_gear_change_num(result_status->segs_lengths);
      res.init_gear = (result_status->segs_lengths.front() > 0.0);
    } else {
      res.total_length = std::numeric_limits<double>::infinity();
      res.gear_change_num = 0;
      res.init_gear = true;
    }
    return res;
  }

  res.total_length = kRSTableTotalLength[x][y][theta] / max_kappa;
  res.gear_change_num = kRSTableGearChangeNum[x][y][theta];
  res.init_gear = kRSTableInitGear[x][y][theta];
  return res;
}

}  // namespace planner
}  // namespace qcraft
