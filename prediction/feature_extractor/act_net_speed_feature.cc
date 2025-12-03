#include "onboard/prediction/feature_extractor/act_net_speed_feature.h"

#include <string>
#include <vector>

#include "absl/strings/str_format.h"

#include "onboard/planner/common/plot_util.h"
#include "onboard/vis/common/color.h"

namespace qcraft {
namespace prediction {
namespace {
std::vector<Vec2d> PathFeatureToSmoothCoordinate(const PathFeature& path,
                                                 const Vec2d& ref_position,
                                                 double rot_rad) {
  const auto& xy = path.path_xy_interp;
  std::vector<Vec2d> smooth_xy;
  const int point_num = static_cast<int>(xy.size() / 2.0);
  smooth_xy.reserve(point_num);
  for (int i = 0; i < point_num; ++i) {
    const int query_idx = 2 * i;
    const Vec2d xy_agent(static_cast<double>(xy[query_idx]),
                         static_cast<double>(xy[query_idx + 1]));
    smooth_xy.push_back(xy_agent.Rotate(-rot_rad) + ref_position);
  }
  return smooth_xy;
}
}  // namespace
void SendActNetSpeedFeatureToCanvas(const ActNetSpeedFeature& feature) {
  const auto& ref_position = feature.ref_position;
  const auto& rot_rad = feature.rot_rad;
  const auto& agent_path = feature.agent_path;
  const auto& av_path = feature.av_path;
  const auto& agent_pts =
      PathFeatureToSmoothCoordinate(agent_path, ref_position, rot_rad);
  const auto& av_pts =
      PathFeatureToSmoothCoordinate(av_path, ref_position, rot_rad);
  const std::string channel_agent =
      absl::StrFormat("%s_%d", feature.agent_id, feature.traj_index);
  planner::SendPointsToCanvas(agent_pts, channel_agent,
                              vis::Color::kSalmonPink);
  const std::string channel_av =
      absl::StrFormat("%s_%d_av", feature.agent_id, feature.traj_index);
  planner::SendPointsToCanvas(av_pts, channel_av, vis::Color::kDarkViolet);
}
}  // namespace prediction
}  // namespace qcraft
