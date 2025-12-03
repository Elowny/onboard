#ifndef ONBOARD_PLANNER_ML_CONDITION_FEATURE_CONDITION_FEATURE_H_  // NOLINT
#define ONBOARD_PLANNER_ML_CONDITION_FEATURE_CONDITION_FEATURE_H_  // NOLINT

#include <vector>

namespace qcraft {
namespace planner {
namespace ml {

struct TrajectoryFeature {
  // positions([horizon, 2])
  std::vector<float> pos;
  // Position diff from last point to current point([horizon, 2])
  std::vector<float> pos_diff;
  // speed along heading ([horizon, 1])
  std::vector<float> speed;
  // sin and cos of yaw([horizon, 2])
  std::vector<float> yaw;
  // timestamps in seconds
  std::vector<float> ts;
};

struct LineSegmentsFeature {
  // node([horizon, 2])
  std::vector<float> node;
};

struct PathBoundaryFeature {
  LineSegmentsFeature left_path_boundary;
  LineSegmentsFeature right_path_boundary;
  LineSegmentsFeature smooth_ref_center_line;
};

struct MultiLanePathFeature {
  std::vector<std::vector<float>> lanes;
  std::vector<int> lanes_mask;
};

}  // namespace ml
}  // namespace planner
}  // namespace qcraft

// NOLINTNEXTLINE
#endif  // ONBOARD_PLANNER_ML_CONDITION_FEATURE_CONDITION_FEATURE_H_
