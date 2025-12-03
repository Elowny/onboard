#ifndef ONBOARD_PLANNER_ML_OPTIMIZER_AUTO_TUNING_AUTO_TUNING_COMMON_FLAGS_H_  // NOLINT
#define ONBOARD_PLANNER_ML_OPTIMIZER_AUTO_TUNING_AUTO_TUNING_COMMON_FLAGS_H_  // NOLINT

#include "gflags/gflags.h"

DECLARE_bool(dump_expert_policy);
DECLARE_bool(output_pose_traj);
DECLARE_string(specific_snapshot_folder);
DECLARE_string(auto_tuning_data_filename);

namespace qcraft {
namespace planner {

// The optimizer trajectory length used for auto tuning.
// The actual length will be clamped to be smaller than that of trajectory
// optimizer.
inline constexpr int kDdpTrajectoryStepsDATHint = 50;  // 10s.

}  // namespace planner
}  // namespace qcraft

// NOLINTNEXTLINE
#endif  // ONBOARD_PLANNER_ML_OPTIMIZER_AUTO_TUNING_AUTO_TUNING_COMMON_FLAGS_H_
