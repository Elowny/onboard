#ifndef ONBOARD_PREDICTION_PREDICTION_FLAGS_H_
#define ONBOARD_PREDICTION_PREDICTION_FLAGS_H_

#include "gflags/gflags.h"

namespace qcraft {
namespace prediction {

DECLARE_int32(prediction_thread_pool_size);

DECLARE_double(prediction_max_allowed_pose_delay_ms);

DECLARE_double(prediction_max_allowed_iteration_time);

DECLARE_bool(prediction_use_autonomy_state);

DECLARE_bool(prediction_use_tracker_history);

// ped
DECLARE_bool(prediction_enable_ped_traj_cutoff_at_curb);

// schedule J5
DECLARE_bool(prediction_enable_debug_no_map);
DECLARE_bool(prediction_enable_debug_perception_map);
DECLARE_bool(prediction_enable_debug_noa_map);

// ignore off road object of perception map
DECLARE_bool(prediction_enable_ignore_off_road_object);

DECLARE_bool(prediction_enable_act_net_j5);
DECLARE_bool(prediction_replace_act_net_j5_with_local);
DECLARE_bool(prediction_enable_lane_selection_net_j5);
DECLARE_bool(prediction_lane_selection_ignore_rear_objects);

// schedule CPU&GPU
// machine learning model
DECLARE_bool(prediction_enable_act_net);

DECLARE_bool(prediction_enable_auxiliary_cutin_sl_net);
DECLARE_bool(prediction_enable_auxiliary_cutin_sl_net_j5);

// conflict resolver
DECLARE_bool(prediction_conflict_resolver_visual_on);
DECLARE_bool(prediction_run_post_process_in_planner);

DECLARE_bool(rectify_speed_profile);
// print time stats
DECLARE_bool(print_prediction_time_stats);

DECLARE_bool(only_use_perception_acc);

DECLARE_bool(prediction_debug_drive_passage);

DECLARE_bool(prediction_debug_av_drive_passage);

}  // namespace prediction
}  // namespace qcraft

#endif  // ONBOARD_PREDICTION_PREDICTION_FLAGS_H_
