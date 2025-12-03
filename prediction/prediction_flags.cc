#include "onboard/prediction/prediction_flags.h"

namespace qcraft {
namespace prediction {

DEFINE_int32(prediction_thread_pool_size, 2,
             "The thread num of the prediction thread pool");

DEFINE_double(prediction_max_allowed_pose_delay_ms, 50.0,
              "The maximum allowed pose delay time in milliseconds.");

DEFINE_double(prediction_max_allowed_iteration_time, 0.35,
              "The maximum allowed planner iteration time in seconds.");

DEFINE_bool(prediction_use_autonomy_state, true,
            "Use autonomy state to switch scheduler.");

DEFINE_bool(prediction_use_tracker_history, true, "Use tracker history.");

DEFINE_bool(
    prediction_enable_ped_traj_cutoff_at_curb, true,
    "switch to enable termination of ped trajectory when crossing the curb.");

DEFINE_bool(prediction_enable_debug_no_map, false,
            "Enable prediction noa scheduler using no map.");

DEFINE_bool(prediction_enable_debug_perception_map, false,
            "Enable prediction noa scheduler using perception map.");

DEFINE_bool(prediction_enable_debug_noa_map, false,
            "Enable prediction noa scheduler using noa map.");

DEFINE_bool(
    prediction_enable_ignore_off_road_object, false,
    "Enable prediction ignore off road object when using perception map.");

// machine learning model
DEFINE_bool(prediction_enable_act_net, true, "Enable actnet predictor.");
DEFINE_bool(prediction_enable_auxiliary_cutin_sl_net, true,
            "Enable cutin_sl_net auxiliary predictor for actnet.");

// j5 flags
DEFINE_bool(prediction_enable_act_net_j5, true,
            "Enable actnet in j5 or x86 sim j5");

DEFINE_bool(prediction_replace_act_net_j5_with_local, false,
            "Enable actnet j5 or x86 sim with local coord "
            "model.");

DEFINE_bool(prediction_enable_auxiliary_cutin_sl_net_j5, true,
            "Enable cutin_sl_net auxiliary predictor "
            "for actnet.");
DEFINE_bool(prediction_enable_lane_selection_net_j5, true,
            "Enable lane selection net in j5 or x86 sim j5");
DEFINE_bool(prediction_lane_selection_ignore_rear_objects, true,
            "Lane selection net ignores objects behind av for time saving");

// Conflict Resolver.
DEFINE_bool(prediction_conflict_resolver_visual_on, false,
            "Conflict resolver module debug mode on/off");
DEFINE_bool(prediction_run_post_process_in_planner, true,
            "Whether to run prediction post process in planner module.");

DEFINE_bool(rectify_speed_profile, true,
            "Whether to rectify speed_profile by heuristic method.");

DEFINE_bool(print_prediction_time_stats, false, "Print prediction time stats.");

DEFINE_bool(only_use_perception_acc, true,
            "Only use perception acc for vehicle lane follow predictor.");

DEFINE_bool(prediction_debug_drive_passage, false,
            "Send drive passage cache to prediction debug.");

DEFINE_bool(prediction_debug_av_drive_passage, false,
            "Send av drive passage to prediction debug.");

}  // namespace prediction
}  // namespace qcraft
