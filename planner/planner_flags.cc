#include "onboard/planner/planner_flags.h"

DEFINE_int32(planner_debug, 0, "How much do you want to debug me?");

DEFINE_double(planner_main_loop_interval, 0.1,
              "Planner main loop interval in seconds.");

DEFINE_int32(planner_thread_pool_size, 3, "Planner thread pool size.");

DEFINE_double(planner_lateral_reset_error, 0.35,
              "Lateral reset error for EstPlanner.");

DEFINE_double(planner_max_allowed_iteration_time, 0.45,
              "The maximum allowed planner iteration time in seconds.");

DEFINE_double(planner_max_localization_transform_delay, 5.0,
              "The max delay allowed to use a localization transform message.");
DEFINE_double(planner_max_perception_delay, 3.0,
              "The max delay allowed for perception message.");
DEFINE_double(planner_max_pose_delay, 0.1,
              "The max delay allowed for pose message in seconds.");

DEFINE_bool(planner_allow_async_in_main_thread, true,
            "Whether to allow async operations in planner\'s main thread.");

DEFINE_bool(planner_allow_multi_threads_in_est, true,
            "Whether to allow multiple threads in est-planner.");

DEFINE_bool(planner_run_prev_traj_async, true,
            "Whether to run prev traj planner asynchronously.");

DEFINE_bool(planner_multi_est_in_parallel, true,
            "Whether to run multiple est-planners in parallel.");

DEFINE_int32(planner_async_low_freq_cycle_iterations, 2,
             "The iteration that path planner wait when running in async mode. "
             "When it is 0, path and speed runs in sequential mode.");

DEFINE_int32(planner_alcc_async_low_freq_cycle_iterations, 2,
             "Path planner iter for alcc task in async mode.");

DEFINE_int32(planner_mapless_async_low_freq_cycle_iterations, 2,
             "Path planner iter for mapless task in async mode.");

DEFINE_int32(planner_max_cruise_async_iterations, 3,
             "Max allowable low freq iteration for cruise task.");

DEFINE_int32(planner_max_alcc_async_iterations, 2,
             "Max allowable low freq iteration for alcc task.");

DEFINE_int32(planner_max_mapless_async_iterations, 2,
             "Max allowable low freq iteration for mapless task.");

DEFINE_bool(
    planner_consider_objects, true,
    "Planner consider objects in perception and prediction. This is "
    "useful for testing mode that does not have perception or prediction.");

DEFINE_double(
    planner_check_trajectory_engage_condition_duration, 1.0,
    "Validate if we can engage in the trajectory's first this amount of time.");

DEFINE_double(
    planner_filter_reflected_object_distance, 0.3,
    "Filter reflected objects that in AV's current position's proximity and "
    "the distance to SDC is less than this value (in meters). Reflected "
    "objects are fake unknown stationary perception objects "
    "that are near SDC body. When this value is less than zero, we "
    "will not filter reflected object.");

DEFINE_bool(planner_filter_unknown_roadway_position_object, true,
            "Filter object that has classification of "
            "RoadWayPositionType::RWPT_UNKNOWN");

DEFINE_bool(planner_filter_static_object, false, "Filter static object");

DEFINE_int32(planner_lookforward_time_ms, 200,
             "The planner look forward time when computing trajectory.");

DEFINE_double(planner_prediction_probability_threshold, 0.1,
              "The threshold on probability below which a predicted behavior / "
              "trajectory will be ignored (use at your risk)");

DEFINE_bool(planner_only_use_most_likely_trajectory, false,
            "Only use the most likely trajectory.");

DEFINE_bool(planner_open_door_at_route_end, false,
            "Open door when AV is at end of route and fully stopped.");

DEFINE_double(
    planner_door_state_override_waiting_time, 40.0,
    "Don't change door state for so many seconds after door state override. "
    "This value should be less than the duration for the AV to travel from one "
    "station to another. This is used to prevent software open door "
    "immediately after driver closed the door.");

DEFINE_bool(planner_enable_occluded_objects_inference, true,
            "Enable reasoning occluded objects according sensor fov.");
DEFINE_bool(
    planner_enable_crosswalk_occluded_objects_inference, false,
    "Enable reasoning occluded objects according sensor fov on crosswalks.");

DEFINE_bool(
    planner_ignore_stalled_objects_on_tl_controlled_leftmost_lane, false,
    " Don't make stalled object decision for  objects on left most lane");

DEFINE_bool(
    planner_enable_un_tl_controlled_intersection_reasoning, true,
    "Enable scene reasoning on un traffic light controlled intersection.");

DEFINE_bool(planner_enable_bus_station_stalled_object_filter, false,
            "Enable bus station stalled object filter on L4, default false.");

DEFINE_int32(planner_task_init_type, 1,
             "0: NOA only; 1:Dynamic switch; 2: ACC only; 3:LCC only; "
             "4:Mapless NOA; 5:L2 APA");

DEFINE_bool(planner_export_all_prediction_to_speed_considered, false,
            "Whether to send all time-aligned predictions to speed considered "
            "prediction. Set it true only for simulation debugging purpose.");

DEFINE_bool(planner_check_aeb, true, "Whether to check emergency stop.");
DEFINE_int32(planner_check_aeb_curb_traj_horizon, 20,
             "Timestep num for trajectory curb collision");

DEFINE_bool(planner_publish_chart_data, true,
            "Whether to publish charts data proto.");

DEFINE_bool(planner_simplify_debug_proto, false,
            "Only return limited fields in planenr debug proto if true.");

DEFINE_bool(planner_enable_tja_in_alcc_task, true,
            "If true, use tja function in alcc task when online map fails.");
DEFINE_int32(planner_running_platform, 0, "0: IPC; 1: Orin; 2: X9; 3: S32G");

// Scheduler.
DEFINE_int32(
    planner_est_parallel_branch_num, 2,
    "The maximal amount of target lane paths to be generated for multi-task "
    "est planner. Note that a borrow-lane branch will be created if this flag "
    "is set to 1, so if only one branch is desired, you should also set the "
    "next flag to false to prohibit lane borrowing.");
DEFINE_bool(planner_est_scheduler_seperate_lc_pause, false,
            "Whether to create a seperate lc pause scheduler branch along with "
            "a lc executing branch.");
DEFINE_bool(planner_est_scheduler_allow_borrow, true,
            "Whether to allow a borrow path boundary if only one target lane "
            "is chosen.");
DEFINE_bool(planner_send_lane_graph_to_canvas, false,
            "Whether to send lane graph to canvas.");
DEFINE_bool(planner_consider_all_lanes_virtual, false,
            "Whether to consider all lanes\' type as VIRTUAL, for scenarios "
            "with one single lane for mixed use.");
DEFINE_int32(planner_local_lane_map_debug_level, 0,
             "Debug level for local lane map: 0: no debug info; 1: send to "
             "canvas for viz; 2: some other options");
DEFINE_int32(planner_drive_passage_debug_level, 0,
             "Debug level for drive passage: 0: no debug info; "
             "1: send to canvas for viz; 2: some other options)");
DEFINE_bool(
    planner_enable_path_boundary_debug, false,
    "Enable debug for path boundary. Will draw infos on canvas if enabled.");

DEFINE_bool(
    planner_lc_prepare_when_branch_invalid, true,
    "Enter lane change prepare state when cannot find a valid target branch.");

// Decision
DEFINE_bool(planner_decision_enable_stop_sign, false,
            "Whether enable stop sign decider, default false.");

// Initializer
DEFINE_int32(planner_initializer_debug_level, 0,
             "Initializer debug level: 0: Nothing; 1: Debug info for world "
             "renderer; 2: Debug info for canvas and terminal.");
DEFINE_bool(planner_initializer_only_activate_nodes_near_capnet_traj, false,
            "Only activate geometry nodes near the reference traj provided by "
            "captain net for motion search.");
DEFINE_bool(
    planner_initializer_only_activate_nodes_near_refline, false,
    "Only activate geometry nodes near the reference line for motion search.");
DEFINE_int32(
    planner_initializer_max_multi_traj_num, 3,
    "The maximal amount of leading groups that should be considered for "
    "initializer multiple trajectory selection in certain situations.");
DEFINE_bool(planner_initializer_enable_post_evaluation, false,
            "Whether to re-evaluate the top k trajectories from the hand-tuned "
            "initializer params with learned params and select the best as the "
            "final output.");
DEFINE_bool(planner_initializer_enable_clip, true,
            "Whether to enable to clip initializer for faster calculation.");
DEFINE_bool(
    planner_initializer_astar_inspired_by_reference_line, true,
    "Whether to use reference line and reference speed to inspire Astar");

// Offline DataDumping
DEFINE_bool(
    dumping_initializer_features, false,
    "Whether reading and dumping initializer features' cost of manual driving "
    "trajectory for offline learning");

DEFINE_bool(
    dumping_selector_features, false,
    "Whether reading and dumping selector features' cost of manual driving "
    "trajectory for offline learning");

DEFINE_bool(filter_selector_intention, false,
            "Whether filtering the selector features dumping based on if their "
            "intentions are same as expert. Only work when "
            "dumping_selector_features is true.");

DEFINE_bool(
    planner_dumping_ml_data_in_simulation, false,
    "Whether to use oracle trajectory and dump data for ml to planner_debug");

// Spacetime flags.
DEFINE_bool(planner_alcc_use_st_traj_cutin_filter, false,
            "Whether to filter cutin spacetime trajectories for alcc task.");

// Dopt auto tuning.
DEFINE_bool(auto_tuning_mode, false,
            "When auto tuning mode is on, the optimizer will generate and save "
            "one more output which are the accumulated discounted costs for "
            "different cost type.");
DEFINE_bool(optimizer_data_cleaning, false,
            "Whether to do data cleaning/filtering for optimizer and run "
            "snapshot in optimizer data cleaning mode.");
DEFINE_bool(
    update_learned_alphas, false,
    "Whether to use the cost weight alphas learned in auto tuning mode.");
DEFINE_bool(
    update_learned_alphas_except_lane_change, true,
    "Whether to use the cost weight alphas learned in auto tuning mode when "
    "lane change. Only worked when update_learned_alphas is true. So this will "
    "not influence training process but will influence "
    "evaluation/validation/testing.");
DEFINE_string(traj_opt_params_file_address,
              "onboard/planner/ml/optimizer_auto_tuning/traj_opt_params.pb.txt",
              "The address of the trajectory optimizer params proto file.");
DEFINE_bool(
    compare_different_weight, false,
    "Whether to compare different cost weight learned in auto tuning mode.");
DEFINE_bool(compare_based_on_original_weight, true,
            "Whether to compare the cost weight based on original cost "
            "weight(true) or auto tuned cost weight(false), only works when "
            "compare_different_weight is true.");

// Selector auto tuning.
DEFINE_bool(use_tuned_selector_params, false,
            "Whether to use the auto tuned selector cost weight params.");
DEFINE_string(
    selector_params_file_address,
    "offboard/planner/ml/models/selector_auto_tuning/selector_params.pb.txt",
    "The address of the auto tuned selector params proto file.");

DEFINE_double(planner_path_start_point_time_diff_limit, 0.5,
              "If relative time of closest point on prev traj from plan "
              "start point larger than this time, set path plan start point to "
              "current close point.");
DEFINE_bool(
    enable_path_start_point_look_ahead, true,
    "Whether to use logic about planner_path_start_point_time_diff_limit.");

DEFINE_int32(planner_runtime_uturn_level, 1,
             "0: Disable three point turn by force; 1: Enable by params; 2: "
             "Enable by force.");

DEFINE_bool(planner_enable_dynamic_lane_speed_limit, true,
            "Whether to enable dynamic lane speedlimit");

DEFINE_double(planner_override_lane_speed_limit_proportion, 0.0,
              "Modify lane speed limit by a given proportion");

// Planner ml inference.
DEFINE_bool(planner_enable_selector_scoring_net, false,
            "Whether to enable the inference of selector scoring model.");
DEFINE_bool(planner_enable_captain_net, false,
            "Whether to enable the inference of CaptainNet model.");
DEFINE_bool(planner_enable_captain_net_onnx_trt, true,
            "Whether to enable the onnx & trt inference of CaptainNet model.");
DEFINE_bool(planner_enable_captain_net_j5, false,
            "Enable actnet in j5 or x86 sim j5");
DEFINE_bool(planner_use_ml_trajectory_end_to_end, false,
            "Whether to use model generated trajectory as the final output.");
DEFINE_bool(planner_use_ml_trajectory_as_initializer_ref_traj, true,
            "Whether to use model generated trajectory as the reference "
            "trajectory of initializer.");
DEFINE_bool(
    planner_use_ml_trajectory_to_derive_leading_objects, true,
    "Whether to use model generated trajectory to get leading objects.");
DEFINE_bool(planner_use_ml_trajectory_as_optimizer_ref_traj, false,
            "Whether to use model generated trajectory as the reference "
            "trajectory of optimizer.");
DEFINE_bool(planner_captain_net_align_traj_based_on_time_for_all_points, true,
            "Whether to align captain net trajectory based on time for "
            "all point.");
DEFINE_bool(
    planner_captain_net_align_traj_based_on_time_for_first_point, false,
    "Whether to align captain net trajectory based on time for first point.");
DEFINE_bool(planner_captain_net_post_process_movability_issue, true,
            "Whether to post process the captain net trajectory if it has "
            "movability issue.");
DEFINE_bool(
    planner_capnet_ref_traj_use_mahalanobis_distance, false,
    "Whether to use mahalanobis distance when using captain net trajectory as "
    "initializer reference trajectory.");
DEFINE_bool(planner_captain_net_use_dkm, true,
            "Whether to post process captain net output as dkm or not.");
DEFINE_bool(
    planner_enable_act_net_speed, false,
    "Whether to use act net speed predictor in planner speed decision.");

DEFINE_bool(planner_rebuild_route_navi_info, false,
            "alway rebuild route navi info in snapshot");

// Planner selector
DEFINE_bool(planner_enable_lane_change_in_intersection, true,
            "whether to allow lane change in intersection");
DEFINE_bool(planner_enable_cross_solid_boundary, true,
            "whether to allow cross solid boundary");
DEFINE_bool(planner_enable_obstacle_lane_change, true,
            "whether to allow lane change for obstacle");
DEFINE_int32(planner_begin_lane_change_frame, 6,
             "Can not change until lane change trajectory is better than "
             "lane keep trajectory in successive frames");
DEFINE_int32(planner_begin_radical_lane_change_frame, 2,
             "In certain situations, accelerate lane change decision.");
DEFINE_int32(planner_begin_signal_frame, 4,
             "Turn pre lane change when lane change trajectory is better than "
             "lane keep trajectory in successive frames");
DEFINE_bool(planner_enable_lc_request_in_tricky_scenario, true,
            "whether to allow send lc request in tricky scenario");
DEFINE_double(planner_allow_lc_time_after_activate_selector, 2.0,
              "After activating selector, how long lane change is allowed");
DEFINE_double(planner_max_allow_lc_time_before_give_up, 15.0,
              "How long is maximum allowed duration for a lane change");
DEFINE_double(planner_allow_lc_time_after_give_up_lc, 15.0,
              "After lc giving up,how long lane change is allowed");
DEFINE_int32(planner_lc_begin_request_frame_in_tricky_scenario, 2,
             "In tricky scenario, when to send lane change request");
DEFINE_bool(planner_enable_prefilter_for_selector, false,
            "whether to prefilter 2 trjectory for selector.");
DEFINE_double(planner_allow_opposite_lc_time_after_paddle_lc, 30.0,
              "After paddle lc, how long opposite lane change is allowed");

DEFINE_bool(
    planner_enable_cross_iteration_tf, true,
    "Whether to enable cross-interation smooth coordinate transformation to "
    "compensate for smooth coordinate origin drift.");

// L2 related flag to simulate different situation
DEFINE_bool(planner_force_no_map, false, "Force to not use map.");

// Lane change style setting.
DEFINE_bool(planner_enable_lc_style_params, true,
            "Whether to enable stylistic planner params in lc.");

DEFINE_double(planner_paddle_lane_change_max_prepare_time, 6.0,
              "Max allowed duration for staying in ALC_PREPARE state after "
              "having triggered paddle lane change.");

DEFINE_bool(planner_use_traffic_gap_finder_v2, true,
            "Choose between traffic gap finder v1 and v2.");

// Use previous optimization result as init solution.
DEFINE_bool(
    st_path_planner_lookahead_for_trajectory_optimizer_synchronization, true,
    "When enabled, st path planner start point will be further prolonged.");

DEFINE_bool(traj_opt_init_traj_uses_last_optimized_trajectory, true,
            "If enabled, optimizer will try to "
            "use last optimized trajectory as its init solution.");

// Map preprocessing.
DEFINE_bool(planner_force_route_filtered_smm, false,
            "if enabled, filter elements in smm with route.");

DEFINE_bool(planner_use_lane_change_style_from_hmi, true,
            "If enabled, use lane change style from hmi.");

DEFINE_double(
    planner_alcc_prev_traj_planner_max_time, 1.0,
    "The max duration that the previous-trajectory planner is allowed "
    "to be consecutively used for ALCC.");

DEFINE_bool(
    planner_enable_online_map_auto_correction, false,
    "Move path boundary center to predict online map drifting if enabled.");

DEFINE_bool(planner_enable_route_lane_change_fail, true,
            "Whether to enable route lane change fail signal.");

// Freespace planner.
DEFINE_bool(planner_freespace_path_stop_mode, false,
            "Whether to enable freespace planner path-stop mode, where only "
            "path and stop distance is sent instead of trajectory.");

DEFINE_bool(planner_debug_force_acc_in_lcc, false,
            "Force to perform ACC plan after LCC plan, debug only.");

DEFINE_bool(planner_enable_collision_risk, false,
            "Whether to enable collision risk function.");
