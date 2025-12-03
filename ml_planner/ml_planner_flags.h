#ifndef ONBOARD_ML_PLANNER_ML_PLANNER_FLAGS_H_
#define ONBOARD_ML_PLANNER_ML_PLANNER_FLAGS_H_

#include "gflags/gflags.h"

namespace qcraft {
namespace mlplanner {

DECLARE_int32(ml_planner_thread_pool_size);

DECLARE_bool(ml_planner_use_autonomy_state);

// schedule J5
DECLARE_bool(ml_planner_enable_debug_no_map);

// planner related
DECLARE_double(ml_planner_filter_reflected_object_distance);
DECLARE_double(ml_planner_prediction_probability_threshold);
DECLARE_bool(ml_planner_filter_unknown_roadway_position_object);
DECLARE_bool(ml_planner_enable_captain_net_j5);
DECLARE_bool(ml_planner_enable_captain_net_onnx_trt);
DECLARE_bool(ml_planner_captain_net_use_dkm);

}  // namespace mlplanner
}  // namespace qcraft

#endif  // ONBOARD_ML_PLANNER_ML_PLANNER_FLAGS_H_
