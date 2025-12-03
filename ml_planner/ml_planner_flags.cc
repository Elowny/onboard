#include "onboard/ml_planner/ml_planner_flags.h"

namespace qcraft {
namespace mlplanner {

DEFINE_int32(ml_planner_thread_pool_size, 2,
             "The thread num of the ml planner thread pool");

DEFINE_bool(ml_planner_use_autonomy_state, true,
            "Use autonomy state to switch scheduler.");

DEFINE_bool(ml_planner_enable_debug_no_map, false,
            "Enable ml planner noa scheduler using no map.");

DEFINE_double(
    ml_planner_filter_reflected_object_distance, 0.3,
    "Filter reflected objects that in AV's current position's proximity and "
    "the distance to SDC is less than this value (in meters). Reflected "
    "objects are fake unknown stationary perception objects "
    "that are near SDC body. When this value is less than zero, we "
    "will not filter reflected object.");

DEFINE_double(ml_planner_prediction_probability_threshold, 0.1,
              "The threshold on probability below which a predicted behavior / "
              "trajectory will be ignored (use at your risk)");

DEFINE_bool(ml_planner_filter_unknown_roadway_position_object, true,
            "Filter object that has classification of "
            "RoadWayPositionType::RWPT_UNKNOWN");

DEFINE_bool(ml_planner_enable_captain_net_j5, true,
            "Enable actnet in j5 or x86 sim j5");
DEFINE_bool(ml_planner_enable_captain_net_onnx_trt, false,
            "Whether to enable the onnx & trt inference of CaptainNet model.");
DEFINE_bool(ml_planner_captain_net_use_dkm, true,
            "Whether to post process captain net output as dkm or not.");

}  // namespace mlplanner
}  // namespace qcraft
