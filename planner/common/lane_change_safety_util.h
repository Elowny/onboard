#ifndef ONBOARD_PLANNER_COMMON_LANE_CHANGE_SAFETY_UTIL_H_
#define ONBOARD_PLANNER_COMMON_LANE_CHANGE_SAFETY_UTIL_H_

#include "onboard/math/frenet_common.h"
#include "onboard/planner/object/planner_object.h"
#include "onboard/proto/driving_style.pb.h"
namespace qcraft::planner {
double ComputeEgoLeadTime(double speed_limit, double ego_v, double obj_v);

double ComputeEgoFollowTime(double obj_v, double ego_v);

double ComputeLcConservFactor(const FrenetBox& av_box, double av_half_width,
                              bool lc_left, LaneChangeStyle lc_style);

double ComputeSafeResponseInterval(double v_lead, double v_follow,
                                   double response_time, double lead_time);

double ComputeSafeDecelerationInterval(double v_lead, double v_follow,
                                       double max_allowed_deceleration);

double EstimateObjectSpeed(const PlannerObject& object, double preview_time);
}  // namespace qcraft::planner
#endif
