#include "onboard/planner/plan/acc/test_util.h"

#include "onboard/planner/speed/proto/speed_finder_params.pb.h"
#include "onboard/utils/file_util.h"
#include "onboard/utils/proto_util.h"

namespace qcraft {
namespace planner {

PlannerParamsProto CreateDefaultPlannerParamOnlyFillAccParams() {
  PlannerParamsProto default_planner_params;
  QCHECK(file_util::FileToProto(
      "onboard/planner/params/planner_default_params.pb.txt",
      &default_planner_params));
  // Fill default motion constraint params into default motion constraint
  // params.
  FillInMissingFieldsWithDefault(
      default_planner_params.motion_constraint_params(),
      default_planner_params.mutable_acc_params()
          ->mutable_motion_constraint_params());
  // Fill default speed finder params into default speed finder params.
  SpeedFinderParamsProto default_speed_finder_params;
  QCHECK(file_util::FileToProto(
      "onboard/planner/params/speed_finder_default_params.pb.txt",
      &default_speed_finder_params));
  FillInMissingFieldsWithDefault(default_speed_finder_params,
                                 default_planner_params.mutable_acc_params()
                                     ->mutable_speed_finder_params());
  // Fill customer requirement params.
  QCHECK(file_util::FileToProto(
      "onboard/planner/params/acc_req_params.pb.txt",
      default_planner_params.mutable_acc_params()->mutable_acc_req_params()));
  return default_planner_params;
}

}  // namespace planner
}  // namespace qcraft
