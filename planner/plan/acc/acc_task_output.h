#ifndef ONBOARD_PLANNER_PLAN_ACC_ACC_TASK_OUTPUT_H_
#define ONBOARD_PLANNER_PLAN_ACC_ACC_TASK_OUTPUT_H_

#include "onboard/proto/charts.pb.h"
#include "onboard/proto/hmi_content.pb.h"
#include "onboard/proto/planner.pb.h"
#include "onboard/proto/trajectory.pb.h"

namespace qcraft::planner {

struct AccTaskOutput {
  TrajectoryProto trajectory_info;
  // TODO(weijun): min dependency.
  PlannerDebugProto debug_info;
  vis::vantage::ChartsDataProto chart_data;
  HmiContentProto hmi_content;
  QACCTaskProto acc_task_proto;
  QACCState acc_state = QACCState::ACC_OFF;
  std::optional<double> cruising_speed_limit = std::nullopt;
};

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_PLAN_ACC_ACC_TASK_OUTPUT_H_
