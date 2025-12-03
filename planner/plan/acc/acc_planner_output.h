#ifndef ONBOARD_PLANNER_PLAN_ACC_ACC_PLANNER_OUTPUT_H_
#define ONBOARD_PLANNER_PLAN_ACC_ACC_PLANNER_OUTPUT_H_

#include <map>
#include <memory>
#include <utility>
#include <vector>

#include "common/proto/qacc.pb.h"

#include "onboard/planner/common/planner_status.h"
#include "onboard/planner/plan/acc/acc_corridor.h"
#include "onboard/planner/plan/acc/acc_speed_finder_output.h"
#include "onboard/planner/plan/acc/acc_target.h"
#include "onboard/planner/proto/trajectory_validation.pb.h"
#include "onboard/proto/trajectory_point.pb.h"

namespace qcraft::planner {
struct AccPlannerOutput {
  AccSpeedFinderOutput acc_speed_output;
  AccCorridorSource active_corridor_source;
  std::vector<ApolloTrajectoryPointProto> acc_past_points;
  TrajectoryValidationResultProto traj_validation_result;

  std::map<AccCorridorSource, std::unique_ptr<AccCorridor>> corridors;
  std::map<AccCorridorSource, std::unique_ptr<AccTargetPerCorridor>> targets;
  std::map<AccCorridorSource, PlannerStatus> source_status;

  void ToDebugProto(QACCDebugProto* proto) const {
    proto->set_active_corridor_source(active_corridor_source);
    *proto->mutable_speed_debug() = acc_speed_output.speed_finder_proto;
    *proto->mutable_preliminary_speed_debug() =
        acc_speed_output.preliminary_speed_debug;

    for (const auto& [source, status] : source_status) {
      PlannerStatusProto status_proto;
      status.ToProto(&status_proto);
      proto->mutable_source_status()->insert(
          {AccCorridorSource_Name(source), status_proto});
    }

    for (const auto& [source, corridor] : corridors) {
      if (corridor.get() != nullptr) {
        AccCorridorProto corridor_proto;
        corridor->ToProto(&corridor_proto);
        proto->mutable_source_corridors()->insert(
            {AccCorridorSource_Name(source), corridor_proto});
      }
    }
    for (const auto& [source, target] : targets) {
      if (target.get() != nullptr) {
        QACCTargetsProto targets_proto;
        target->ToProto(&targets_proto);
        proto->mutable_source_targets()->insert(
            {AccCorridorSource_Name(source), targets_proto});
      }
    }
  }
};

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_PLAN_ACC_ACC_PLANNER_OUTPUT_H_
