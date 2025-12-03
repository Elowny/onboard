#ifndef ONBOARD_PLANNER_PLAN_ACC_TARGET_H_
#define ONBOARD_PLANNER_PLAN_ACC_TARGET_H_

#include <limits>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "absl/time/time.h"

#include "common/proto/qacc.pb.h"

#include "onboard/params/vehicle_param_api.h"
#include "onboard/planner/object/planner_object.h"
#include "onboard/planner/object/planner_object_manager.h"
#include "onboard/planner/object/proto/planner_object.pb.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/plan/acc/acc_corridor.h"
#include "onboard/prediction/predicted_trajectory.h"
#include "onboard/proto/positioning.pb.h"

namespace qcraft::planner {

struct AccTargetDecision {
  std::string object_id;
  std::string traj_id;

  void ToProto(QACCTargetProto* target_proto) const {
    target_proto->set_object_id(object_id);
    target_proto->set_traj_id(traj_id);
  }

  void FromProto(const QACCTargetProto& target_proto) {
    object_id = target_proto.object_id();
    traj_id = target_proto.traj_id();
  }
};

struct AccTargetInput {
  const AccCorridor* corridor = nullptr;
  const SpacetimeTrajectoryManager* st_traj_mgr = nullptr;
  const std::vector<AccTargetDecision>* prev_primary_targets = nullptr;
  const PoseProto* av_pose = nullptr;
  const VehicleGeometryParamsProto* vehicle_geom = nullptr;
  const ApolloTrajectoryPointProto* plan_start_point = nullptr;
};

struct AccTargetPerCorridor {
  // Objects filtered by corridor outer boundaries.
  std::vector<AccTargetDecision> leaders;
  std::vector<AccTargetDecision> potential_targets;

  absl::Time timestamp;  // Microseconds in proto.
  AccCorridorSource source;

  std::unique_ptr<SpacetimeTrajectoryManager> acc_st_traj_mgr;

  void ToProto(QACCTargetsProto* targets_proto) const {
    targets_proto->Clear();
    // Generate proto message according to targets choices and timestamp.
    for (const auto& l : leaders) {
      l.ToProto(targets_proto->add_leaders());
    }
    for (const auto& pc : potential_targets) {
      pc.ToProto(targets_proto->add_potential_targets());
    }
    targets_proto->set_timestamp(absl::ToUnixMicros(timestamp));
    targets_proto->set_source(source);
  }

  const SpacetimeTrajectoryManager* st_traj_mgr() const {
    return acc_st_traj_mgr.get();
  }

  bool has_leaders() const { return !leaders.empty(); }
};
}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_PLAN_ACC_TARGET_H_
