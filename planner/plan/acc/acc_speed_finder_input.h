#ifndef ONBOARD_PLANNER_SPEED_ACC_SPEED_FINDER_INPUT_H_
#define ONBOARD_PLANNER_SPEED_ACC_SPEED_FINDER_INPUT_H_

#include <map>
#include <set>
#include <string>
#include <vector>

#include "absl/container/flat_hash_set.h"

#include "onboard/planner/common/discretized_path.h"
#include "onboard/planner/decision/constraint_manager.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/planner/speed/proto/speed_finder_params.pb.h"
#include "onboard/planner/speed/speed_limit.h"
#include "onboard/planner/speed/speed_vector.h"
#include "onboard/proto/planner.pb.h"
#include "onboard/proto/vehicle.pb.h"
namespace qcraft::planner {

struct AccSpeedFinderInput {
  std::string base_name;
  const SpacetimeTrajectoryManager* traj_mgr = nullptr;
  const DiscretizedPath* path = nullptr;
  const SpeedLimit* speed_limit = nullptr;
  const std::vector<ApolloTrajectoryPointProto>* time_aligned_prev_traj =
      nullptr;
  double plan_start_v = 0.0;
  double plan_start_a = 0.0;
  double user_speed_limit = 0.0;  // m/s.
  absl::Time plan_time;
  const std::set<std::string>* crowded_side_objs = nullptr;
  const MotionConstraintParamsProto* motion_constraint_params = nullptr;
  const SpeedFinderParamsProto* speed_finder_params = nullptr;
  const VehicleGeometryParamsProto* vehicle_geom = nullptr;
};

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_SPEED_ACC_SPEED_FINDER_INPUT_H_
