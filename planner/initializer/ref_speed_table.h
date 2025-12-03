#ifndef ONBOARD_PLANNER_INITIALIZER_REF_SPEED_TABLE_H_
#define ONBOARD_PLANNER_INITIALIZER_REF_SPEED_TABLE_H_

#include <string>
#include <utility>
#include <vector>

#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/router/drive_passage.h"

namespace qcraft::planner {

class RefSpeedVec {
 public:
  RefSpeedVec(const DrivePassage& drive_passage,
              const std::vector<std::pair<double, double>>& obj_info,
              double stop_s);
  double FastComputeRefSpeed(double s) const;

 private:
  double start_s_, end_s_;
  std::vector<double> discretized_ref_speed_by_s_;
};

class RefSpeedTable {
 public:
  RefSpeedTable(const SpacetimeTrajectoryManager& st_traj_mgr,
                const std::vector<std::string>& leading_objs,
                const DrivePassage& drive_passage,
                const std::vector<double>& stop_s);
  // first - speed_limit, second - ref_speed
  std::pair<double, double> LookUpRefSpeed(double time, double span) const;

  const std::vector<RefSpeedVec>& ref_speed_table() const {
    return ref_speed_table_;
  }

 private:
  std::vector<double> station_accum_s_, station_speed_limits_;
  std::vector<RefSpeedVec> ref_speed_table_;
};

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_INITIALIZER_REF_SPEED_TABLE_H_
