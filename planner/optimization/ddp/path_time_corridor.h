#ifndef ONBOARD_PLANNER_OPTIMIZATION_DDP_PATH_TIME_CORRIDOR_H_
#define ONBOARD_PLANNER_OPTIMIZATION_DDP_PATH_TIME_CORRIDOR_H_

#include <limits>
#include <map>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "absl/status/statusor.h"

#include "onboard/math/frenet_frame.h"
#include "onboard/planner/common/path_sl_boundary.h"
#include "onboard/planner/decision/proto/constraint.pb.h"
#include "onboard/planner/object/spacetime_object_trajectory.h"
#include "onboard/planner/object/spacetime_planner_object_trajectories.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/planner/trajectory_point.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {
namespace planner {
namespace optimizer {

class PathTimeCorridor {
 public:
  struct BoundaryInfo {
    enum Type {
      kCurb = 0,
      kLaneBoundary = 1,
      kObject = 2,
      kUnknown = 3,
    };
    Type type;
    double l_boundary;
    double l_curb;
    double l_object;
    const SpacetimeObjectTrajectory* object_ptr = nullptr;

    static Type GetType(StationBoundaryType type) {
      switch (type) {
        case StationBoundaryType::BROKEN_WHITE:
        case StationBoundaryType::SOLID_WHITE:
        case StationBoundaryType::BROKEN_YELLOW:
        case StationBoundaryType::SOLID_YELLOW:
        case StationBoundaryType::SOLID_DOUBLE_YELLOW:
        case StationBoundaryType::BROKEN_LEFT_DOUBLE_WHITE:
        case StationBoundaryType::BROKEN_RIGHT_DOUBLE_WHITE:
        case StationBoundaryType::UNKNOWN_TYPE:
          return Type::kLaneBoundary;
          break;
        case StationBoundaryType::CURB:
        case StationBoundaryType::VIRTUAL_CURB:
        case StationBoundaryType::PREDICTION_VIRTUAL_CURB:
          return Type::kCurb;
          break;
      }
    }
    static BoundaryInfo CreateDefaultLeftBoundaryInfo() {
      return BoundaryInfo{.type = BoundaryInfo::Type::kLaneBoundary,
                          .l_boundary = std::numeric_limits<double>::infinity(),
                          .l_curb = std::numeric_limits<double>::infinity(),
                          .l_object = std::numeric_limits<double>::infinity(),
                          .object_ptr = nullptr};
    }
    static BoundaryInfo CreateDefaultRightBoundaryInfo() {
      return BoundaryInfo{
          .type = BoundaryInfo::Type::kLaneBoundary,
          .l_boundary = -std::numeric_limits<double>::infinity(),
          .l_curb = -std::numeric_limits<double>::infinity(),
          .l_object = -std::numeric_limits<double>::infinity(),
          .object_ptr = nullptr};
    }
  };

  PathTimeCorridor(const PathSlBoundary* path_sl_boundary,
                   std::vector<std::vector<BoundaryInfo>> left_boundary,
                   std::vector<std::vector<BoundaryInfo>> right_boundary,
                   std::vector<double> time_points);

  std::pair<const BoundaryInfo*, const BoundaryInfo*> QueryBoundaryL(
      double s_start, double s_end, double t, int* min_lane_width_idx) const;

  std::pair<BoundaryInfo, BoundaryInfo> QueryNarrowestBoundaryAllTypes(
      double s_start, double s_end, double t) const;

 private:
  const PathSlBoundary* path_sl_boundary_;
  std::vector<std::vector<BoundaryInfo>> left_boundary_;
  std::vector<std::vector<BoundaryInfo>> right_boundary_;
  std::vector<double> time_points_;
};

absl::StatusOr<PathTimeCorridor> BuildPathTimeCorridor(
    std::string_view base_name, const std::vector<TrajectoryPoint>& init_traj,
    const DrivePassage& drive_passage, const PathSlBoundary& path_sl_boundary,
    const FrenetFrame& init_traj_frenet_frame,
    const std::map<std::string, ConstraintProto::LeadingObjectProto>&
        leading_trajs,
    const SpacetimePlannerObjectTrajectories& st_planner_object_traj,
    const VehicleGeometryParamsProto& veh_geo_params);

}  // namespace optimizer
}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_OPTIMIZATION_DDP_PATH_TIME_CORRIDOR_H_
