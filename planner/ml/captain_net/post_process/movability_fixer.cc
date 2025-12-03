#include "onboard/planner/ml/captain_net/post_process/movability_fixer.h"

#include <cmath>   // for abs
#include <vector>  // for vector

#include "onboard/math/vec.h"  // for Vec2d
#include "onboard/planner/planner_flags.h"  // for FLAGS_planner_captain_net_post_process_movability_issue
#include "onboard/planner/trajectory_util.h"             // for LerpTrajPoint
#include "onboard/planner/util/vehicle_geometry_util.h"  // for ComputeCenterMaxCurvature
#include "onboard/proto/trajectory_point.pb.h"  // for ApolloTrajectoryPointProto, PathPoint

namespace qcraft::planner::ml {

void PostProcessTrajectoryMovability(
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const VehicleDriveParamsProto& vehicle_drive_params,
    captain_net::CaptainNetOutput* output) {
  constexpr double kCurvatureRelaxFactor = 10.0;
  const double curvature_threshold =
      kCurvatureRelaxFactor *
      ComputeCenterMaxCurvature(vehicle_geometry_params, vehicle_drive_params);
  double max_abs_kappa = 0.0;
  int i = 1;
  const auto& traj_points = output->traj_points;
  while (i < traj_points.size() &&
         Vec2d(traj_points[0].path_point().x(), traj_points[0].path_point().y())
                 .DistanceTo(Vec2d(traj_points[i].path_point().x(),
                                   traj_points[i].path_point().y())) <=
             vehicle_geometry_params.length()) {
    const double cur_abs_kappa = std::abs(traj_points[i].path_point().kappa());
    if (cur_abs_kappa > max_abs_kappa) {
      max_abs_kappa = cur_abs_kappa;
    }
    i++;
  }
  if (max_abs_kappa < curvature_threshold) {
    return;
  }

  output->validation.is_kinematic_infeasible = true;

  if (FLAGS_planner_captain_net_post_process_movability_issue) {
    i--;
    const auto alpha_step = 1.0 / i;
    double alpha = 0.0;
    for (int idx = 1; idx < i; ++idx) {
      alpha += alpha_step;
      LerpTrajPoint(traj_points[0], traj_points[i], alpha,
                    &output->traj_points[idx]);
    }
    output->is_post_processed = true;
  }
}

}  // namespace qcraft::planner::ml
