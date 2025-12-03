#include "onboard/planner/freespace/ipopt_global_path_smoother.h"

#include <algorithm>
#include <cmath>
#include <ostream>
#include <string>
#include <utility>

#define HAVE_STDDEF_H
#include "absl/status/status.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "coin/IpIpoptApplication.hpp"
#include "coin/IpOptionsList.hpp"
#include "coin/IpReturnCodes.hpp"
#include "coin/IpSmartPtr.hpp"
#include "coin/IpTNLP.hpp"
#include "glog/logging.h"

#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/math/util.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/vis/canvas/canvas.h"
#include "onboard/vis/common/color.h"

#include "offboard/vis/vantage/vantage_server/vantage_canvas_client.h"

#undef HAVE_STDDEF_H

#include "onboard/math/vec.h"
#include "onboard/planner/freespace/ipopt_global_path_smoother_model.h"
#include "onboard/planner/optimization/ipopt/ipopt_util.h"
#include "onboard/planner/util/path_util.h"
#include "onboard/planner/util/vehicle_geometry_util.h"

namespace qcraft {
namespace planner {
namespace {

using ModelInput = IpoptGlobalPathSmootherInput;

constexpr double kDefaultPosVelocity = 1.0;   // m/s.
constexpr double kDefaultNegVelocity = -1.0;  // m/s.
constexpr double kMaxPosVelocity = 3.0;       // m/s.
constexpr double kMaxNegVelocity = -2.0;      // m/s.
constexpr double kStepSize = 0.2;             // m.
constexpr double kEps = 1e-6;

constexpr int kPrintLevel = 0;
constexpr int kMaxIterator = 3000;
constexpr double kTol = 1e-4;

void SendToCanvas(const std::vector<PathPoint>& path_points) {
  std::vector<Vec3d> points;
  points.reserve(path_points.size());
  for (const auto& pt : path_points) {
    points.emplace_back(pt.x(), pt.y(), 0.0);
  }
  auto* canvas = &(vis::vantage::GetCanvasClient()->GetCanvas(
      "planner/global_path_smoother"));
  canvas->SetGroundZero(1);
  canvas->DrawLineStrip(points, qcraft::vis::Color::kRed);
  for (const auto& point : points) {
    canvas->DrawCircle(point, 0.08, qcraft::vis::Color::kBlue);
  }
}

ModelInput MakeModelInput(const std::vector<DirectionalPath>& init_path,
                          const VehicleGeometryParamsProto& veh_geo_params,
                          const VehicleDriveParamsProto& vehicle_drive_params) {
  ModelInput model_input;
  double accumulate_s = 0.0;
  for (int i = 0; i < init_path.size(); ++i) {
    const auto& curr_path = init_path[i];
    QCHECK_GE(curr_path.path.size(), 2);
    const double end_s = curr_path.path.back().s();
    while (accumulate_s <= end_s + kEps) {
      PathPoint pt = curr_path.path.Evaluate(accumulate_s);
      accumulate_s += kStepSize;
      double init_v = kDefaultPosVelocity;
      double init_kappa = pt.kappa();
      if (!curr_path.forward) {
        pt.set_theta(NormalizeAngle(pt.theta() + M_PI));
        init_v = kDefaultNegVelocity;
        init_kappa *= -1.0;
      }
      model_input.init_path.push_back(std::move(pt));
      model_input.init_v.push_back(init_v);
      model_input.init_kappa.push_back(init_kappa);
    }
  }

  // TODO(ping): use goal as the last point of the init path.
  model_input.v_upper = kMaxPosVelocity;
  model_input.v_lower = kMaxNegVelocity;
  model_input.max_kappa =
      ComputeCenterMaxCurvature(veh_geo_params, vehicle_drive_params);
  return model_input;
}

}  // namespace

absl::StatusOr<DiscretizedPath> SmoothGlobalPath(
    const std::vector<DirectionalPath>& init_path,
    const VehicleGeometryParamsProto& veh_geo_params,
    const VehicleDriveParamsProto& vehicle_drive_params) {
  auto model_input =
      MakeModelInput(init_path, veh_geo_params, vehicle_drive_params);
  const int ne = model_input.init_path.size();

  Ipopt::SmartPtr<IpoptGlobalPathSmootherModel> smooth_model =
      new IpoptGlobalPathSmootherModel(std::move(model_input));
  Ipopt::SmartPtr<Ipopt::IpoptApplication> app = IpoptApplicationFactory();

  // TODO(ping): put these setting into config.
  app->Options()->SetIntegerValue("print_level", kPrintLevel);
  app->Options()->SetIntegerValue("max_iter", kMaxIterator);
  app->Options()->SetStringValue("linear_solver", "mumps");
  app->Options()->SetNumericValue("tol", kTol);

  const auto start = absl::Now();
  Ipopt::ApplicationReturnStatus status = app->OptimizeTNLP(smooth_model);
  VLOG(2) << "NLP cost time: "
          << absl::ToDoubleMilliseconds(absl::Now() - start);

  const auto status_info = IpoptReturnStatusToString(status);
  VLOG(2) << "Ipopt return status: " << status_info;

  if (status != Ipopt::Solve_Succeeded &&
      status != Ipopt::Solved_To_Acceptable_Level) {
    return absl::UnavailableError(status_info);
  }

  const auto& result = smooth_model->result();

  std::vector<PathPoint> path_points;
  path_points.reserve(ne);
  double s = 0.0;
  for (int i = 1; i <= ne; ++i) {
    PathPoint pt;
    pt.set_x(result[i]);
    pt.set_y(result[i + ne]);
    pt.set_theta(NormalizeAngle(result[i + 2 * ne]));
    pt.set_kappa(result[i + 4 * ne]);
    if (i != 1) {
      s += DistanceTo(path_points.back(), pt);
    }
    pt.set_s(s);
    path_points.push_back(std::move(pt));
  }
  SendToCanvas(path_points);

  return DiscretizedPath(std::move(path_points));
}

}  // namespace planner
}  // namespace qcraft
