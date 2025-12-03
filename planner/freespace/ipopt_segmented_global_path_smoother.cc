#include "onboard/planner/freespace/ipopt_segmented_global_path_smoother.h"

#include <algorithm>
#include <cmath>
#include <ostream>
#include <string>
#include <utility>

#define HAVE_STDDEF_H
#include "absl/status/status.h"
#include "absl/strings/str_format.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "coin/IpIpoptApplication.hpp"
#include "coin/IpOptionsList.hpp"
#include "coin/IpReturnCodes.hpp"
#include "coin/IpSmartPtr.hpp"
#include "coin/IpTNLP.hpp"
#include "gflags/gflags.h"
#include "glog/logging.h"

#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/math/geometry/segment2d.h"
#include "onboard/math/util.h"
#include "onboard/planner/common/discretized_path.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/vis/canvas/canvas.h"
#include "onboard/vis/common/color.h"

#undef HAVE_STDDEF_H

#include "onboard/global/trace.h"
#include "onboard/math/vec.h"
#include "onboard/planner/freespace/ipopt_segmented_global_path_smoother_model.h"
#include "onboard/planner/optimization/ipopt/ipopt_util.h"
#include "onboard/planner/util/path_util.h"
#include "onboard/planner/util/vehicle_geometry_util.h"

#include "offboard/vis/vantage/vantage_server/vantage_canvas_client.h"

DEFINE_int32(freespace_global_smooth_path_canvas_level, 0,
             "Freespace global smooth path canvas level.");

namespace qcraft {
namespace planner {
namespace {

using ModelInput = IpoptSegmentedGlobalPathSmootherInput;

constexpr int kPrintLevel = 0;
constexpr int kMaxIterator = 3000;
constexpr double kTol = 1e-4;

void SendInputToCanvas(const ModelInput& input) {
  vis::Canvas* canvas_support_plane = nullptr;
  constexpr double kPlaneWidth = 1.0;      // m.
  constexpr double kPlaneDirLength = 0.3;  // m.
  const auto& init_traj = input.init_traj;
  for (int i = 0; i < init_traj.size(); ++i) {
    for (int j = 0; j < init_traj[i].supports.size(); ++j) {
      for (int k = 0; k < init_traj[i].supports[j].size(); ++k) {
        const auto& plane = init_traj[i].supports[j][k];
        canvas_support_plane =
            &vis::vantage::GetCanvasClient()->GetCanvas(absl::StrFormat(
                "freespace/init_traj/seg_%03d/point_%03d/plane_%03d", i, j, k));
        QCHECK_NOTNULL(canvas_support_plane)
            ->DrawLine(Vec3d(plane.ref + kPlaneWidth * plane.dir.Perp()),
                       Vec3d(plane.ref - kPlaneWidth * plane.dir.Perp()),
                       vis::Color(0.4, 0.5, 0.8));
        QCHECK_NOTNULL(canvas_support_plane)
            ->DrawLine(Vec3d(plane.ref),
                       Vec3d(plane.ref + kPlaneDirLength * plane.dir),
                       vis::Color(0.4, 0.5, 0.8));
      }
    }
  }
}

void SendResultToCanvas(const std::vector<DirectionalPath>& paths) {
  vis::Canvas& canvas = vis::vantage::GetCanvasClient()->GetCanvas(
      "freespace/global_smooth_path");
  for (int i = 0; i < paths.size(); ++i) {
    std::vector<Vec3d> points;
    points.reserve(paths[i].path.size());
    for (int j = 0; j < paths[i].path.size(); ++j) {
      points.emplace_back(paths[i].path[j].x(), paths[i].path[j].y(), 0.0);
    }
    vis::Color color =
        paths[i].forward ? vis::Color::kLightGreen : vis::Color::kLightRed;
    for (const auto& point : points) {
      canvas.DrawCircle(point, 0.08, color);
    }
    canvas.DrawLineStrip(points, color);
  }
}

ModelInput MakeModelInput(const std::vector<DirectionalPath>& init_path,
                          const FreespaceMap& freespace_map,
                          const VehicleGeometryParamsProto& veh_geo_params,
                          const VehicleDriveParamsProto& vehicle_drive_params) {
  using TrajectorySegment = ModelInput::TrajectorySegment;
  constexpr double kDefaultPosVelocity = 1.0;   // m/s.
  constexpr double kDefaultNegVelocity = -1.0;  // m/s.
  constexpr double kMaxPosVelocity = 2.0;       // m/s.
  constexpr double kMaxNegVelocity = 1.5;       // m/s.
  constexpr double kStepSize = 0.3;             // m.

  ModelInput model_input;
  std::vector<TrajectorySegment> init_traj;
  init_traj.reserve(init_path.size());
  for (int i = 0; i < init_path.size(); ++i) {
    const auto& curr_path = init_path[i];
    QCHECK_GE(curr_path.path.size(), 2);
    TrajectorySegment traj_seg;
    traj_seg.forward = curr_path.forward;
    const double end_s = curr_path.path.back().s();
    const int num = CeilToInt(end_s / kStepSize) + 1;
    traj_seg.path.reserve(num);
    traj_seg.v.reserve(num);
    traj_seg.supports.reserve(num);
    double accumulate_s = 0.0;
    // Make sure the last path point is sampled.
    while (true) {
      PathPoint pt = curr_path.path.Evaluate(accumulate_s);
      double v = kDefaultPosVelocity;
      if (!curr_path.forward) {
        pt.set_theta(NormalizeAngle(pt.theta() + M_PI));
        pt.set_kappa(-pt.kappa());
        v = kDefaultNegVelocity;
      }
      traj_seg.path.push_back(pt);
      traj_seg.v.push_back(v);
      // Generate support planes for each path point.
      const auto& boundaries = freespace_map.boundaries;
      std::vector<ModelInput::SupportPlane> supports;
      supports.reserve(boundaries.size());
      const Vec2d p = ToVec2d(pt);
      for (const auto& b : boundaries) {
        for (int i = 0; i + 1 < b.points.size(); ++i) {
          Vec2d ref_point, inner_dir;
          const Segment2d seg(b.points[i], b.points[i + 1]);
          const Vec2d x0 = p - seg.start();
          const double proj = x0.Dot(seg.unit_direction());
          if (proj <= 0.0) {
            ref_point = seg.start();
            inner_dir = x0.Unit();
          } else if (proj >= seg.length()) {
            ref_point = seg.end();
            inner_dir = (p - seg.end()).Unit();
          } else {
            ref_point = seg.start() + proj * seg.unit_direction();
            inner_dir = seg.unit_direction().CrossProd(x0) > 0.0
                            ? seg.unit_direction().Perp()
                            : -seg.unit_direction().Perp();
          }
          supports.emplace_back(ref_point, inner_dir);
        }
      }
      traj_seg.supports.push_back(std::move(supports));
      if (accumulate_s >= end_s) break;
      accumulate_s += kStepSize;
    }
    init_traj.push_back(std::move(traj_seg));
  }

  model_input.init_traj = std::move(init_traj);
  // TODO(renjie): Load params from config.
  model_input.max_forward_speed = kMaxPosVelocity;
  model_input.max_reverse_speed = kMaxNegVelocity;
  model_input.max_accel = 2.0;
  model_input.min_accel = -4.0;
  model_input.max_curvature =
      ComputeCenterMaxCurvature(veh_geo_params, vehicle_drive_params);
  model_input.max_curvature_rate = 0.1;
  model_input.max_time = 50.0;
  model_input.buffer = 0.5 * veh_geo_params.width();
  return model_input;
}

}  // namespace

absl::StatusOr<std::vector<DirectionalPath>> SmoothGlobalPath(
    const std::vector<DirectionalPath>& init_path,
    const FreespaceMap& freespace_map,
    const VehicleGeometryParamsProto& veh_geo_params,
    const VehicleDriveParamsProto& vehicle_drive_params) {
  SCOPED_QTRACE("SmoothGlobalPath");

  auto model_input = MakeModelInput(init_path, freespace_map, veh_geo_params,
                                    vehicle_drive_params);
  if (FLAGS_freespace_global_smooth_path_canvas_level > 1) {
    SendInputToCanvas(model_input);
  }
  Ipopt::SmartPtr<IpoptSegmentedGlobalPathSmootherModel> smooth_model =
      new IpoptSegmentedGlobalPathSmootherModel(std::move(model_input));
  Ipopt::SmartPtr<Ipopt::IpoptApplication> app = IpoptApplicationFactory();

  // TODO(ping): put these setting into config.
  app->Options()->SetIntegerValue("print_level", kPrintLevel);
  app->Options()->SetIntegerValue("max_iter", kMaxIterator);
  app->Options()->SetStringValue("linear_solver", "mumps");
  app->Options()->SetNumericValue("tol", kTol);

  const auto start = absl::Now();
  Ipopt::ApplicationReturnStatus status = app->OptimizeTNLP(smooth_model);
  VLOG(2) << "Global path smoother cost time (ms): "
          << absl::ToDoubleMilliseconds(absl::Now() - start);

  const auto status_info = IpoptReturnStatusToString(status);
  VLOG(2) << "Ipopt return status: " << status_info;

  if (!(status == Ipopt::Solve_Succeeded ||
        status == Ipopt::Feasible_Point_Found ||
        status == Ipopt::Maximum_Iterations_Exceeded ||
        status == Ipopt::Maximum_CpuTime_Exceeded ||
        status == Ipopt::Search_Direction_Becomes_Too_Small ||
        status == Ipopt::Solved_To_Acceptable_Level ||
        status == Ipopt::Restoration_Failed ||
        status == Ipopt::User_Requested_Stop)) {
    return absl::UnavailableError(status_info);
  }

  auto& res_paths = smooth_model->result_paths();
  QCHECK_EQ(res_paths.size(), init_path.size());
  std::vector<DirectionalPath> res;
  res.reserve(init_path.size());
  for (int i = 0; i < init_path.size(); ++i) {
    DirectionalPath dir_path;
    dir_path.path = DiscretizedPath(res_paths[i]);
    dir_path.forward = init_path[i].forward;
    res.push_back(std::move(dir_path));
  }

  if (FLAGS_freespace_global_smooth_path_canvas_level > 0) {
    SendResultToCanvas(res);
  }

  return res;
}

}  // namespace planner
}  // namespace qcraft
