#include "onboard/planner/freespace/tob_path_smoother.h"

#include <algorithm>
#include <cmath>
#include <complex>  // IWYU pragma: keep
#include <memory>
#include <ostream>
#include <regex>  // IWYU pragma: keep
#include <string_view>
#include <utility>
#include <vector>

#include "Eigen/Cholesky"     // IWYU pragma: keep
#include "Eigen/Householder"  // IWYU pragma: keep
#include "Eigen/Jacobi"       // IWYU pragma: keep
#include "Eigen/LU"           // IWYU pragma: keep
#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "absl/types/span.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "google/protobuf/repeated_ptr_field.h"

#include "onboard/global/buffered_logger.h"
#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/math/fast_math.h"
#include "onboard/math/frenet_common.h"
#include "onboard/math/frenet_frame.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/math/geometry/proto/aabox3d.pb.h"
#include "onboard/math/geometry/segment2d.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/common/discretized_path.h"
#include "onboard/planner/object/planner_object.h"
#include "onboard/planner/object/spacetime_object_trajectory.h"
#include "onboard/planner/optimization/ddp/ddp_optimizer.h"
#include "onboard/planner/optimization/ddp/ddp_optimizer_debug_hook.h"
#include "onboard/planner/optimization/ddp/object_cost_util.h"
#include "onboard/planner/optimization/ddp/trajectory_optimizer_util.h"
#include "onboard/planner/optimization/problem/av_model_helper.h"
#include "onboard/planner/optimization/problem/backward_speed_cost.h"
#include "onboard/planner/optimization/problem/cost.h"
#include "onboard/planner/optimization/problem/curvature_cost.h"
#include "onboard/planner/optimization/problem/forward_speed_cost.h"
#include "onboard/planner/optimization/problem/intrinsic_jerk_cost.h"
#include "onboard/planner/optimization/problem/msd_static_boundary_cost.h"
#include "onboard/planner/optimization/problem/partitioned_object_cost.h"
#include "onboard/planner/optimization/problem/partitioned_static_boundary_cost.h"
#include "onboard/planner/optimization/problem/reference_control_deviation_cost.h"
#include "onboard/planner/optimization/problem/reference_line_deviation_cost.h"
#include "onboard/planner/optimization/problem/reference_state_deviation_cost.h"
#include "onboard/planner/optimization/problem/third_order_bicycle.h"
#include "onboard/planner/optimization/problem/tob_curvature_rate_cost.h"
#include "onboard/planner/optimization/proto/optimizer.pb.h"
#include "onboard/planner/second_order_trajectory_point.h"
#include "onboard/planner/util/min_segment_distance_problem.h"
#include "onboard/planner/util/qtfm_segment_matcher_v2.h"
#include "onboard/planner/util/trajectory_plot_util.h"
#include "onboard/planner/util/vehicle_geometry_util.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/utils/status_macros.h"
#include "onboard/vis/canvas/canvas.h"
#include "onboard/vis/common/color.h"

#include "offboard/vis/vantage/vantage_server/vantage_canvas_client.h"

DEFINE_int32(freespace_local_smoother_verbosity_level, 0,
             "Freespace local smoother verbosity level");
DEFINE_bool(send_freespace_local_smoother_path_to_canvas, false,
            "whether send freespace local smoother path to canvas");
DEFINE_bool(
    draw_boundary_cost_buffer_circle, false,
    "whether send buffer circle of partitioned static boundary cost to canvas");
DEFINE_bool(send_msd_static_boundary_to_canvas, false,
            "whether send msd static boundary to canvas");

namespace qcraft {
namespace planner {
namespace {

constexpr int kTobSteps = 50;
constexpr double kTobDt = 0.2;
using Tob = ThirdOrderBicycle;
using ObjectCost = PartitionedObjectCost<Tob>;
using ModelHelper = AvModelHelper<Tob>;

std::vector<TrajectoryPoint> GetInitTrajectoryByPurePursuit(
    const TrajectoryPoint& plan_start_point,
    const BruteForceFrenetFrame& reference_line,
    const VehicleGeometryParamsProto& vehicle_geo_params,
    const VehicleDriveParamsProto& vehicle_drive_params, bool forward,
    double max_speed) {
  const auto current_sl = reference_line.XYToSL(plan_start_point.pos());
  const double s_step =
      (reference_line.end_s() - current_sl.s) / static_cast<double>(kTobSteps);
  const double target_v = std::min(max_speed, s_step / kTobDt);

  constexpr double kLateralLookAhead = 0.4;  // s.
  Tob::ControlsType us = Tob::ControlsType::Zero(kTobSteps * Tob::kControlSize);
  Tob::StatesType xs = Tob::StatesType::Zero(kTobSteps * Tob::kStateSize);
  Tob::StateType x;
  // Init state.
  Tob::StateSetX(plan_start_point.pos().x(), &x);
  Tob::StateSetY(plan_start_point.pos().y(), &x);
  Tob::StateSetTheta(forward ? plan_start_point.theta()
                             : NormalizeAngle(plan_start_point.theta() + M_PI),
                     &x);
  Tob::StateSetKappa(
      forward ? plan_start_point.kappa() : -plan_start_point.kappa(), &x);
  Tob::StateSetV(target_v, &x);
  Tob::StateSetA(0.0, &x);
  Tob::StateSetS(0.0, &x);
  constexpr double kMaxKappaRelaxFactor = 2.0;
  const double vehicle_max_kappa =
      kMaxKappaRelaxFactor *
      ComputeCenterMaxCurvature(vehicle_geo_params, vehicle_drive_params);
  // Pure pursuit process.
  for (int k = 0; k < kTobSteps; ++k) {
    Tob::SetStateAtStep(x, k, &xs);
    const double lateral_look_ahead_dist =
        kLateralLookAhead * target_v + vehicle_geo_params.wheel_base();
    const Vec2d lateral_target = reference_line.SLToXY(
        {Tob::StateGetS(x) + lateral_look_ahead_dist + current_sl.s, 0.0});
    const double theta = Tob::StateGetTheta(x);
    const double alpha =
        Vec2d(lateral_target - Tob::StateGetPos(x)).Angle() - theta;
    const double kappa =
        std::clamp(2.0 * fast_math::Sin(alpha) / lateral_look_ahead_dist,
                   -vehicle_max_kappa, vehicle_max_kappa);
    const double psi = (kappa - Tob::StateGetKappa(x)) / kTobDt;

    const auto u = Tob::MakeControl(psi, /*j=*/0.0);
    x = Tob::EvaluateF(k, x, u, kTobDt);
    Tob::SetControlAtStep(u, k, &us);
  }
  // Extract result.
  std::vector<TrajectoryPoint> res(kTobSteps);
  for (int k = 0; k < kTobSteps; ++k) {
    Tob::ExtractTrajectoryPoint(k, Tob::GetStateAtStep(xs, k),
                                Tob::GetControlAtStep(us, k), kTobDt,
                                &(res[k]));
  }
  return res;
}

std::vector<TrajectoryPoint> GetInitTrajectoryByPrevPath(
    const TrajectoryPoint& plan_start_point,
    const std::vector<PathPoint>& prev_path, bool forward) {
  std::vector<TrajectoryPoint> res;
  res.reserve(kTobSteps);

  DiscretizedPath path(prev_path);
  const auto current_sl = path.XYToSL(plan_start_point.pos());
  const double s_step = (prev_path.back().s() - current_sl.s) / kTobSteps;
  const double target_v = s_step / kTobDt;

  for (int i = 0; i < kTobSteps; ++i) {
    const auto pt = path.Evaluate(current_sl.s + s_step * i);
    TrajectoryPoint traj_pt;
    traj_pt.set_pos(Vec2d(pt.x(), pt.y()));
    traj_pt.set_theta(pt.theta());
    traj_pt.set_kappa(pt.kappa());
    traj_pt.set_s(s_step * i);
    traj_pt.set_v(target_v);
    traj_pt.set_a(0.0);
    traj_pt.set_j(0.0);
    traj_pt.set_psi(0.0);
    traj_pt.set_t(i * kTobDt);
    res.push_back(std::move(traj_pt));
  }
  // BANDAID(fengzhuang): This is a hack, replace the first point with plan
  // start point.
  res.front().set_pos(plan_start_point.pos());
  res.front().set_theta(forward
                            ? plan_start_point.theta()
                            : NormalizeAngle(plan_start_point.theta() + M_PI));
  res.front().set_kappa(forward ? plan_start_point.kappa()
                                : -plan_start_point.kappa());
  return res;
}

std::unique_ptr<ModelHelper> ConstructAvModelHelper(
    const VehicleCircleModelParamsProto& vehicle_model_params) {
  std::vector<double> dists_to_rac;
  std::vector<double> angles_to_axis;
  const int circle_size = vehicle_model_params.circles_size() +
                          vehicle_model_params.mirror_circles_size();
  dists_to_rac.reserve(circle_size);
  angles_to_axis.reserve(circle_size);
  for (const auto& circle : vehicle_model_params.circles()) {
    dists_to_rac.push_back(circle.dist_to_rac());
    angles_to_axis.push_back(circle.angle_to_axis());
  }
  for (const auto& circle : vehicle_model_params.mirror_circles()) {
    dists_to_rac.push_back(circle.dist_to_rac());
    angles_to_axis.push_back(circle.angle_to_axis());
  }
  return std::make_unique<ModelHelper>(kTobSteps, dists_to_rac, angles_to_axis,
                                       "TobAvModelHelper");
}

void AddObjectCosts(
    const VehicleGeometryParamsProto& veh_geo_params,
    const FreespaceLocalSmootherParamsProto& local_smoother_params,
    const VehicleCircleModelParamsProto& vehicle_model_params,
    const ModelHelper* av_model_helper,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const absl::flat_hash_set<std::string>& stalled_object_ids,
    const DirectionalPath& input_path,
    const std::vector<TrajectoryPoint>& init_traj, Tob* problem) {
  QCHECK_EQ(init_traj.size(), kTobSteps);
  constexpr double kGain = 1.0;
  constexpr double kNearbyBoundaryBuffer = 3.0;

  std::vector<Box2d> traj_boxes_with_buffer;
  traj_boxes_with_buffer.reserve(input_path.path.size());
  for (const auto& path_point : input_path.path) {
    const Vec2d av_pos(path_point.x(), path_point.y());
    Box2d av_box_with_buffer =
        ComputeAvBoxWithBuffer(av_pos, path_point.theta(), veh_geo_params,
                               kNearbyBoundaryBuffer, kNearbyBoundaryBuffer);
    traj_boxes_with_buffer.push_back(std::move(av_box_with_buffer));
  }

  const auto is_nearby_objects =
      [&traj_boxes_with_buffer](const Polygon2d& object) -> bool {
    for (const auto& av_box : traj_boxes_with_buffer) {
      if (object.HasOverlap(av_box)) return true;
    }
    return false;
  };
  // Compute buffers.
  const double buffer = local_smoother_params.object_buffer();
  const int vehicle_circle_size = vehicle_model_params.circles_size();
  const int mirror_circle_size = vehicle_model_params.mirror_circles_size();
  const int total_circle_size = vehicle_circle_size + mirror_circle_size;

  std::vector<double> dist_to_rac;
  std::vector<double> angle_to_axis;
  std::vector<double> circle_buffers;
  dist_to_rac.reserve(total_circle_size);
  angle_to_axis.reserve(total_circle_size);
  circle_buffers.reserve(total_circle_size);
  double max_buffer = 0.0;
  double max_model_dist = 0.0;
  for (const auto& circle : vehicle_model_params.circles()) {
    dist_to_rac.push_back(circle.dist_to_rac());
    angle_to_axis.push_back(circle.angle_to_axis());
    circle_buffers.push_back(circle.radius() + buffer);
    max_buffer = std::max(max_buffer, circle.radius() + buffer);
    max_model_dist = std::max(max_model_dist, circle.dist_to_rac());
  }
  for (const auto& circle : vehicle_model_params.mirror_circles()) {
    dist_to_rac.push_back(circle.dist_to_rac());
    angle_to_axis.push_back(circle.angle_to_axis());
    circle_buffers.push_back(circle.radius() + buffer);
  }
  const double filter_offset = max_buffer + max_model_dist;
  // Add costs.
  for (const auto& traj : st_traj_mgr.trajectories()) {
    if (!stalled_object_ids.contains(traj.object_id())) {
      continue;
    }
    const Polygon2d& contour = traj.contour();
    if (!is_nearby_objects(contour)) continue;

    const auto& object_proto = traj.planner_object().object_proto();
    const double object_height = object_proto.max_z() - object_proto.ground_z();
    const bool consider_mirror =
        veh_geo_params.has_left_mirror() &&
        object_height + local_smoother_params.mirror_height_buffer() >
            veh_geo_params.left_mirror().z() -
                veh_geo_params.left_mirror().height() * 0.5;
    const int circle_size =
        consider_mirror ? total_circle_size : vehicle_circle_size;
    const Vec2d obj_x = traj.pose().pos();
    const auto obj_id = std::string(traj.object_id());
    std::vector<ObjectCost::filter> filters;
    std::vector<std::vector<ObjectCost::Object>> objects;
    objects.resize(kTobSteps);
    filters.resize(kTobSteps);
    for (int k = 0; k < kTobSteps; ++k) {
      auto& objects_k = objects[k];
      const Vec2d x = init_traj[k].pos();
      for (int idx = 0; idx < circle_size; ++idx) {
        const Vec2d tangent =
            Vec2d::FastUnitFromAngle(init_traj[k].theta() + angle_to_axis[idx]);
        const Vec2d x_center = x + tangent * dist_to_rac[idx];

        std::vector<Segment2d> lines;
        Vec2d ref_x;
        Vec2d ref_tangent;
        double offset = 0.0;
        optimizer::CalcPartitionHalfContourInfo(x_center, obj_x, contour,
                                                circle_buffers[idx], &lines,
                                                &ref_x, &ref_tangent, &offset);
        QCHECK(!lines.empty());
        objects_k.push_back(ObjectCost::Object{.lines = lines,
                                               .buffers = {circle_buffers[idx]},
                                               .gains = {kGain},
                                               .ref_x = ref_x,
                                               .offset = offset,
                                               .ref_tangent = ref_tangent,
                                               .enable = true});
      }

      // Update filter.
      Vec2d ref_x = {0.0, 0.0};
      Vec2d ref_tangent = {0.0, 0.0};
      for (const auto& object : objects_k) {
        ref_x += object.ref_x;
        ref_tangent += object.ref_tangent;
      }
      ref_x /= static_cast<double>(objects_k.size());
      ref_tangent =
          (ref_tangent / static_cast<double>(objects_k.size())).normalized();

      Vec2d front, back;
      int front_index, back_index;
      contour.ExtremePoints(ref_tangent, &back_index, &front_index, &back,
                            &front);
      const double offset = (front - ref_x).dot(ref_tangent) + filter_offset;

      ObjectCost::filter& filter = filters[k];
      filter.ref_x = ref_x;
      filter.ref_tangent = ref_tangent;
      filter.offset = offset;
    }
    problem->AddCost(std::make_unique<ObjectCost>(
        std::move(objects), filters,
        std::vector<double>(dist_to_rac.begin(),
                            dist_to_rac.begin() + circle_size),
        std::vector<double>(angle_to_axis.begin(),
                            angle_to_axis.begin() + circle_size),
        /*cascade_gains=*/std::vector<double>({1.0}), av_model_helper,
        /*sub_names=*/std::vector<std::string>({""}),
        /*using_hessian_approximate=*/false,
        absl::StrFormat("Partition AV Object: for %s", obj_id),
        local_smoother_params.object_cost_weight()));
  }
}

void AddMsdStaticBoundaryCost(
    std::string_view base_name, std::string_view source_name,
    const VehicleGeometryParamsProto& veh_geo_params,
    const VehicleCircleModelParamsProto& vehicle_model_params,
    bool consider_vehicle_mirrors, const std::vector<int>& start_segment_ids,
    std::vector<MsdProblemWithBuffer::SegmentType> named_segments,
    std::vector<std::string> sub_names, std::vector<double> cascade_buffers,
    std::vector<double> cascade_gains, Tob* problem) {
  constexpr double kScale = 1.0;
  constexpr double kCutoffDistExtraBuffer = 0.1;
  const double cutoff_distance =
      *std::max_element(cascade_buffers.begin(), cascade_buffers.end()) +
      0.5 * veh_geo_params.width() + kCutoffDistExtraBuffer;
  MsdProblemWithBuffer msd(std::move(named_segments), cutoff_distance,
                           start_segment_ids);

  if (FLAGS_send_msd_static_boundary_to_canvas) {
    vis::Canvas* canvas = &vis::vantage::GetCanvasClient()->GetCanvas(
        absl::StrFormat("%s/qtfm_for_%s", base_name, source_name));
    msd.DrawQtfmSegmentMatcher(canvas);
  }

  std::vector<Vec2d> circle_center_offsets;
  std::vector<double> circle_radiuses;
  const int vehicle_circle_size = vehicle_model_params.circles_size();
  const int mirror_circle_size = vehicle_model_params.mirror_circles_size();
  const int total_circle_size = vehicle_circle_size + mirror_circle_size;
  circle_center_offsets.reserve(total_circle_size);
  circle_radiuses.reserve(total_circle_size);
  for (const auto& circle : vehicle_model_params.circles()) {
    double cos_sin[2];
    fast_math::CosAndSin<7>(circle.angle_to_axis(), cos_sin);
    circle_center_offsets.emplace_back(circle.dist_to_rac() * cos_sin[0],
                                       circle.dist_to_rac() * cos_sin[1]);
    circle_radiuses.push_back(circle.radius());
  }
  for (const auto& circle : vehicle_model_params.mirror_circles()) {
    double cos_sin[2];
    fast_math::CosAndSin<7>(circle.angle_to_axis(), cos_sin);
    circle_center_offsets.push_back(Vec2d(circle.dist_to_rac() * cos_sin[0],
                                          circle.dist_to_rac() * cos_sin[1]));
    circle_radiuses.push_back(circle.radius());
  }

  const int circle_size =
      consider_vehicle_mirrors ? total_circle_size : vehicle_circle_size;
  problem->AddCost(std::make_unique<MsdStaticBoundaryCost<Tob>>(
      kTobSteps, veh_geo_params, std::move(msd),
      /*center_line_helper=*/nullptr, std::move(sub_names),
      std::move(cascade_buffers), std::move(cascade_gains),
      std::vector<Vec2d>(circle_center_offsets.begin(),
                         circle_center_offsets.begin() + circle_size),
      std::vector<double>(circle_radiuses.begin(),
                          circle_radiuses.begin() + circle_size),
      /*effect_index=*/kTobSteps,
      /*ignore_invasion_second_order_derivative=*/true, "MsdStaticBoundaryCost",
      kScale));
}

// NOLINTNEXTLINE
void AddStaticBoundaryCost(
    const VehicleGeometryParamsProto& veh_geo_params,
    const FreespaceLocalSmootherParamsProto& local_smoother_params,
    const VehicleCircleModelParamsProto& vehicle_model_params,
    const FreespaceMap& static_map, const DirectionalPath& input_path,
    Tob* problem, FreespaceLocalSmootherDebugProto* debug_info) {
  const auto add_boundary_segments =
      [&](const std::vector<Vec2d>& points, const std::string& id,
          std::vector<int>* start_segment_ids,
          std::vector<MsdProblemWithBuffer::SegmentType>* named_segments) {
        if (points.empty()) return;
        bool has_no_prev_segment = true;
        Vec2d prev = points.front();
        for (int i = 1; i < points.size(); ++i) {
          // Skip short segments.
          if (prev.DistanceTo(points[i]) <=
              QtfmSegmentMatcherV2::kMinSegmentLength) {
            continue;
          }
          // Append a segment.
          named_segments->push_back({absl::StrCat("id:", id, ",seg:", i),
                                     Segment2d(prev, points[i]), 0.0});
          prev = points[i];

          if (has_no_prev_segment) {
            start_segment_ids->push_back(named_segments->size() - 1);
            has_no_prev_segment = false;
          }
        }
      };

  const auto add_partition_static_boundary_cost =
      [&problem, &vehicle_model_params, &veh_geo_params](
          const std::vector<Vec2d>& points, const std::string& id,
          const std::vector<double>& buffers,
          const std::vector<double>& weights,
          const std::vector<std::string>& sub_names) {
        constexpr double kScale = 1.0;
        for (int i = 0; i + 1 < points.size(); ++i) {
          const Segment2d seg(points[i], points[i + 1]);
          for (const auto& param : vehicle_model_params.circles()) {
            auto buffers_with_radius = buffers;
            std::for_each(buffers_with_radius.begin(),
                          buffers_with_radius.end(),
                          [&param](double& x) { x += param.radius(); });
            problem->AddCost(
                std::make_unique<PartitionedStaticBoundaryCost<Tob>>(
                    kTobSteps, &veh_geo_params, seg,
                    PartitionedStaticBoundaryCost<Tob>::Type::kCor,
                    param.dist_to_rac(), param.angle_to_axis(), sub_names,
                    /*using_hessian_approximate=*/false, buffers_with_radius,
                    weights,
                    absl::StrFormat(
                        "PartitionStaticBoundaryCost (%s) of %s",
                        VehicleCircleModelParamsProto::Type_Name(param.type()),
                        id),
                    kScale));
          }
        }
      };

  const auto is_nearby_boundary =
      [&input_path](const std::vector<Vec2d>& points) {
        constexpr double kNearbyDistance = 15.0;  // m.
        for (int i = 0; i + 1 < points.size(); ++i) {
          Segment2d segment(points[i], points[i + 1]);
          for (const auto& pt : input_path.path) {
            if (segment.DistanceSquareTo(Vec2d(pt.x(), pt.y())) <
                Sqr(kNearbyDistance)) {
              return true;
            }
          }
        }
        return false;
      };

  // Use MsdStaticBoundaryCost for curb, dashed lane line and solid
  // lane line, use PartitionedStaticBoundaryCost for others.
  std::vector<int> low_curb_start_segment_ids, high_curb_start_segment_ids,
      solid_line_start_segment_ids, dashed_line_start_segment_ids;
  std::vector<MsdProblemWithBuffer::SegmentType> low_curb_segments,
      high_curb_segments, solid_line_segments, dashed_line_segments;

  for (const auto& boundary : static_map.boundaries) {
    if (!is_nearby_boundary(boundary.points)) continue;
    switch (boundary.type) {
      case FreespaceMapProto::CURB: {
        if (veh_geo_params.has_left_mirror() &&
            boundary.height + local_smoother_params.mirror_height_buffer() >
                veh_geo_params.left_mirror().z() -
                    veh_geo_params.left_mirror().height() * 0.5) {
          add_boundary_segments(boundary.points, boundary.id,
                                &high_curb_start_segment_ids,
                                &high_curb_segments);
        } else {
          add_boundary_segments(boundary.points, boundary.id,
                                &low_curb_start_segment_ids,
                                &low_curb_segments);
        }
      } break;
      case FreespaceMapProto::BARRIER:
        add_partition_static_boundary_cost(
            boundary.points, boundary.id,
            /*buffers=*/{local_smoother_params.barrier_buffer()},
            /*weights=*/{local_smoother_params.barrier_cost_weight()},
            /*sub_names=*/{""});
        break;
      case FreespaceMapProto::YELLOW_SOLID_LANE:
      case FreespaceMapProto::WHITE_SOLID_LANE:
        add_partition_static_boundary_cost(
            boundary.points, boundary.id,
            /*buffers=*/{local_smoother_params.solid_lane_buffer()},
            /*weights=*/{local_smoother_params.solid_lane_cost_weight()},
            /*sub_names=*/{""});
        break;
      case FreespaceMapProto::PARKING_SPOT:
        add_partition_static_boundary_cost(
            boundary.points, boundary.id,
            /*buffers=*/{local_smoother_params.spot_line_buffer()},
            /*weights=*/{local_smoother_params.spot_line_cost_weight()},
            /*sub_names=*/{""});
        break;
      case FreespaceMapProto::VIRTUAL:
        add_partition_static_boundary_cost(
            boundary.points, boundary.id,
            /*buffers=*/{local_smoother_params.virtual_boundary_buffer()},
            /*weights=*/{local_smoother_params.virtual_boundary_cost_weight()},
            /*sub_names=*/{""});
        break;
      case FreespaceMapProto::YELLOW_DASHED_LANE:
      case FreespaceMapProto::WHITE_DASHED_LANE:
      case FreespaceMapProto::PARKING_STOPPER:
      case FreespaceMapProto::OTHER:
        add_partition_static_boundary_cost(boundary.points, boundary.id,
                                           /*buffers=*/{0.0},
                                           /*weights=*/{1.0},
                                           /*sub_names=*/{""});
        break;
    }
  }

  // Add MsdStaticBoundaryCost.
  if (!low_curb_segments.empty()) {
    AddMsdStaticBoundaryCost(
        /*base_name=*/"freespace", /*source_name=*/"curb", veh_geo_params,
        vehicle_model_params,
        /*consider_vehicle_mirrors=*/false, low_curb_start_segment_ids,
        std::move(low_curb_segments),
        /*sub_names=*/{"CurbSoft", "CurbHard"},
        /*cascade_buffers=*/
        {local_smoother_params.curb_soft_buffer(),
         local_smoother_params.curb_hard_buffer()},
        /*cascade_gains=*/
        {local_smoother_params.curb_soft_cost_weight(),
         local_smoother_params.curb_hard_cost_weight()},
        problem);
  }
  if (!high_curb_segments.empty()) {
    AddMsdStaticBoundaryCost(
        /*base_name=*/"freespace", /*source_name=*/"curb", veh_geo_params,
        vehicle_model_params,
        /*consider_vehicle_mirrors=*/true, high_curb_start_segment_ids,
        std::move(high_curb_segments),
        /*sub_names=*/{"CurbSoft", "CurbHard"},
        /*cascade_buffers=*/
        {local_smoother_params.curb_soft_buffer(),
         local_smoother_params.curb_hard_buffer()},
        /*cascade_gains=*/
        {local_smoother_params.curb_soft_cost_weight(),
         local_smoother_params.curb_hard_cost_weight()},
        problem);
  }
  if (!solid_line_segments.empty()) {
    AddMsdStaticBoundaryCost(
        /*base_name=*/"freespace", /*source_name=*/"solid_line", veh_geo_params,
        vehicle_model_params,
        /*consider_vehicle_mirrors=*/false, solid_line_start_segment_ids,
        std::move(solid_line_segments),
        /*sub_names=*/{"SolidLine"},
        /*cascade_buffers=*/{local_smoother_params.solid_lane_buffer()},
        /*cascade_gains=*/{local_smoother_params.solid_lane_cost_weight()},
        problem);
  }

  // Add costs of dashed lane lines. We only consider lane line that has no
  // intersection with path.
  std::vector<Segment2d> seg_path;
  seg_path.reserve(input_path.path.size() - 1);
  for (int i = 0; i + 1 < input_path.path.size(); ++i) {
    seg_path.emplace_back(
        Vec2d(input_path.path[i].x(), input_path.path[i].y()),
        Vec2d(input_path.path[i + 1].x(), input_path.path[i + 1].y()));
  }
  const auto has_overlap =
      [&seg_path](const SpecialBoundary& boundary) -> bool {
    for (int i = 0; i + 1 < boundary.points.size(); ++i) {
      Segment2d segment(boundary.points[i], boundary.points[i + 1]);
      const double min_x = segment.min_x();
      const double min_y = segment.min_y();
      const double max_x = segment.max_x();
      const double max_y = segment.max_y();
      for (const auto& seg : seg_path) {
        if (min_x > seg.max_x() || min_y > seg.max_y() || max_x < seg.min_x() ||
            max_y < seg.min_y()) {
          continue;
        }
        if (segment.HasIntersect(seg)) return true;
      }
    }
    return false;
  };
  // Add costs.
  for (const auto& boundary : static_map.special_boundaries) {
    if (!input_path.forward) break;
    if (boundary.type != FreespaceMapProto::CROSSABLE_LANE_LINE) {
      continue;
    }
    if (!is_nearby_boundary(boundary.points)) continue;
    if (has_overlap(boundary)) continue;
    add_boundary_segments(boundary.points, boundary.id,
                          &dashed_line_start_segment_ids,
                          &dashed_line_segments);
    // Add to debug info here.
    debug_info->add_enabled_crossable_boundary_ids(boundary.id);
  }
  // Add MsdStaticBoundaryCost.
  if (!dashed_line_segments.empty()) {
    AddMsdStaticBoundaryCost(
        /*base_name=*/"freespace", /*source_name=*/"dashed_line",
        veh_geo_params, vehicle_model_params,
        /*consider_vehicle_mirrors=*/false, dashed_line_start_segment_ids,
        std::move(dashed_line_segments),
        /*sub_names=*/{"DashedLine"},
        /*cascade_buffers=*/{local_smoother_params.crossable_lane_buffer()},
        /*cascade_gains=*/{local_smoother_params.crossable_lane_cost_weight()},
        problem);
  }
}

void ToDebugProto(double end_xy_error, double end_theta_error,
                  const std::vector<TrajectoryPoint>& init_traj,
                  const std::vector<TrajectoryPoint>& res_traj,
                  const OptimizerSolverDebugHook<Tob>& debug_hook,
                  FreespaceLocalSmootherDebugProto* debug_info) {
  QCHECK_NOTNULL(debug_info);
  debug_info->mutable_end_pose_error()->set_xy(end_xy_error);
  debug_info->mutable_end_pose_error()->set_theta(end_theta_error);
  for (int k = 0; k < init_traj.size(); ++k) {
    init_traj[k].ToProto(debug_info->add_init_traj());
  }
  for (int k = 0; k < res_traj.size(); ++k) {
    res_traj[k].ToProto(debug_info->add_res_traj());
  }
  const auto& init_costs = debug_hook.init_costs;
  debug_info->mutable_init_costs()->set_cost(init_costs.cost);
  for (int i = 0; i < init_costs.ddp_costs.size(); ++i) {
    TrajectoryOptimizerCost* cost_proto =
        debug_info->mutable_init_costs()->add_costs();
    cost_proto->set_name(init_costs.ddp_costs[i].name);
    cost_proto->set_cost(init_costs.ddp_costs[i].value);
  }
  const auto& final_costs = debug_hook.final_costs;
  debug_info->mutable_final_costs()->set_cost(final_costs.cost);
  for (int i = 0; i < final_costs.ddp_costs.size(); ++i) {
    TrajectoryOptimizerCost* cost_proto =
        debug_info->mutable_final_costs()->add_costs();
    cost_proto->set_name(final_costs.ddp_costs[i].name);
    cost_proto->set_cost(final_costs.ddp_costs[i].value);
  }
  for (const auto& iteration : debug_hook.iterations) {
    TrajectoryOptimizerCostInfo* iters_proto = debug_info->add_iter_costs();
    iters_proto->set_cost(iteration.final_cost);
  }
}

}  // namespace

// NOLINTNEXTLINE
absl::StatusOr<DirectionalPath> SmoothLocalPath(
    const VehicleGeometryParamsProto& veh_geo_params,
    const VehicleDriveParamsProto& veh_drive_params,
    const MotionConstraintParamsProto& motion_constraint_params,
    const FreespaceLocalSmootherParamsProto& local_smoother_params,
    const VehicleCircleModelParamsProto& vehicle_model_params,
    const std::string& owner, const FreespaceMap& static_map,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const PlannerClusterObjectManager& /*cluster_obj_mgr*/,
    const absl::flat_hash_set<std::string>& stalled_object_ids,
    const absl::flat_hash_set<PlannerClusterObject::Id>&
    /*stalled_cluster_object_ids*/,
    const DirectionalPath& input_path, const TrajectoryPoint& plan_start_point,
    bool reset, const std::vector<PathPoint>& prev_path,
    FreespaceLocalSmootherDebugProto* debug_info,
    vis::vantage::ChartsDataProto* charts_data) {
  SCOPED_QTRACE("SmoothLocalPath");

  constexpr double kScale = 1.0;
  constexpr double kPathGain = 1.0;
  constexpr double kEndStateGain = 1.0;
  constexpr double kCurvatureBufferGain = 0.75;
  constexpr double kCurvatureRateBufferGain = 0.9;
  constexpr double kJerkBufferRatio = 0.75;
  constexpr double kSpeedDirectionGain = 10.0;
  constexpr double kMaxSpeed = 2.0;

  // Create reference line.
  std::vector<Vec2d> reference_line;
  constexpr double kPathStep = 0.1;  // 0.1m
  double s = input_path.path.front().s();
  while (s <= input_path.path.back().s()) {
    const auto pt = input_path.path.Evaluate(s);
    reference_line.emplace_back(Vec2d(pt.x(), pt.y()));
    s += kPathStep;
  }
  if (reference_line.size() <= 2) {
    return absl::InternalError(absl::StrFormat(
        "Local smoother ref line size is only %d", reference_line.size()));
  }
  std::vector<double> ref_line_deviation_gains(
      reference_line.size() - 1,
      local_smoother_params.ref_path_deviation_cost_weight());

  ASSIGN_OR_RETURN(const auto frenet_ref_line,
                   BuildBruteForceFrenetFrame(reference_line,
                                              /*down_sample_raw_points=*/true));
  // Create init trajectory.
  std::vector<TrajectoryPoint> init_traj;
  if (reset || prev_path.empty()) {
    init_traj = GetInitTrajectoryByPurePursuit(
        plan_start_point, frenet_ref_line, veh_geo_params, veh_drive_params,
        input_path.forward, kMaxSpeed);
    debug_info->set_init_traj_type(
        FreespaceLocalSmootherDebugProto::PURE_PURSUIT);
  } else {
    // TODO(fengzhuang): Maybe just repalce the first point is enough.
    init_traj = GetInitTrajectoryByPrevPath(plan_start_point, prev_path,
                                            input_path.forward);
    debug_info->set_init_traj_type(FreespaceLocalSmootherDebugProto::PREV_PATH);
  }
  if (!input_path.forward) {
    for (auto& pt : init_traj) {
      pt.set_theta(NormalizeAngle(pt.theta() + M_PI));
      pt.set_kappa(-pt.kappa());
      pt.set_v(-pt.v());
      pt.set_a(-pt.a());
      pt.set_s(-pt.s());
      pt.set_j(-pt.j());
      pt.set_psi(-pt.psi());
    }
  }
  if (FLAGS_send_freespace_local_smoother_path_to_canvas) {
    vis::Canvas& canvas = vis::vantage::GetCanvasClient()->GetCanvas(
        "freespace/local_smooth_path/init_traj");
    for (const auto& pt : init_traj) {
      canvas.DrawPoint(Vec3d(pt.pos()), vis::Color::kLightGray, 2);
    }
  }

  // Create reference states and control for regularization.
  constexpr double kStateDeviationGain = 1.0e-5;
  constexpr double kControlDeviationGain = 1.0e-5;
  const auto ref_xs = Tob::FitState(init_traj);
  const auto ref_us =
      Tob::FitControl(init_traj, Tob::GetStateAtStep(ref_xs, 0));

  // Create end attraction.
  Tob::StatesType end_attraction_xs =
      Tob::StatesType::Zero(kTobSteps * Tob::kStateSize);
  const auto current_sl = frenet_ref_line.XYToSL(plan_start_point.pos());
  const double s_step =
      (frenet_ref_line.end_s() - current_sl.s) / static_cast<double>(kTobSteps);
  const double target_v = s_step / kTobDt;
  // If target_v is very large, the end point should not be the end of
  // reference line.
  PathPoint end_point;
  if (target_v > kMaxSpeed) {
    const double end_s = current_sl.s + kMaxSpeed * kTobDt * kTobSteps;
    end_point = input_path.path.Evaluate(end_s + input_path.path.front().s());
  } else {
    end_point = input_path.path.back();
  }
  Tob::set_x(end_point.x(), kTobSteps - 1, &end_attraction_xs);
  Tob::set_y(end_point.y(), kTobSteps - 1, &end_attraction_xs);
  if (input_path.forward) {
    Tob::set_theta(end_point.theta(), kTobSteps - 1, &end_attraction_xs);
  } else {
    Tob::set_theta(NormalizeAngle(end_point.theta() + M_PI), kTobSteps - 1,
                   &end_attraction_xs);
  }
  std::vector<double> end_attraction_weights(end_attraction_xs.size(), 0.0);
  end_attraction_weights[(kTobSteps - 1) * Tob::kStateSize +
                         Tob::kStateXIndex] =
      local_smoother_params.end_pose_deviation_cost_weight();
  end_attraction_weights[(kTobSteps - 1) * Tob::kStateSize +
                         Tob::kStateYIndex] =
      local_smoother_params.end_pose_deviation_cost_weight();
  end_attraction_weights[(kTobSteps - 1) * Tob::kStateSize +
                         Tob::kStateThetaIndex] =
      local_smoother_params.end_theta_deviation_cost_weight();

  Tob problem(&motion_constraint_params, &veh_geo_params, &veh_drive_params,
              kTobSteps, kTobDt, /*enable_dynamic_2nd_derivatives=*/false,
              /*enable_post_process =*/false);

  // TODO(fengzhuang): Maybe need cascade buffer for ref_line.
  std::vector<double> ref_ls(reference_line.size() - 1, 0.0);
  problem.AddCost(std::make_unique<ReferenceLineDeviationCost<Tob>>(
      kTobSteps, kPathGain, kEndStateGain, std::move(ref_ls), reference_line,
      /*center_line_helper=*/nullptr, std::move(ref_line_deviation_gains),
      "ReferenceLineDeviationCost", kScale));
  problem.AddCost(std::make_unique<ReferenceStateDeviationCost<Tob>>(
      ref_xs,
      std::vector<double>(kTobSteps * Tob::kStateSize, kStateDeviationGain),
      "ReferenceStateDeviationCost", kScale));
  problem.AddCost(std::make_unique<ReferenceControlDeviationCost<Tob>>(
      ref_us,
      std::vector<double>(kTobSteps * Tob::kControlSize, kControlDeviationGain),
      "ReferenceControlDeviationCost", kScale));
  problem.AddCost(std::make_unique<CurvatureCost<Tob>>(
      kCurvatureBufferGain *
          ComputeCenterMaxCurvature(veh_geo_params, veh_drive_params),
      kTobSteps, "CurvatureCost",
      local_smoother_params.curvature_cost_weight()));
  if (input_path.forward) {
    problem.AddCost(std::make_unique<ForwardSpeedCost<Tob>>(
        "ForwardSpeedCost", kSpeedDirectionGain));
  } else {
    problem.AddCost(std::make_unique<BackwardSpeedCost<Tob>>(
        "BackwardSpeedCost", kSpeedDirectionGain));
  }
  problem.AddCost(std::make_unique<ReferenceStateDeviationCost<Tob>>(
      end_attraction_xs, std::move(end_attraction_weights), "EndAttractionCost",
      kScale));
  problem.AddCost(std::make_unique<TobCurvatureRateCost<Tob>>(
      motion_constraint_params.max_psi() * kCurvatureRateBufferGain,
      "CurvatureRateCost", local_smoother_params.curvature_rate_cost_weight()));
  problem.AddCost(std::make_unique<IntrinsicJerkCost<Tob>>(
      motion_constraint_params.max_accel_jerk() * kJerkBufferRatio,
      motion_constraint_params.max_decel_jerk() * kJerkBufferRatio,
      "IntrinsicJerkCost", local_smoother_params.intrinsic_jerk_cost_weight()));
  std::unique_ptr<ModelHelper> av_model_helper =
      ConstructAvModelHelper(vehicle_model_params);
  AddObjectCosts(veh_geo_params, local_smoother_params, vehicle_model_params,
                 av_model_helper.get(), st_traj_mgr, stalled_object_ids,
                 input_path, init_traj, &problem);
  problem.AddCostHelper(std::move(av_model_helper));
  AddStaticBoundaryCost(veh_geo_params, local_smoother_params,
                        vehicle_model_params, static_map, input_path, &problem,
                        debug_info);

  DdpOptimizer<Tob> smoother(
      &problem, kTobSteps, owner,
      /*verbosity=*/FLAGS_freespace_local_smoother_verbosity_level,
      local_smoother_params.optimizer_params());
  OptimizerSolverDebugHook<Tob> debug_hook(kTobSteps, init_traj.front(),
                                           st_traj_mgr);
  smoother.AddHook(&debug_hook);

  const auto solve_start_time = absl::Now();
  const auto output =
      smoother.Solve(init_traj, DdpOptimizer<Tob>::SolveConfig::Onboard());
  const absl::Duration time_consuming = absl::Now() - solve_start_time;

  VLOG(2) << "Freespace local smoother time(ms): "
          << absl::ToDoubleMilliseconds(time_consuming);
  if (!output.ok()) {
    debug_info->set_status(output.status().ToString());
    return absl::InternalError("Local path smoother: DDP failed! " +
                               output.status().ToString());
  }
  const double end_xy_error = Hypot(end_point.x() - output->back().pos().x(),
                                    end_point.y() - output->back().pos().y());
  const double end_theta_error = std::abs(NormalizeAngle(
      (input_path.forward ? end_point.theta() : (end_point.theta() + M_PI)) -
      output->back().theta()));
  VLOG(2) << "Endpoint pose error = " << end_xy_error
          << ", theta error = " << end_theta_error;
  ToDebugProto(end_xy_error, end_theta_error, init_traj, *output, debug_hook,
               debug_info);
  std::vector<TrajectoryPlotInfo> plot_trajs = {
      {.traj = *output, .name = "res", .color = vis::Color::kRed}};
  if (charts_data != nullptr) {
    optimizer::AddTrajCharts("freespace/local_smoother", plot_trajs,
                             charts_data->mutable_charts());
  }
  if (FLAGS_send_freespace_local_smoother_path_to_canvas) {
    vis::Canvas& canvas = vis::vantage::GetCanvasClient()->GetCanvas(
        "freespace/local_smooth_path/res_traj");
    for (const auto& pt : *output) {
      canvas.DrawPoint(Vec3d(pt.pos()), vis::Color::kWhite, 2);
    }
  }
  if (FLAGS_draw_boundary_cost_buffer_circle) {
    const Vec2d rac = output->front().pos();
    const double heading = output->front().theta();
    const auto get_circle_center = [&rac, &heading](double dist_to_rac,
                                                    double angle_to_axis) {
      const auto tangent = Vec2d::FastUnitFromAngle(heading + angle_to_axis);
      return rac + dist_to_rac * tangent;
    };
    vis::Canvas& canvas = vis::vantage::GetCanvasClient()->GetCanvas(
        "freespace/local_smooth_path/boundary_cost_buffer_circle");
    for (int i = 0; i < vehicle_model_params.circles_size(); ++i) {
      const auto& param = vehicle_model_params.circles(i);
      const auto pos =
          get_circle_center(param.dist_to_rac(), param.angle_to_axis());
      canvas.DrawCircle(Vec3d(pos), param.radius(), vis::Color::kWhite, 1);
    }
  }

  // Extract result.
  std::vector<PathPoint> raw_path_points;
  raw_path_points.reserve(output->size());
  const double first_traj_pt_s = output->front().s();
  for (const auto& traj_pt : *output) {
    PathPoint pt;
    pt.set_x(traj_pt.pos()[0]);
    pt.set_y(traj_pt.pos()[1]);
    pt.set_theta(input_path.forward ? traj_pt.theta()
                                    : NormalizeAngle(traj_pt.theta() + M_PI));
    pt.set_kappa(input_path.forward ? traj_pt.kappa() : -traj_pt.kappa());
    // Make sure path point s starts from zero.
    pt.set_s(input_path.forward ? traj_pt.s() - first_traj_pt_s
                                : first_traj_pt_s - traj_pt.s());
    raw_path_points.push_back(std::move(pt));
  }

  // Delete zigzag points.
  auto prev_iter = raw_path_points.begin();
  for (auto iter = raw_path_points.begin() + 1;
       iter != raw_path_points.end();) {
    if (iter->s() <= prev_iter->s()) {
      iter = raw_path_points.erase(iter);
    } else {
      iter++;
      prev_iter++;
    }
  }
  // Keep at least two points to build path.
  if (raw_path_points.size() == 1) {
    raw_path_points.push_back(raw_path_points.front());
  }

  DirectionalPath res = {DiscretizedPath(std::move(raw_path_points)),
                         input_path.forward};
  debug_info->set_status("success");
  return res;
}
}  // namespace planner
}  // namespace qcraft
