#include "onboard/planner/optimization/ddp/object_cost_util.h"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <ostream>
#include <string>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "google/protobuf/repeated_ptr_field.h"

#include "onboard/async/parallel_for.h"
#include "onboard/global/buffered_logger.h"
#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/math/fast_math.h"
#include "onboard/math/frenet_common.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/math/geometry/proto/aabox3d.pb.h"
#include "onboard/math/piecewise_const_function.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/math/util.h"
#include "onboard/planner/object/planner_object.h"
#include "onboard/planner/object/proto/planner_object.pb.h"
#include "onboard/planner/object/spacetime_object_state.h"
#include "onboard/planner/object/spacetime_object_trajectory.h"
#include "onboard/planner/optimization/problem/aggregate_static_object_cost.h"
#include "onboard/planner/optimization/problem/partitioned_object_cost.h"
#include "onboard/planner/optimization/problem/unidirectional_object_cost.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/planner_util.h"
#include "onboard/planner/second_order_trajectory_point.h"
#include "onboard/planner/trajectory_point.h"
#include "onboard/planner/util/perception_util.h"
#include "onboard/planner/util/vehicle_geometry_util.h"
#include "onboard/prediction/predicted_trajectory.h"
#include "onboard/proto/perception/fusion/object.pb.h"
#include "onboard/utils/map_util.h"
#include "onboard/utils/status_macros.h"
#include "onboard/vis/canvas/canvas.h"
#include "onboard/vis/common/color.h"

#include "offboard/vis/vantage/vantage_server/vantage_canvas_client.h"

DEFINE_bool(traj_opt_draw_object_canvas, false,
            "If send object cost reults to canvas.");

namespace qcraft {
namespace planner {
namespace optimizer {
namespace {
// Increment in Z-axis to draw trajectory canvas.
constexpr double kTrajVisZInc =
    kSpaceTimeVisualizationDefaultTimeScale * kTrajectoryTimeStep;

struct NudgeParam {
  double buffer = 0.0;
  double weight = 1.0;
};

struct ObjectNudgeBufferInfo {
  bool is_oncoming = false;
  double ttc = std::numeric_limits<double>::infinity();
};

double ComputeConstObjectSpeedTtc(double v_ego, double v_obj, double a_ego,
                                  double d_obj, double d_safe) {
  QCHECK_GE(v_ego, 0.0);
  QCHECK_GE(d_safe, 0.0);

  const double d_rel = d_obj - d_safe;
  if (d_rel < 0.0) return 0.0;

  const double v_rel = v_ego - v_obj;
  if (v_rel <= 0.0 && a_ego <= 0.0) {
    return std::numeric_limits<double>::infinity();
  }

  if (a_ego == 0.0) return d_rel / v_rel;
  const double d_safe_rel = v_rel * v_rel + 2.0 * a_ego * d_rel;
  if (d_safe_rel <= 0.0) {
    std::numeric_limits<double>::infinity();
  }
  const double ttc = (-v_rel + std::sqrt(d_safe_rel)) / a_ego;
  if (v_obj >= 0.0) {
    return ttc;
  }
  const double av_stop_t =
      a_ego < 0.0 ? v_ego / a_ego : std::numeric_limits<double>::infinity();
  if (ttc < av_stop_t) {
    return ttc;
  }
  const double dist_after_av_stop =
      d_rel - v_rel * av_stop_t - 0.5 * a_ego * Sqr(av_stop_t);
  return av_stop_t - dist_after_av_stop / v_obj;
}

absl::StatusOr<ObjectNudgeBufferInfo> ComputeObjectNudgeBufferInfo(
    const TrajectoryPoint& plan_start_point,
    const prediction::PredictedTrajectoryPoint& object_start_point,
    const DrivePassage& drive_passage, const FrenetBox& av_frenet_box,
    const FrenetBox& object_frenet_box) {
  // TODO(Runbing): Use unbounded frenet query.
  ASSIGN_OR_RETURN(const auto av_dp_dir,
                   drive_passage.QueryTangentAtS(av_frenet_box.center_s()));
  ASSIGN_OR_RETURN(const auto object_dp_dir,
                   drive_passage.QueryTangentAtS(object_frenet_box.center_s()));
  const double av_dir_dp_dir_dot =
      Vec2d::FastUnitFromAngle(plan_start_point.theta()).dot(av_dp_dir);
  const double object_dir_dp_dir_dot =
      Vec2d::FastUnitFromAngle(object_start_point.theta()).dot(object_dp_dir);
  const double av_speed_dp = plan_start_point.v() * av_dir_dp_dir_dot;
  const double av_accel_dp = plan_start_point.a() * av_dir_dp_dir_dot;
  const double object_speed_dp = object_start_point.v() * object_dir_dp_dir_dot;
  return ObjectNudgeBufferInfo{
      .is_oncoming = object_speed_dp < 0.0,
      .ttc = ComputeConstObjectSpeedTtc(
          std::max(av_speed_dp, 0.0), object_speed_dp, av_accel_dp,
          object_frenet_box.s_min - av_frenet_box.s_max,
          /*d_safe=*/0.0)};
}

NudgeParam GenerateNudgeBufferAndWeightForObject(
    double t, const ObjectNudgeBufferInfo& object_nudge_buffer_info,
    double nudge_buffer, const Vec2d& object_velocity,
    const TrajectoryPoint& plan_start_point, const FrenetBox& object_frenet_box,
    const FrenetBox& av_frenet_box, const PathTimeCorridor& path_time_corridor,
    const TrajectoryOptimizerCostWeightParamsProto::NudgeBufferParamsProto&
        nudge_buffer_params,
    double vehicle_width) {
  const PiecewiseLinearFunction<double> nudge_buffer_av_speed_plf =
      PiecewiseLinearFunctionFromProto(
          nudge_buffer_params.nudge_front_buffer_object_speed_plf());
  const PiecewiseLinearFunction<double> min_nudge_buffer_speed_plf =
      PiecewiseLinearFunctionFromProto(
          nudge_buffer_params.min_nudge_buffer_speed_plf());
  nudge_buffer = nudge_buffer * nudge_buffer_av_speed_plf(plan_start_point.v());

  // Consider lane boundary for all object.
  double nudge_buffer_consider_lane = nudge_buffer;
  const auto boudnary_info = path_time_corridor.QueryBoundaryL(
      object_frenet_box.s_min, object_frenet_box.s_max,
      t, /*min_lane_width_idx=*/
      nullptr);
  const double min_nudge_buffer = nudge_buffer_params.min_nudge_buffer();
  const double nudge_buffer_lane_width_gain =
      nudge_buffer_params.nudge_buffer_lane_width_gain();
  const double vehicle_half_width = vehicle_width * 0.5;
  if (object_frenet_box.l_max < 0.0) {
    // If object on the right side of stations.
    nudge_buffer_consider_lane =
        std::clamp((-boudnary_info.first->l_boundary - vehicle_half_width) *
                       nudge_buffer_lane_width_gain,
                   min_nudge_buffer, nudge_buffer);
  } else if (object_frenet_box.l_min > 0.0) {
    nudge_buffer_consider_lane =
        std::clamp((boudnary_info.second->l_boundary - vehicle_half_width) *
                       nudge_buffer_lane_width_gain,
                   min_nudge_buffer, nudge_buffer);
  }
  nudge_buffer_consider_lane =
      std::min(nudge_buffer_consider_lane,
               std::clamp(((boudnary_info.second->l_boundary -
                            boudnary_info.first->l_boundary) *
                               0.5 -
                           vehicle_half_width) *
                              nudge_buffer_lane_width_gain,
                          min_nudge_buffer, nudge_buffer));
  const Vec2d av_local_dir = Vec2d::FastUnitFromAngle(plan_start_point.theta());
  const double obj_v_av_local = object_velocity.dot(av_local_dir);
  const double nudge_buffer_min =
      nudge_buffer * min_nudge_buffer_speed_plf(
                         std::max(obj_v_av_local, plan_start_point.v()));
  nudge_buffer = std::max(nudge_buffer_consider_lane, nudge_buffer_min);
  const auto object_boundary =
      path_time_corridor.QueryNarrowestBoundaryAllTypes(
          object_frenet_box.s_min, object_frenet_box.s_max, t);
  nudge_buffer = std::min(
      nudge_buffer, 0.5 * (object_boundary.second.l_object -
                           object_boundary.first.l_object - vehicle_width));
  if (object_nudge_buffer_info.is_oncoming) {
    return {.buffer = nudge_buffer, .weight = 1.0};
  }
  double l_dist = 0.0;
  // TODO(Runbing): Don't use object frenet to check nudge dir.
  if (object_frenet_box.l_min > av_frenet_box.l_max > 0.0) {
    l_dist = object_frenet_box.l_min - av_frenet_box.l_max;
  } else if (av_frenet_box.l_min - object_frenet_box.l_max > 0.0) {
    l_dist = av_frenet_box.l_min - object_frenet_box.l_max;
  } else {
    l_dist = 0.0;
  }
  const double ttc = object_nudge_buffer_info.ttc;
  const double nudge_buffer_without_clamp_object = nudge_buffer;
  if (ttc == 0.0) {
    constexpr double kVelDiff = 1.0;  // m/s
    const double alpha = std::clamp(
        (plan_start_point.v() - obj_v_av_local) / kVelDiff, 0.0, 1.0);
    nudge_buffer = Lerp(std::min(l_dist, nudge_buffer), nudge_buffer, alpha);
  } else if (ttc > 0.0) {
    constexpr double kTTCClampNudgeBufferBase = 10.0;   // s
    constexpr double kTTCClampNudgeBufferOffset = 4.0;  // s
    const double alpha = std::clamp(
        (ttc - kTTCClampNudgeBufferOffset) / kTTCClampNudgeBufferBase, 0.0,
        1.0);
    nudge_buffer = Lerp(nudge_buffer, std::min(l_dist, nudge_buffer), alpha);
  }
  constexpr double kNudgeBufferClampWeightBase = 0.5;
  const double weight_alpha =
      std::clamp((nudge_buffer_without_clamp_object - nudge_buffer) /
                     kNudgeBufferClampWeightBase,
                 0.0, 1.0);
  constexpr double kWeightFactor = 0.1;
  const double weight = 1.0 / (1.0 - weight_alpha + kWeightFactor);
  return {.buffer = nudge_buffer, .weight = weight};
}

bool AddPartitionAvObjectCost(
    double trajectory_time_step, std::string_view base_name,
    double nudge_buffer, const ObjectNudgeBufferInfo& object_nudge_buffer_info,
    double vehicle_width, bool consider_mirrors, const Vec2d& object_velocity,
    const FrenetBox& av_frenet_box,
    const std::vector<SpacetimeObjectState>& states,
    const std::vector<TrajectoryPoint>& init_traj, bool is_stationary,
    const DrivePassage& drive_passage,
    const PathTimeCorridor& path_time_corridor,
    const TrajectoryOptimizerCostWeightParamsProto& cost_weight_params,
    const VehicleCircleModelParamsProto&
        trajectory_optimizer_vehicle_model_params,
    const std::unique_ptr<AvModelHelper<Mfob>>& av_model_helpers,
    absl::string_view traj_id, double gain,
    std::optional<double>* first_nudge_buffer,
    std::vector<std::unique_ptr<Cost<Mfob>>>* costs) {
  using ObjectCost = PartitionedObjectCost<Mfob>;
  QCHECK_GT(trajectory_time_step, 0.0);

  constexpr double kSafeBuffer = 0.5;  // m.
  const int num_points = static_cast<int>(states.size());

  const auto& object_cost_params = cost_weight_params.object_cost_params();
  std::vector<double> cascade_gains = {
      object_cost_params.object_b_cost_weight(),
      object_cost_params.object_a_cost_weight()};

  google::protobuf::RepeatedPtrField<
      ::qcraft::VehicleCircleModelParamsProto_CircleParams>
      circles = trajectory_optimizer_vehicle_model_params.circles();
  if (consider_mirrors) {
    for (const auto& circle :
         trajectory_optimizer_vehicle_model_params.mirror_circles()) {
      *circles.Add() = circle;
    }
  }

  const int circle_size = circles.size();
  std::vector<double> dists_to_rac;
  std::vector<double> angles_to_axis;
  std::vector<double> circles_radius;
  dists_to_rac.reserve(circle_size);
  angles_to_axis.reserve(circle_size);
  circles_radius.reserve(circle_size);
  double max_buffer_plus_radius = 0.0;
  double max_model_dist = 0.0;
  for (const auto& circle : circles) {
    dists_to_rac.push_back(circle.dist_to_rac());
    angles_to_axis.push_back(circle.angle_to_axis());
    circles_radius.push_back(circle.radius());
    max_buffer_plus_radius = std::max(max_buffer_plus_radius, circle.radius());
    max_model_dist = std::max(max_model_dist, circle.dist_to_rac());
  }

  std::vector<ObjectCost::filter> filters;
  std::vector<std::vector<ObjectCost::Object>> objects;
  objects.reserve(num_points);
  filters.reserve(num_points);

  const PiecewiseConstFunction<double, double> nudge_buffer_time_gain_pcf =
      PiecewiseConstFunctionFromProto(object_cost_params.nudge_buffer_params()
                                          .nudge_buffer_time_gain_pcf());
  const double dp_real_s =
      drive_passage.station(drive_passage.last_real_station_index())
          .accumulated_s();

  std::optional<FrenetCoordinate> last_real_frenet_pt;
  const SpacetimeObjectState* last_real_state = nullptr;

  for (int k = 0; k < num_points; ++k) {
    const Vec2d x = init_traj[k].pos();
    const auto& traj_point = *states[k].traj_point;
    const Vec2d obj_x = traj_point.pos();
    const Polygon2d& contour = states[k].contour;
    const double t = static_cast<double>(k) * trajectory_time_step;
    const double gain = is_stationary ? 1.0 : nudge_buffer_time_gain_pcf(t);

    const auto frenet_box_or = drive_passage.QueryFrenetBoxAtContour(contour);
    if (!frenet_box_or.ok()) break;

    std::optional<Polygon2d> contour_modified;

    if (frenet_box_or->s_min > dp_real_s && last_real_frenet_pt.has_value() &&
        last_real_state != nullptr) {
      const auto frenet_obj_x = drive_passage.QueryFrenetCoordinateAt(obj_x);
      if (!frenet_obj_x.ok()) break;
      const auto lane_theta_at_obj_s =
          drive_passage.QueryTangentAngleAtS(frenet_obj_x->s);
      if (!lane_theta_at_obj_s.ok()) break;
      const auto obj_x_modified = drive_passage.QueryPointXYAtSL(
          frenet_obj_x->s, last_real_frenet_pt->l);
      if (!obj_x_modified.ok()) break;
      const Vec2d rotation = Vec2d::FastUnitFromAngle(
          *lane_theta_at_obj_s - last_real_state->box.heading());
      contour_modified = last_real_state->contour.Transform(
          last_real_state->traj_point->pos(), rotation.x(), rotation.y(),
          *obj_x_modified - last_real_state->traj_point->pos());
    } else {
      last_real_state = &states[k];
      const auto frenet_obj_x = drive_passage.QueryFrenetCoordinateAt(obj_x);
      if (frenet_obj_x.ok()) {
        last_real_frenet_pt = *frenet_obj_x;
      }
    }

    objects.emplace_back();
    auto& objects_k = objects.back();
    const auto nudge_param_k = GenerateNudgeBufferAndWeightForObject(
        t, object_nudge_buffer_info, nudge_buffer, object_velocity,
        init_traj.front(), *frenet_box_or, av_frenet_box, path_time_corridor,
        object_cost_params.nudge_buffer_params(), vehicle_width);
    if (!first_nudge_buffer->has_value()) {
      *first_nudge_buffer = nudge_param_k.buffer;
    }
    std::vector<double> nudge_buffers_k = {
        std::min(nudge_param_k.buffer, kSafeBuffer), nudge_param_k.buffer};
    const double filter_offset =
        max_buffer_plus_radius + max_model_dist +
        *std::max_element(nudge_buffers_k.begin(), nudge_buffers_k.end());

    for (int idx = 0; idx < circle_size; ++idx) {
      std::vector<double> circle_buffers = nudge_buffers_k;
      std::vector<Segment2d> lines;
      Vec2d ref_x;
      Vec2d ref_tangent;
      double offset = 0.0;

      const Vec2d tangent =
          Vec2d::FastUnitFromAngle(init_traj[k].theta() + angles_to_axis[idx]);
      const Vec2d x_center = x + tangent * dists_to_rac[idx];

      const double circle_radius = circles_radius[idx];
      CalcPartitionHalfContourInfo(
          x_center, obj_x,
          contour_modified.has_value() ? *contour_modified : contour,
          *std::max_element(nudge_buffers_k.begin(), nudge_buffers_k.end()) +
              circle_radius,
          &lines, &ref_x, &ref_tangent, &offset);
      QCHECK(!lines.empty());
      for (int i = 0; i < circle_buffers.size(); ++i) {
        circle_buffers[i] = nudge_buffers_k[i] * gain + circle_radius;
      }
      objects_k.push_back(ObjectCost::Object{
          .lines = lines,
          .buffers = std::move(circle_buffers),
          .gains = {nudge_param_k.weight, nudge_param_k.weight},
          .ref_x = ref_x,
          .offset = offset,
          .ref_tangent = ref_tangent,
          .enable = true});

      if (FLAGS_traj_opt_draw_object_canvas) {
        vis::Canvas* canvas_line = nullptr;
        vis::Canvas* canvas_contour = nullptr;

        canvas_line = &vis::vantage::GetCanvasClient()->GetCanvas(
            absl::StrFormat("%s/object/%s/partition/line_%03f/%03d", base_name,
                            traj_id, dists_to_rac[idx], k));
        canvas_contour = &vis::vantage::GetCanvasClient()->GetCanvas(
            absl::StrFormat("%s/object/%s/partition/contour_%03f/%03d",
                            base_name, traj_id, dists_to_rac[idx], k));
        const double z = k * kTrajVisZInc;
        // Draw rac
        for (int i = 0; i < lines.size(); i++) {
          const auto& line = lines[i];
          QCHECK_NOTNULL(canvas_line)
              ->DrawLine({line.start().x(), line.start().y(), z},
                         {line.end().x(), line.end().y(), z},
                         vis::Color(0.7, 0.2, 0.2));
        }
        canvas_line->SetGroundZero(1);
        QCHECK_NOTNULL(canvas_contour)
            ->DrawPolygon(
                contour_modified.has_value() ? *contour_modified : contour, z,
                vis::Color(0.7, 0.2, 0.2));
        canvas_contour->SetGroundZero(1);
      }
    }

    // Update filter.
    Vec2d ref_x = {0.0, 0.0};
    Vec2d ref_tangent = {0.0, 0.0};
    for (const auto& object : objects_k) {
      ref_x += object.ref_x;
      ref_tangent += object.ref_tangent;
    }
    ref_x /= static_cast<double>(objects_k.size());
    ref_tangent /= static_cast<double>(objects_k.size());
    ref_tangent = ref_tangent.normalized();

    Vec2d front, back;
    int front_index, back_index;
    contour.ExtremePoints(ref_tangent, &back_index, &front_index, &back,
                          &front);
    const double offset = (front - ref_x).dot(ref_tangent) + filter_offset;

    filters.emplace_back();
    ObjectCost::filter& filter = filters.back();
    filter.ref_x = std::move(ref_x);
    filter.ref_tangent = std::move(ref_tangent);
    filter.offset = offset;
  }

  costs->emplace_back(std::make_unique<ObjectCost>(
      std::move(objects), std::move(filters), std::move(dists_to_rac),
      std::move(angles_to_axis), std::move(cascade_gains),
      av_model_helpers.get(),
      /*sub_names=*/std::vector<std::string>({"Inner", "Outer"}),
      /*using_hessian_approximate=*/true,
      absl::StrFormat("Partition AV Object: for %s", traj_id),
      gain * cost_weight_params.object_cost_weight(),
      /*cost_type=*/Cost<Mfob>::CostType::GROUP_OBJECT));
  return true;
}

bool AddUnidirectionalObjectCostForLeading(
    double trajectory_time_step, std::string_view base_name,
    const std::vector<SpacetimeObjectState>& states,
    const std::vector<TrajectoryPoint>& init_traj,
    const DrivePassage& drive_passage, const PathSlBoundary& path_boundary,
    const std::map<std::string, ConstraintProto::LeadingObjectProto>&
        leading_trajs,
    const TrajectoryOptimizerCostWeightParamsProto& cost_weight_params,
    const VehicleGeometryParamsProto& veh_geo_params,
    const MotionConstraintParamsProto& motion_constraint_params,
    absl::string_view traj_id, double gain,
    std::vector<LeadingInfo>* leading_min_s,
    std::vector<std::unique_ptr<Cost<Mfob>>>* costs) {
  QCHECK_GT(trajectory_time_step, 0.0);
  FUNC_QTRACE();

  QCHECK_GE(leading_min_s->size(), states.size());

  using ObjectCost = UnidirectionalObjectCost<Mfob>;
  const auto& object_cost_params = cost_weight_params.object_cost_params();

  const double follow_buffer = object_cost_params.acc_standstill_standoff();
  const double half_width = 0.5 * veh_geo_params.width();
  const std::vector<double> buffers = {
      follow_buffer + half_width,
      object_cost_params.acc_safe_standstill_standoff() + half_width};

  std::vector<ObjectCost::Object> objects;
  const int num_points = states.size();
  objects.reserve(num_points);

  double t_range = kSpacetimePlannerTrajectoryHorizon;
  const Vec2d x = init_traj.front().pos();
  const Vec2d tangent = Vec2d::FastUnitFromAngle(init_traj.front().theta());
  const double dist_to_rac = veh_geo_params.front_edge_to_center() - half_width;
  const Vec2d circle_center = x + tangent * dist_to_rac;

  const auto& leading_object_proto =
      FindOrNull(leading_trajs, std::string(traj_id));
  if (leading_object_proto != nullptr &&
      !leading_object_proto->st_constraints().empty()) {
    const int st_constraints_length =
        leading_object_proto->st_constraints_size();
    t_range =
        leading_object_proto->st_constraints(st_constraints_length - 1).t();
  }

  const double plan_start_point_v = init_traj.front().v();
  const auto plan_start_point_sl_or =
      drive_passage.QueryFrenetCoordinateAt(circle_center);
  if (!plan_start_point_sl_or.ok()) {
    return false;
  }
  const double plan_start_point_s_on_drive_passage = plan_start_point_sl_or->s;
  const double max_deceleration = motion_constraint_params.max_deceleration();
  // Time of slowing down to zero. Assume that v > 0, so time shouldn't be
  // smaller than zero.
  const double slow_down_zero_time =
      std::max(0.0, plan_start_point_v / -max_deceleration);
  constexpr double kPenetrationOffset = 1.0;  // m.

  for (int k = 0; k < states.size(); ++k) {
    const auto& traj_point = *states[k].traj_point;
    if (traj_point.t() > t_range) {
      break;
    }
    const Polygon2d& contour = states[k].contour;

    const double t = std::min(static_cast<double>(k) * trajectory_time_step,
                              slow_down_zero_time);
    // Compute s that if av slow down with max deceleration from start s to now,
    // leading s should not be smaller than it to avoid abnormal braking traj.
    const double leading_min_s_on_drive_passage =
        plan_start_point_s_on_drive_passage + plan_start_point_v * t +
        0.5 * max_deceleration * Sqr(t) + follow_buffer + half_width -
        kPenetrationOffset;

    const auto frenet_box_or = drive_passage.QueryFrenetBoxAtContour(contour);
    if (!frenet_box_or.ok()) {
      break;
    }

    const auto& frenet_box = *frenet_box_or;
    if (frenet_box.s_min > path_boundary.end_s()) {
      break;
    }

    const double leading_cost_s =
        std::max(leading_min_s_on_drive_passage, frenet_box.s_min);
    (*leading_min_s)[k] = LeadingInfo{.s = leading_cost_s, .v = traj_point.v()};

    const auto [right_boundary_point, left_boundary_point] =
        path_boundary.QueryBoundaryXY(leading_cost_s);
    Segment2d mid_line(left_boundary_point, right_boundary_point);

    // Draw leading.
    if (FLAGS_traj_opt_draw_object_canvas) {
      vis::Canvas* canvas_line = nullptr;
      vis::Canvas* canvas_contour = nullptr;

      canvas_line = &vis::vantage::GetCanvasClient()->GetCanvas(
          absl::StrFormat("%s/object/%s/leading/%03d", base_name, traj_id, k));
      canvas_contour = &vis::vantage::GetCanvasClient()->GetCanvas(
          absl::StrFormat("%s/object/%s/leading/%03d", base_name, traj_id, k));
      const double z = k * kTrajVisZInc;
      // Draw rac
      QCHECK_NOTNULL(canvas_line)
          ->DrawLine({mid_line.start().x(), mid_line.start().y(), z},
                     {mid_line.end().x(), mid_line.end().y(), z},
                     vis::Color(0.7, 0.2, 0.2));
      canvas_line->SetGroundZero(1);
      QCHECK_NOTNULL(canvas_contour)
          ->DrawPolygon(contour, z, vis::Color(0.7, 0.2, 0.2));
      canvas_contour->SetGroundZero(1);
    }

    objects.push_back(ObjectCost::Object{
        .dir = -mid_line.unit_direction().Perp(),
        .ref = (mid_line.start() + mid_line.end()) * 0.5,
        .lateral_extent = mid_line.length() * 0.5,
        .buffers = buffers,
        .gains = {object_cost_params.leading_object_a_cost_weight(),
                  object_cost_params.leading_object_b_cost_weight()},
        .enable = true});
  }

  std::vector<double> dist_to_rac_vec = {dist_to_rac};
  std::vector<double> angle_to_axis_vec = {0.0};

  costs->emplace_back(std::make_unique<ObjectCost>(
      std::move(objects), std::move(dist_to_rac_vec),
      std::move(angle_to_axis_vec),
      /*sub_names=*/std::vector<std::string>({SoftNameString, HardNameString}),
      /*using_hessian_approximate=*/true,
      absl::StrFormat("Leading Object (F): for %s", traj_id),
      gain * cost_weight_params.object_cost_weight(),
      /*cost_type=*/Cost<Mfob>::CostType::GROUP_OBJECT));
  return true;
}

using UndirectionalObjectCostParams = TrajectoryOptimizerCostWeightParamsProto::
    UnidirectionalNudgeObjectCostParamsProto;
std::string GetUnidirectionalObjectSourceString(
    UndirectionalObjectCostParams::Source source) {
  switch (source) {
    case UndirectionalObjectCostParams::LARGE_VEHICLE:
      return "Large Vehicle";
    case UndirectionalObjectCostParams::DRIVE_IN:
      return "Drive In";
    case UndirectionalObjectCostParams::VEHICLE:
      return "Vehicle";
  }
}

enum class Direction {
  kLeft = 1,
  kNormal = 0,
  kRight = -1,
};

absl::Status ComputeUnidirectionalObjectCostSegment(
    const DrivePassage& drive_passage, double l_offset, Direction direction,
    const FrenetBox& frenet_box, double speed, double look_ahead_time,
    double look_back_time, Segment2d* line) {
  if (direction == Direction::kLeft) {
    const auto end = drive_passage.QueryPointXYAtSL(
        std::min(drive_passage.end_s(),
                 frenet_box.s_min - speed * look_back_time),
        l_offset);
    const auto start = drive_passage.QueryPointXYAtSL(
        std::max(drive_passage.front_s(),
                 frenet_box.s_max + speed * look_ahead_time),
        l_offset);
    if (!(end.ok() && start.ok())) {
      return absl::InternalError("End or Start frenet to xy failed.");
    }
    *line = Segment2d(*start, *end);
  } else if (direction == Direction::kRight) {
    const auto end = drive_passage.QueryPointXYAtSL(
        std::min(drive_passage.end_s(),
                 frenet_box.s_max + speed * look_ahead_time),
        l_offset);
    const auto start = drive_passage.QueryPointXYAtSL(
        std::max(drive_passage.front_s(),
                 frenet_box.s_min - speed * look_back_time),
        l_offset);
    if (!(end.ok() && start.ok())) {
      return absl::InternalError("End or Start frenet to xy failed.");
    }
    *line = Segment2d(*start, *end);
  }
  return absl::OkStatus();
}

bool AddUnidirectionalObjectCost(
    double trajectory_time_step, const Vec2d& object_velocity,
    std::string_view base_name, double vehicle_width,
    double front_edge_to_center,
    const ObjectNudgeBufferInfo& object_nudge_buffer_info,
    const FrenetBox& av_frenet_box, const TrajectoryPoint& plan_start_point,
    const std::vector<SpacetimeObjectState>& states,
    const PathTimeCorridor& path_time_corridor,
    const DrivePassage& drive_passage, const PathSlBoundary& path_boundary,
    const UndirectionalObjectCostParams& undirectional_nudge_object_cost_params,
    absl::string_view traj_id, double gain,
    std::optional<double>* first_nudge_buffer,
    std::vector<std::unique_ptr<Cost<Mfob>>>* costs) {
  QCHECK_GT(trajectory_time_step, 0.0);
  FUNC_QTRACE();

  using ObjectCost = UnidirectionalObjectCost<Mfob>;

  const double vehicle_half_width = 0.5 * vehicle_width;

  std::vector<ObjectCost::Object> objects;
  const int num_points = states.size();
  objects.reserve(num_points);

  const double large_vehicle_nudge_time =
      undirectional_nudge_object_cost_params.large_vehicle_nudge_time();
  const double l_min_offset_to_center =
      undirectional_nudge_object_cost_params.l_min_offset_to_center();
  const double min_dist_to_lane_boundary =
      undirectional_nudge_object_cost_params.min_dist_to_lane_boundary();
  const double look_ahead_time =
      undirectional_nudge_object_cost_params.look_ahead_time();
  const double look_back_time =
      undirectional_nudge_object_cost_params.look_back_time();
  const double l_hold_time =
      undirectional_nudge_object_cost_params.l_hold_time();
  const std::string& source = GetUnidirectionalObjectSourceString(
      undirectional_nudge_object_cost_params.source());

  const PiecewiseConstFunction<double, double> nudge_buffer_time_gain_pcf =
      PiecewiseConstFunctionFromProto(
          undirectional_nudge_object_cost_params.nudge_buffer_params()
              .nudge_buffer_time_gain_pcf());
  const double nudge_buffer_decay =
      undirectional_nudge_object_cost_params.nudge_buffer_decay();

  // Generate nudge direction.
  Direction direction = Direction::kNormal;
  const auto& object_contour = states[0].contour;
  const auto frenet_box_or =
      drive_passage.QueryFrenetBoxAtContour(object_contour);
  if (!frenet_box_or.ok()) {
    return false;
  }
  const double center_l =
      path_boundary.QueryReferenceCenterL(frenet_box_or->center_s());
  double av_l_now = 0.0;
  if (frenet_box_or->l_max < center_l) {
    direction = Direction::kLeft;
    av_l_now = av_frenet_box.l_min;
  } else if (frenet_box_or->l_min > center_l) {
    direction = Direction::kRight;
    av_l_now = av_frenet_box.l_max;
  } else {
    return false;
  }

  const double dp_real_s =
      drive_passage.station(drive_passage.last_real_station_index())
          .accumulated_s();

  const double max_penetration_base =
      undirectional_nudge_object_cost_params.max_penetration_base();
  const double max_penetration_gain =
      undirectional_nudge_object_cost_params.max_penetration_gain();
  std::optional<double> last_real_offset;
  for (int k = 0; k < states.size(); ++k) {
    const auto& traj_point = *states[k].traj_point;
    if (traj_point.t() > large_vehicle_nudge_time) {
      break;
    }
    const Polygon2d& contour = states[k].contour;
    const auto frenet_box_or = drive_passage.QueryFrenetBoxAtContour(contour);
    if (!frenet_box_or.ok()) {
      break;
    }

    const auto nudge_param_k = GenerateNudgeBufferAndWeightForObject(
        traj_point.t(), object_nudge_buffer_info,
        undirectional_nudge_object_cost_params.nudge_buffer_a(),
        object_velocity, plan_start_point, *frenet_box_or, av_frenet_box,
        path_time_corridor,
        undirectional_nudge_object_cost_params.nudge_buffer_params(),
        vehicle_width);
    if (!first_nudge_buffer->has_value()) {
      *first_nudge_buffer = nudge_param_k.buffer;
    }
    Segment2d line;
    if ((traj_point.t() > l_hold_time && last_real_offset.has_value()) ||
        (frenet_box_or->s_min > dp_real_s && last_real_offset.has_value())) {
      if (!ComputeUnidirectionalObjectCostSegment(
               drive_passage, *last_real_offset, direction, *frenet_box_or,
               traj_point.v(), look_ahead_time, look_back_time, &line)
               .ok()) {
        break;
      }
    } else {
      double l_offset = 0.0;
      const auto lane_theta =
          drive_passage.QueryTangentAngleAtS(frenet_box_or->s_min);
      if (!lane_theta.ok()) break;
      constexpr double kLookAheadTime = 2.0;  // s
      const auto boundary = path_time_corridor.QueryNarrowestBoundaryAllTypes(
          std::max(0.0, frenet_box_or->s_min - traj_point.v() * look_back_time),
          frenet_box_or->s_max + kLookAheadTime * traj_point.v(),
          traj_point.t());
      std::optional<double> object_l_offset;
      if (std::isfinite(boundary.second.l_object) &&
          std::isfinite(boundary.first.l_object)) {
        object_l_offset =
            (boundary.second.l_object + boundary.first.l_object) * 0.5;
      }
      const double center_l =
          path_boundary.QueryReferenceCenterL(frenet_box_or->center_s());
      const double nudge_buffer =
          nudge_param_k.buffer * nudge_buffer_time_gain_pcf(traj_point.t());
      const double nudge_buffer_normal = nudge_buffer * nudge_buffer_decay;
      constexpr double kCurbBesideLaneCheckFactor = 1.5;
      if (direction == Direction::kLeft) {
        const double l_offset_min_based_av =
            av_l_now + max_penetration_base +
            traj_point.t() * max_penetration_gain;
        double l_offset_without_object = 0.0;
        const double nudge_lane_based_center =
            std::min(l_offset_min_based_av,
                     center_l + l_min_offset_to_center - vehicle_half_width);
        const double boundary_base_l_max = boundary.second.l_boundary -
                                           min_dist_to_lane_boundary -
                                           vehicle_width;
        if (boundary.second.l_curb <
            boundary.second.l_boundary * kCurbBesideLaneCheckFactor) {
          const double center_l_with_curb =
              (boundary.second.l_curb + frenet_box_or->l_max) * 0.5 -
              vehicle_half_width;
          l_offset_without_object =
              std::min({frenet_box_or->l_max + nudge_buffer_normal,
                        boundary_base_l_max, center_l_with_curb});
        } else {
          l_offset_without_object =
              std::clamp(frenet_box_or->l_max + nudge_buffer,
                         nudge_lane_based_center, boundary_base_l_max);
          l_offset_without_object =
              std::max(l_offset_without_object,
                       frenet_box_or->l_max + nudge_buffer_normal);
        }
        l_offset = object_l_offset.has_value()
                       ? std::min(*object_l_offset - vehicle_half_width,
                                  l_offset_without_object)
                       : l_offset_without_object;
      } else if (direction == Direction::kRight) {
        const double l_offset_max_based_av =
            av_l_now - max_penetration_base -
            traj_point.t() * max_penetration_gain;
        double l_offset_without_object = 0.0;

        const double nudge_lane_based_center =
            std::max(l_offset_max_based_av,
                     center_l - l_min_offset_to_center + vehicle_half_width);
        const double boundary_base_l_min = boundary.first.l_boundary +
                                           min_dist_to_lane_boundary +
                                           vehicle_width;
        if (boundary.first.l_curb >
            boundary.first.l_boundary * kCurbBesideLaneCheckFactor) {
          const double center_l_with_curb =
              (frenet_box_or->l_min + boundary.first.l_curb) * 0.5 +
              vehicle_half_width;
          l_offset_without_object =
              std::max({frenet_box_or->l_min - nudge_buffer_normal,
                        center_l_with_curb, boundary_base_l_min});
        } else {
          l_offset_without_object =
              std::clamp(frenet_box_or->l_min - nudge_buffer,
                         boundary_base_l_min, nudge_lane_based_center);
          l_offset_without_object =
              std::min(l_offset_without_object,
                       frenet_box_or->l_min - nudge_buffer_normal);
        }
        l_offset = object_l_offset.has_value()
                       ? std::max(*object_l_offset + vehicle_half_width,
                                  l_offset_without_object)
                       : l_offset_without_object;
      }
      if (!ComputeUnidirectionalObjectCostSegment(
               drive_passage, l_offset, direction, *frenet_box_or,
               traj_point.v(), look_ahead_time, look_back_time, &line)
               .ok()) {
        break;
      }
      last_real_offset = l_offset;
    }

    // Draw leading.
    if (FLAGS_traj_opt_draw_object_canvas) {
      vis::Canvas* canvas_line = nullptr;
      vis::Canvas* canvas_contour = nullptr;
      canvas_line = &vis::vantage::GetCanvasClient()->GetCanvas(absl::StrFormat(
          "%s/object/%s/%s/line/%03d", base_name, source, traj_id, k));
      canvas_contour =
          &vis::vantage::GetCanvasClient()->GetCanvas(absl::StrFormat(
              "%s/object/%s/%s/contour/%03d", base_name, source, traj_id, k));
      const double z = k * kTrajVisZInc;
      // Draw rac
      QCHECK_NOTNULL(canvas_line)
          ->DrawLine({line.start().x(), line.start().y(), z},
                     {line.end().x(), line.end().y(), z},
                     vis::Color(0.7, 0.2, 0.2));
      canvas_line->SetGroundZero(1);
      QCHECK_NOTNULL(canvas_contour)
          ->DrawPolygon(contour, z, vis::Color(0.7, 0.2, 0.2));
      canvas_contour->SetGroundZero(1);
    }

    objects.push_back(ObjectCost::Object{
        .dir = -line.unit_direction().Perp(),
        .ref = (line.start() + line.end()) * 0.5,
        .lateral_extent = line.length() * 0.5,
        .buffers = {vehicle_half_width},
        .gains =
            {undirectional_nudge_object_cost_params.object_a_cost_weight() *
             nudge_param_k.weight},
        .enable = true});
  }
  std::vector<double> dist_to_rac = {0.0, front_edge_to_center};
  std::vector<double> angle_to_axis = {0.0, 0.0};
  costs->emplace_back(std::make_unique<ObjectCost>(
      objects, std::move(dist_to_rac), std::move(angle_to_axis),
      /*sub_names=*/std::vector<std::string>({SoftNameString}),
      /*using_hessian_approximate=*/true,
      absl::StrFormat(" %s Object: for %s", source, traj_id), gain,
      /*cost_type=*/Cost<Mfob>::CostType::GROUP_OBJECT));
  return true;
}

void GetClosestLeadingObjectInfo(
    int trajectory_steps, double trajectory_time_step,
    const DrivePassage& drive_passage,
    const std::map<std::string, ConstraintProto::LeadingObjectProto>&
        leading_trajs,
    absl::Span<const SpacetimeObjectTrajectory* const> spacetime_trajs,
    const TrajectoryOptimizerCostWeightParamsProto& cost_weight_params,
    std::optional<
        std::pair<std::vector<FrenetBox>, const SpacetimeObjectTrajectory*>>*
        stationary_closest_leading_object_info,
    std::optional<
        std::pair<std::vector<FrenetBox>, const SpacetimeObjectTrajectory*>>*
        moving_closest_leading_object_info) {
  QCHECK_GT(trajectory_steps, 0);
  QCHECK_GT(trajectory_time_step, 0.0);
  // End s can't be larger than max leading object s.
  // first: traj_id, second: closest_s.
  double stationary_closest_min_s = std::numeric_limits<double>::infinity();
  double moving_closest_min_s = std::numeric_limits<double>::infinity();
  const double acc_standstill_standoff =
      cost_weight_params.object_cost_params().acc_standstill_standoff();

  for (const auto& traj_ptr : spacetime_trajs) {
    const bool is_leading =
        leading_trajs.find(std::string(traj_ptr->traj_id())) !=
        leading_trajs.end();
    if (is_leading) {
      const Polygon2d& contour = traj_ptr->contour();
      const auto frenet_box_or = drive_passage.QueryFrenetBoxAtContour(contour);
      if (!frenet_box_or.ok()) {
        continue;
      }
      if (traj_ptr->is_stationary()) {
        if (double leading_s = frenet_box_or->s_min - acc_standstill_standoff;
            leading_s < stationary_closest_min_s) {
          *stationary_closest_leading_object_info = {{*frenet_box_or},
                                                     traj_ptr};
          stationary_closest_min_s = leading_s;
        }
      } else {
        if (double leading_s = frenet_box_or->s_min - acc_standstill_standoff;
            leading_s < moving_closest_min_s) {
          *moving_closest_leading_object_info = {std::vector<FrenetBox>(),
                                                 traj_ptr};
          moving_closest_min_s = leading_s;
        }
      }
    }
  }
  if (moving_closest_leading_object_info->has_value() &&
      moving_closest_leading_object_info->value().second != nullptr) {
    const auto states = SampleObjectStates(
        trajectory_steps, trajectory_time_step,
        moving_closest_leading_object_info->value().second->states());
    std::vector<FrenetBox>& frenet_boxes =
        moving_closest_leading_object_info->value().first;
    frenet_boxes.reserve(states.size());
    for (const auto& state : states) {
      const Polygon2d& contour = state.contour;
      const auto frenet_box_or = drive_passage.QueryFrenetBoxAtContour(contour);
      if (!frenet_box_or.ok()) {
        return;
      }
      frenet_boxes.push_back(frenet_box_or.value());
    }
  }
}

bool IgnoreObjectCost(
    std::string_view base_name, const SpacetimeObjectTrajectory& traj,
    const std::vector<SpacetimeObjectState>& sampled_states,
    const DrivePassage& drive_passage,
    const TrajectoryOptimizerCostWeightParamsProto& cost_weight_params,
    const std::optional<
        std::pair<std::vector<FrenetBox>, const SpacetimeObjectTrajectory*>>&
        stationary_closest_leading_object_info,
    const std::optional<
        std::pair<std::vector<FrenetBox>, const SpacetimeObjectTrajectory*>>&
        moving_closest_leading_object_info) {
  if (!stationary_closest_leading_object_info.has_value() &&
      !moving_closest_leading_object_info.has_value()) {
    return false;
  }
  // Don't ignore closest leading object itself.
  if (stationary_closest_leading_object_info.has_value() &&
      traj.traj_id() ==
          stationary_closest_leading_object_info->second->traj_id()) {
    return false;
  }
  // Ignore spacetime trajectories whose entire trajectory is behind the
  // closest leading object.
  const double acc_standstill_standoff =
      cost_weight_params.object_cost_params().acc_standstill_standoff();
  if (stationary_closest_leading_object_info.has_value()) {
    const double ignore_s =
        stationary_closest_leading_object_info->first.front().s_min;
    if (traj.is_stationary()) {
      const auto frenet_box_or =
          drive_passage.QueryFrenetBoxAtContour(traj.contour());
      if (!frenet_box_or.ok()) {
        VLOG(2) << base_name << " ignores stationary st-trajectory "
                << traj.traj_id()
                << " because we can't query its contour on drive passage.";
        return true;
      }
      if ((frenet_box_or->s_min - acc_standstill_standoff) < ignore_s) {
        return false;
      }
    } else {
      for (int k = 0; k < sampled_states.size(); ++k) {
        const Polygon2d& contour = sampled_states[k].contour;
        const auto frenet_box_or =
            drive_passage.QueryFrenetBoxAtContour(contour);
        if (!frenet_box_or.ok()) {
          return true;
        }
        if ((frenet_box_or->s_min - acc_standstill_standoff) < ignore_s) {
          return false;
        }
      }
    }
  }
  if (moving_closest_leading_object_info.has_value() &&
      traj.traj_id() == moving_closest_leading_object_info->second->traj_id()) {
    return false;
  }
  if (moving_closest_leading_object_info.has_value()) {
    const std::vector<FrenetBox>& leading_frenet_boxes =
        moving_closest_leading_object_info->first;
    if (sampled_states.size() > leading_frenet_boxes.size()) {
      return false;
    }
    for (int k = 0; k < sampled_states.size(); ++k) {
      const Polygon2d& contour = sampled_states[k].contour;
      const auto frenet_box_or = drive_passage.QueryFrenetBoxAtContour(contour);
      if (!frenet_box_or.ok()) {
        return true;
      }
      const FrenetBox& leading_frenet_box = leading_frenet_boxes[k];
      if ((frenet_box_or->s_min - acc_standstill_standoff) <
          leading_frenet_box.s_min) {
        return false;
      }
    }
  }
  VLOG(2) << base_name << " ignores st-trajectory " << traj.traj_id()
          << " because its entire trajectory is behind closest leading object.";
  return true;
}

double GenerateNudgeBufferForStatic(
    double av_speed, const Polygon2d& object_contour,
    const DrivePassage& drive_passage,
    const PathTimeCorridor& path_time_corridor, double vehicle_width,
    const TrajectoryOptimizerCostWeightParamsProto::
        AgggregateObjectCostParamsProto& aggregate_object_cost_params) {
  const PiecewiseLinearFunction<double> nudge_buffer_speed_plf =
      PiecewiseLinearFunctionFromProto(
          aggregate_object_cost_params.nudge_buffer_speed_plf());
  double nudge_buffer = nudge_buffer_speed_plf(av_speed);
  // Consider lane boundary for all object.
  const auto frenet_box = drive_passage.QueryFrenetBoxAtContour(object_contour);
  if (!frenet_box.ok()) return nudge_buffer;
  const auto boudnary_info = path_time_corridor.QueryBoundaryL(
      frenet_box->s_min, frenet_box->s_max, /*t=*/0.0,
      /*min_lane_width_idx=*/nullptr);
  double lane_width =
      std::min(boudnary_info.second->l_curb, boudnary_info.second->l_object) -
      std::max(boudnary_info.first->l_curb, boudnary_info.first->l_object);
  nudge_buffer = std::min(nudge_buffer, 0.5 * (lane_width - vehicle_width));
  return nudge_buffer;
}

bool AddAggregateStaticObjectCost(
    double trajectory_time_step, const TrajectoryPoint& plan_start_point,
    const DrivePassage& drive_passage,
    const PathTimeCorridor& path_time_corridor,
    absl::Span<const SpacetimeObjectTrajectory* const> spacetime_trajs,
    double min_mirror_height_avg, double max_mirror_height_avg,
    double vehicle_width, double vehicle_width_with_mirror,
    const TrajectoryOptimizerCostWeightParamsProto& cost_weight_params,
    const VehicleCircleModelParamsProto&
        trajectory_optimizer_vehicle_model_params,
    std::vector<double>* static_object_nudge_buffers,
    std::vector<char>* aggregate_objects_consider_mirror,
    std::vector<std::unique_ptr<Cost<Mfob>>>* costs) {
  std::vector<Segment2d> segments;
  std::vector<double> segments_buffers;
  std::vector<Segment2d> segments_consider_mirrors;
  std::vector<double> segments_consider_mirrors_buffers;
  static_object_nudge_buffers->reserve(spacetime_trajs.size());
  aggregate_objects_consider_mirror->reserve(spacetime_trajs.size());

  const auto& object_cost_params =
      cost_weight_params.aggregate_object_cost_params();

  constexpr int kEstimateLineCountsPerObject = 6;
  QCHECK_GT(trajectory_time_step, 0.0);
  const int free_index = static_cast<int>(
      (kTrajectorySteps - 1) * kTrajectoryTimeStep / trajectory_time_step);
  const int estimate_size =
      kEstimateLineCountsPerObject * spacetime_trajs.size();
  segments.reserve(estimate_size);
  segments_buffers.reserve(estimate_size);
  segments_consider_mirrors.reserve(estimate_size);
  segments_consider_mirrors_buffers.reserve(estimate_size);
  for (int idx = 0; idx < spacetime_trajs.size(); ++idx) {
    const auto& traj = *spacetime_trajs[idx];
    const auto& object_segments = traj.contour().line_segments();
    if (IsConsiderMirrorObject(traj.planner_object().object_proto(),
                               min_mirror_height_avg, max_mirror_height_avg)) {
      const double nudge_buffer = GenerateNudgeBufferForStatic(
          plan_start_point.v(), traj.contour(), drive_passage,
          path_time_corridor, vehicle_width_with_mirror, object_cost_params);
      segments_consider_mirrors_buffers.insert(
          segments_consider_mirrors_buffers.end(), object_segments.size(),
          nudge_buffer);
      segments_consider_mirrors.insert(segments_consider_mirrors.end(),
                                       object_segments.begin(),
                                       object_segments.end());
      static_object_nudge_buffers->push_back(nudge_buffer);
      aggregate_objects_consider_mirror->push_back(true);
    } else {
      const double nudge_buffer = GenerateNudgeBufferForStatic(
          plan_start_point.v(), traj.contour(), drive_passage,
          path_time_corridor, vehicle_width, object_cost_params);
      segments_buffers.insert(segments_buffers.end(), object_segments.size(),
                              nudge_buffer);
      segments.insert(segments.end(), object_segments.begin(),
                      object_segments.end());
      static_object_nudge_buffers->push_back(nudge_buffer);
      aggregate_objects_consider_mirror->push_back(false);
    }
  }
  if (segments.empty() && segments_consider_mirrors.empty()) {
    return true;
  }

  std::vector<double> gains = {object_cost_params.object_a_cost_weight(),
                               object_cost_params.object_b_cost_weight()};
  std::vector<std::string> sub_names = {"a", "b"};
  std::vector<double> dist_to_rac, angle_to_axis;
  std::vector<std::vector<double>> circle_buffers;
  const auto& circles = trajectory_optimizer_vehicle_model_params.circles();
  dist_to_rac.reserve(circles.size());
  angle_to_axis.reserve(circles.size());
  circle_buffers.reserve(circles.size());
  for (const auto& circle : circles) {
    dist_to_rac.push_back(circle.dist_to_rac());
    angle_to_axis.push_back(circle.angle_to_axis());
    circle_buffers.push_back({circle.radius(), circle.radius()});
  }
  if (!segments.empty()) {
    std::vector<std::vector<std::vector<double>>> buffers;
    buffers.reserve(segments.size());
    for (int i = 0; i < segments.size(); ++i) {
      buffers.push_back(circle_buffers);
      auto& buffers_back = buffers.back();
      for (int j = 0; j < buffers_back.size(); ++j) {
        buffers_back[j].front() += segments_buffers[i];
        buffers_back[j].back() += segments_buffers[i] * 0.5;
      }
    }
    costs->push_back(std::make_unique<AggregateStaticObjectCost<Mfob>>(
        segments, dist_to_rac, angle_to_axis, buffers, gains, sub_names,
        free_index,
        /*using_hessian_approximate=*/
        true, absl::StrFormat("Aggregate Static Object"),
        cost_weight_params.object_cost_weight(),
        /*cost_type=*/Cost<Mfob>::CostType::GROUP_OBJECT));
  }
  if (!segments_consider_mirrors.empty()) {
    const auto& mirror_circles =
        trajectory_optimizer_vehicle_model_params.mirror_circles();
    for (const auto& circle : mirror_circles) {
      dist_to_rac.push_back(circle.dist_to_rac());
      angle_to_axis.push_back(circle.angle_to_axis());
      circle_buffers.push_back({circle.radius(), circle.radius()});
    }
    std::vector<std::vector<std::vector<double>>> buffers;
    buffers.reserve(segments_consider_mirrors.size());
    for (int i = 0; i < segments_consider_mirrors.size(); ++i) {
      buffers.push_back(circle_buffers);
      auto& buffers_back = buffers.back();
      for (int j = 0; j < buffers_back.size(); ++j) {
        buffers_back[j].front() += segments_consider_mirrors_buffers[i];
        buffers_back[j].back() += segments_consider_mirrors_buffers[i] * 0.5;
      }
    }
    costs->push_back(std::make_unique<AggregateStaticObjectCost<Mfob>>(
        segments_consider_mirrors, std::move(dist_to_rac),
        std::move(angle_to_axis), std::move(buffers), std::move(gains),
        std::move(sub_names), free_index,
        /*using_hessian_approximate=*/
        true, absl::StrFormat("Aggregate Static Object Consider Mirrors"),
        cost_weight_params.object_cost_weight(),
        /*cost_type=*/Cost<Mfob>::CostType::GROUP_OBJECT));
  }
  return true;
}

void GetCorridorWithObjectFirstState(
    double nudge_buffer, double ttc, const DrivePassage& drive_passage,
    const PathSlBoundary& path_boundary,
    const FrenetFrame& init_traj_frenet_frame, double init_end_s,
    bool is_enforce_expand_corridor, const TrajectoryPoint& plan_start_point,
    const SecondOrderTrajectoryPoint& object_pose,
    const Polygon2d& object_contour, double /*front_edge_to_center*/,
    double vehicle_width, std::vector<double>* left_l_boundary_for_nudge,
    std::vector<double>* right_l_boundary_for_nudge, char* is_dist_update) {
  QCHECK_NOTNULL(left_l_boundary_for_nudge);
  QCHECK_NOTNULL(right_l_boundary_for_nudge);
  QCHECK_EQ(left_l_boundary_for_nudge->size(), path_boundary.size());
  QCHECK_EQ(right_l_boundary_for_nudge->size(), path_boundary.size());
  bool expand_corridor = false;
  if (is_enforce_expand_corridor) {
    expand_corridor = true;
  } else {
    const auto frenet_box =
        drive_passage.QueryFrenetBoxAtContour(object_contour);
    if (frenet_box.ok()) {
      constexpr double kMeetTimeTheThreshold = 4.0;  // s
      if (ttc < kMeetTimeTheThreshold) {
        expand_corridor = true;
      }
    }
  }
  if (expand_corridor) {
    bool contour_out_boundary = true;
    const auto& contour_points = object_contour.points();
    constexpr double kSafeBuffer = 0.5;
    for (const auto& pt : contour_points) {
      const auto frenet_pt = drive_passage.QueryFrenetCoordinateAt(pt);
      if (!frenet_pt.ok()) return;
      const auto boundary_l = path_boundary.QueryBoundaryL(frenet_pt->s);
      if (frenet_pt->l > (boundary_l.first - kSafeBuffer) &&
          frenet_pt->l < (boundary_l.second + kSafeBuffer)) {
        contour_out_boundary = false;
        break;
      }
    }
    if (contour_out_boundary) return;

    const auto object_frenet_box =
        drive_passage.QueryFrenetBoxAtContour(object_contour);
    if (!object_frenet_box.ok()) return;
    const double center_l_at_object =
        path_boundary.QueryReferenceCenterL(object_frenet_box->center_s());

    bool left;
    if (object_frenet_box->s_min > init_end_s) {
      left = object_frenet_box->center_l() > center_l_at_object;
    } else {
      const FrenetCoordinate frenet_center =
          init_traj_frenet_frame.XYToSL(object_pose.pos());
      left = frenet_center.l > 0.0;
    }

    std::vector<double>* object_dists =
        left ? right_l_boundary_for_nudge : left_l_boundary_for_nudge;
    *is_dist_update = true;
    const double object_l_boundary =
        left ? object_frenet_box->l_min - nudge_buffer - vehicle_width
             : object_frenet_box->l_max + nudge_buffer + vehicle_width;
    const auto& reference_center_l_vector =
        path_boundary.reference_center_l_vector();

    const auto obj_min_dist_dp_station_pt_index =
        drive_passage.FindNearestStationIndex(object_pose.pos());
    const auto& obj_min_dist_dp_station_pt =
        drive_passage.station(obj_min_dist_dp_station_pt_index);
    const Vec2d& min_dist_dp_station_pt_theta_tangent =
        obj_min_dist_dp_station_pt.tangent();

    Vec2d front, back;
    object_contour.ExtremePoints(min_dist_dp_station_pt_theta_tangent, &back,
                                 &front);
    const double contour_length =
        (front - back).dot(min_dist_dp_station_pt_theta_tangent);

    constexpr double kGainSRangeBase = 5.0;
    constexpr double kGainSRangeCoeff = 3.0;
    const double gain_s = kGainSRangeBase + contour_length * 0.5;
    const double vel_s_offset = std::max(
        kGainSRangeCoeff * plan_start_point.v(),
        object_pose.v() *
            fast_math::Cos(min_dist_dp_station_pt_theta_tangent.Angle() -
                           object_pose.theta()) *
            kGainSRangeCoeff);
    double base_s = obj_min_dist_dp_station_pt.accumulated_s();
    if (!is_enforce_expand_corridor) {
      base_s = object_frenet_box->center_s() + ttc * object_pose.v();
    }
    const double s_max = base_s + gain_s + vel_s_offset;
    const double s_min = base_s - gain_s - vel_s_offset;

    constexpr double kMinGain = 0.005;

    const auto& path_boundary_s_vector = path_boundary.s_vector();
    int mid_path_boundary_index = obj_min_dist_dp_station_pt_index.value();
    for (int i = mid_path_boundary_index;
         i < path_boundary.size() && path_boundary_s_vector[i] < s_max; ++i) {
      const double factor = (s_max - path_boundary_s_vector[i]) / gain_s;
      const double l = object_l_boundary +
                       (reference_center_l_vector[i] - object_l_boundary) *
                           std::pow(kMinGain, factor);
      (*object_dists)[i] = left ? std::min(l, (*object_dists)[i])
                                : std::max(l, (*object_dists)[i]);
    }
    for (int i = std::min(mid_path_boundary_index,
                          static_cast<int>(path_boundary.size()) - 1);
         i >= 0 && path_boundary_s_vector[i] > s_min; --i) {
      const double factor = (path_boundary_s_vector[i] - s_min) / gain_s;
      const double l = object_l_boundary +
                       (reference_center_l_vector[i] - object_l_boundary) *
                           std::pow(kMinGain, factor);
      (*object_dists)[i] = left ? std::min(l, (*object_dists)[i])
                                : std::max(l, (*object_dists)[i]);
    }
  }
}

bool IsObjectOnStraightWay(const DrivePassage& drive_passage, double s,
                           double object_v, double object_theta,
                           double lane_theta) {
  constexpr double kLookAheadTime = 3.0;             // s
  constexpr double kStraightKappaThreshold = 0.005;  // m^-1;
  const double look_ahead_s =
      kLookAheadTime * object_v * fast_math::Cos(object_theta - lane_theta);
  const auto& station_preview =
      drive_passage.FindNearestStationAtS(s + look_ahead_s);
  const double theta_preview = station_preview.tangent().FastAngle();
  const double kappa_fd =
      std::abs(NormalizeAngle(theta_preview - lane_theta) / look_ahead_s);
  return kappa_fd < kStraightKappaThreshold;
}

bool CanUseUndirectionedObjectCost(
    const FrenetCoordinate& av_frenet_pt, const SpacetimeObjectTrajectory& traj,
    const SpacetimePlannerObjectTrajectories::TrajectoryInfo& traj_info,
    const PathTimeCorridor& path_time_corridor,
    const DrivePassage& drive_passage) {
  if (traj_info.reason == SpacetimePlannerObjectTrajectoryReason::DRIVE_IN) {
    return true;
  }
  if (traj.object_type() == ObjectType::OT_LARGE_VEHICLE ||
      traj.object_type() == ObjectType::OT_VEHICLE) {
    const auto frenet_box =
        drive_passage.QueryFrenetBoxAtContour(traj.contour());
    if (!frenet_box.ok()) return false;
    const auto boundary = path_time_corridor.QueryBoundaryL(
        frenet_box->s_min, frenet_box->s_max, /*t=*/0.0,
        /*min_lane_width_idx=*/nullptr);
    constexpr double kBoundaryBuffer = 0.5;  // m
    const auto& right_boundary = *boundary.first;
    const auto& left_boundary = *boundary.second;
    const bool is_object_on_right =
        std::isfinite(right_boundary.l_boundary) &&
        frenet_box->l_max < (right_boundary.l_boundary + kBoundaryBuffer) &&
        frenet_box->l_max > (right_boundary.l_boundary - kDefaultLaneWidth);
    const bool is_object_on_left =
        std::isfinite(left_boundary.l_boundary) &&
        frenet_box->l_min > (left_boundary.l_boundary - kBoundaryBuffer) &&
        frenet_box->l_min < (left_boundary.l_boundary + kDefaultLaneWidth);
    if (is_object_on_right || is_object_on_left) {
      // Don't overlap on l with av.
      if ((av_frenet_pt.l < right_boundary.l_boundary && is_object_on_right) ||
          (av_frenet_pt.l > left_boundary.l_boundary && is_object_on_left)) {
        return false;
      }
      constexpr double kThetaLaneThreshold = 0.02;  // rad
      const auto theta =
          drive_passage.QueryTangentAngleAtS(frenet_box->center_s());
      const auto speed_limit =
          drive_passage.QuerySpeedLimitAtS(frenet_box->center_s());
      constexpr double kVelocityThreshold = 15.0;  // m/s^2
      if (!theta.ok()) return false;
      if (!speed_limit.ok()) return false;
      if (*speed_limit > kVelocityThreshold &&
          std::abs(NormalizeAngle(traj.pose().theta() - *theta)) <
              kThetaLaneThreshold &&
          IsObjectOnStraightWay(drive_passage, frenet_box->center_s(),
                                traj.pose().v(), traj.pose().theta(), *theta)) {
        return true;
      }
    }
  }
  return false;
}

const UndirectionalObjectCostParams& GetUndirectionalObjectCostParams(
    SpacetimePlannerObjectTrajectoryReason::Type reason, ObjectType type,
    const Box2d& object_box,
    const TrajectoryOptimizerCostWeightParamsProto& cost_weight_params) {
  constexpr double kLargeVehicleNudgeLengthThreshold = 7.5;  // m
  if (reason == SpacetimePlannerObjectTrajectoryReason::DRIVE_IN) {
    return cost_weight_params.drive_in_object_cost_params();
  } else if (type == ObjectType::OT_LARGE_VEHICLE &&
             object_box.length() > kLargeVehicleNudgeLengthThreshold) {
    return cost_weight_params.large_vehicle_object_cost_params();
  } else {
    return cost_weight_params.nudge_object_cost_params();
  }
}

}  // namespace

void AddObjectCosts(
    int trajectory_steps, double trajectory_time_step,
    std::string_view base_name, const std::vector<TrajectoryPoint>& init_traj,
    const DrivePassage& drive_passage, const PathSlBoundary& path_boundary,
    const PathTimeCorridor& path_time_corridor,
    const FrenetFrame& init_traj_frenet_frame,
    const std::map<std::string, ConstraintProto::LeadingObjectProto>&
        leading_trajs,
    const SpacetimePlannerObjectTrajectories& st_planner_object_traj,
    const TrajectoryOptimizerCostWeightParamsProto& cost_weight_params,
    const VehicleGeometryParamsProto& veh_geo_params,
    const MotionConstraintParamsProto& motion_constraint_params,
    const VehicleCircleModelParamsProto&
        trajectory_optimizer_vehicle_model_params,
    const std::unique_ptr<AvModelHelper<Mfob>>& av_model_helpers,
    std::vector<LeadingInfo>* leading_min_s,
    std::vector<double>* left_l_boundary_for_nudge,
    std::vector<double>* right_l_boundary_for_nudge,
    std::vector<std::unique_ptr<Cost<Mfob>>>* costs, ThreadPool* thread_pool) {
  FUNC_QTRACE();

  QCHECK_GT(trajectory_steps, 0);
  QCHECK_GT(trajectory_time_step, 0.0);
  const int free_index = static_cast<int>(
      (kTrajectorySteps - 1) * kTrajectoryTimeStep / trajectory_time_step);

  QCHECK_GE(init_traj.size(), trajectory_steps);
  // Use a map-reduce strategy to parallelize the obstacle cost collection.
  // const auto spacetime_trajs = st_traj_mgr.spacetime_planner_trajs();
  const auto& spacetime_trajs = st_planner_object_traj.trajectories;
  const int num_trajs = spacetime_trajs.size();
  using TrajectoryInfo = SpacetimePlannerObjectTrajectories::TrajectoryInfo;
  const auto& spacetime_traj_infos = st_planner_object_traj.trajectory_infos;

  // Group st_trajs with different object cost types.
  // Trajs added to aggregate object cost.
  std::vector<const SpacetimeObjectTrajectory*> static_spacetime_trajs;
  // Trajs used to generate partition and leading object costs.
  std::vector<const SpacetimeObjectTrajectory*> generic_spacetime_trajs;
  // Trajs used to generate partition and leading object costs.
  std::vector<const TrajectoryInfo*> generic_spacetime_traj_infos;

  static_spacetime_trajs.reserve(num_trajs);
  generic_spacetime_trajs.reserve(num_trajs);
  generic_spacetime_traj_infos.reserve(num_trajs);
  for (int idx = 0; idx < num_trajs; ++idx) {
    const SpacetimeObjectTrajectory& traj = spacetime_trajs[idx];
    if (IsStaticObjectType(traj.object_type()) &&
        leading_trajs.find(std::string(traj.traj_id())) ==
            leading_trajs.end()) {
      static_spacetime_trajs.push_back(&traj);
    } else {
      generic_spacetime_trajs.push_back(&traj);
      generic_spacetime_traj_infos.push_back(&spacetime_traj_infos[idx]);
    }
  }

  // Compute AV box.
  const Box2d av_box = ComputeAvBox(init_traj.front().pos(),
                                    init_traj.front().theta(), veh_geo_params);
  const auto av_frenet_box = drive_passage.QueryFrenetBoxAt(av_box);
  if (!av_frenet_box.ok()) return;

  // Compute AV min & max mirror average height.
  const auto min_max_mirror_average_height =
      ComputeMinMaxMirrorAverageHeight(veh_geo_params);
  // Note: Structured bindings cannot be captured by lambda expressions until
  // C++20.
  const double min_mirror_height_avg = min_max_mirror_average_height.first;
  const double max_mirror_height_avg = min_max_mirror_average_height.second;

  const double vehicle_width_with_mirror =
      veh_geo_params.left_mirror().y() - veh_geo_params.right_mirror().y() +
      0.5 * (veh_geo_params.right_mirror().length() +
             veh_geo_params.left_mirror().length());
  const double vehicle_width = veh_geo_params.width();

  const auto init_traj_end_frenet_pt =
      drive_passage.QueryUnboundedFrenetCoordinateAt(init_traj.back().pos());
  if (!init_traj_end_frenet_pt.ok()) return;
  const double init_end_s =
      init_traj_end_frenet_pt->s + veh_geo_params.front_edge_to_center();

  // Add aggregate static object cost first.
  std::vector<double> aggregate_object_nudge_buffers;
  std::vector<char> aggregate_objects_consider_mirror;
  AddAggregateStaticObjectCost(
      trajectory_time_step, init_traj.front(), drive_passage,
      path_time_corridor, static_spacetime_trajs, min_mirror_height_avg,
      max_mirror_height_avg, vehicle_width, vehicle_width_with_mirror,
      cost_weight_params, trajectory_optimizer_vehicle_model_params,
      &aggregate_object_nudge_buffers, &aggregate_objects_consider_mirror,
      costs);
  // Expand corridor for static object.
  for (int idx = 0; idx < static_spacetime_trajs.size(); ++idx) {
    const auto& traj = static_spacetime_trajs[idx];
    char is_dist_update = false;
    GetCorridorWithObjectFirstState(
        aggregate_object_nudge_buffers[idx], /*ttc=*/0.0, drive_passage,
        path_boundary, init_traj_frenet_frame, init_end_s,
        /*is_enforce_expand_corridor=*/true, init_traj.front(), traj->pose(),
        traj->contour(), veh_geo_params.front_edge_to_center(),
        aggregate_objects_consider_mirror[idx] ? vehicle_width_with_mirror
                                               : vehicle_width,
        left_l_boundary_for_nudge, right_l_boundary_for_nudge, &is_dist_update);
  }

  // Next, add partition and leading object costs.
  const int num_generic_spacetime_trajs = generic_spacetime_trajs.size();
  std::vector<std::vector<double>> left_l_boundary_for_nudge_all_trajs(
      num_generic_spacetime_trajs,
      std::vector<double>(left_l_boundary_for_nudge->size(), 0.0));
  std::vector<std::vector<double>> right_l_boundary_for_nudge_all_trajs(
      num_generic_spacetime_trajs,
      std::vector<double>(right_l_boundary_for_nudge->size(), 0.0));
  std::vector<std::vector<std::unique_ptr<Cost<Mfob>>>> costs_all_trajs(
      num_generic_spacetime_trajs);
  std::vector<char> is_dists_update(num_generic_spacetime_trajs, false);
  std::vector<std::vector<LeadingInfo>> leading_min_s_all_trajs(
      num_generic_spacetime_trajs, *leading_min_s);

  // Get Clostest leading object min_s, we will use the value to filter
  // objects whose prediction traj point s all larger than min_s.
  std::optional<
      std::pair<std::vector<FrenetBox>, const SpacetimeObjectTrajectory*>>
      stationary_closest_leading_object_info,
      moving_closest_leading_object_info;
  GetClosestLeadingObjectInfo(trajectory_steps, trajectory_time_step,
                              drive_passage, leading_trajs,
                              generic_spacetime_trajs, cost_weight_params,
                              &stationary_closest_leading_object_info,
                              &moving_closest_leading_object_info);

  const auto av_frenet_pt =
      drive_passage.QueryFrenetCoordinateAt(init_traj.front().pos());
  if (!av_frenet_pt.ok()) return;
  // Don't apply parallelism in debugging mode as canvas updating is not
  // thread safe.
  ThreadPool* used_tp =
      FLAGS_traj_opt_draw_object_canvas ? nullptr : thread_pool;
  ParallelFor(0, num_generic_spacetime_trajs, used_tp, [&](int i) {
    const auto& traj = *(generic_spacetime_trajs[i]);
    const auto& traj_info = *(generic_spacetime_traj_infos[i]);
    const auto states = SampleObjectStates(trajectory_steps,
                                           trajectory_time_step, traj.states());
    if (!IgnoreObjectCost(base_name, traj, states, drive_passage,
                          cost_weight_params,
                          stationary_closest_leading_object_info,
                          moving_closest_leading_object_info)) {
      const bool is_leading_object =
          leading_trajs.find(std::string(traj.traj_id())) !=
          leading_trajs.end();
      if (is_leading_object) {
        if (!cost_weight_params.object_cost_params().ignore_leading()) {
          AddUnidirectionalObjectCostForLeading(
              trajectory_time_step, base_name, states, init_traj, drive_passage,
              path_boundary, leading_trajs, cost_weight_params, veh_geo_params,
              motion_constraint_params, traj.traj_id(),
              traj.trajectory().probability(), &leading_min_s_all_trajs[i],
              &costs_all_trajs[i]);
        }
      } else {
        const auto frenet_box_or =
            drive_passage.QueryFrenetBoxAtContour(traj.contour());
        if (!frenet_box_or.ok()) return;

        const auto object_nudge_buffer_info = ComputeObjectNudgeBufferInfo(
            init_traj.front(), *states.front().traj_point, drive_passage,
            *av_frenet_box, *frenet_box_or);
        if (!object_nudge_buffer_info.ok()) return;
        const bool consider_mirrors = IsConsiderMirrorObject(
            traj.planner_object().object_proto(), min_mirror_height_avg,
            max_mirror_height_avg);
        const bool can_use_unidirectioned_nudge_cost =
            CanUseUndirectionedObjectCost(*av_frenet_pt, traj, traj_info,
                                          path_time_corridor, drive_passage);
        double nudge_buffer = 0.0;
        std::optional<double> first_nudge_buffer;
        const double vehicle_width_used =
            consider_mirrors ? vehicle_width_with_mirror : vehicle_width;
        if (can_use_unidirectioned_nudge_cost) {
          const auto& undirectional_object_cost_params =
              GetUndirectionalObjectCostParams(
                  traj_info.reason, traj.object_type(), traj.bounding_box(),
                  cost_weight_params);
          AddUnidirectionalObjectCost(
              trajectory_time_step, traj.planner_object().velocity(), base_name,
              vehicle_width_used, veh_geo_params.front_edge_to_center(),
              *object_nudge_buffer_info, *av_frenet_box, init_traj.front(),
              states, path_time_corridor, drive_passage, path_boundary,
              undirectional_object_cost_params, traj.traj_id(),
              traj.trajectory().probability() *
                  cost_weight_params.object_cost_weight(),
              &first_nudge_buffer, &costs_all_trajs[i]);

        } else {
          const double nudge_buffer_a =
              IsCameraObject(traj.planner_object().object_proto()) &&
                      !traj.planner_object().is_large_vehicle()
                  ? 0.85
                  : 1.0;
          AddPartitionAvObjectCost(
              trajectory_time_step, base_name, nudge_buffer_a,
              *object_nudge_buffer_info, vehicle_width_used, consider_mirrors,
              traj.planner_object().velocity(), *av_frenet_box, states,
              init_traj, traj.is_stationary(), drive_passage,
              path_time_corridor, cost_weight_params,
              trajectory_optimizer_vehicle_model_params, av_model_helpers,
              traj.traj_id(), traj.trajectory().probability(),
              &first_nudge_buffer, &costs_all_trajs[i]);
        }
        if (first_nudge_buffer.has_value()) {
          GetCorridorWithObjectFirstState(
              nudge_buffer, object_nudge_buffer_info->ttc, drive_passage,
              path_boundary, init_traj_frenet_frame, init_end_s,
              traj.is_stationary(), init_traj.front(), traj.pose(),
              traj.contour(), veh_geo_params.front_edge_to_center(),
              vehicle_width_used, &left_l_boundary_for_nudge_all_trajs[i],
              &right_l_boundary_for_nudge_all_trajs[i], &is_dists_update[i]);
        }
      }
    }
  });

  // Collect results from each trajectory.
  for (int idx = 0; idx < num_generic_spacetime_trajs; ++idx) {
    const auto& leading_min_s_per_traj = leading_min_s_all_trajs[idx];
    for (int i = 0; i <= free_index; ++i) {
      if (leading_min_s_per_traj[i].s < (*leading_min_s)[i].s) {
        (*leading_min_s)[i] = leading_min_s_per_traj[i];
      }
    }
    if (is_dists_update[idx]) {
      const auto& left_dists_per_traj =
          left_l_boundary_for_nudge_all_trajs[idx];
      const auto& right_dists_per_traj =
          right_l_boundary_for_nudge_all_trajs[idx];
      QCHECK_EQ(left_dists_per_traj.size(), right_dists_per_traj.size());
      for (int i = 0; i < left_dists_per_traj.size(); ++i) {
        (*left_l_boundary_for_nudge)[i] =
            std::max((*left_l_boundary_for_nudge)[i], left_dists_per_traj[i]);
        (*right_l_boundary_for_nudge)[i] =
            std::min((*right_l_boundary_for_nudge)[i], right_dists_per_traj[i]);
      }
    }
  }
  for (auto& costs_per_traj : costs_all_trajs) {
    std::move(costs_per_traj.begin(), costs_per_traj.end(),
              std::back_inserter(*costs));
  }
}

void CalcPartitionHalfContourInfo(const Vec2d& x, const Vec2d& obj_x,
                                  const Polygon2d& contour, double buffer,
                                  std::vector<Segment2d>* lines, Vec2d* ref_x,
                                  Vec2d* ref_tangent, double* offset) {
  const Vec2d force_dir = (x - obj_x).normalized();
  const Vec2d force_right = -force_dir.Perp();

  Vec2d left, right, front, back;
  int left_index, right_index, front_index, back_index;
  contour.ExtremePoints(force_dir, &back_index, &front_index, &back, &front);
  contour.ExtremePoints(force_right, &left_index, &right_index, &left, &right);

  const auto& contour_lines = contour.line_segments();
  lines->reserve(contour_lines.size());

  // Insert right border
  constexpr double kBorderExtent = 2.0;
  const Segment2d& right_line = contour_lines[right_index];
  lines->emplace_back(right_line.start() - force_dir * kBorderExtent,
                      right_line.start());
  if (front_index >= right_index && front_index <= left_index) {
    lines->insert(lines->end(), contour_lines.begin() + right_index,
                  contour_lines.begin() + left_index);
  } else {
    lines->insert(lines->end(), contour_lines.begin() + right_index,
                  contour_lines.end());
    if (left_index != 0) {
      lines->insert(lines->end(), contour_lines.begin(),
                    contour_lines.begin() + left_index);
    }
  }
  // Insert left border
  const Segment2d& left_line =
      contour_lines[left_index == 0 ? (contour_lines.size() - 1)
                                    : (left_index - 1)];
  lines->emplace_back(left_line.end(),
                      left_line.end() - force_dir * kBorderExtent);

  // Fill filter variables
  *ref_x = Vec2d((left.x() + right.x()) * 0.5, (front.y() + back.y()) * 0.5);
  *ref_tangent = force_dir;
  *offset = (front - (*ref_x)).dot(force_dir) + buffer;
}

std::vector<SpacetimeObjectState> SampleObjectStates(
    int trajectory_steps, double trajectory_time_step,
    absl::Span<const SpacetimeObjectState> states) {
  QCHECK_GT(trajectory_steps, 0);
  QCHECK_GT(trajectory_time_step, 0.0);

  const int sample_step =
      static_cast<int>(trajectory_time_step / kTrajectoryTimeStep + 0.5);

  // TODO(fengzhuang): When modify DDP horizon to 15s, this check should be
  // removed.
  std::vector<SpacetimeObjectState> sampled_states;
  sampled_states.reserve(states.size() / sample_step);
  for (int i = 0; i < trajectory_steps; ++i) {
    if (i * sample_step >= states.size()) break;
    sampled_states.push_back(states[i * sample_step]);
  }
  return sampled_states;
}

}  // namespace optimizer
}  // namespace planner
}  // namespace qcraft
