#include "onboard/planner/speed/speed_finder_util.h"

#include <algorithm>
#include <limits>
#include <map>
#include <ostream>
#include <utility>

#include "absl/strings/str_cat.h"

#include "onboard/eval/qevent.h"
#include "onboard/eval/qevent_base.h"
#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/lite/logging.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/math/geometry/proto/aabox3d.pb.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/speed/proto/speed_finder_params.pb.h"
#include "onboard/planner/speed/speed_limit.h"
#include "onboard/planner/speed/speed_point.h"
#include "onboard/planner/speed/st_boundary.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/utils/map_util.h"

namespace qcraft {
namespace planner {

void SetStBoundaryDebugInfo(
    absl::Span<const StBoundaryWithDecision> st_boundaries_with_decision,
    SpeedFinderDebugProto* speed_finder_proto) {
  QCHECK_NOTNULL(speed_finder_proto);
  speed_finder_proto->mutable_st_boundaries()->clear();
  for (const auto& boundary_with_decision : st_boundaries_with_decision) {
    StBoundaryProto st_boundary_proto;
    const auto* st_boundary = boundary_with_decision.st_boundary();
    st_boundary_proto.set_decision_type(boundary_with_decision.decision_type());
    st_boundary_proto.set_decision_reason(
        boundary_with_decision.decision_reason());
    st_boundary_proto.set_decision_info(
        std::string(boundary_with_decision.decision_info()));
    st_boundary_proto.set_follow_standstill_distance(
        boundary_with_decision.follow_standstill_distance());
    st_boundary_proto.set_lead_standstill_distance(
        boundary_with_decision.lead_standstill_distance());
    st_boundary_proto.set_object_type(st_boundary->object_type());
    st_boundary_proto.set_probability(st_boundary->probability());
    st_boundary_proto.set_min_s(st_boundary->min_s());
    st_boundary_proto.set_max_s(st_boundary->max_s());
    st_boundary_proto.set_min_t(st_boundary->min_t());
    st_boundary_proto.set_max_t(st_boundary->max_t());
    *st_boundary_proto.mutable_decision_prob() =
        boundary_with_decision.decision_prob();
    st_boundary_proto.set_is_stationary(st_boundary->is_stationary());

    speed_finder_proto->mutable_st_boundaries()->insert(
        {st_boundary->id(), st_boundary_proto});
  }
}

int GetSpeedFinderTrajectorySteps(double init_v, int max_traj_steps) {
  constexpr double kMaxSpeedOfFixedTrajectorySteps = 22.23;    // m/s.->80km/h
  constexpr double kMaxSpeedOfDynamicTrajectorySteps = 33.33;  // m/s.->120km/h
  QCHECK_GE(max_traj_steps, kTrajectorySteps);
  const PiecewiseLinearFunction<int> plf = {
      {kMaxSpeedOfFixedTrajectorySteps, kMaxSpeedOfDynamicTrajectorySteps},
      {kTrajectorySteps, max_traj_steps}};
  return plf(init_v);
}

void PostProcessSpeedByFullStop(
    const SpeedFinderParamsProto& speed_finder_params,
    SpeedVector* speed_data) {
  QCHECK_NOTNULL(speed_data);
  // TODO(renjie): Tune the full stop threshold down after control
  // performance improves.
  bool force_brake = true;
  int i = 0;
  while ((*speed_data)[i].t() <
         speed_finder_params.full_stop_traj_time_threshold()) {
    if ((*speed_data)[i].v() >
        speed_finder_params.full_stop_speed_threshold()) {
      force_brake = false;
      break;
    }
    ++i;
  }
  if (speed_data->TotalLength() >=
      speed_finder_params.full_stop_traj_length_threshold()) {
    force_brake = false;
  }
  if (force_brake) {
    for (int i = 0; i < speed_data->size(); ++i) {
      (*speed_data)[i].set_s(0.0);
      (*speed_data)[i].set_v(0.0);
      (*speed_data)[i].set_a(0.0);
      (*speed_data)[i].set_j(0.0);
    }
  }
}

std::vector<VehicleShapeBasePtr> BuildAvShapes(
    const VehicleGeometryParamsProto& vehicle_geom,
    const DiscretizedPath& path_points) {
  const double half_length = vehicle_geom.length() * 0.5;
  const double half_width = vehicle_geom.width() * 0.5;
  const double rac_to_center = half_length - vehicle_geom.back_edge_to_center();
  std::vector<VehicleShapeBasePtr> av_shapes;
  const int num_points = path_points.size();
  av_shapes.reserve(num_points);
  for (int i = 0; i < num_points; ++i) {
    const auto& pt = path_points[i];
    const double theta = pt.theta();
    const Vec2d rac(pt.x(), pt.y());
    const Vec2d tangent = Vec2d::FastUnitFromAngle(theta);
    const Vec2d center = rac + tangent * rac_to_center;
    av_shapes.push_back(std::make_unique<VehicleBoxShape>(
        vehicle_geom, rac, center, tangent, theta, half_length, half_width));
  }
  return av_shapes;
}

std::unique_ptr<SegmentMatcherKdtree> BuildPathKdTree(
    const DiscretizedPath& path_points) {
  std::vector<Vec2d> points;
  points.reserve(path_points.size());
  for (const auto& point : path_points) {
    points.emplace_back(point.x(), point.y());
  }
  return std::make_unique<SegmentMatcherKdtree>(points);
}

std::optional<PathApprox> BuildPathApproxForMirrors(
    const PathApprox& path_approx,
    const VehicleGeometryParamsProto& vehicle_geom) {
  if (!vehicle_geom.has_left_mirror() || !vehicle_geom.has_right_mirror()) {
    return std::nullopt;
  }
  // left_mirror().y() is positive.
  const auto& mirror = vehicle_geom.left_mirror();
  if (!mirror.has_width() || !mirror.has_length()) {
    return std::nullopt;
  }
  std::vector<PathSegment> mirror_segments;
  mirror_segments.reserve(path_approx.segments().size());
  const double mirrors_center_offset = 0.5 * vehicle_geom.length() -
                                       vehicle_geom.front_edge_to_center() +
                                       mirror.x();
  for (const auto& segment : path_approx.segments()) {
    const Vec2d mirrors_center =
        segment.center() + segment.tangent() * mirrors_center_offset;
    const double mirror_box_length =
        std::max(segment.length() - vehicle_geom.length(), 0.0) +
        mirror.width();
    Box2d path_segment_box(0.5 * mirror_box_length,
                           0.5 * mirror.length() + mirror.y(), mirrors_center,
                           segment.heading(), segment.tangent());
    mirror_segments.emplace_back(segment.first_index(), segment.last_index(),
                                 segment.first_ra(), segment.last_ra(),
                                 segment.first_s(), segment.last_s(),
                                 std::move(path_segment_box));
  }
  return PathApprox(std::move(mirror_segments), path_approx.path_kd_tree());
}

std::vector<PartialSpacetimeObjectTrajectory> GetConsideredStObjects(
    const std::vector<StBoundaryWithDecision>& st_boundaries_with_decision,
    const SpacetimeTrajectoryManager& obj_mgr,
    std::unordered_map<std::string, SpacetimeObjectTrajectory>
        processed_st_objects) {
  FUNC_QTRACE();
  std::unordered_map<std::string, PartialSpacetimeObjectTrajectory>
      considered_st_objects_map;
  considered_st_objects_map.reserve(st_boundaries_with_decision.size());
  for (const auto& st_boundary_with_decision : st_boundaries_with_decision) {
    const auto decision_type = st_boundary_with_decision.decision_type();
    if (decision_type == StBoundaryProto::IGNORE ||
        decision_type == StBoundaryProto::UNKNOWN ||
        st_boundary_with_decision.st_boundary()->source_type() !=
            StBoundarySourceTypeProto::ST_OBJECT) {
      continue;
    }
    const auto& traj_id = st_boundary_with_decision.traj_id();
    QCHECK(traj_id.has_value());
    const auto* raw_st_boundary = st_boundary_with_decision.raw_st_boundary();
    if (considered_st_objects_map.find(*traj_id) ==
        considered_st_objects_map.end()) {
      // Judge whether the spacetime_object corresponding to current st-boundary
      // has been processed.
      if (processed_st_objects.find(*traj_id) != processed_st_objects.end()) {
        // Use st_object from processed_st_objects.
        considered_st_objects_map.emplace(
            *traj_id, PartialSpacetimeObjectTrajectory(std::move(
                          FindOrDie(processed_st_objects, *traj_id))));
      } else {
        // Use st_object from obj_mgr.
        considered_st_objects_map.emplace(
            *traj_id, PartialSpacetimeObjectTrajectory(*QCHECK_NOTNULL(
                          obj_mgr.FindTrajectoryById(*traj_id))));
      }
    }
    QCHECK(decision_type == StBoundaryProto::LEAD ||
           decision_type == StBoundaryProto::FOLLOW);
    FindOrDie(considered_st_objects_map, *traj_id)
        .AppendTimeRangeAndDecisonType(
            raw_st_boundary->min_t(), raw_st_boundary->max_t(),
            decision_type == StBoundaryProto::LEAD
                ? PartialSpacetimeObjectTrajectory::DecisionType::LEAD
                : PartialSpacetimeObjectTrajectory::DecisionType::FOLLOW);
  }
  std::vector<PartialSpacetimeObjectTrajectory> considered_st_objects;
  considered_st_objects.reserve(considered_st_objects_map.size());
  for (auto& [_, st_obj] : considered_st_objects_map) {
    considered_st_objects.push_back(std::move(st_obj));
  }
  return considered_st_objects;
}

void CutoffSpeedByTimeHorizon(SpeedVector* speed_data) {
  QCHECK_NOTNULL(speed_data);
  constexpr double kTimeHorizon = kTrajectoryTimeStep * kTrajectorySteps;
  speed_data->erase(
      std::lower_bound(speed_data->begin(), speed_data->end(), kTimeHorizon,
                       [](const auto& pt, double t) { return pt.t() < t; }),
      speed_data->end());
}

// If the st-traj has been modified, the original one will also be inserted in
// the map.
std::unordered_map<std::string, const SpacetimeObjectTrajectory*>
GetAllOverlappedStObjectTrajs(
    const std::unordered_map<std::string, double>& considered_trajs,
    const std::unordered_map<std::string, SpacetimeObjectTrajectory>&
        processed_st_objects,
    const SpacetimeTrajectoryManager& traj_mgr) {
  FUNC_QTRACE();
  std::unordered_map<std::string, const SpacetimeObjectTrajectory*>
      st_trajs_map;
  for (const auto& [traj_id, _] : considered_trajs) {
    if (processed_st_objects.find(traj_id) != processed_st_objects.end()) {
      st_trajs_map.emplace(absl::StrCat(traj_id, "|m"),
                           &FindOrDie(processed_st_objects, traj_id));
      st_trajs_map.emplace(
          absl::StrCat(traj_id, "|raw"),
          QCHECK_NOTNULL(traj_mgr.FindTrajectoryById(traj_id)));
    } else {
      st_trajs_map.emplace(
          traj_id, QCHECK_NOTNULL(traj_mgr.FindTrajectoryById(traj_id)));
    }
  }
  return st_trajs_map;
}

SpeedVector GenerateReferenceSpeed(
    const std::vector<SpeedBoundWithInfo>& min_speed_limit, double init_v,
    double ref_speed_bias, double ref_speed_static_limit_bias, double max_accel,
    double max_decel, double total_time, double delta_t) {
  QCHECK_GT(delta_t, 0.0);
  constexpr double kMaxComfortAcc = 1.4;  // m/s^2.
  const double max_speed_limit = init_v + kMaxComfortAcc * total_time;
  const double init_v_ref = std::max(init_v, 0.0) + ref_speed_bias;
  std::vector<double> reference_speed;
  reference_speed.reserve(min_speed_limit.size());
  reference_speed.push_back(std::min(
      init_v_ref, min_speed_limit[0].bound + ref_speed_static_limit_bias));
  for (int i = 1; i < min_speed_limit.size(); ++i) {
    const double min_static_limit =
        min_speed_limit[i].bound + ref_speed_static_limit_bias;
    const double bound = std::min(min_static_limit, max_speed_limit);
    reference_speed.push_back(
        std::min(reference_speed.back() + max_accel * delta_t, bound));
  }
  for (int i = reference_speed.size() - 2; i >= 0; --i) {
    reference_speed[i] = std::min(reference_speed[i],
                                  reference_speed[i + 1] - max_decel * delta_t);
  }
  SpeedVector speed_points;
  speed_points.reserve(reference_speed.size());
  double s = 0.0;
  double t = 0.0;
  for (int i = 0; i < reference_speed.size() - 1; ++i) {
    speed_points.emplace_back(
        /*t=*/t, /*s=*/s,
        /*v=*/reference_speed[i],
        /*a=*/(reference_speed[i + 1] - reference_speed[i]) / delta_t,
        /*j=*/0.0);
    t += delta_t;
    s += (reference_speed[i + 1] + reference_speed[i]) * 0.5 * delta_t;
  }
  speed_points.emplace_back(/*t=*/t, /*s=*/s,
                            /*v=*/reference_speed.back(),
                            /*a=*/0.0,
                            /*j=*/0.0);
  return speed_points;
}

VtSpeedLimit GetVtSpeedLimitFromSpeedVector(
    const SpeedVector& preliminary_speed, int traj_steps, double time_step,
    double buffer) {
  VtSpeedLimit vt_speed_limit;
  for (double t = 0.0; t < traj_steps * kTrajectoryTimeStep; t += time_step) {
    double min_speed_upper_bound = std::numeric_limits<double>::max();
    std::string min_type;
    const auto speed_pt_or = preliminary_speed.EvaluateByTime(t);
    if (speed_pt_or.has_value()) {
      min_speed_upper_bound = speed_pt_or->v() + buffer;
      min_type = "preliminary speed";
    }
    vt_speed_limit.emplace_back(min_speed_upper_bound, std::move(min_type));
  }
  return vt_speed_limit;
}

const PiecewiseLinearFunction<double, double> kAvAndObjSpeedLerpFactorPlf = {
    {0.0, 6.0}, {0.4, 0.7}};

SpeedVector GeneratePredictedAvSpeed(const SpeedVector& preliminary_speed,
                                     double av_speed, double av_acc,
                                     double plan_total_time) {
  QCHECK(!preliminary_speed.empty());
  constexpr double kTimeStep = 1.0;  // s.
  const int num = CeilToInt(plan_total_time / kTimeStep) + 1;
  SpeedVector predicted_av_speed;
  predicted_av_speed.reserve(num);
  double s = 0.0, t = 0.0;
  for (int i = 0; i < num; ++i) {
    constexpr double kMaxPredictTime = 1.5;             // s.
    constexpr double kMaxAllowedSpeedDivergence = 3.0;  // m/s.
    const double raw_predicted_speed =
        av_speed + std::clamp(std::min(kMaxPredictTime, t) * av_acc,
                              -kMaxAllowedSpeedDivergence,
                              kMaxAllowedSpeedDivergence);
    const double lerp_factor = kAvAndObjSpeedLerpFactorPlf(t);
    const double preliminary_speed_at_t =
        preliminary_speed.EvaluateByTime(t)
            .value_or(preliminary_speed.back())
            .v();
    const double speed =
        Lerp(raw_predicted_speed, preliminary_speed_at_t, lerp_factor);
    if (i != 0) {
      auto& prev_speed_pt = predicted_av_speed.back();
      prev_speed_pt.set_a((speed - prev_speed_pt.v()) / kTimeStep);
      s += 0.5 * kTimeStep * (speed + prev_speed_pt.v());
    }
    predicted_av_speed.emplace_back(/*t=*/t, /*s=*/s, /*v=*/speed, /*a=*/0.0,
                                    /*j=*/0.0);
    t += kTimeStep;
  }
  return predicted_av_speed;
}

SpeedBoundMapType EstimateSpeedBound(
    const SpeedLimitProvider& speed_limit_provider,
    const SpeedVector& preliminary_speed, double init_v,
    double allowed_max_speed, int knot_num, double delta_t,
    std::string_view base_name) {
  QCHECK(!preliminary_speed.empty());
  const auto fill_speed_bound =
      [](const std::optional<SpeedLimit::SpeedLimitInfo>& speed_limit_info,
         SpeedBoundWithInfo* speed_bound, double allowed_max_speed) {
        QCHECK_NOTNULL(speed_bound);
        speed_bound->bound = speed_limit_info.has_value()
                                 ? speed_limit_info->speed_limit
                                 : allowed_max_speed;
        speed_bound->info =
            speed_limit_info.has_value() ? speed_limit_info->info : "";
      };

  SpeedBoundMapType speed_upper_bound_map;

  std::vector<double> estimated_s;
  estimated_s.reserve(knot_num);
  estimated_s.push_back(0.0);
  for (int i = 1; i < knot_num; ++i) {
    const double t = i * delta_t;
    const auto speed_point = preliminary_speed.EvaluateByTime(t);
    const double s =
        speed_point.has_value()
            ? speed_point->s()
            : (estimated_s.back() + preliminary_speed.back().v() * delta_t);
    estimated_s.push_back(s);
  }

  // Emplace default speed limit.
  speed_upper_bound_map.emplace(
      SpeedLimitTypeProto::DEFAULT,
      std::vector<SpeedBoundWithInfo>(
          knot_num,
          SpeedBoundWithInfo{.bound = allowed_max_speed, .info = "Default."}));
  // Estimate speed bound for static speed limit.
  for (const auto& [type, speed_limit] :
       speed_limit_provider.static_speed_limit_map()) {
    std::vector<SpeedBoundWithInfo> speed_bounds_with_info;
    speed_bounds_with_info.reserve(knot_num);
    for (int i = 0; i < knot_num; ++i) {
      const auto speed_limit_info =
          speed_limit.GetSpeedLimitInfoByS(estimated_s[i]);
      fill_speed_bound(speed_limit_info, &speed_bounds_with_info.emplace_back(),
                       allowed_max_speed);
    }
    speed_upper_bound_map.emplace(type, std::move(speed_bounds_with_info));
  }

  // Estimate speed bound for dynamic speed limit.
  constexpr double kCloseTrajHardBrakeThres = -1.0;
  std::vector<SpeedBoundWithInfo> speed_bounds_with_info;
  speed_bounds_with_info.reserve(knot_num);
  for (int i = 0; i < knot_num; ++i) {
    const double t = i * delta_t;
    const auto dynamic_speed_limit_info =
        speed_limit_provider.GetDynamicSpeedLimitInfoByTimeAndS(t,
                                                                estimated_s[i]);
    SpeedBoundWithInfo speed_bound;
    fill_speed_bound(dynamic_speed_limit_info, &speed_bound, allowed_max_speed);
    // Add hard brake qevent for close traj.
    const double decel = i == 0 ? 0.0 : (speed_bound.bound - init_v) / t;
    if (decel < kCloseTrajHardBrakeThres) {
      QEVENT_EVERY_N_SECONDS("yuhang", "close_traj_hard_brake", 1.0,
                             [&](QEvent* qevent) {
                               qevent->AddField("decel", decel)
                                   .AddField("time", t)
                                   .AddField("speed_limit", speed_bound.bound)
                                   .AddField("info: ", speed_bound.info)
                                   .AddField("base_name", base_name);
                             });
    }
    speed_bounds_with_info.push_back(std::move(speed_bound));
  }
  speed_upper_bound_map.emplace(SpeedLimitTypeProto::MOVING_CLOSE_TRAJ,
                                std::move(speed_bounds_with_info));

  // Get speed bound for vt speed limit.
  for (const auto& [type, _] : speed_limit_provider.vt_speed_limit_map()) {
    const bool has_type =
        speed_upper_bound_map.find(type) != speed_upper_bound_map.end();

    for (int i = 0; i < knot_num; ++i) {
      const double t = i * delta_t;
      const auto vt_speed_limit_info =
          speed_limit_provider.GetVtSpeedLimitInfoByTypeAndTime(type, t);
      if (!has_type) {
        SpeedBoundWithInfo speed_bound = {
            .bound = vt_speed_limit_info.has_value()
                         ? vt_speed_limit_info->speed_limit
                         : allowed_max_speed,
            .info = vt_speed_limit_info.has_value()
                        ? absl::StrCat(vt_speed_limit_info->info, "-vt")
                        : ""};
        speed_upper_bound_map[type].push_back(std::move(speed_bound));
      } else {
        // Merge vt speed limit to a particular type.
        if (!vt_speed_limit_info.has_value()) continue;
        SpeedBoundWithInfo& origin = speed_upper_bound_map[type][i];
        if (vt_speed_limit_info->speed_limit < origin.bound) {
          origin.bound = vt_speed_limit_info->speed_limit;
          origin.info = absl::StrCat(vt_speed_limit_info->info, "-vt");
        }
      }
    }
  }

  return speed_upper_bound_map;
}

std::vector<SpeedBoundWithInfo> GenerateMinSpeedLimit(
    const std::vector<SpeedBoundWithInfo>& lim_1,
    const std::vector<SpeedBoundWithInfo>& lim_2, const std::string& info) {
  QCHECK_EQ(lim_1.size(), lim_2.size());
  const int knot_num = static_cast<int>(lim_1.size());
  std::vector<SpeedBoundWithInfo> min_speed_lim;
  min_speed_lim.reserve(knot_num);
  for (int i = 0; i < knot_num; ++i) {
    min_speed_lim.push_back(SpeedBoundWithInfo{
        .bound = std::min(lim_1[i].bound, lim_2[i].bound),
        .info = info,
    });
  }
  return min_speed_lim;
}
}  // namespace planner
}  // namespace qcraft
