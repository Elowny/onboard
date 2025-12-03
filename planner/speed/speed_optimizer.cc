#include "onboard/planner/speed/speed_optimizer.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <iterator>
#include <limits>
#include <map>
#include <optional>
#include <string>

#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "absl/types/span.h"
#include "glog/logging.h"
#include "osqp/osqp.h"

#include "onboard/eval/qevent.h"
#include "onboard/eval/qevent_base.h"
#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/lite/logging.h"
#include "onboard/math/util.h"
#include "onboard/planner/speed/proto/speed_finder_params.pb.h"
#include "onboard/planner/speed/speed_point.h"
#include "onboard/utils/status_macros.h"

namespace qcraft::planner {
namespace {

using DecisionType = StBoundaryProto::DecisionType;
using ConstraintMgr = SpeedOptimizerConstraintManager;

const double kInfinity = std::numeric_limits<double>::infinity();
constexpr double kVCheckThreshold = -1e-2;
constexpr double kSDiffCheckThreshold = -1e-2;
constexpr double kStartSCheckThreshold = 1e-2;
constexpr double kLowSpeedThreshold = 1.0;
constexpr double kPathLengthThreshold = 0.1;  // m.

constexpr double kMaxProbOfAllowableCollision = 0.1;
constexpr double kMinProbOfNotShrunk = 0.8;

const PiecewiseLinearFunction<double, double> kTtcStopWeightGainPlf = {
    {1.0, 2.0, 3.0, 4.0, 5.0, 6.0}, {0.8, 0.2, 0.1, 0.06, 0.04, 0.013}};

const PiecewiseLinearFunction<double, double> kLowSpeedFollowWeightRatioPlf = {
    {0.0, 5.0, 10.0}, {1.2, 1.4, 1.0}};
// Soft jerk constraint param.
constexpr double kTimeToAddJerkConstraint = 4.0;  // s.
const PiecewiseLinearFunction<double, double> kAVSpeedJerkLowerWeightPlf = {
    {5.0, 10.0, 20.0}, {0.01, 3.0, 6.0}};
const PiecewiseLinearFunction<double, double> kAVSpeedJerkUpperWeightPlf = {
    {2.0, 10.0}, {0.1, 3.0}};

const PiecewiseLinearFunction<double, double>
    kAvSpeedFollowDistanceLowestGainPlf = {{0.0, 6.0, 10.0}, {0.9, 0.6, 0.1}};

absl::Status PostProcessForValidity(std::string_view base_name,
                                    SpeedVector* speed_data) {
  QCHECK_NOTNULL(speed_data);
  for (int i = 0; i < speed_data->size(); ++i) {
    const double s = (*speed_data)[i].s();
    if (i == 0) {
      if (std::fabs(s) < kStartSCheckThreshold) {
        (*speed_data)[i].set_s(0.0);
      } else {
        QEVENT_EVERY_N_SECONDS(
            "pingshi", "start_s_not_zero", 1.0,
            [&](QEvent* qevent) { qevent->AddField("start_s", s); });

        return absl::InternalError(
            absl::StrCat("Start s is not close to 0.0, fabs(", s,
                         ") >= ", kStartSCheckThreshold, "."));
      }
    } else {
      // Monotomic requirement.
      const double prev_s = (*speed_data)[i - 1].s();
      const double s_diff = s - prev_s;
      if (s_diff < kSDiffCheckThreshold) {
        QEVENT_EVERY_N_SECONDS("pingshi", "speed_point_s_nonmonotomic", 1.0,
                               [&](QEvent* qevent) {
                                 qevent->AddField("s_diff", s_diff)
                                     .AddField("prev_s", prev_s)
                                     .AddField("now_s", s)
                                     .AddField("index", i)
                                     .AddField("base_name", base_name);
                               });

        QLOG_EVERY_N_SEC(WARNING, 1.0) << absl::StrFormat(
            "The cumulative distance of speed point is nonmonotonic, the "
            "threshold: %.3f has been exceeded, prev_s: %.3f now_s: %.3f.",
            kSDiffCheckThreshold, prev_s, s);
      }
      (*speed_data)[i].set_s(std::max(s, (*speed_data)[i - 1].s()));
    }
    const double v = (*speed_data)[i].v();
    if (v < kVCheckThreshold) {
      QEVENT_EVERY_N_SECONDS("pingshi", "osqp_neg_speed_too_large", 1.0,
                             [&](QEvent* qevent) {
                               qevent->AddField("v", v)
                                   .AddField("index", i)
                                   .AddField("base_name", base_name);
                             });

      QLOG_EVERY_N_SEC(WARNING, 1.0) << absl::StrFormat(
          "The negative speed %.3f calculated by osqp is too large,"
          "the threshold: %.3f has been exceeded.",
          v, kVCheckThreshold);
    }
    (*speed_data)[i].set_v(std::max(v, 0.0));
    VLOG(3) << "speed_data:[" << i << "] = " << (*speed_data)[i].DebugString();
  }

  return absl::OkStatus();
}

double MakeUpperBoundByProbability(double prob, double reaction_bound,
                                   double collision_bound,
                                   double max_path_length) {
  QCHECK_GE(max_path_length, collision_bound);
  QCHECK_GE(collision_bound, reaction_bound);
  const PiecewiseLinearFunction<double> upper_bound_plf = {
      {0.0, kMaxProbOfAllowableCollision, kMinProbOfNotShrunk, 1.0},
      {max_path_length, collision_bound, reaction_bound, reaction_bound}};
  return upper_bound_plf(prob);
}

double MakeLowerBoundByProbability(double prob, double reaction_bound,
                                   double collision_bound) {
  QCHECK_GE(reaction_bound, collision_bound);
  QCHECK_GE(collision_bound, 0.0);
  const PiecewiseLinearFunction<double> lower_bound_plf = {
      {0.0, kMaxProbOfAllowableCollision, kMinProbOfNotShrunk, 1.0},
      {0.0, collision_bound, reaction_bound, reaction_bound}};
  return lower_bound_plf(prob);
}

double GetObjectTimeGain(double t, const std::vector<double>& time_ranges,
                         const std::vector<double>& time_gains) {
  QCHECK_GT(time_ranges.size(), 1);
  QCHECK_EQ(time_ranges.size() - 1, time_gains.size());
  const int dist = std::distance(
      time_ranges.begin(),
      std::lower_bound(time_ranges.begin(), time_ranges.end(), t));
  const int idx =
      std::clamp(dist - 1, 0, static_cast<int>(time_ranges.size() - 2));
  return time_gains[idx];
}

SpeedVector MakeComfortableBrakeSpeed(double init_v, double init_a,
                                      double total_time, double max_decel,
                                      double const_jerk) {
  constexpr double kDeltaT = 0.2;  // s.
  SpeedVector speed_points;
  speed_points.reserve(static_cast<int>(total_time / kDeltaT) + 2);
  SpeedPoint prev_speed_point(/*t=*/0.0, /*s=*/0.0, std::max(0.0, init_v),
                              init_a,
                              /*j=*/0.0);
  speed_points.push_back(prev_speed_point);

  for (double t = kDeltaT; t <= total_time + kDeltaT; t += kDeltaT) {
    const double prev_a = prev_speed_point.a();
    const double prev_v = prev_speed_point.v();
    const double prev_s = prev_speed_point.s();
    if (prev_v < 1e-6) {
      prev_speed_point.set_t(t);
      prev_speed_point.set_s(prev_s);
      prev_speed_point.set_v(0.0);
      prev_speed_point.set_a(0.0);
      prev_speed_point.set_j(0.0);
      speed_points.push_back(prev_speed_point);
      continue;
    }

    // Flip jerk.
    constexpr double kDecelDurationTime = 4.0;
    double a, jerk;
    if (t > kDecelDurationTime && prev_a < 0.0) {
      jerk = -const_jerk;
      a = std::min({prev_a + jerk * kDeltaT, 0.0, init_a});
    } else {
      jerk = const_jerk;
      a = std::max(prev_a + jerk * kDeltaT, max_decel);
    }
    const double v = std::max(0.0, prev_v + (prev_a + a) * kDeltaT * 0.5);
    a = 2.0 * (v - prev_v) / kDeltaT - prev_a;
    const double prev_j = (a - prev_a) / kDeltaT;
    constexpr double kOneSixth = 1.0 / 6.0;  // one-sixth.
    const double s =
        prev_s + std::max(prev_v * kDeltaT + 0.5 * prev_a * Sqr(kDeltaT) +
                              kOneSixth * prev_j * Cube(kDeltaT),
                          0.0);
    speed_points.back().set_j(prev_j);
    prev_speed_point.set_t(t);
    prev_speed_point.set_s(s);
    prev_speed_point.set_v(v);
    prev_speed_point.set_a(a);
    prev_speed_point.set_j(0.0);
    speed_points.push_back(prev_speed_point);
  }
  return speed_points;
}

SpeedVector GenerateMaxBrakingSpeed(double init_v, double init_a,
                                    double max_decel, double total_time,
                                    double delta_t) {
  constexpr double kJerk = -10.0;  // m/(s^3)
  SpeedVector speed_points;
  speed_points.reserve(static_cast<int>(total_time / delta_t) + 2);
  SpeedPoint prev_speed_point(/*t=*/0.0, /*s=*/0.0, std::max(0.0, init_v),
                              init_a,
                              /*j=*/0.0);
  speed_points.push_back(prev_speed_point);
  const double delta_t_sqr = Sqr(delta_t);
  const double delta_t_cube = delta_t_sqr * delta_t;
  for (double t = delta_t; t <= total_time + delta_t; t += delta_t) {
    const double prev_a = prev_speed_point.a();
    const double prev_v = prev_speed_point.v();
    const double prev_s = prev_speed_point.s();
    if (prev_v < 1e-6) {
      prev_speed_point.set_t(t);
      prev_speed_point.set_s(prev_s);
      prev_speed_point.set_v(0.0);
      prev_speed_point.set_a(0.0);
      prev_speed_point.set_j(0.0);
      speed_points.push_back(prev_speed_point);
      continue;
    }
    double a = std::max(prev_a + kJerk * delta_t, max_decel);
    const double v = std::max(0.0, prev_v + (prev_a + a) * delta_t * 0.5);
    a = 2.0 * (v - prev_v) / delta_t - prev_a;
    const double prev_j = (a - prev_a) / delta_t;
    constexpr double kOneSixth = 1.0 / 6.0;  // one-sixth.
    const double s =
        prev_s + std::max(prev_v * delta_t + 0.5 * prev_a * delta_t_sqr +
                              kOneSixth * prev_j * delta_t_cube,
                          0.0);
    speed_points.back().set_j(prev_j);
    prev_speed_point.set_t(t);
    prev_speed_point.set_s(s);
    prev_speed_point.set_v(v);
    prev_speed_point.set_a(a);
    prev_speed_point.set_j(0.0);
    speed_points.push_back(prev_speed_point);
  }
  return speed_points;
}

double ComputeMaxDesiredDecel(const SpeedOptimizerObjectManager& opt_obj_mgr,
                              double av_speed, double delta_t) {
  double max_desired_decel = 0.0;
  constexpr double kStartTime = 1.0;        // s.
  constexpr double kLookForwardTime = 4.0;  // s.
  const int start_time_index = static_cast<int>(kStartTime / delta_t);
  const int look_forward_time_index =
      static_cast<int>(kLookForwardTime / delta_t);
  for (int time_idx = start_time_index; time_idx < look_forward_time_index;
       ++time_idx) {
    const double time = time_idx * delta_t;
    for (const SpeedOptimizerObject& opt_obj :
         opt_obj_mgr.MovingFollowObjects()) {
      const auto& overlap_state = opt_obj.GetOverlapStateByIndex(time_idx);
      if (!overlap_state.has_value()) continue;
      if (opt_obj.source_type() != StBoundarySourceTypeProto::ST_OBJECT) {
        continue;
      }
      const double weak_bound =
          std::max(overlap_state->bound - overlap_state->lon_buffer, 0.0);
      const double desired_decel =
          2.0 * (weak_bound - av_speed * time) / Sqr(time);
      if (desired_decel < max_desired_decel) {
        max_desired_decel = desired_decel;
      }
    }
  }
  return max_desired_decel;
}

SpeedVector GenerateComfortBrakeSpeedByMaxDesiredDecel(
    double init_v, double init_a, double total_time, double max_decel,
    double max_desired_decel) {
  const PiecewiseLinearFunction<double, double>
      comfort_jerk_rel_desired_decel_plf = {
          {-10.0, -5.0, -3.0, -2.0, -1.0, -0.5, -0.2},
          {-9.0, -4.4, -2.5, -1.6, -0.7, -0.3, -0.2}};
  const double jerk =
      comfort_jerk_rel_desired_decel_plf(max_desired_decel - init_a);
  VLOG(2) << "Comfort brake speed jerk : " << jerk;
  return MakeComfortableBrakeSpeed(init_v, init_a, total_time, max_decel, jerk);
}

SpeedLimitLevelProto::Level SpeedLimitTypeToLevel(
    SpeedLimitTypeProto::Type speed_limit_type) {
  switch (speed_limit_type) {
    case SpeedLimitTypeProto::CURVATURE:
    case SpeedLimitTypeProto::UNCERTAIN_PEDESTRAIN:
    case SpeedLimitTypeProto::STEER_RATE:
    case SpeedLimitTypeProto::LANE:
      return SpeedLimitLevelProto::STRONG;
    case SpeedLimitTypeProto::UNCERTAIN_VEHICLE:
    case SpeedLimitTypeProto::NEAR_PARALLEL_VEHICLE:
    case SpeedLimitTypeProto::IGNORE_OBJECT:
    case SpeedLimitTypeProto::CLOSE_CURB:
    case SpeedLimitTypeProto::DEFAULT:
      return SpeedLimitLevelProto::MEDIUM;
    case SpeedLimitTypeProto::EXTERNAL:
    case SpeedLimitTypeProto::MOVING_CLOSE_TRAJ:
    case SpeedLimitTypeProto::COMBINATION:
      return SpeedLimitLevelProto::WEAK;
  }
}

}  // namespace

SpeedOptimizer::SpeedOptimizer(
    std::string_view base_name, double init_v, double init_a,
    const MotionConstraintParamsProto* motion_constraint_params,
    const SpeedFinderParamsProto* speed_finder_params, double path_length,
    double default_speed_limit, double delta_t)
    : base_name_(std::string(base_name)),
      init_v_(init_v),
      init_a_(init_a),
      motion_constraint_params_(QCHECK_NOTNULL(motion_constraint_params)),
      speed_finder_params_(QCHECK_NOTNULL(speed_finder_params)),
      speed_optimizer_params_(&speed_finder_params->speed_optimizer_params()),
      delta_t_(delta_t),
      knot_num_(speed_optimizer_params_->knot_num()),
      total_time_(delta_t_ * (knot_num_ - 1)),
      allowed_max_speed_(Mph2Mps(default_speed_limit)),
      max_path_length_(path_length - kPathLengthThreshold),
      probability_gain_plf_(PiecewiseLinearFunctionFromProto(
          speed_finder_params_->probability_gain_plf())),
      accel_weight_gain_plf_(PiecewiseLinearFunctionFromProto(
          speed_optimizer_params_->accel_weight_gain_plf())),
      piecewise_time_range_(
          speed_optimizer_params_->piecewise_time_range().begin(),
          speed_optimizer_params_->piecewise_time_range().end()),
      moving_obj_time_gain_(
          speed_optimizer_params_->time_gain_for_moving_object().begin(),
          speed_optimizer_params_->time_gain_for_moving_object().end()),
      static_obj_time_gain_(
          speed_optimizer_params_->time_gain_for_static_object().begin(),
          speed_optimizer_params_->time_gain_for_static_object().end()),
      accel_lower_bound_plf_(PiecewiseLinearFunctionFromProto(
          speed_optimizer_params_->accel_lower_bound_plf())),
      ref_speed_time_gain_(PiecewiseLinearFunctionFromProto(
          speed_optimizer_params_->ref_speed_time_gain())) {}

absl::Status SpeedOptimizer::Optimize(
    const SpeedOptimizerObjectManager& opt_obj_mgr,
    const SpeedBoundMapType& speed_bound_map,
    const SpeedVector& reference_speed,
    const std::vector<ApolloTrajectoryPointProto>* time_aligned_prev_traj,
    SpeedVector* optimized_speed,
    SpeedFinderDebugProto* speed_finder_debug_proto) {
  QCHECK_NOTNULL(optimized_speed);
  QCHECK_NOTNULL(speed_finder_debug_proto);

  auto* speed_optimizer_debug =
      speed_finder_debug_proto->mutable_speed_optimizer();

  const double max_desired_decel =
      ComputeMaxDesiredDecel(opt_obj_mgr, init_v_, delta_t_);
  VLOG(2) << "Max desired decel: " << max_desired_decel;

  auto comfortable_brake_speed = GenerateComfortBrakeSpeedByMaxDesiredDecel(
      init_v_, init_a_, total_time_,
      motion_constraint_params_->max_deceleration(), max_desired_decel);

  optimized_speed->clear();
  for (const auto& speed_pt : comfortable_brake_speed) {
    speed_pt.ToProto(speed_optimizer_debug->add_comfortable_brake_speed());
  }
  comfortable_brake_speed_ = std::move(comfortable_brake_speed);

  auto max_brake_speed = GenerateMaxBrakingSpeed(
      init_v_, init_a_, motion_constraint_params_->max_deceleration(),
      total_time_, delta_t_);
  for (const auto& speed_pt : max_brake_speed) {
    speed_pt.ToProto(speed_optimizer_debug->add_max_brake_speed());
  }
  max_brake_speed_ = max_brake_speed;

  const auto accel_bound =
      std::make_pair(motion_constraint_params_->max_deceleration(),
                     motion_constraint_params_->max_acceleration());

  ConstraintMgr constraint_mgr(piecewise_time_range_, delta_t_, knot_num_);
  for (int i = 1; i < knot_num_; ++i) {
    SCOPED_QTRACE("SpeedOptimizer::MakeConstraint");
    MakeSConstraint(i, opt_obj_mgr, &constraint_mgr, speed_finder_debug_proto);
    MakeSpeedConstraint(i, speed_bound_map, &constraint_mgr,
                        speed_finder_debug_proto);
    MakeAccelConstraint(i, init_v_, accel_bound, &constraint_mgr);
  }
  // Add hard s constraint for last knot.
  constraint_mgr.AddHardSConstraint(knot_num_ - 1, 0.0, max_path_length_);
  if (time_aligned_prev_traj != nullptr && !time_aligned_prev_traj->empty()) {
    MakeJerkConstraint(*time_aligned_prev_traj, &constraint_mgr);
  }

  const int soft_s_lower_num = constraint_mgr.GetSlackNumOfLowerS();
  const int soft_s_upper_num = constraint_mgr.GetSlackNumOfUpperS();
  const int soft_v_lower_num = constraint_mgr.GetSlackNumOfLowerSpeed();
  const int soft_v_upper_num = constraint_mgr.GetSlackNumOfUpperSpeed();
  const int soft_a_lower_num = constraint_mgr.GetSlackNumOfLowerAccel();
  const int soft_a_upper_num = constraint_mgr.GetSlackNumOfUpperAccel();
  const int soft_j_lower_num = constraint_mgr.GetSlackNumOfLowerJerk();
  const int soft_j_upper_num = constraint_mgr.GetSlackNumOfUpperJerk();

  // Set debug proto.
  speed_optimizer_debug->set_init_v(init_v_);
  speed_optimizer_debug->set_init_a(init_a_);
  speed_optimizer_debug->set_delta_t(delta_t_);
  speed_optimizer_debug->set_lower_s_slack_term_num(soft_s_lower_num);
  speed_optimizer_debug->set_upper_s_slack_term_num(soft_s_upper_num);
  speed_optimizer_debug->set_lower_v_slack_term_num(soft_v_lower_num);
  speed_optimizer_debug->set_upper_v_slack_term_num(soft_v_upper_num);
  speed_optimizer_debug->set_lower_a_slack_term_num(soft_a_lower_num);
  speed_optimizer_debug->set_upper_a_slack_term_num(soft_a_upper_num);

  VLOG(3) << "lower_s_num = " << soft_s_lower_num;
  VLOG(3) << "upper_s_num = " << soft_s_upper_num;

  solver_ = std::make_unique<PiecewiseJerkQpSolver>(
      speed_optimizer_params_->knot_num(), delta_t_, soft_s_lower_num,
      soft_s_upper_num, soft_v_lower_num, soft_v_upper_num, soft_a_lower_num,
      soft_a_upper_num, soft_j_lower_num, soft_j_upper_num);

  if (!AddConstarints(init_v_, init_a_, constraint_mgr)) {
    const std::string err_msg = "Failed to add constraint.";
    QLOG(ERROR) << err_msg;
    QEVENT_EVERY_N_SECONDS("pingshi", "speed_optimizer_failure_add_constraint",
                           2.0, [&](QEvent* qevent) {
                             qevent->AddField("error_message", err_msg)
                                 .AddField("base_name", base_name_);
                           });
    return absl::InternalError(err_msg);
  }
  for (const auto& speed_pt : reference_speed) {
    speed_pt.ToProto(speed_optimizer_debug->add_ref_speed());
  }
  if (!AddKernel(constraint_mgr, reference_speed)) {
    const std::string err_msg = "Failed to add kernel.";
    QLOG(ERROR) << err_msg;
    QEVENT_EVERY_N_SECONDS("pingshi", "speed_optimizer_failure_add_kernel", 2.0,
                           [&](QEvent* qevent) {
                             qevent->AddField("error_message", err_msg)
                                 .AddField("base_name", base_name_);
                           });
    return absl::InternalError(err_msg);
  }

  if (const absl::Status status = Solve(); !status.ok()) {
    const auto err_msg = status.ToString();
    QLOG(ERROR) << err_msg;
    QEVENT_EVERY_N_SECONDS("pingshi", "speed_optimizer_failure_qp_solver", 2.0,
                           [&](QEvent* qevent) {
                             qevent->AddField("error_message", err_msg)
                                 .AddField("base_name", base_name_);
                           });

    return absl::InternalError(err_msg);
  }

  // extract output
  optimized_speed->clear();
  for (double t = 0.0; t < total_time_;
       t += speed_optimizer_params_->output_time_resolution()) {
    std::array<double, 4> res = solver_->Evaluate(t);
    SpeedPoint& speed_point =
        optimized_speed->emplace_back(t, res[0], res[1], res[2], res[3]);
    speed_point.ToProto(speed_optimizer_debug->add_optimized_speed());
  }

  RETURN_IF_ERROR(PostProcessForValidity(base_name_, optimized_speed));

  const double speed_length = optimized_speed->TotalLength();
  const double path_length = max_path_length_ + kPathLengthThreshold;
  if (speed_length > path_length) {
    QEVENT_EVERY_N_SECONDS(
        "pingshi", "speed_length_is_larger_than_path_length", 1.0,
        [&](QEvent* qevent) {
          qevent->AddField("path_length", path_length)
              .AddField("optimized_speed_length", speed_length)
              .AddField("length_diff", path_length - speed_length)
              .AddField("base_name", base_name_);
        });
  }

  return absl::OkStatus();
}

void SpeedOptimizer::MakeStationaryObjectFollowConstraint(
    int knot_idx, const ObjectOverlapState& overlap_state, absl::string_view id,
    double standstill, double time_gain, ConstraintMgr* constraint_mgr,
    SpeedFinderDebugProto* speed_finder_debug_proto) {
  QCHECK_NOTNULL(constraint_mgr);
  QCHECK_NOTNULL(speed_finder_debug_proto);

  const double upper_s = overlap_state.bound;
  const double follow_dist = overlap_state.lon_buffer;

  const double time = knot_idx * delta_t_;
  const auto max_braking_pt = max_brake_speed_.EvaluateByTime(time);
  const double s_upper = max_braking_pt.has_value()
                             ? std::max(upper_s, max_braking_pt->s())
                             : upper_s;

  const double upper_bound = std::max(0.0, s_upper - follow_dist);
  const double hard_upper_bound = std::max(0.0, s_upper - standstill);
  constexpr double kFullStopSpeedThres = 0.3;  // m/s.
  double weight = speed_optimizer_params_->s_stop_weight();
  if (init_v_ >= kFullStopSpeedThres) {
    const double acc = 0.5 * Sqr(init_v_) / hard_upper_bound;
    constexpr double kMaxDecelThres = 2.5;     // m/s^2.
    constexpr double kMinDistanceThres = 3.0;  // m.
    if (acc < kMaxDecelThres || hard_upper_bound < kMinDistanceThres) {
      // Calculate stop weight gain based on ttc if AV not fully stopped.
      const double ttc = hard_upper_bound / init_v_;
      weight *= kTtcStopWeightGainPlf(ttc);
    }
  }
  weight *= time_gain;
  constraint_mgr->AddSoftSUpperConstraint(
      ConstraintMgr::BoundStrType::MEDIUM,
      {.knot_idx = knot_idx, .weight = weight, .bound = upper_bound});

  const std::string chart_tips =
      absl::StrFormat("following_distance: %.2fm\nweight: %.2f(stop_weight)",
                      follow_dist, weight);

  auto* speed_optimizer_debug =
      speed_finder_debug_proto->mutable_speed_optimizer();
  auto& plot_data = (*speed_optimizer_debug->mutable_soft_s_upper_bound())[id];
  plot_data.add_value(upper_bound);
  plot_data.add_time(time);
  plot_data.add_soft_bound_distance(follow_dist);
  plot_data.add_info(chart_tips);
}

void SpeedOptimizer::MakeMovingObjectFollowConstraint(
    int knot_idx, const ObjectOverlapState& overlap_state, absl::string_view id,
    double follow_standstill, double time_att_gain,
    ConstraintMgr* constraint_mgr,
    SpeedFinderDebugProto* speed_finder_debug_proto) {
  QCHECK_NOTNULL(constraint_mgr);
  QCHECK_NOTNULL(speed_finder_debug_proto);

  const double time = knot_idx * delta_t_;
  const double obj_speed = std::max(0.0, overlap_state.speed);
  const double upper_s = overlap_state.bound;
  const double obj_average_prob = overlap_state.prob;

  // Step 1: Calculate following distance.
  constexpr double kNegDistanceBuffer = 0.5;
  const double follow_dist =
      std::min(std::max(overlap_state.lon_buffer, follow_standstill),
               upper_s + kNegDistanceBuffer);

  // Step 2: Clamp weak upper bound by comfortable brake speed.
  const PiecewiseLinearFunction<double, double>
      comfortable_brake_bound_violation_rel_speed_plf = {
          {-8.0, -4.0, -2.0, 0.0, 2.0, 4.0}, {12.0, 7.0, 4.0, 2.0, 1.0, 0.5}};
  const auto comfortable_brake_pt =
      comfortable_brake_speed_.EvaluateByTime(time);
  double weak_upper_bound = upper_s - follow_dist;
  if (speed_optimizer_params_->enable_comfort_brake_speed() &&
      comfortable_brake_pt.has_value()) {
    const double comfortable_brake_upper_bound =
        comfortable_brake_pt->s() -
        comfortable_brake_bound_violation_rel_speed_plf(obj_speed - init_v_);
    if (comfortable_brake_upper_bound > weak_upper_bound) {
      weak_upper_bound = std::max(
          -kNegDistanceBuffer,
          std::min(comfortable_brake_upper_bound, upper_s - follow_standstill));
    }
  }

  // Step 3: Add weak and strong soft constraints to constraint manager.
  const double low_speed_follow_weight_ratio =
      kLowSpeedFollowWeightRatioPlf(init_v_);
  double weak_upper_bound_by_prob = MakeUpperBoundByProbability(
      obj_average_prob, weak_upper_bound, upper_s, max_path_length_);
  const double strong_weight =
      low_speed_follow_weight_ratio * time_att_gain *
      speed_optimizer_params_->s_follow_strong_weight();
  if (obj_speed < kLowSpeedThreshold) {
    // When the target object speed is low, only add a strong soft constraint
    // and use the weak upper_s as the bound.
    // Make strong constraints larger than 0 to accelerate computation.
    weak_upper_bound_by_prob = std::max(0.0, weak_upper_bound_by_prob);
    constraint_mgr->AddSoftSUpperConstraint(
        ConstraintMgr::BoundStrType::STRONG,
        {.knot_idx = knot_idx,
         .weight = strong_weight,
         .bound = weak_upper_bound_by_prob});
  } else {
    const double follow_safety_distance = std::min(
        speed_finder_params_->follow_safety_distance(), follow_standstill);
    const double strong_upper_bound =
        std::max(upper_s - follow_safety_distance, weak_upper_bound);
    const double strong_upper_bound_by_prob = MakeUpperBoundByProbability(
        obj_average_prob, strong_upper_bound, upper_s, max_path_length_);
    const double weak_weight = low_speed_follow_weight_ratio * time_att_gain *
                               speed_optimizer_params_->s_follow_weak_weight();
    constraint_mgr->AddSoftSUpperConstraint(
        ConstraintMgr::BoundStrType::WEAK, {.knot_idx = knot_idx,
                                            .weight = weak_weight,
                                            .bound = weak_upper_bound_by_prob});
    constraint_mgr->AddSoftSUpperConstraint(
        ConstraintMgr::BoundStrType::STRONG,
        {.knot_idx = knot_idx,
         .weight = strong_weight,
         .bound = std::max(0.0, strong_upper_bound_by_prob)});
  }

  // Step 4: Debug and plot.
  const double actual_follow_dist =
      upper_s - weak_upper_bound_by_prob;  // Could be negative.
  const std::string chart_tips = absl::StrFormat(
      "prob: %.2f\nraw_dist: %.1f\ndist: %.1f\nobj_v: %.1f\ntime_gain: %.2f",
      obj_average_prob, follow_dist, actual_follow_dist, obj_speed,
      time_att_gain);
  auto* speed_optimizer_debug =
      speed_finder_debug_proto->mutable_speed_optimizer();
  auto& plot_data = (*speed_optimizer_debug->mutable_soft_s_upper_bound())[id];
  plot_data.add_value(weak_upper_bound_by_prob);
  plot_data.add_time(time);
  plot_data.add_soft_bound_distance(actual_follow_dist);
  plot_data.add_info(chart_tips);
}

void SpeedOptimizer::MakeMovingObjectLeadConstraint(
    int knot_idx, const ObjectOverlapState& overlap_state, absl::string_view id,
    double time_att_gain, ConstraintMgr* constraint_mgr,
    SpeedFinderDebugProto* speed_finder_debug_proto) {
  QCHECK_NOTNULL(constraint_mgr);
  QCHECK_NOTNULL(speed_finder_debug_proto);

  const double lower_s = overlap_state.bound;
  const double obj_average_prob = overlap_state.prob;
  const double lead_dist = overlap_state.lon_buffer;
  const double obj_speed = std::max(0.0, overlap_state.speed);

  // Step 1: Calculate leading distance.
  // TODO(ping): consider relative speed.
  const double weak_lower_bound = lower_s + lead_dist;
  const double strong_lower_bound = lower_s;

  // Step 2: Make lower bound by the probability.
  const double weak_lower_bound_by_prob =
      MakeLowerBoundByProbability(obj_average_prob, weak_lower_bound, lower_s);
  const double strong_lower_bound_by_prob = MakeLowerBoundByProbability(
      obj_average_prob, strong_lower_bound, lower_s);

  // Step 3: Make slack variable's weight.
  const double strong_weight =
      time_att_gain * speed_optimizer_params_->s_lead_strong_weight();
  const double weak_weight =
      time_att_gain * speed_optimizer_params_->s_lead_weak_weight();

  // Step 4: Add constraint data to constraint manager.
  constraint_mgr->AddSoftSLowerConstraint(ConstraintMgr::BoundStrType::WEAK,
                                          {.knot_idx = knot_idx,
                                           .weight = weak_weight,
                                           .bound = weak_lower_bound_by_prob});
  constraint_mgr->AddSoftSLowerConstraint(
      ConstraintMgr::BoundStrType::STRONG,
      {.knot_idx = knot_idx,
       .weight = strong_weight,
       .bound = strong_lower_bound_by_prob});

  // Step 5: Debug and plot.
  const double actual_lead_dist =
      weak_lower_bound_by_prob - lower_s;  // Could be negative.
  auto* speed_optimizer_debug =
      speed_finder_debug_proto->mutable_speed_optimizer();

  auto& plot_data = (*speed_optimizer_debug->mutable_soft_s_lower_bound())[id];
  plot_data.add_value(weak_lower_bound_by_prob);
  plot_data.add_time(knot_idx * delta_t_);
  plot_data.add_soft_bound_distance(actual_lead_dist);
  plot_data.add_info(absl::StrFormat(
      "final prob: %.2f\nraw_lead_dist: "
      "%.2f\nactual_lead_dist: "
      "%.2f\nweight: %.2f\nobj_speed : %.2fm/s\ntime_gain: %.2f",
      obj_average_prob, lead_dist, actual_lead_dist, weak_weight, obj_speed,
      time_att_gain));
}

void SpeedOptimizer::MakeSConstraint(
    int knot_idx, const SpeedOptimizerObjectManager& opt_obj_mgr,
    ConstraintMgr* constraint_mgr,
    SpeedFinderDebugProto* speed_finder_debug_proto) {
  QCHECK_NOTNULL(constraint_mgr);
  QCHECK_NOTNULL(speed_finder_debug_proto);

  const double time_att_gain = GetObjectTimeGain(
      knot_idx * delta_t_, piecewise_time_range_, moving_obj_time_gain_);

  // Add moving follow objects constraints.
  for (const SpeedOptimizerObject& obj : opt_obj_mgr.MovingFollowObjects()) {
    const auto& overlap_state = obj.GetOverlapStateByIndex(knot_idx);
    if (!overlap_state.has_value()) continue;
    MakeMovingObjectFollowConstraint(knot_idx, *overlap_state, obj.id(),
                                     obj.standstill(), time_att_gain,
                                     constraint_mgr, speed_finder_debug_proto);
  }

  const double static_time_att_gain = GetObjectTimeGain(
      knot_idx * delta_t_, piecewise_time_range_, static_obj_time_gain_);
  // Add stationary follow objects constraints.
  for (const SpeedOptimizerObject& obj : opt_obj_mgr.StationaryObjects()) {
    const auto& overlap_state = obj.GetOverlapStateByIndex(knot_idx);
    if (!overlap_state.has_value()) continue;
    MakeStationaryObjectFollowConstraint(
        knot_idx, *overlap_state, obj.id(), obj.standstill(),
        static_time_att_gain, constraint_mgr, speed_finder_debug_proto);
  }

  // Add moving lead objects constraints.
  for (const SpeedOptimizerObject& obj : opt_obj_mgr.MovingLeadObjects()) {
    const auto& overlap_state = obj.GetOverlapStateByIndex(knot_idx);
    if (!overlap_state.has_value()) continue;
    MakeMovingObjectLeadConstraint(knot_idx, *overlap_state, obj.id(),
                                   time_att_gain, constraint_mgr,
                                   speed_finder_debug_proto);
  }

  constraint_mgr->FilterObjectSConstraint(knot_idx);
}

void SpeedOptimizer::MakeSpeedConstraint(
    int knot_idx,
    const std::map<SpeedLimitTypeProto::Type, std::vector<SpeedBoundWithInfo>>&
        speed_limit_map,
    ConstraintMgr* constraint_mgr,
    SpeedFinderDebugProto* speed_finder_debug) const {
  QCHECK_NOTNULL(constraint_mgr);
  QCHECK_NOTNULL(speed_finder_debug);

  auto* speed_optimizer_debug = speed_finder_debug->mutable_speed_optimizer();

  constexpr double kDecel = -1.0;  // m/s^2
  constexpr double kAllowedIgnoreTime = 2.0;
  const double time = knot_idx * delta_t_;
  const double v_at_time = std::max(init_v_ + kDecel * time, 0.0);

  const auto get_slack_weight = [](auto level, const auto& params) {
    switch (level) {
      case SpeedLimitLevelProto::STRONG:
        return params.speed_limit_strong_weight();
      case SpeedLimitLevelProto::MEDIUM:
        return params.speed_limit_medium_weight();
      case SpeedLimitLevelProto::WEAK:
        return params.speed_limit_weak_weight();
    }
  };

  for (const auto& [type, speed_limit] : speed_limit_map) {
    if (type == SpeedLimitTypeProto::COMBINATION) continue;
    const double speed_upper_bound = speed_limit[knot_idx].bound;

    // Avoid the close range speed limits lower than the init speed, it may
    // increase the time cost and failure probability.
    // BANDAID(ping): Ignore curvature speed limit may cause steering
    // rate exceed the limitation.
    if (speed_upper_bound < v_at_time && time < kAllowedIgnoreTime &&
        type != SpeedLimitTypeProto::CURVATURE &&
        type != SpeedLimitTypeProto::UNCERTAIN_PEDESTRAIN &&
        type != SpeedLimitTypeProto::UNCERTAIN_VEHICLE &&
        type != SpeedLimitTypeProto::NEAR_PARALLEL_VEHICLE &&
        type != SpeedLimitTypeProto::IGNORE_OBJECT) {
      continue;
    }

    // Add constraint.
    const auto level = SpeedLimitTypeToLevel(type);
    constraint_mgr->AddSoftSpeedUpperConstraints(
        level, {.knot_idx = knot_idx,
                .weight = get_slack_weight(level, *speed_optimizer_params_),
                .bound = speed_upper_bound});

    // Add debug info for plot speed limit in chart.
    auto& plot_data =
        (*speed_optimizer_debug
              ->mutable_speed_limit())[SpeedLimitTypeProto::Type_Name(type)];
    plot_data.add_value(std::min(speed_upper_bound, allowed_max_speed_));
    plot_data.add_time(knot_idx * delta_t_);
    plot_data.add_info(speed_limit[knot_idx].info);
  }

  // Filter the constraints of different levels at same knot_idx.
  constraint_mgr->FilterObjectSpeedConstraint(knot_idx);

  constraint_mgr->AddHardSpeedConstraint(knot_idx, 0.0, kInfinity);
}

bool SpeedOptimizer::AddConstarints(double init_v, double init_a,
                                    const ConstraintMgr& constraint_mgr) {
  SCOPED_QTRACE("SpeedOptimizer::AddConstarints");

  /******************* add init constraint **********************/
  solver_->AddNthOrderEqualityConstraint(/*order=*/0, std::vector<int>{0},
                                         std::vector<double>{1.0}, 0.0);
  solver_->AddNthOrderEqualityConstraint(/*order=*/1, std::vector<int>{0},
                                         std::vector<double>{1.0},
                                         std::max(0.0, init_v));
  solver_->AddNthOrderEqualityConstraint(
      /*order=*/2, std::vector<int>{0}, std::vector<double>{1.0}, init_a);

  // set last jerk to 0.0
  solver_->AddNthOrderEqualityConstraint(/*order=*/3,
                                         std::vector<int>{knot_num_ - 1},
                                         std::vector<double>{1.0}, 0.0);

  /******************** add s constraint ********************/
  for (const auto& data : constraint_mgr.hard_s_constraint()) {
    solver_->AddNthOrderInequalityConstraint(
        /*order=*/0, {data.knot_idx},
        /*coeff=*/{1.0}, data.lower_bound, data.upper_bound);
  }

  for (const auto& data : constraint_mgr.GetLowerConstraintOfSoftS()) {
    solver_->AddZeroOrderInequalityConstraintWithOneSideSlack(
        {data.knot_idx},
        /*coeff=*/{1.0}, data.slack_idx, BoundType::LOWER_BOUND, data.bound);
  }

  for (const auto& data : constraint_mgr.GetUpperConstraintOfSoftS()) {
    solver_->AddZeroOrderInequalityConstraintWithOneSideSlack(
        {data.knot_idx},
        /*coeff=*/{1.0}, data.slack_idx, BoundType::UPPER_BOUND, data.bound);
  }

  /******************** add v constraint ********************/
  for (const auto& data : constraint_mgr.hard_v_constraint()) {
    solver_->AddNthOrderInequalityConstraint(
        /*order=*/1, {data.knot_idx},
        /*coeff=*/{1.0}, data.lower_bound, data.upper_bound);
  }

  for (const auto& data : constraint_mgr.GetLowerConstraintOfSpeed()) {
    solver_->AddFirstOrderInequalityConstraintWithOneSideSlack(
        {data.knot_idx},
        /*coeff=*/{1.0}, data.slack_idx, BoundType::LOWER_BOUND, data.bound);
  }

  for (const auto& data : constraint_mgr.GetUpperConstraintOfSpeed()) {
    solver_->AddFirstOrderInequalityConstraintWithOneSideSlack(
        {data.knot_idx},
        /*coeff=*/{1.0}, data.slack_idx, BoundType::UPPER_BOUND, data.bound);
  }

  /******************** add accel constraint ******************/
  for (const auto& data : constraint_mgr.hard_a_constraint()) {
    solver_->AddNthOrderInequalityConstraint(
        /*order=*/2, {data.knot_idx},
        /*coeff=*/{1.0}, data.lower_bound, data.upper_bound);
  }

  for (const auto& data : constraint_mgr.GetLowerConstraintOfSoftAccel()) {
    solver_->AddSecondOrderInequalityConstraintWithOneSideSlack(
        {data.knot_idx},
        /*coeff=*/{1.0}, data.slack_idx, BoundType::LOWER_BOUND, data.bound);
  }

  for (const auto& data : constraint_mgr.GetUpperConstraintOfSoftAccel()) {
    solver_->AddSecondOrderInequalityConstraintWithOneSideSlack(
        {data.knot_idx},
        /*coeff=*/{1.0}, data.slack_idx, BoundType::UPPER_BOUND, data.bound);
  }

  /******************** add jerk constraint ******************/
  for (const auto& data : constraint_mgr.GetLowerConstraintOfSoftJerk()) {
    solver_->AddThirdOrderInequalityConstraintWithOneSideSlack(
        {data.knot_idx},
        /*coeff=*/{1.0}, data.slack_idx, BoundType::LOWER_BOUND, data.bound);
  }

  for (const auto& data : constraint_mgr.GetUpperConstraintOfSoftJerk()) {
    solver_->AddThirdOrderInequalityConstraintWithOneSideSlack(
        {data.knot_idx},
        /*coeff=*/{1.0}, data.slack_idx, BoundType::UPPER_BOUND, data.bound);
  }

  return true;
}

bool SpeedOptimizer::AddKernel(const ConstraintMgr& constraint_mgr,
                               const SpeedVector& reference_speed) {
  SCOPED_QTRACE("SpeedOptimizer::AddKernel");
  QCHECK_EQ(reference_speed.size(), knot_num_);

  // Add s and v weight.
  for (int i = 0; i < knot_num_; ++i) {
    const auto time = static_cast<double>(i) * delta_t_;
    solver_->AddNthOrderReferencePointKernel</*order=*/0>(
        i, max_path_length_, speed_optimizer_params_->s_kernel_weight());

    const double ref_v = speed_optimizer_params_->enable_const_speed_ref_v()
                             ? init_v_
                             : reference_speed[i].v();

    const auto ref_speed_time_gain = ref_speed_time_gain_(time);
    const auto ref_speed_weight =
        ref_speed_time_gain * speed_optimizer_params_->speed_kernel_weight();
    solver_->AddNthOrderReferencePointKernel</*order=*/1>(i, ref_v,
                                                          ref_speed_weight);
  }

  const double accel_weight_gain = accel_weight_gain_plf_(init_v_);
  solver_->AddSecondOrderDerivativeKernel(
      accel_weight_gain * speed_optimizer_params_->accel_kernel_weight());

  solver_->AddThirdOrderDerivativeKernel(
      speed_optimizer_params_->jerk_kernel_weight());

  // Add s slack weight.
  for (const auto& [slack_idx, slack_weight] :
       constraint_mgr.GetLowerSlackWeightOfSoftS()) {
    solver_->AddZeroOrderSlackVarKernel(BoundType::LOWER_BOUND, {slack_idx},
                                        {slack_weight});
  }

  for (const auto& [slack_idx, slack_weight] :
       constraint_mgr.GetUpperSlackWeightOfSoftS()) {
    solver_->AddZeroOrderSlackVarKernel(BoundType::UPPER_BOUND, {slack_idx},
                                        {slack_weight});
  }

  // Add v slack weight.
  for (const auto& [slack_idx, slack_weight] :
       constraint_mgr.GetLowerSlackWeightOfSpeed()) {
    solver_->AddFirstOrderSlackVarKernel(BoundType::LOWER_BOUND, {slack_idx},
                                         {slack_weight});
  }
  for (const auto& [slack_idx, slack_weight] :
       constraint_mgr.GetUpperSlackWeightOfSpeed()) {
    solver_->AddFirstOrderSlackVarKernel(BoundType::UPPER_BOUND, {slack_idx},
                                         {slack_weight});
  }

  // Add a slack weight.
  for (const auto& [slack_idx, slack_weight] :
       constraint_mgr.GetLowerSlackWeightOfAccel()) {
    solver_->AddSecondOrderSlackVarKernel(BoundType::LOWER_BOUND, {slack_idx},
                                          {slack_weight});
  }
  for (const auto& [slack_idx, slack_weight] :
       constraint_mgr.GetUpperSlackWeightOfAccel()) {
    solver_->AddSecondOrderSlackVarKernel(BoundType::UPPER_BOUND, {slack_idx},
                                          {slack_weight});
  }

  // Add jerk slack weight
  for (const auto& [slack_idx, slack_weight] :
       constraint_mgr.GetLowerSlackWeightOfJerk()) {
    solver_->AddThirdOrderSlackVarKernel(BoundType::LOWER_BOUND, {slack_idx},
                                         {slack_weight});
  }

  for (const auto& [slack_idx, slack_weight] :
       constraint_mgr.GetUpperSlackWeightOfJerk()) {
    solver_->AddThirdOrderSlackVarKernel(BoundType::UPPER_BOUND, {slack_idx},
                                         {slack_weight});
  }

  // Add regularization.
  solver_->AddRegularization(speed_optimizer_params_->regularization());

  return true;
}

absl::Status SpeedOptimizer::Solve() {
  SCOPED_QTRACE("SpeedOptimizer::Solve");
  auto status = solver_->Optimize();
  const auto iters = solver_->osqp_iter();
  if (constexpr int kCheckNum = 1000; iters >= kCheckNum) {
    QEVENT_EVERY_N_SECONDS("pingshi",
                           absl::StrCat("speed_optimizer_osqp_iters_reached_",
                                        std::to_string(kCheckNum)),
                           1.0, [&](QEvent* qevent) {
                             qevent->AddField("osqp_iter", iters)
                                 .AddField("base_name", base_name_);
                           });
  }

  if (solver_->status_val() == OSQP_MAX_ITER_REACHED) {
    // Usually, it doesn't trigger often.
    QLOG_EVERY_N_SEC(INFO, 1.0) << "Use max iteration result.";
    QEVENT_EVERY_N_SECONDS(
        "pingshi", "speed_optimizer_osqp_max_iter_reached", 1.0,
        [&](QEvent* qevent) {
          qevent->AddField("max_iter", iters).AddField("base_name", base_name_);
        });
  }
  return status;
}

void SpeedOptimizer::MakeAccelConstraint(
    int knot_idx, double reference_speed,
    const std::pair<double, double>& accel_bound,
    ConstraintMgr* constraint_mgr) const {
  QCHECK_NOTNULL(constraint_mgr);
  const double accel_lower_bound =
      std::max(accel_lower_bound_plf_(reference_speed),
               motion_constraint_params_->max_deceleration());
  constraint_mgr->AddAccelSoftLowerConstraint(
      {.knot_idx = knot_idx,
       .weight = speed_optimizer_params_->accel_lower_slack_weight(),
       .bound = accel_lower_bound});
  constraint_mgr->AddAccelConstraint(knot_idx, accel_bound.first,
                                     accel_bound.second);
}

void SpeedOptimizer::MakeJerkConstraint(
    const std::vector<ApolloTrajectoryPointProto>& time_aligned_prev_traj,
    SpeedOptimizerConstraintManager* constraint_mgr) const {
  QCHECK_NOTNULL(constraint_mgr);
  int knot_idx = 0;
  double time = knot_idx * delta_t_;
  const double lower_weight = kAVSpeedJerkLowerWeightPlf(init_v_);
  const double upper_weight = kAVSpeedJerkUpperWeightPlf(init_v_);
  constexpr double kConsiderPrevTrajTime = 2.0;  // s.
  double min_jerk = 0.0, max_jerk = 0.0;
  for (const auto& traj_point : time_aligned_prev_traj) {
    if (traj_point.relative_time() > kConsiderPrevTrajTime) {
      break;
    }
    const double jerk = traj_point.j();
    min_jerk = std::min(min_jerk, jerk);
    max_jerk = std::max(max_jerk, jerk);
  }
  constexpr double kJerkLowerBoundRatio = 0.7;
  const double lower_bound = std::min(0.0, min_jerk * kJerkLowerBoundRatio);
  const double upper_bound = std::max(0.0, max_jerk);
  while (time <= kTimeToAddJerkConstraint) {
    constraint_mgr->AddJerkSoftLowerConstraint(
        {.knot_idx = knot_idx, .weight = lower_weight, .bound = lower_bound});
    constraint_mgr->AddJerkSoftUpperConstraint(
        {.knot_idx = knot_idx, .weight = upper_weight, .bound = upper_bound});
    time = ++knot_idx * delta_t_;
  }
}

}  // namespace qcraft::planner
