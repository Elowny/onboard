#include "onboard/prediction/predictor/predictor_util.h"

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type
#include <algorithm>
#include <cmath>
#include <numeric>
#include <ostream>
#include <queue>
#include <string>
#include <utility>

#include "absl/container/flat_hash_set.h"
#include "glog/logging.h"

#include "onboard/global/buffered_logger.h"
#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/math/fitter_def.h"
#include "onboard/math/frenet_common.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/math/polynomial_fitter.h"
#include "onboard/math/polynomialxd.h"
#include "onboard/math/util.h"
#include "onboard/planner/common/discretized_path.h"
#include "onboard/planner/speed/speed_vector.h"
#include "onboard/prediction/container/object_history_span.h"
#include "onboard/prediction/container/prediction_object.h"
#include "onboard/prediction/util/kinematic_model.h"
#include "onboard/prediction/util/pole_placement_util.h"
#include "onboard/prediction/util/trajectory_developer.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/utils/elements_history.h"
#include "onboard/utils/status_macros.h"

namespace qcraft {
namespace prediction {
namespace {
constexpr double kCutOffKappa = 0.5;                // m^-1.
constexpr double kSetToStaticSpeed = 0.2;           // m/s.
constexpr double kAngleDiffThreshold = M_PI / 6.0;  // rad.
constexpr double kMinObjectLength = 1.0;            // m.
// Look ahead time for curvature related acceleration heuristic.
constexpr double kAccLookAheadTime = 3.0;  // s.
// Minimal look ahead time for curvature related acceleration heuristic.
constexpr double kAccMinLookAheadTime = 1.0;     // s.
constexpr double kMaxLateralAcceleration = 2.5;  // m/s^2.
const PiecewiseLinearFunction<double, double> kMinAccPLF({2.0, 5.0, 8.0},
                                                         {-4.0, -3.5, -2.5});
const PiecewiseLinearFunction<double, double> kMaxAccPLF({2.0, 5.0, 8.0},
                                                         {1.5, 1.0, 1.0});
const PiecewiseLinearFunction<double, double> kHeuristicSpeedAccMaintainTimePLF(
    {0.0, 0.5, 1.0, 1.5}, {3.0, 2.5, 2.0, 1.5});

const PiecewiseLinearFunction<double, double>
    kMaxPredictionFutureSpeedIncreasePLF({1.0, 3.0, 10.0, 20.0},
                                         {0.0, 0.2, 1.0, 2.0});
/*
 *  Pure pursuit path generation related parameters.
 */

// Look ahead time for pure pursuit path generation.
constexpr double kNominalDtForPath = 0.2;      // s.
constexpr double kNominalHorizonForPath = 12;  // s.
constexpr double kCurvatureEpsilon = 1e-6;
// For cutin sl Net
constexpr double kMaxCutinSLConsiderAngle = 0.5;   // about 30 degrees.
constexpr double kMinCutinSLConsiderRange = 50.0;  // m.
const PiecewiseLinearFunction<double, double> kVelToMaxSteerMultRatio(
    std::vector<double>{4.0, 6.0}, std::vector<double>{1.4, 1.0});
struct ObjectKeyFilterField {
  const ObjectHistory* object_hist;
  double dist_to_ego;
};

ObjectKeyFilterField CreateObjectKeyFilterField(const ObjectHistory& obj_hist,
                                                const Box2d& ego_box) {
  const auto& hist = obj_hist.GetHistory();
  const double dist_to_ego = ego_box.DistanceTo(hist.back().val.bounding_box());
  return ObjectKeyFilterField{
      .object_hist = &obj_hist,
      .dist_to_ego = dist_to_ego,
  };
}

// Closer object has higher priority to be selected as Prophnet object.
struct ObjectKeyFilterFieldCmp {
  bool operator()(const ObjectKeyFilterField& object_a,
                  const ObjectKeyFilterField& object_b) {
    return object_a.dist_to_ego > object_b.dist_to_ego;
  }
};

constexpr int kFitterDegree = 5;
constexpr double kInitIndexWeight = 10.0;  // s.

std::pair<PolynomialXd<kFitterDegree>, PolynomialXd<kFitterDegree>>
GetTrajPolynomials(absl::Span<const Vec2d> traj_points,
                   absl::Span<const double> vec_t,
                   int polyfit_downsample_step) {
  const int size = traj_points.size();
  std::vector<Vec2d> x_series, y_series;
  std::vector<double> x_weights, y_weights;
  const int reserved_size = size / polyfit_downsample_step + 2;
  x_series.reserve(reserved_size);
  y_series.reserve(reserved_size);
  for (int i = 0; i < size; i += polyfit_downsample_step) {
    if (i == 0) {
      x_weights.push_back(kInitIndexWeight);
      y_weights.push_back(kInitIndexWeight);
    } else {
      x_weights.push_back(1.0);
      y_weights.push_back(1.0);
    }
    x_series.emplace_back(vec_t[i], traj_points[i].x());
    y_series.emplace_back(vec_t[i], traj_points[i].y());
  }
  if ((size - 1) % polyfit_downsample_step != 0) {
    x_weights.push_back(1.0);
    y_weights.push_back(1.0);
    x_series.emplace_back(vec_t.back(), traj_points.back().x());
    y_series.emplace_back(vec_t.back(), traj_points.back().y());
  }
  // Note: must ensure number of points larger than fitter degree.
  QCHECK_GT(x_series.size(), kFitterDegree);
  PolynomialXd<kFitterDegree> x_polynomial =
      *FitPolynomialXdToData<kFitterDegree>(x_series, x_weights,
                                            LS_SOLVER::kQr);
  PolynomialXd<kFitterDegree> y_polynomial =
      *FitPolynomialXdToData<kFitterDegree>(y_series, y_weights,
                                            LS_SOLVER::kQr);
  return {x_polynomial, y_polynomial};
}

planner::SpeedVector ComputeHeuristicSpeed(
    double cur_speed, double accel, double pred_horizon,
    const planner::DiscretizedPath& path) {
  // Approximatively compute decel by curvature limit.
  const double dist = kAccLookAheadTime * cur_speed;
  double max_abs_curvature = 0.0;
  for (const auto& pt : path) {
    if (pt.s() < kAccMinLookAheadTime * cur_speed) {
      continue;
    }
    if (pt.s() > dist) {
      break;
    }
    max_abs_curvature = std::max(max_abs_curvature, std::fabs(pt.kappa()));
  }
  double des_speed = std::sqrt(kMaxLateralAcceleration /
                               std::max(max_abs_curvature, kCurvatureEpsilon));
  if (des_speed < cur_speed) {
    accel = std::min(accel, (des_speed - cur_speed) / kAccLookAheadTime);
  }
  const double min_acc = kMinAccPLF(cur_speed);
  const double max_acc = kMaxAccPLF(cur_speed);
  // Get the final acceleration.
  accel = std::clamp(accel, min_acc, max_acc);

  const double acc_maintain_time =
      kHeuristicSpeedAccMaintainTimePLF(std::fabs(accel));

  // Apply acceleration
  return DevelopConstAccSpeedProfile(cur_speed, accel, acc_maintain_time,
                                     /*min_speed=*/0.0, kPredictionTimeStep,
                                     pred_horizon);
}

std::map<PredictTypePrio, std::vector<const ObjectHistory*>>
SelectObjectsByTypePrio(absl::Span<const ObjectHistory* const> objs_to_predict,
                        const TypePrioMap& type_prio_map) {
  std::map<PredictTypePrio, std::vector<const ObjectHistory*>> prio_obj_map;
  for (const auto& [prio, type_set] : type_prio_map) {
    prio_obj_map[prio].reserve(objs_to_predict.size());
    for (int i = 0; i < objs_to_predict.size(); ++i) {
      if (type_set.contains(objs_to_predict[i]->type())) {
        prio_obj_map[prio].push_back(objs_to_predict[i]);
      }
    }
  }
  return prio_obj_map;
}

std::map<PredictTypePrio, int> CalculateDynamicMaxNumMapForPrio(
    const std::map<PredictTypePrio, std::vector<const ObjectHistory*>>&
        prio_objs_map,
    const std::map<PredictTypePrio, int>& type_max_num_map,
    const int max_objects_num) {
  std::map<PredictTypePrio, int> dynamic_type_max_num_map;
  int selected_objs_num = 0;
  for (const auto& [prio, objs] : prio_objs_map) {
    dynamic_type_max_num_map[prio] =
        std::min(type_max_num_map.at(prio), static_cast<int>(objs.size()));
    selected_objs_num += dynamic_type_max_num_map[prio];
  }
  // If total selected objects num < total max objects number, modify high
  // Priority type objects max number.
  if (selected_objs_num < max_objects_num) {
    int remaining_spots_num = max_objects_num - selected_objs_num;
    for (const auto& [prio, objs] : prio_objs_map) {
      int not_select_objs_num = std::max(
          static_cast<int>(objs.size()) - type_max_num_map.at(prio), 0);
      dynamic_type_max_num_map[prio] +=
          std::min(remaining_spots_num, not_select_objs_num);
      remaining_spots_num -= std::min(remaining_spots_num, not_select_objs_num);
      if (remaining_spots_num == 0) {
        break;
      }
    }
  }
  return dynamic_type_max_num_map;
}

bool IsFastMovingTrajectory(
    absl::Span<const PredictedTrajectoryPoint> raw_traj_pts) {
  constexpr double kMovingSpeedThreshold = 2.0;
  constexpr double kMovingDistanceThreshold = 16.0;

  // If any point in traj has a speed lower than threshold ,then it is not a
  // fast moving traj.
  for (const auto& pt : raw_traj_pts) {
    if (pt.v() < kMovingSpeedThreshold) {
      return false;
    }
  }
  // If the total traj length is less than certain threshold, then it is not a
  // fast moving traj.
  if (raw_traj_pts.back().s() < kMovingDistanceThreshold) {
    return false;
  }
  return true;
}

bool IsObjectHeadOverAvHeadWithBuffer(
    const planner::DrivePassage& drive_passage, const Box2d& av_box,
    const Box2d& obj_box) {
  constexpr double kCutinInterfereDistBuffer = 2.0;  // m
  const Vec2d av_front_center_point = av_box.FrontCenterPoint();
  const Vec2d obj_front_center_point = obj_box.FrontCenterPoint();
  const auto av_front_center_sl_pos_or =
      drive_passage.QueryUnboundedFrenetCoordinateAt(av_front_center_point);
  const auto obj_front_center_sl_pos_or =
      drive_passage.QueryUnboundedFrenetCoordinateAt(obj_front_center_point);
  if (obj_front_center_sl_pos_or.ok() && av_front_center_sl_pos_or.ok()) {
    if (obj_front_center_sl_pos_or.value().s >
        av_front_center_sl_pos_or.value().s - kCutinInterfereDistBuffer) {
      return true;
    }
  }
  return false;
}
}  // namespace.

absl::StatusOr<std::vector<PredictedTrajectoryPoint>>
GeneratePredictedTrajectoryPoints(const ObjectMotionState& cur_state,
                                  const planner::DrivePassage& dp, double accel,
                                  double pred_horizon, double target_l) {
  const auto& pos = cur_state.pos;

  BicycleModelState obj_state{
      .x = pos.x(),
      .y = pos.y(),
      .v = cur_state.vel.norm(),
      .heading = cur_state.heading,
      .acc = 0.0,
      .front_wheel_angle = 0.0,
  };
  const auto max_steer = kLengthToMaxFrontSteerPlf(cur_state.bbox.length());
  const auto wheelbase = kLengthToWheelbasePlf(cur_state.bbox.length());
  ASSIGN_OR_RETURN(
      const planner::DiscretizedPath path,
      DevelopPolePlacementPath(obj_state, dp, kNominalDtForPath,
                               kNominalHorizonForPath, target_l,
                               cur_state.bbox.length(), max_steer, wheelbase));
  const double cur_speed = std::fabs(obj_state.v);
  const auto speed_vec =
      ComputeHeuristicSpeed(cur_speed, accel, pred_horizon, path);
  return CombinePathAndSpeedForPredictedTrajectoryPoints(path, speed_vec);
}

// Closer object has higher priority to be considered as prophnet object.
std::vector<const ObjectHistory*> ScreenPredictObjectsByDistance(
    const Box2d& ego_box,
    absl::Span<const ObjectHistory* const> objs_to_predict,
    int max_objects_num) {
  std::vector<const ObjectHistory*> pred_objects;
  pred_objects.reserve(max_objects_num);
  std::priority_queue<ObjectKeyFilterField, std::vector<ObjectKeyFilterField>,
                      ObjectKeyFilterFieldCmp>
      object_pq;
  for (int i = 0; i < objs_to_predict.size(); ++i) {
    object_pq.push(CreateObjectKeyFilterField(*objs_to_predict[i], ego_box));
  }
  while (!object_pq.empty()) {
    pred_objects.push_back(object_pq.top().object_hist);
    if (pred_objects.size() == max_objects_num) {
      return pred_objects;
    }
    object_pq.pop();
  }
  return pred_objects;
}

// Select objects by type_max_num_map first, then put high Priority type objects
// into empty spots.
std::vector<const ObjectHistory*> SelectPredictedObjectsByTypePriority(
    const Box2d& ego_box, const int max_objects_num,
    absl::Span<const ObjectHistory* const> objs_to_predict,
    const TypePrioMap& type_prio_map,
    const std::map<PredictTypePrio, int>& type_max_num_map) {
  FUNC_QTRACE();
  const auto prio_objs_map =
      SelectObjectsByTypePrio(objs_to_predict, type_prio_map);

  const std::map<PredictTypePrio, int> dynamic_type_max_num_map =
      CalculateDynamicMaxNumMapForPrio(prio_objs_map, type_max_num_map,
                                       max_objects_num);

  std::vector<const ObjectHistory*> result;
  for (const auto& [prio, objs] : prio_objs_map) {
    if (objs.size() > type_max_num_map.at(prio)) {
      const auto screened_objs = ScreenPredictObjectsByDistance(
          ego_box, objs, dynamic_type_max_num_map.at(prio));
      std::for_each(screened_objs.begin(), screened_objs.end(),
                    [&result](const ObjectHistory* object_history) {
                      if (!object_history->empty()) {
                        result.push_back(object_history);
                      }
                    });
    } else {
      std::for_each(objs.begin(), objs.end(),
                    [&result](const ObjectHistory* object_history) {
                      if (!object_history->empty()) {
                        result.push_back(object_history);
                      }
                    });
    }
  }
  return result;
}

// Note(Jinyun): If no objects, use av pose latest time as current timestamp
double GetCurrentTimeStamp(double av_latest_timestamp,
                           absl::Span<const ObjectHistory* const> objs) {
  return objs.empty() ? av_latest_timestamp
                      : std::accumulate(objs.begin(), objs.end(), 0.0,
                                        [](const double sum, const auto& obj) {
                                          return sum + obj->timestamp();
                                        }) /
                            objs.size();
}

std::vector<PredictedTrajectoryPoint> Vec2dPointsToPredTrajPoints(
    absl::Span<const Vec2d> vec2d_points, absl::Span<const double> vec_t,
    int traj_points_size, bool use_pos_fitter, int polyfit_downsample_step) {
  // Fit trajectory points by polynomial fitting.
  const auto [x_polynomial, y_polynomial] =
      GetTrajPolynomials(vec2d_points, vec_t, polyfit_downsample_step);

  PiecewiseLinearFunction<Vec2d, double> pos_interp(
      std::vector<double>(vec_t.begin(), vec_t.end()),
      std::vector<Vec2d>(vec2d_points.begin(), vec2d_points.end()));
  std::vector<PredictedTrajectoryPoint> traj_pts;
  traj_pts.resize(traj_points_size);
  double s = 0.0;
  Vec2d prev_pos;
#if (defined(__ARM_NEON) && defined(__J5__))
  auto xy_polynomial = PolynomialXdx2<kFitterDegree>(x_polynomial.coeffs(),
                                                     y_polynomial.coeffs());
#endif
  double point_t = -kPredictionTimeStep;
  for (int i = 0; i < traj_points_size; ++i) {
    point_t += kPredictionTimeStep;
    const double t = vec_t[0] + point_t;
    auto& point = traj_pts[i];
    point.set_t(point_t);
    Vec2d cur_pos;
    if (use_pos_fitter) {
#if (defined(__ARM_NEON) && defined(__J5__))
      auto xy = xy_polynomial.Evaluate(t);
      vst1q_f64(&cur_pos.x(), xy);
#else
      const double x = x_polynomial.Evaluate(t);
      const double y = y_polynomial.Evaluate(t);
      cur_pos = Vec2d(x, y);
#endif
      point.set_pos(cur_pos);
    } else {
      cur_pos = pos_interp(point_t);
      point.set_pos(cur_pos);
    }
#if (defined(__ARM_NEON) && defined(__J5__))
    double len12[2];
    double len[2];
    auto dxy = xy_polynomial.EvaluateDerivative<1>(t);
    auto dxy2 = xy_polynomial.EvaluateDerivative<2>(t);
    len12[0] = vaddvq_f64(vmulq_f64(dxy, dxy));
    len12[1] = vaddvq_f64(vmulq_f64(dxy2, dxy2));
    auto d2_dot_d = vaddvq_f64(vmulq_f64(dxy, dxy2));
    vst1q_f64(len, vsqrtq_f64(vld1q_f64(len12)));
    double d_len3 = len12[0] * len[0];
    vst1q_f64(len12, dxy);
    point.set_v(len[0]);
    if (d2_dot_d < 0) {
      len[1] = -len[1];
    }
    point.set_a(len[1]);
    len[0] = -len12[1];
    point.set_theta(fast_math::Atan2(len12[1], len12[0]));
    len12[1] = len12[0];
    len12[0] = len[0];
    point.set_kappa(vaddvq_f64(vmulq_f64(dxy2, vld1q_f64(len12))) / d_len3);
#else
    const double dx = x_polynomial.EvaluateDerivative</*order=*/1>(t);
    const double d2x = x_polynomial.EvaluateDerivative</*order=*/2>(t);
    const double dy = y_polynomial.EvaluateDerivative</*order=*/1>(t);
    const double d2y = y_polynomial.EvaluateDerivative</*order=*/2>(t);

    Vec2d d(dx, dy);
    Vec2d d2(d2x, d2y);
    const double d_len2 = dx * dx + dy * dy;
    const double d_len = std::sqrt(d_len2);
    point.set_v(d_len);
    if (d2.dot(d) > 0) {
      point.set_a(d2.norm());
    } else {
      point.set_a(-d2.norm());
    }
    point.set_theta(d.FastAngle());
    point.set_kappa((dx * d2y - dy * d2x) / (d_len2 * d_len));
#endif
    if (i > 0) {
      s += (cur_pos - prev_pos).norm();
    }
    prev_pos = cur_pos;
    point.set_s(s);
  }
  return traj_pts;
}

std::vector<PredictedTrajectoryPoint> PostProcessModelOutputTrajPts(
    std::vector<PredictedTrajectoryPoint> raw_traj_pts,
    const ObjectMotionState& obj_state, double time_step, double duration,
    double perception_acc, bool rectify_speed) {
  std::vector<PredictedTrajectoryPoint> traj_pts = std::move(raw_traj_pts);

  // If a traj is not fast moving, we try to smooth it because we may find
  // some strange twists in a low speed trajectory. If it is fast moving, then
  // we do not perform any post-process.
  if (!IsFastMovingTrajectory(traj_pts)) {
    traj_pts[0].set_theta(obj_state.heading);
    for (int i = 0; i < traj_pts.size(); ++i) {
      if (traj_pts[i].v() < kSetToStaticSpeed) {
        traj_pts[i].set_v(0.0);
      }
      const double cur_kappa = traj_pts[i].kappa();
      traj_pts[i].set_kappa(std::clamp(cur_kappa, -kCutOffKappa, kCutOffKappa));
    }
    for (int i = 1; i < traj_pts.size(); ++i) {
      const double angle_diff =
          NormalizeAngle(traj_pts[i].theta() - traj_pts[i - 1].theta());
      traj_pts[i].set_theta(NormalizeAngle(
          traj_pts[i - 1].theta() +
          std::clamp(angle_diff, -kAngleDiffThreshold, kAngleDiffThreshold)));
    }
    const double length = std::max(obj_state.bbox.length(), kMinObjectLength);
    const auto max_steer = kVelToMaxSteerMultRatio(traj_pts[0].v()) *
                           kLengthToMaxFrontSteerPlf(length);
    const auto wheelbase = kLengthToWheelbasePlf(length);

    auto tracked_traj_pts = TrackTrajectoryByPolePlacement(
        traj_pts, ObjectMotionStateToBicycleModelState(obj_state), length,
        max_steer, wheelbase, time_step, duration);
    // Tracked trajectory should contain at least some points.
    const bool tracked_contains_less_point =
        (tracked_traj_pts.size() <=
             static_cast<int>(kEmergencyGuardHorizon / time_step) &&
         tracked_traj_pts.size() < traj_pts.size());
    if (!tracked_contains_less_point) {
      traj_pts = std::move(tracked_traj_pts);
    }
    VLOG(2) << " slow moving trajectory, use pp tracking";
  }

  if (rectify_speed) {
    const auto path = PredictedTrajectoryPointsToDiscretizedPath(traj_pts);
    const double start_v = obj_state.vel.norm();
    // Replace predicted trajectory speed with const acc heuristic speed
    // profile.
    planner::SpeedVector speed_vec = ComputeHeuristicSpeed(
        start_v, perception_acc, traj_pts.back().t(), path);
    auto new_traj_pts =
        CombinePathAndSpeedForPredictedTrajectoryPoints(path, speed_vec);
    if (new_traj_pts.size() >= kPredictionMinPointNum) {
      traj_pts = std::move(new_traj_pts);
    }
  }

  return traj_pts;
}

std::vector<AgentCentricObjectProbTraj> FilterTrajsByProb(
    absl::Span<const AgentCentricObjectProbTraj> prob_trajs,
    double prob_threshold) {
  std::vector<AgentCentricObjectProbTraj> filtered_trajs;
  filtered_trajs.reserve(prob_trajs.size());
  // Trajectory probability is ordered from large to small.
  filtered_trajs.push_back(prob_trajs.front());
  int valid_traj_idx = 1;
  double sum_prob = prob_trajs.front().mode_prob;
  while (valid_traj_idx < prob_trajs.size()) {
    if (prob_trajs[valid_traj_idx].mode_prob < prob_threshold) {
      break;
    }
    filtered_trajs.push_back(prob_trajs[valid_traj_idx]);
    sum_prob += prob_trajs[valid_traj_idx].mode_prob;
    ++valid_traj_idx;
  }
  QCHECK_GT(sum_prob, 0.0);
  for (auto& prob_traj : filtered_trajs) {
    prob_traj.mode_prob /= sum_prob;
  }
  return filtered_trajs;
}

// Closer object has higher priority to be considered as to predict object.
std::vector<ObjectIDType> SelectCutinSLNetPredictedObjects(
    const ObjectHistorySpan& av_history_span, int max_objects_num,
    absl::Span<const ObjectHistory* const> objs_to_predict,
    const planner::DrivePassage& drive_passage,
    const ObjectHistorySampler& obj_sampler) {
  FUNC_QTRACE();
  std::vector<ObjectIDType> cutin_sl_objs_ids;
  cutin_sl_objs_ids.reserve(max_objects_num);
  const Vec2d& av_pos = av_history_span.pos();
  const auto av_cur_sl_pos_or =
      drive_passage.QueryUnboundedFrenetCoordinateAt(av_pos);
  if (!av_cur_sl_pos_or.ok()) {
    return cutin_sl_objs_ids;
  }
  for (const ObjectHistory* const obj : objs_to_predict) {
    const auto& obj_motion_hist =
        obj_sampler.GetResampledMotionHistoryById(obj->id());
    const auto& obj_last_state = obj_motion_hist.states.back();
    const auto& obj_pos = obj_last_state.pos;

    // Don't consider objects behind av.
    ASSIGN_OR_CONTINUE(const auto obj_cur_sl_pos,
                       drive_passage.QueryUnboundedFrenetCoordinateAt(obj_pos));

    if (std::abs(obj_cur_sl_pos.l) < kHalfChannelWidth) {
      continue;
    }

    if (!IsObjectHeadOverAvHeadWithBuffer(drive_passage,
                                          av_history_span.bounding_box(),
                                          obj_last_state.bbox)) {
      continue;
    }

    ASSIGN_OR_CONTINUE(const auto dp_angle,
                       drive_passage.QueryTangentAngleAtS(obj_cur_sl_pos.s));

    if (IsBicycleModelLike(obj_motion_hist.type) &&
        (obj_pos.DistanceTo(av_pos) <
         std::max(kMinCutinSLConsiderRange,
                  av_history_span.v() * kCutinSLHorizon)) &&
        (std::abs(NormalizeAngle(av_history_span.heading() -
                                 obj_last_state.heading)) <
             kMaxCutinSLConsiderAngle ||
         std::abs(NormalizeAngle(dp_angle - obj_last_state.heading)) <
             kMaxCutinSLConsiderAngle)) {
      cutin_sl_objs_ids.push_back(obj->id());
    }
  }

  if (cutin_sl_objs_ids.size() > max_objects_num) {
    // Sort the objects according to its distance to av
    std::sort(
        cutin_sl_objs_ids.begin(), cutin_sl_objs_ids.end(),
        [&av_pos, &obj_sampler](const ObjectIDType& x, const ObjectIDType& y) {
          const auto& x_motion_hist =
              obj_sampler.GetResampledMotionHistoryById(x);
          const auto& y_motion_hist =
              obj_sampler.GetResampledMotionHistoryById(y);
          const auto x_pos = x_motion_hist.states.back().pos;
          const auto y_pos = y_motion_hist.states.back().pos;
          return x_pos.DistanceSquareTo(av_pos) <
                 y_pos.DistanceSquareTo(av_pos);
        });
    cutin_sl_objs_ids.resize(max_objects_num);
  }

  return cutin_sl_objs_ids;
}

std::unordered_map<ObjectIDType, ParallelRiskType>
SelectParallelHighRiskObjects(
    const ObjectHistorySpan& av_history_span,
    absl::Span<const ObjectHistory* const> objs_to_predict,
    const planner::DrivePassage& drive_passage,
    const ObjectHistorySampler& obj_sampler) {
  FUNC_QTRACE();
  std::unordered_map<ObjectIDType, ParallelRiskType> high_risk_objs_map;
  high_risk_objs_map.reserve(objs_to_predict.size());
  const Vec2d& av_pos = av_history_span.pos();
  const auto av_cur_sl_pos_or =
      drive_passage.QueryUnboundedFrenetCoordinateAt(av_pos);
  if (!av_cur_sl_pos_or.ok()) {
    return high_risk_objs_map;
  }
  constexpr double kBackRange = -50.0;
  constexpr double kFrontRange = 100.0;
  for (const ObjectHistory* const obj : objs_to_predict) {
    const auto& obj_motion_hist =
        obj_sampler.GetResampledMotionHistoryById(obj->id());
    const auto& obj_last_state = obj_motion_hist.states.back();
    const auto& obj_pos = obj_last_state.pos;

    ASSIGN_OR_CONTINUE(const auto obj_cur_sl_pos,
                       drive_passage.QueryUnboundedFrenetCoordinateAt(obj_pos));
    // too far from av
    if (std::abs(obj_cur_sl_pos.l) > kDefaultHalfLaneWidth * 5.0) {
      continue;
    }

    // in boundary to considered as leading or following object
    if (std::abs(obj_cur_sl_pos.l) < kDefaultHalfLaneWidth * 0.5) {
      continue;
    }

    const double s_diff = obj_cur_sl_pos.s - av_cur_sl_pos_or->s;
    if (s_diff < kBackRange || s_diff > kFrontRange) {
      continue;
    }

    ASSIGN_OR_CONTINUE(const auto dp_angle,
                       drive_passage.QueryTangentAngleAtS(obj_cur_sl_pos.s));

    // Filter heading and distance.
    const bool is_parallel_object =
        (obj_pos.DistanceTo(av_pos) <
         std::max(kMinCutinSLConsiderRange,
                  av_history_span.v() * kCutinSLHorizon)) &&
        (std::abs(NormalizeAngle(av_history_span.heading() -
                                 obj_last_state.heading)) <
             kMaxCutinSLConsiderAngle ||
         std::abs(NormalizeAngle(dp_angle - obj_last_state.heading)) <
             kMaxCutinSLConsiderAngle);

    if (!is_parallel_object && IsBicycleModelLike(obj_motion_hist.type)) {
      continue;
    }
    const auto risk_type = obj_motion_hist.type == OT_LARGE_VEHICLE
                               ? ParallelRiskType::kNearLargeVehicle
                               : ParallelRiskType::kNearNormalVehicle;
    high_risk_objs_map.emplace(obj->id(), risk_type);
  }

  return high_risk_objs_map;
}

const planner::DrivePassage* FindBestMatchingDrivePassage(
    absl::Span<const std::unique_ptr<planner::DrivePassage>> dps,
    const Vec2d& pos, const double heading) {
  struct DpInfo {
    double dis;
    FrenetCoordinate sl;
    const planner::DrivePassage* dp;
  };
  std::vector<DpInfo> dp_infos;
  dp_infos.reserve(dps.size());
  constexpr double kMaxHeadingDiff = M_PI / 6.0;
  for (const auto& dp : dps) {
    ASSIGN_OR_CONTINUE(const auto sl,
                       dp->QueryLaterallyUnboundedFrenetCoordinateAt(pos));
    ASSIGN_OR_CONTINUE(const auto tangent_angle,
                       dp->QueryTangentAngleAtS(sl.s));
    ASSIGN_OR_CONTINUE(const auto bounds,
                       dp->QueryNearestBoundaryLateralOffset(sl.s));
    if (sl.l < bounds.first || sl.l > bounds.second) {
      continue;
    }
    if (std::fabs(NormalizeAngle(tangent_angle - heading)) > kMaxHeadingDiff) {
      continue;
    }
    dp_infos.push_back(
        DpInfo{.dis = std::fabs(sl.l), .sl = sl, .dp = dp.get()});
  }
  if (dp_infos.empty()) {
    return nullptr;
  }
  std::sort(dp_infos.begin(), dp_infos.end(),
            [](const auto& info1, const auto& info2) {
              return info1.dis < info2.dis;
            });
  return dp_infos.front().dp;
}

std::vector<const planner::DrivePassage*> FindNearbyDrivePassages(
    absl::Span<const std::unique_ptr<planner::DrivePassage>> dps,
    const Vec2d& pos, const double heading, double max_offset,
    double max_heading_diff, int max_num) {
  struct DpInfo {
    double dis;
    FrenetCoordinate sl;
    const planner::DrivePassage* dp;
  };
  std::vector<DpInfo> dp_infos;
  dp_infos.reserve(dps.size());
  for (const auto& dp : dps) {
    ASSIGN_OR_CONTINUE(const auto sl,
                       dp->QueryLaterallyUnboundedFrenetCoordinateAt(pos));
    ASSIGN_OR_CONTINUE(const auto tangent_angle,
                       dp->QueryTangentAngleAtS(sl.s));
    if (sl.l < -max_offset || sl.l > max_offset) {
      continue;
    }
    if (std::fabs(NormalizeAngle(tangent_angle - heading)) > max_heading_diff) {
      continue;
    }
    dp_infos.push_back(
        DpInfo{.dis = std::fabs(sl.l), .sl = sl, .dp = dp.get()});
  }
  std::vector<const planner::DrivePassage*> res;
  if (dp_infos.empty()) {
    return res;
  }
  std::sort(dp_infos.begin(), dp_infos.end(),
            [](const auto& info1, const auto& info2) {
              return info1.dis < info2.dis;
            });
  for (int i = 0; i < std::min<int>(max_num, dp_infos.size()); ++i) {
    res.push_back(dp_infos[i].dp);
  }
  return res;
}

std::vector<const planner::DrivePassage*> FilterDrivePassagesByCTRA(
    absl::Span<const planner::DrivePassage* const> dps,
    const ObjectMotionState& cur_state, int max_num) {
  constexpr double kCTRARollOutHorizon = 1.5;
  const auto probe_traj_pts = DevelopForwardCTRATrajectory(
      ObjectMotionStateToUniCycleState(cur_state), kPredictionTimeStep,
      kCTRARollOutHorizon, kCTRARollOutHorizon, kEmergencyGuardHorizon);
  std::vector<const planner::DrivePassage*> res;
  for (const auto* dp : dps) {
    bool find_dp = false;
    for (const auto& pt : probe_traj_pts) {
      ASSIGN_OR_CONTINUE(
          const auto sl,
          dp->QueryLaterallyUnboundedFrenetCoordinateAt(pt.pos()));
      ASSIGN_OR_CONTINUE(const auto bounds,
                         dp->QueryNearestBoundaryLateralOffset(sl.s));
      if (sl.l >= bounds.first && sl.l <= bounds.second) {
        find_dp = true;
        break;
      }
    }
    if (find_dp) {
      res.push_back(dp);
    }
  }
  if (res.size() > max_num) {
    res.resize(max_num);
  }
  return res;
}

}  // namespace prediction
}  // namespace qcraft
