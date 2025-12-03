#include "onboard/prediction/feature_extractor/object_history_sampler.h"

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type
#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <ostream>
#include <string>
#include <utility>

// IWYU pragma: no_include <google/protobuf/repeated_ptr_field.h>
#include "absl/status/status.h"
#include "absl/status/statusor.h"

#include "onboard/global/buffered_logger.h"
#include "onboard/global/car_common.h"
#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/math/fast_math.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/math/geometry/proto/box2d.pb.h"
#include "onboard/math/geometry/util.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/math/polynomial.h"
#include "onboard/math/polynomial_fitter.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/prediction/container/object_history.h"
#include "onboard/prediction/container/prediction_object.h"
#include "onboard/prediction/prediction_util.h"
#include "onboard/prediction/util/perception_util.h"
#include "onboard/proto/perception/fusion/object.pb.h"
#include "onboard/utils/elements_history.h"
#include "onboard/utils/map_util.h"

namespace qcraft {
namespace prediction {

namespace {
constexpr double kMinDynamicForPolyFit = 0.5;
constexpr int kPolyFitDegree = 3;
constexpr double kDecayFactor = 0.5;
constexpr double kConsistencyFactor = 0.5;
constexpr double kConsistencySpeedTolerance = 4.0;  // m/s.
ObjectMotionState AlignObjectMotionState(const ObjectMotionState& cur_state,
                                         double dt) {
  const Vec2d new_pos = cur_state.pos + cur_state.vel * dt;
  const double length = cur_state.bbox.length();
  const double width = cur_state.bbox.width();
  return ObjectMotionState{
      .timestamp = cur_state.timestamp + dt,
      .pos = new_pos,
      .heading = cur_state.heading,
      .vel = cur_state.vel,
      .bbox = Box2d(new_pos, cur_state.heading, length, width),
  };
}

int GetValidStateEndIndex(const ObjectProto& obj) {
  for (int i = 0; i < obj.trajectory_size() - 1; ++i) {
    const auto& cur_state_proto = obj.trajectory(i);
    const auto& next_state_proto = obj.trajectory(i + 1);
    const double cur_heading = cur_state_proto.heading();
    const double next_heading = next_state_proto.heading();
    if (std::abs(AngleDifference(cur_heading, next_heading)) >= M_PI_4) {
      return i;
    }
  }
  return obj.trajectory_size() - 1;
}

}  // namespace

std::vector<ObjectProto> ResampleObjectHistorySpan(
    const ObjectHistorySpan& obj_history, double current_ts, double time_step,
    int max_steps, bool enable_smoothing) {
  FUNC_QTRACE();
  const double last_t = obj_history.timestamp();
  const auto visit_hist =
      obj_history.GetHistoryFrom(current_ts - time_step * max_steps);
  std::vector<ObjectProto> vec_objects;
  std::vector<double> vec_ts;
  std::vector<Vec2d> vec_xt;
  std::vector<Vec2d> vec_yt;
  std::vector<double> weights;
  vec_objects.reserve(max_steps);
  vec_ts.reserve(max_steps);
  vec_xt.reserve(max_steps);
  vec_yt.reserve(max_steps);
  weights.reserve(max_steps);
  {
    SCOPED_QTRACE("ResampleObjectHistorySpan::GetObjectsInfo");
    for (int i = 0; i < visit_hist.size(); ++i) {
      const auto& cur_node = visit_hist[i];
      const auto& cur_obj = cur_node.val.object_proto();
      // NOTE:(zheng): If the object is partially observed, and the pos source
      // type has changed, the pos may be not consistent, we can clear the
      // history.
      if (i > 0) {
        const auto& prev_obj = visit_hist[i - 1].val.object_proto();
        if (prev_obj.has_observation_state() &&
            prev_obj.has_pos_source_type() && cur_obj.has_observation_state() &&
            cur_obj.has_pos_source_type()) {
          if ((cur_obj.observation_state() ==
                   ObservationState::OS_PARTIALLY_OBSERVED ||
               prev_obj.observation_state() ==
                   ObservationState::OS_PARTIALLY_OBSERVED) &&
              cur_obj.pos_source_type() != prev_obj.pos_source_type()) {
            vec_ts.clear();
            vec_xt.clear();
            vec_yt.clear();
            vec_objects.clear();
            weights.clear();
          }
        }
      }
      vec_ts.push_back(cur_node.time);
      const double rel_ts = cur_node.time - current_ts;
      const double final_ts_diff = last_t - cur_node.time;
      vec_xt.push_back(Vec2d(rel_ts, cur_node.val.pos().x()));
      vec_yt.push_back(Vec2d(rel_ts, cur_node.val.pos().y()));
      vec_objects.push_back(cur_node.val.object_proto());
      weights.push_back(std::pow(kDecayFactor, final_ts_diff));
    }
  }
  if (vec_objects.size() == 1) {
    ObjectProto object = obj_history.back().val.object_proto();
    const auto align_status = AlignPerceptionObjectTime(current_ts, &object);
    return {object};
  }
  // 1. Count dynamic state num, only smooth the result if most states are
  // dynamic.
  // 2. If object is static, then force the static states to be the same (avoid
  // position shift).
  int count_dynamic = 0;
  for (int i = vec_objects.size() - 2; i >= 0; --i) {
    auto& cur_obj = vec_objects[i];
    if (cur_obj.moving_state() == ObjectProto::MS_MOVING) {
      count_dynamic++;
    }
    const auto& next_obj = vec_objects[i + 1];
    if (cur_obj.moving_state() == ObjectProto::MS_STATIC &&
        next_obj.moving_state() == ObjectProto::MS_STATIC) {
      double cur_ts = cur_obj.timestamp();
      cur_obj = next_obj;
      cur_obj.set_timestamp(cur_ts);
    }
    vec_xt[i] = Vec2d(vec_xt[i].x(), cur_obj.pos().x());
    vec_yt[i] = Vec2d(vec_xt[i].x(), cur_obj.pos().y());
  }

  class ObjectProtoLerper {
   public:
    ObjectProto operator()(const ObjectProto& a, const ObjectProto& b,
                           double alpha) const {
      return LerpObjectProto(a, b, alpha);
    }
  };
  std::optional<Polynomial> x_polynomial, y_polynomial;
  // Ensure past traj stability
  const bool use_poly_fit =
      (vec_xt.size() > kPolyFitDegree) && (obj_history.id() != kAvObjectId) &&
      (count_dynamic / static_cast<double>(vec_objects.size()) >
       kMinDynamicForPolyFit) &&
      enable_smoothing;
  if (use_poly_fit) {
    x_polynomial = *FitPolynomialToData(kPolyFitDegree, vec_xt);
    y_polynomial = *FitPolynomialToData(kPolyFitDegree, vec_yt);
  }

  PiecewiseLinearFunction<ObjectProto, double, ObjectProtoLerper>
      object_proto_plf(vec_ts, vec_objects);
  std::vector<ObjectProto> raw_resampled_objects;
  // vec_ts.front() can be larger than current_ts, thus we must ensure we can at
  // least find one point.
  int valid_samples = std::max(static_cast<int>(std::floor(
                                   (current_ts - vec_ts.front()) / time_step)),
                               0) +
                      1;
  int num_step = std::min(max_steps, valid_samples);
  raw_resampled_objects.reserve(num_step);
  {
    SCOPED_QTRACE("ResampleObjectHistorySpan::RawResampledObjects");
    for (int i = 0; i < num_step; ++i) {
      const double ts = current_ts + (i - num_step + 1) * time_step;
      auto resampled_obj = object_proto_plf.EvaluateWithExtrapolation(ts);
      if (x_polynomial.has_value() && y_polynomial.has_value()) {
        Vec2dProto pos;
        pos.set_x(x_polynomial->Evaluate(ts - current_ts));
        pos.set_y(y_polynomial->Evaluate(ts - current_ts));
        *resampled_obj.mutable_pos() = pos;
      }
      raw_resampled_objects.push_back(std::move(resampled_obj));
    }
  }
  // Check consistency and remove bad history sequence.
  std::vector<ObjectProto> resampled_objects;
  resampled_objects.reserve(raw_resampled_objects.size());
  {
    SCOPED_QTRACE("ResampleObjectHistorySpan::CheckHistoryConsistency");
    for (int i = 0; i < raw_resampled_objects.size(); ++i) {
      if (i + 1 < raw_resampled_objects.size()) {
        const auto& next_obj = raw_resampled_objects[i + 1];
        double t_diff =
            next_obj.timestamp() - raw_resampled_objects[i].timestamp();
        if (!CheckHistoryConsistency(raw_resampled_objects[i], next_obj,
                                     t_diff)) {
          resampled_objects.clear();
          continue;
        }
      }
      resampled_objects.push_back(std::move(raw_resampled_objects[i]));
    }
  }
  return resampled_objects;
}

ObjectMotionState LerpObjectMotionState(const ObjectMotionState& a,
                                        const ObjectMotionState& b,
                                        double alpha) {
  const double lerped_ts = Lerp(a.timestamp, b.timestamp, alpha);
  const Vec2d lerped_pos = Lerp(a.pos, b.pos, alpha);
  const Vec2d lerped_vel = Lerp(a.vel, b.vel, alpha);
  const double lerped_yaw =
      NormalizeAngle(LerpAngle(a.heading, b.heading, alpha));
  const double length = b.bbox.length();
  const double width = b.bbox.width();

  return ObjectMotionState{
      .timestamp = lerped_ts,
      .pos = lerped_pos,
      .heading = lerped_yaw,
      .vel = lerped_vel,
      .bbox = Box2d(lerped_pos, lerped_yaw, length, width),
  };
}

absl::StatusOr<ObjectMotionHistory>
ResampleObjectMotionHistoryFromTrackerHistory(const ObjectProto& obj,
                                              double current_ts,
                                              double time_step, int max_steps) {
  FUNC_QTRACE();
  const int traj_size = obj.trajectory_size();
  std::vector<ObjectMotionState> raw_states;
  raw_states.reserve(traj_size);
  std::vector<double> vec_ts;
  vec_ts.reserve(traj_size);
  const double length = obj.bounding_box().length();
  const double width = obj.bounding_box().width();

  // If unknown static, return current state.
  if (obj.type() == OT_UNKNOWN_STATIC) {
    ObjectMotionState cur_state = {
        .timestamp = current_ts,
        .pos = Vec2dFromProto(obj.pos()),
        .heading = NormalizeAngle(obj.yaw()),
        .vel = Vec2d(0.0, 0.0),
        .bbox = Box2d(obj.bounding_box()),
    };
    return ObjectMotionHistory{
        .id = obj.id(),
        .type = obj.type(),
        .states = {cur_state},
    };
  }
  const int valid_state_end_index = GetValidStateEndIndex(obj);
  for (int i = valid_state_end_index; i >= 0; --i) {
    const auto& state_proto = obj.trajectory(i);
    const auto pos = Vec2dFromProto(state_proto.pos());
    const auto heading = state_proto.heading();
    ObjectMotionState cur_state = {
        .timestamp = state_proto.timestamp(),
        .pos = pos,
        .heading = heading,
        .vel = Vec2d(
            state_proto.velocity() * fast_math::Cos(state_proto.heading()),
            state_proto.velocity() * fast_math::Sin(state_proto.heading())),
        .bbox = Box2d(pos, heading, length, width),
    };
    // Ensure the strict time ordering of motion state and do not add two motion
    // states with timestamps too close to each other, which might lead to
    // extrapolation error.
    constexpr double kTrackerHistoryMinTimestep = 25e-3;  // s.
    if (!vec_ts.empty() &&
        (cur_state.timestamp - vec_ts.back()) < kTrackerHistoryMinTimestep) {
      continue;
    }
    vec_ts.push_back(cur_state.timestamp);
    raw_states.push_back(std::move(cur_state));
  }
  if (raw_states.size() == 0) {
    return absl::NotFoundError("No tracker history available");
  }
  if (raw_states.size() == 1) {
    const auto& prev_state = raw_states.back();
    raw_states = {
        AlignObjectMotionState(prev_state, current_ts - prev_state.timestamp)};
    return ObjectMotionHistory{
        .id = obj.id(),
        .type = obj.type(),
        .states = std::move(raw_states),
    };
  }

  class ObjectMotionStateLerper {
   public:
    ObjectMotionState operator()(const ObjectMotionState& a,
                                 const ObjectMotionState& b,
                                 double alpha) const {
      return LerpObjectMotionState(a, b, alpha);
    }
  };
  PiecewiseLinearFunction<ObjectMotionState, double, ObjectMotionStateLerper>
      object_plf(vec_ts, raw_states);
  ObjectMotionHistory resampled_history;
  resampled_history.id = obj.id();
  resampled_history.type = obj.type();
  resampled_history.states.reserve(max_steps);

  for (int i = 0; i < max_steps; ++i) {
    const double ts = current_ts - i * time_step;
    // Stop resampling if reaching end of history.
    if (ts < vec_ts.front() && i != 0) {
      break;
    }
    auto resample_state = object_plf.EvaluateWithExtrapolation(ts);
    resampled_history.states.push_back(std::move(resample_state));
  }
  std::reverse(resampled_history.states.begin(),
               resampled_history.states.end());
  return resampled_history;
}

ObjectMotionHistory ResampleObjectHistorySpanToMotionHistory(
    const ObjectHistorySpan& obj_history, double current_ts, double time_step,
    int max_steps, bool enable_smoothing) {
  const double length = obj_history.bounding_box().length();
  const double width = obj_history.bounding_box().width();

  const auto resampled_objs = ResampleObjectHistorySpan(
      obj_history, current_ts, time_step, max_steps, enable_smoothing);
  ObjectMotionHistory history;
  history.id = obj_history.id();
  history.type = obj_history.type();
  history.states.reserve(resampled_objs.size());
  for (const auto& obj : resampled_objs) {
    const auto pos = Vec2dFromProto(obj.pos());
    const auto yaw = obj.yaw();
    history.states.push_back(
        ObjectMotionState{.timestamp = obj.timestamp(),
                          .pos = pos,
                          .heading = yaw,
                          .vel = Vec2dFromProto(obj.vel()),
                          .bbox = Box2d(pos, yaw, length, width)});
  }
  return history;
}

bool CheckHistoryConsistency(const ObjectProto& obj,
                             const ObjectProto& next_obj, double ts) {
  const auto dpos = Vec2dFromProto(next_obj.pos()) - Vec2dFromProto(obj.pos());
  const auto dv_by_pos_diff = dpos / ts;
  const auto vel = Vec2dFromProto(obj.vel());
  const double vx_lb = vel.x() * (1 - kConsistencyFactor);
  const double vx_ub = vel.x() * (1 + kConsistencyFactor);
  const double vy_lb = vel.y() * (1 - kConsistencyFactor);
  const double vy_ub = vel.y() * (1 + kConsistencyFactor);
  const double min_vx =
      std::min(vel.x() - kConsistencySpeedTolerance, std::min(vx_lb, vx_ub));
  const double max_vx =
      std::max(vel.x() + kConsistencySpeedTolerance, std::max(vx_lb, vx_ub));
  const double min_vy =
      std::min(vel.y() - kConsistencySpeedTolerance, std::min(vy_lb, vy_ub));
  const double max_vy =
      std::max(vel.y() + kConsistencySpeedTolerance, std::max(vy_lb, vy_ub));
  if (dv_by_pos_diff.x() < min_vx || dv_by_pos_diff.x() > max_vx) {
    return false;
  }
  if (dv_by_pos_diff.y() < min_vy || dv_by_pos_diff.y() > max_vy) {
    return false;
  }
  return true;
}

ObjectHistorySampler::ObjectHistorySampler(
    absl::Span<const ObjectHistory* const> objs_to_predict,
    const ObjectHistory& av_history, double current_time, double time_step,
    int max_steps, bool enable_smoothing, bool use_tracker_history)
    : current_time_(current_time),
      time_step_(time_step),
      max_steps_(max_steps) {
  FUNC_QTRACE();
  resampled_histories_[kAvObjectId] = std::make_unique<ObjectMotionHistory>(
      ResampleObjectHistorySpanToMotionHistory(av_history.GetHistory(),
                                               current_time_, time_step_,
                                               max_steps_, enable_smoothing));
  if (use_tracker_history) {
    SCOPED_QTRACE("ObjectHistorySampler::UseTrackerHistory");
    if (IsOnboardMode() && !objs_to_predict.empty()) {
      const auto& check_obj = objs_to_predict[0]->object_proto();
      QCHECK(!check_obj.trajectory().empty())
          << "Tracker trajectory can't be empty.";
    }

    for (const auto& object_history : objs_to_predict) {
      auto obj_id = object_history->id();
      const auto& obj = object_history->object_proto();
      auto motion_hist_or = ResampleObjectMotionHistoryFromTrackerHistory(
          obj, current_time_, time_step_, max_steps_);
      if (motion_hist_or.ok()) {
        resampled_histories_[obj_id] =
            std::make_unique<ObjectMotionHistory>(motion_hist_or.value());
      } else {
        // If we do not have tracker history, use cached object history.
        const auto hist = object_history->GetHistoryFrom(
            current_time_ - time_step_ * (max_steps_ + 1));
        resampled_histories_[obj_id] = std::make_unique<ObjectMotionHistory>(
            ResampleObjectHistorySpanToMotionHistory(
                hist, current_time_, time_step_, max_steps_, enable_smoothing));
      }
    }
  } else {
    SCOPED_QTRACE(
        "ObjectHistorySampler::ResampleObjectsHistorySpanToMotionHistory");
    for (const auto& object_history : objs_to_predict) {
      auto obj_id = object_history->id();
      const auto hist = object_history->GetHistoryFrom(
          current_time_ - time_step_ * (max_steps_ + 1));
      resampled_histories_[obj_id] = std::make_unique<ObjectMotionHistory>(
          ResampleObjectHistorySpanToMotionHistory(
              hist, current_time_, time_step_, max_steps_, enable_smoothing));
    }
  }
}

const ObjectMotionHistory*
ObjectHistorySampler::GetResampledMotionHistoryPtrById(
    const ObjectIDType& obj_id) const {
  auto* hist_ptr = FindOrNull(resampled_histories_, obj_id);
  return hist_ptr == nullptr ? nullptr : (*hist_ptr).get();
}

const ObjectMotionHistory& ObjectHistorySampler::GetResampledMotionHistoryById(
    const ObjectIDType& obj_id) const {
  auto* resampled_hist = FindOrNull(resampled_histories_, obj_id);
  QCHECK_NOTNULL(resampled_hist);
  return *(*resampled_hist);
}

std::vector<const ObjectMotionHistory*>
ObjectHistorySampler::GetResampledMotionHistoryWithAVInBox2d(
    const ObjectIDType& agent_id, const Box2d& region_box,
    int num_other_objs) const {
  // No object is needed.
  if (num_other_objs == 0) {
    return std::vector<const ObjectMotionHistory*>();
  }
  // Always return AV as the first history.
  if (num_other_objs == 1) {
    return {resampled_histories_.at(kAvObjectId).get()};
  }

  const auto& box_center = region_box.center();

  struct DistanceInfo {
    const ObjectMotionHistory* obj_resampled;
    double dist = std::numeric_limits<double>::max();
  };
  std::vector<DistanceInfo> dist_infos;
  dist_infos.reserve(resampled_histories_.size());
  for (const auto& [obj_id, resampled_hist] : resampled_histories_) {
    // Cannot be AV ID
    if (obj_id == kAvObjectId) {
      continue;
    }
    if (obj_id == agent_id) {
      continue;
    }

    if (resampled_hist->type == OT_FOD ||
        resampled_hist->type == OT_VEGETATION ||
        resampled_hist->type == OT_BARRIER || resampled_hist->type == OT_CONE ||
        resampled_hist->type == OT_WARNING_TRIANGLE) {
      continue;
    }

    if (resampled_hist->states.size() == 0) {
      continue;
    }
    dist_infos.push_back({.obj_resampled = resampled_hist.get(),
                          .dist = box_center.DistanceSquareTo(
                              Vec2d(resampled_hist->states.back().pos))});
  }
  // Get distance to agent information.
  std::sort(dist_infos.begin(), dist_infos.end(),
            [](const DistanceInfo& a, const DistanceInfo& b) {
              return a.dist < b.dist;
            });

  std::vector<const ObjectMotionHistory*> objects_in_box;
  objects_in_box.reserve(num_other_objs);
  // Put AV inside
  objects_in_box.push_back(resampled_histories_.at(kAvObjectId).get());
  for (const auto& dist_info : dist_infos) {
    // Put N-1 context objects.
    if (objects_in_box.size() > num_other_objs - 1) {
      break;
    }
    const auto& obj_pos = dist_info.obj_resampled->states.back().pos;
    if (!region_box.IsPointIn(obj_pos)) {
      continue;
    }
    objects_in_box.push_back(dist_info.obj_resampled);
  }
  return objects_in_box;
}

}  // namespace prediction
}  // namespace qcraft
