#include "onboard/planner/ml/context_feature_extractor/object_history_sampler.h"

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type
// IWYU pragma: no_include <google/protobuf/repeated_ptr_field.h>

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <memory>
#include <optional>
#include <ostream>
#include <string>
#include <utility>

#include "absl/status/status.h"
#include "absl/strings/str_format.h"
#include "glog/logging.h"

#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/math/fast_math.h"
#include "onboard/math/geometry/proto/box2d.pb.h"
#include "onboard/math/geometry/util.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/object/object_vector.h"
#include "onboard/planner/object/planner_object.h"
#include "onboard/planner/object/spacetime_object_trajectory.h"
#include "onboard/prediction/feature_extractor/object_history_sampler.h"
#include "onboard/prediction/prediction_defs.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/utils/map_util.h"

namespace qcraft {
namespace planner {
namespace ml {
namespace {
using ObjectMotionState = prediction::ObjectMotionState;
using ObjectMotionHistory = prediction::ObjectMotionHistory;
absl::StatusOr<ObjectMotionHistory> FillMissingTrackingWithCurrentState(
    const ObjectProto& obj, double current_ts) {
  const auto dt = current_ts - obj.timestamp();
  const auto vel = Vec2dFromProto(obj.vel());
  const auto new_pos = Vec2dFromProto(obj.pos()) + vel * dt;
  const auto heading = NormalizeAngle(obj.yaw());
  ObjectMotionState cur_state = {
      .timestamp = current_ts,
      .pos = new_pos,
      .heading = heading,
      .vel = vel,
      .bbox = Box2d(new_pos, heading, obj.bounding_box().length(),
                    obj.bounding_box().width()),
  };
  return ObjectMotionHistory{
      .id = obj.id(),
      .type = obj.type(),
      .states = {cur_state},
  };
}

constexpr double kInconsistencyFactor = 3.0;
constexpr double kInconsistencySpeedDiff = 5.0;
constexpr double kMaxPossibleSpeed = 69.4;  // m/s.
constexpr double kMaxPossibleAcc = 9.8;     // m/s^2.

std::optional<ObjectMotionHistory> TruncateInvalidTracking(
    const ObjectMotionHistory& history) {
  const auto& states = history.states;
  if (states.size() <= 1) return std::nullopt;
  int truncate_idx = -1;
  for (int i = states.size() - 1; i >= 1; --i) {
    const auto& cur_obj = states[i];
    const auto& prev_obj = states[i - 1];
    // Add a non-smoothness (velocity inconsistency) checker.
    const double delta_t = cur_obj.timestamp - prev_obj.timestamp;
    const double dis = cur_obj.pos.DistanceTo(prev_obj.pos);
    const double pseudo_speed = std::fabs(dis / delta_t);
    const double speed_diff =
        std::fabs(cur_obj.vel.Length() - prev_obj.vel.Length());
    const double pseudo_acc = std::fabs(speed_diff / delta_t);
    if ((pseudo_speed >
             std::fabs(cur_obj.vel.Length()) * kInconsistencyFactor &&
         std::fabs(pseudo_speed - std::fabs(cur_obj.vel.Length())) >
             kInconsistencySpeedDiff) ||
        pseudo_speed > kMaxPossibleSpeed ||
        cur_obj.vel.Length() > kMaxPossibleSpeed ||
        pseudo_acc > kMaxPossibleAcc) {
      truncate_idx = i;
      break;
    }
  }
  if (truncate_idx == -1) return std::nullopt;
  std::vector<ObjectMotionState> truncated;
  std::copy(std::next(states.begin(), truncate_idx), states.end(),
            std::back_inserter(truncated));
  return ObjectMotionHistory{
      .id = history.id, .type = history.type, .states = std::move(truncated)};
}

}  // namespace

ObjectHistorySampler::ObjectHistorySampler(
    const PlannerObjectManager& objs_mgr,
    const prediction::ObjectHistory& av_history, double current_time,
    double time_step, int max_steps, bool enable_smoothing)
    : current_time_(current_time),
      time_step_(time_step),
      max_steps_(max_steps) {
  SCOPED_QTRACE("ObjectHistorySampler::UseTrackerHistory");
  resampled_histories_[prediction::kAvObjectId] =
      std::make_unique<ObjectMotionHistory>(
          prediction::ResampleObjectHistorySpanToMotionHistory(
              av_history.GetHistory(), current_time_, time_step_, max_steps_,
              enable_smoothing));
  // TODO(Jinyun): Parallelize it.
  for (const auto& object : objs_mgr.planner_objects()) {
    const auto& obj = object.object_proto();
    auto motion_hist_or =
        prediction::ResampleObjectMotionHistoryFromTrackerHistory(
            obj, current_time_, time_step_, max_steps_);
    if (!motion_hist_or.ok()) {
      motion_hist_or = FillMissingTrackingWithCurrentState(obj, current_time_);
    }
    auto truncated_motion_hist = TruncateInvalidTracking(*motion_hist_or);
    resampled_histories_[obj.id()] = std::make_unique<ObjectMotionHistory>(
        truncated_motion_hist.has_value()
            ? std::move(truncated_motion_hist).value()
            : std::move(motion_hist_or).value());
  }
}

ObjectHistorySampler::ObjectHistorySampler(
    const std::shared_ptr<const ObjectsProto>& real_objects,
    const std::shared_ptr<const ObjectsProto>& virtual_objects,
    const prediction::ObjectHistory& av_history, double current_time,
    double time_step, int max_steps, bool enable_smoothing)
    : current_time_(current_time),
      time_step_(time_step),
      max_steps_(max_steps) {
  SCOPED_QTRACE("ObjectHistorySampler::UseTrackerHistory");
  resampled_histories_[prediction::kAvObjectId] =
      std::make_unique<ObjectMotionHistory>(
          prediction::ResampleObjectHistorySpanToMotionHistory(
              av_history.GetHistory(), current_time_, time_step_, max_steps_,
              enable_smoothing));
  // TODO(Jinyun): Parallelize it.
  if (real_objects != nullptr) {
    for (const auto& obj : real_objects->objects()) {
      auto motion_hist_or =
          prediction::ResampleObjectMotionHistoryFromTrackerHistory(
              obj, current_time_, time_step_, max_steps_);
      if (!motion_hist_or.ok()) {
        motion_hist_or =
            FillMissingTrackingWithCurrentState(obj, current_time_);
      }
      auto truncated_motion_hist =
          TruncateInvalidTracking(motion_hist_or.value());
      resampled_histories_[obj.id()] = std::make_unique<ObjectMotionHistory>(
          truncated_motion_hist.has_value() ? *truncated_motion_hist
                                            : motion_hist_or.value());
    }
  }

  // Note(Jinyun): For simulation environment.
  if (virtual_objects != nullptr) {
    for (const auto& obj : virtual_objects->objects()) {
      auto motion_hist_or =
          prediction::ResampleObjectMotionHistoryFromTrackerHistory(
              obj, current_time_, time_step_, max_steps_);
      if (!motion_hist_or.ok()) {
        motion_hist_or =
            FillMissingTrackingWithCurrentState(obj, current_time_);
      }
      auto truncated_motion_hist =
          TruncateInvalidTracking(motion_hist_or.value());
      resampled_histories_[obj.id()] = std::make_unique<ObjectMotionHistory>(
          truncated_motion_hist.has_value() ? *truncated_motion_hist
                                            : motion_hist_or.value());
    }
  }
}

ObjectHistorySampler::ObjectHistorySampler(
    const std::map<std::string, std::vector<int>>& trajs_to_sample,
    const ObjectsProto* real_objects, const ObjectsProto* virtual_objects,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const prediction::ObjectHistory& av_history, double predicted_plan_time,
    double time_step, int max_steps)
    : current_time_(predicted_plan_time),
      time_step_(time_step),
      max_steps_(max_steps) {
  SCOPED_QTRACE("ObjectHistorySampler::OnlyUseTrackerHistoryWithLookAheadTime");
  // Sample av history.
  // TODO(changqing): check whether to sample av history at predicted time.
  resampled_histories_[prediction::kAvObjectId] =
      std::make_unique<ObjectMotionHistory>(
          prediction::ResampleObjectHistorySpanToMotionHistory(
              av_history.GetHistory(), current_time_, time_step_, max_steps_,
              /*enable_smoothing=*/false));

  std::vector<const ObjectsProto*> protos;
  if (real_objects != nullptr) {
    protos.push_back(real_objects);
  }
  if (virtual_objects != nullptr) {
    protos.push_back(virtual_objects);
  }

  for (const auto* proto_ptr : protos) {
    const auto& objects_proto = *proto_ptr;
    for (const auto& object_proto : objects_proto.objects()) {
      const auto& id = object_proto.id();
      const auto* idxes_ptr = FindOrNull(trajs_to_sample, id);
      if (idxes_ptr == nullptr) {
        continue;
      }
      for (const auto& idx : *idxes_ptr) {
        const auto traj_id =
            SpacetimeObjectTrajectory::MakeTrajectoryId(id, idx);
        const auto* st_traj_ptr = st_traj_mgr.FindTrajectoryById(traj_id);
        if (st_traj_ptr == nullptr) continue;
        const auto& predicted_trajectory = st_traj_ptr->trajectory();
        const auto& planner_object = st_traj_ptr->planner_object();
        auto motion_hist_or =
            ResampleObjectMotionHistoryFromTrackerHistoryWithLookAhead(
                object_proto, predicted_trajectory,
                planner_object.proto_timestamp(), predicted_plan_time,
                time_step, max_steps);
        if (!motion_hist_or.ok()) {
          continue;
        }
        resampled_histories_[traj_id] =
            std::make_unique<ObjectMotionHistory>(*motion_hist_or);
      }
    }
  }
}

const ObjectMotionHistory&
ObjectHistorySampler::GetResampledMotionHistoryByObjectId(
    const prediction::ObjectIDType& obj_id) const {
  auto* resampled_hist = FindOrNull(resampled_histories_, obj_id);
  QCHECK_NOTNULL(resampled_hist);
  return *(*resampled_hist);
}

std::vector<const ObjectMotionHistory*>
ObjectHistorySampler::GetResampledMotionHistoryInBox2d(
    const prediction::ObjectIDType& agent_id, const Box2d& region_box,
    int num_objs) const {
  // No object is needed.
  if (num_objs == 0) {
    return std::vector<const ObjectMotionHistory*>();
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
    if (obj_id == prediction::kAvObjectId) {
      continue;
    }
    if (obj_id == agent_id) {
      continue;
    }
    if (resampled_hist->states.size() == 0) {
      LOG(INFO) << "resampled_hist->states.size() == 0";
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
  objects_in_box.reserve(num_objs);
  for (const auto& dist_info : dist_infos) {
    // Put N-1 context objects.
    if (objects_in_box.size() == num_objs) {
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

absl::StatusOr<ObjectMotionHistory>
ResampleObjectMotionHistoryFromTrackerHistoryWithLookAhead(
    const ObjectProto& obj,
    const prediction::PredictedTrajectory& predicted_trajectory,
    double pred_traj_ts, double predicted_plan_time, double time_step,
    int max_steps) {
  FUNC_QTRACE();
  // obj proto timestamp(from perception) might be different from predicted
  // trajectory timestamp since they are from planner module and prediction
  // module.

  int future_start_idx = 0;
  const auto& pred_pts = predicted_trajectory.points();
  for (int i = 0; i < pred_pts.size(); ++i) {
    const auto absl_pred_pt_time = pred_traj_ts + pred_pts[i].t();
    if (absl_pred_pt_time > predicted_plan_time) {
      // Current predicted point is in planned future.
      future_start_idx = i;
      break;
    }
  }
  VLOG(3) << obj.id() << "-" << predicted_trajectory.index()
          << absl::StrFormat(
                 "pred_traj_ts: %.6f, predicted_plan_time: %.6f, future start "
                 "index: "
                 "%d.",
                 pred_traj_ts, predicted_plan_time, future_start_idx);

  // Add tracker history & predicted point before future start idx to raw
  // states to resample.
  const auto& tracker_hist = obj.trajectory();
  std::vector<ObjectMotionState>
      raw_states;  // Arranged from older states to newer states.
  raw_states.reserve(tracker_hist.size() + future_start_idx);
  std::vector<double> vec_ts;
  vec_ts.reserve(tracker_hist.size() + future_start_idx);

  // Add predicted point as ObjectMotionState. (newer).
  const double length = obj.bounding_box().length();
  const double width = obj.bounding_box().width();

  // Add tracker history. (older).
  for (int i = tracker_hist.size() - 1; i >= 0; --i) {
    const auto& state_proto = obj.trajectory(i);
    const auto pos = Vec2dFromProto(state_proto.pos());
    const auto heading = state_proto.heading();
    double heading_cos_sin[2];
    fast_math::CosAndSin<7>(state_proto.heading(), heading_cos_sin);
    ObjectMotionState cur_state = {
        .timestamp = state_proto.timestamp(),
        .pos = pos,
        .heading = heading,
        .vel = Vec2d(state_proto.velocity() * heading_cos_sin[0],
                     state_proto.velocity() * heading_cos_sin[1]),
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

  for (int i = future_start_idx - 1; i >= 0; --i) {
    const auto& pred_pt = pred_pts[i];
    double theta_cos_sin[2];
    fast_math::CosAndSin<7>(pred_pt.theta(), theta_cos_sin);
    ObjectMotionState cur_state = {
        .timestamp = pred_traj_ts + pred_pt.t(),
        .pos = pred_pt.pos(),
        .heading = pred_pt.theta(),
        .vel = Vec2d(pred_pt.v() * theta_cos_sin[0],
                     pred_pt.v() * theta_cos_sin[1]),
        .bbox = Box2d(pred_pt.pos(), pred_pt.theta(), length, width),
    };
    if (!vec_ts.empty() && cur_state.timestamp <= vec_ts.back()) {
      continue;
    }
    vec_ts.push_back(cur_state.timestamp);
    raw_states.push_back(std::move(cur_state));
  }

  if (raw_states.size() == 0) {
    return absl::NotFoundError("No tracker history available");
  }
  if (raw_states.size() == 1) {
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
    const double ts = predicted_plan_time - i * time_step;
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

}  // namespace ml
}  // namespace planner
}  // namespace qcraft
