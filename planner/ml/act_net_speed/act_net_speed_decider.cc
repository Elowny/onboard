#include "onboard/planner/ml/act_net_speed/act_net_speed_decider.h"

// IWYU pragma: no_include <google/protobuf/repeated_ptr_field.h>

#include <algorithm>
#include <deque>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <ostream>
#include <set>
#include <string>
#include <string_view>
#include <utility>

#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "absl/types/span.h"
#include "gflags/gflags.h"
#include "glog/logging.h"

#include "onboard/eval/qevent.h"
#include "onboard/eval/qevent_base.h"
#include "onboard/global/clock.h"
#include "onboard/global/logging.h"
#include "onboard/lite/check.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/math/geometry/segment2d.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/ml/context_feature_extractor/object_history_sampler.h"
#include "onboard/planner/object/spacetime_object_state.h"
#include "onboard/planner/object/spacetime_object_trajectory.h"
#include "onboard/planner/speed/overlap_info.h"
#include "onboard/planner/speed/proto/speed_finder.pb.h"
#include "onboard/planner/speed/st_boundary.h"
#include "onboard/planner/util/vehicle_geometry_util.h"
#include "onboard/prediction/container/object_history.h"
#include "onboard/prediction/feature_extractor/act_net_speed_feature.h"
#include "onboard/prediction/feature_extractor/act_net_speed_feature_extractor.h"
#include "onboard/prediction/inferencer/act_net_speed_inferencer.h"
#include "onboard/prediction/predicted_trajectory.h"
#include "onboard/prediction/prediction_defs.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/utils/map_util.h"
#include "onboard/utils/time_util.h"

namespace qcraft {
namespace planner {
namespace {

using ml::ObjectHistorySampler;
using planner::SpacetimeObjectTrajectory;
using prediction::ActNetSpeedFeature;
using prediction::ActNetSpeedPlannerTarget;
using prediction::kActNetSpeedConfig;
using prediction::ObjectHistory;
using prediction::ObjectIDType;
using prediction::ObjectMotionHistory;
using prediction::ObjectMotionState;
using TargetsMap =
    std::map<ObjectIDType, std::vector<ActNetSpeedPlannerTarget>>;

// Min number of poitnts to sample from lowest_idx to highest_idx.
constexpr int kAVPathPointMinSampleNum = 5;
constexpr double kAVPathPointSampleStep = 0.2;  // m.

// Decision uncertainty: only when yield prob > 0.6 to follow. Otherwise
// UNKNOWN.
constexpr double kYieldOverPassConfidentRatio = 1.5;
const double kPassOverYieldConfidentRatio = 1.0 / kYieldOverPassConfidentRatio;

DEFINE_int32(planner_speed_use_act_net_speed_decision_level, 0,
             "Whether to replace speed's pre decider decision with act net "
             "speed decision with level: 0 (do not replace), 1 (replace with "
             "uncertainty decision), 2 (replace with absolute decision)");

enum class ActNetSpeedDecisionLevel {
  kNoDecision = 0,
  kUncertainDecision = 1,  // Check yield/pass ratio threshold.
  kAbsoluteDecision = 2,
};

inline bool AreDifferentSpeedDecisions(
    const StBoundaryProto::DecisionType& planner,
    const StBoundaryProto::DecisionType& model) {
  const auto planner_decision =
      planner == StBoundaryProto::IGNORE ? StBoundaryProto::LEAD : planner;
  return planner_decision != model;
}

inline bool HasValidDecisionProb(const StBoundaryProto::DecisionProb& prob) {
  constexpr double kValidProbSumEpsilon = 1e-1;
  return (prob.has_yield() && prob.has_pass() &&
          (prob.yield() + prob.pass() > kValidProbSumEpsilon));
}

inline StBoundaryProto::DecisionType DecisionProbToDecisionType(
    const StBoundaryProto::DecisionProb& prob) {
  if (prob.yield() > prob.pass()) {
    return StBoundaryProto::FOLLOW;
  }
  if (prob.yield() < prob.pass()) {
    return StBoundaryProto::LEAD;
  }
  return StBoundaryProto::UNKNOWN;
}

inline StBoundaryProto::DecisionType ToSpeedDecisionTypeWithUncertainty(
    prediction::AVObjectRelation relation_prob) {
  const auto ratio = relation_prob.yield / relation_prob.pass;
  if (ratio > kYieldOverPassConfidentRatio) return StBoundaryProto::FOLLOW;
  if (ratio < kPassOverYieldConfidentRatio) return StBoundaryProto::LEAD;
  // Don't make decision if not confident enough.
  return StBoundaryProto::UNKNOWN;
}

inline StBoundaryProto::DecisionType ToSpeedDecisionType(
    prediction::AVObjectRelation relation_prob) {
  return relation_prob.yield >= relation_prob.pass ? StBoundaryProto::FOLLOW
                                                   : StBoundaryProto::LEAD;
}

// Use overlap info from st graph directly to determine interaction point.
std::optional<ActNetSpeedPlannerTarget> ConstructTargetFromStBoundary(
    const DiscretizedPath& av_path, const SpacetimeObjectTrajectory& st_traj,
    const std::vector<OverlapInfo>& overlap_infos,
    const VehicleGeometryParamsProto& vehicle_geom) {
  if (overlap_infos.empty()) return std::nullopt;
  const auto& states = st_traj.states();
  for (const auto& overlap : overlap_infos) {
    const auto& obj_state = states[overlap.obj_idx];
    const auto& obj_pred_pt = *obj_state.traj_point;
    const auto& av_start_point = av_path[overlap.av_start_idx];

    const auto av_pos = Vec2d(av_start_point.x(), av_start_point.y());
    const auto av_box =
        ComputeAvBox(av_pos, av_start_point.theta(), vehicle_geom);
    const auto& obj_box = obj_state.box;
    if (obj_box.HasOverlap(av_box)) {
      double obj_s = 0.0;
      for (int i = 1; i < overlap.obj_idx; ++i) {
        const auto& prev_pos = states[i - 1].box.center();
        const auto& pos = states[i].box.center();
        obj_s += prev_pos.DistanceTo(pos);
      }
      const auto target = ActNetSpeedPlannerTarget({
          .object_id = std::string(st_traj.object_id()),
          .st_traj = &st_traj,
          .av_s = static_cast<double>(av_start_point.s()),
          .object_s = obj_s,
      });
      VLOG(3) << absl::StrFormat(
          "Found target: object %s, traj id %s, av_s at %.4f, object_s at "
          "%.4f. Object predicted point s at %.4f",
          target.object_id, target.st_traj->traj_id(), target.av_s,
          target.object_s, obj_pred_pt.s());
      return target;
    }
  }
  return std::nullopt;
}

// Finer search for interaction point.
[[maybe_unused]] std::optional<ActNetSpeedPlannerTarget>
ConstructTargetFromStBoundaryWithSearch(
    const DiscretizedPath& av_path, const SpacetimeObjectTrajectory& st_traj,
    const std::vector<OverlapInfo>& overlap_infos,
    const VehicleGeometryParamsProto& /*vehicle_geom*/) {
  double closest_dist = std::numeric_limits<double>::infinity();
  const auto& states = st_traj.states();
  int obj_closest_idx = states.size();
  int av_closest_s = av_path.length();
  Vec2d av_closest_pt = Vec2d::Zero();
  bool found_intersection = false;
  for (int idx = 0; idx < overlap_infos.size(); ++idx) {
    const auto& overlap_info = overlap_infos[idx];
    if (overlap_info.obj_idx == states.size() - 1) {
      continue;
    }
    const auto& obj_state = states[overlap_info.obj_idx];
    const auto& obj_next_state = states[overlap_info.obj_idx + 1];
    const auto& obj_pred_pt = *(obj_state.traj_point);  // Pred traj point.
    const auto& obj_pred_next_pt = *(obj_next_state.traj_point);
    const Segment2d obj_seg(obj_pred_pt.pos(), obj_pred_next_pt.pos());

    const auto& av_start_pt = av_path[overlap_info.av_start_idx];
    const auto& av_end_pt = av_path[overlap_info.av_end_idx];
    const int sample_path_num = std::max<int>(
        kAVPathPointMinSampleNum,
        FloorToInt((av_end_pt.s() - av_start_pt.s()) / kAVPathPointSampleStep));
    const double ds = (av_end_pt.s() - av_start_pt.s()) / (sample_path_num - 1);
    double evaluate_s = av_start_pt.s();
    for (int i = 0; i < sample_path_num; ++i) {
      const auto av_pt =
          av_path.Evaluate(std::min<double>(av_path.length(), evaluate_s));
      const auto av_pos = Vec2d(av_pt.x(), av_pt.y());
      const auto distance = obj_seg.DistanceTo(av_pos);
      if (distance < std::max<double>(0.5 * ds, 0.5 * obj_seg.length())) {
        // Object segment intersect with av path segment. (av path seg
        // estimated, not constructed).
        closest_dist = distance;
        obj_closest_idx = overlap_info.obj_idx;
        av_closest_s = av_pt.s();
        av_closest_pt = av_pos;
        found_intersection = true;
      }
      evaluate_s += ds;
    }
    if (found_intersection) {
      VLOG(3) << st_traj.traj_id() << ": closest dist: " << closest_dist
              << " obj_idx " << obj_closest_idx << " av_s: " << av_closest_s;
      break;
    }
  }
  if (!found_intersection) {
    return std::nullopt;
  }
  if (overlap_infos.empty()) return std::nullopt;

  const auto& first_overlap = overlap_infos.front();
  const auto& obj_closest_state = states[first_overlap.obj_idx];
  const auto& obj_closest_pred_pt = *obj_closest_state.traj_point;
  const auto& av_pt = av_path[first_overlap.av_start_idx];

  double obj_s = 0.0;
  for (int i = 1; i < first_overlap.obj_idx; ++i) {
    const auto& prev_pos = states[i - 1].box.center();
    const auto& pos = states[i].box.center();
    obj_s += prev_pos.DistanceTo(pos);
  }

  VLOG(3) << "use av_s:" << static_cast<double>(av_pt.s()) << " object s"
          << obj_s << " pred point s: " << obj_closest_pred_pt.s();

  return ActNetSpeedPlannerTarget({
      .object_id = std::string(st_traj.object_id()),
      .st_traj = &st_traj,
      .av_s = static_cast<double>(av_pt.s()),
      .object_s = obj_s,
  });
}

struct StBoundInfo {
  std::string traj_id;
  double probability;
  const StBoundary* raw_st_bound = nullptr;
};

TargetsMap ConstructTargetsFromStBoundaries(
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const DiscretizedPath& av_path,
    const std::vector<StBoundaryWithDecision>& st_bounds_with_decision,
    const VehicleGeometryParamsProto& vehicle_geom) {
  TargetsMap targets_map;

  std::map<std::string, std::deque<StBoundInfo>> obj_trajs;
  for (const auto& bound_wd : st_bounds_with_decision) {
    const auto* raw_st_bound = bound_wd.raw_st_boundary();
    if (raw_st_bound == nullptr) continue;
    // Ignore boundaries with source other than object.
    if (raw_st_bound->source_type() != StBoundarySourceTypeProto::ST_OBJECT) {
      continue;
    }
    const auto& traj_id_or = bound_wd.traj_id();
    if (!traj_id_or.has_value()) continue;
    const auto object_id =
        SpacetimeObjectTrajectory::GetObjectIdFromTrajectoryId(*traj_id_or);
    const auto* st_traj = st_traj_mgr.FindTrajectoryById(*traj_id_or);
    if (st_traj == nullptr) continue;
    const auto& pred_traj = st_traj->trajectory();
    obj_trajs[object_id].push_back(StBoundInfo{
        .traj_id = *traj_id_or,
        .probability = pred_traj.probability(),
        .raw_st_bound = raw_st_bound,
    });
  }

  // Order target trajectories for each object from the most probable to the
  // least. Decide for each object the most probable one first.
  int count = 0;
  for (auto& [obj_id, traj_probs] : obj_trajs) {
    std::stable_sort(traj_probs.begin(), traj_probs.end(),
                     [](const StBoundInfo& t1, const StBoundInfo& t2) {
                       return t1.probability > t2.probability;
                     });
    count += traj_probs.size();
  }

  std::vector<StBoundInfo> targets;
  targets.reserve(count);
  while (targets.size() != count) {
    for (auto& [obj_id, traj_probs] : obj_trajs) {
      if (traj_probs.empty()) continue;
      targets.push_back(traj_probs.front());
      traj_probs.pop_front();
    }
  }

  for (const auto& target : targets) {
    const auto* st_traj = st_traj_mgr.FindTrajectoryById(target.traj_id);
    if (st_traj == nullptr) continue;
    auto target_or = ConstructTargetFromStBoundary(
        av_path, *st_traj, target.raw_st_bound->overlap_infos(), vehicle_geom);
    if (target_or.has_value()) {
      const auto object_id = std::string(st_traj->object_id());
      targets_map[object_id].push_back(std::move(*target_or));
    }
  }
  return targets_map;
}

[[maybe_unused]] double GetCurrentTimeForObjects(
    const ObjectsProto& objects_proto, const ObjectHistory& av_history,
    const std::set<ObjectIDType>& objects_to_sample) {
  int count = 1;
  double ts = av_history.timestamp();
  for (const auto& object_proto : objects_proto.objects()) {
    if (objects_to_sample.find(object_proto.id()) == objects_to_sample.end()) {
      continue;
    }
    count++;
    ts += object_proto.timestamp();
  }
  return ts / count;
}

std::vector<ActNetSpeedFeature> ExtractActNetSpeedFeatureWithAvailableHistory(
    const TargetsMap& targets_map,
    const ml::ObjectHistorySampler& obj_hist_sampler,
    const DiscretizedPath& av_path) {
  std::vector<ActNetSpeedFeature> all_features;
  for (const auto& [id, targets] : targets_map) {
    for (const auto& target : targets) {
      const auto* object_hist_ptr =
          obj_hist_sampler.GetResampledMotionHistoryPtrByTrajectoryId(
              std::string(target.st_traj->traj_id()));
      const auto& av_hist = obj_hist_sampler.GetResampledAVMotionHistory();
      if (object_hist_ptr == nullptr) continue;
      const auto act_net_feature =
          ExtractActNetFeatureWithAVOnly(id, *object_hist_ptr, av_hist);
      auto feature = ExtractActNetSpeedFeaturesForPlannerTarget(
          act_net_feature, av_path, target);
      all_features.push_back(std::move(feature));
    }
  }

  if (all_features.size() > kActNetSpeedConfig.max_pred_num) {
    all_features.resize(kActNetSpeedConfig.max_pred_num);
  }

  return all_features;
}

}  // namespace

absl::Status ActNetMakeInteractiveSpeedDecision(
    const ActNetInteractionDecisionInput& input,
    std::vector<StBoundaryWithDecision>* st_boundaries_with_decision) {
  QCHECK_NOTNULL(input.st_traj_mgr);
  QCHECK_NOTNULL(input.av_path);
  if (input.av_context == nullptr || input.planner_model_pool == nullptr) {
    return absl::OkStatus();
  }

  if (input.real_objects == nullptr && input.virtual_objects == nullptr) {
    return absl::OkStatus();
  }

  const auto* act_net_speed_inferencer =
      input.planner_model_pool->GetActNetSpeedInferencer();
  if (act_net_speed_inferencer == nullptr) {
    return absl::NotFoundError("Planner act net speed inferencer not found.");
  }

  const auto& st_traj_mgr = *(input.st_traj_mgr);
  const auto& av_path = *(input.av_path);
  const auto& av_context = *input.av_context;

  // 1. Find intersection infos and create targets according to st boundary
  // overlap infos.
  const auto targets_map = ConstructTargetsFromStBoundaries(
      st_traj_mgr, av_path, *st_boundaries_with_decision, *input.vehicle_geom);
  std::map<std::string, std::vector<int>> trajs_to_sample;
  for (const auto& [id, targets] : targets_map) {
    for (const auto& target : targets) {
      trajs_to_sample[id].push_back(target.st_traj->traj_index());
    }
  }
  const auto& av_history = av_context.GetAvObjectHistory();
  std::unique_ptr<ObjectHistorySampler> obj_sampler =
      std::make_unique<ObjectHistorySampler>(
          trajs_to_sample, input.real_objects, input.virtual_objects,
          st_traj_mgr, av_history, ToUnixDoubleSeconds(input.plan_time),
          prediction::kFeatureV2HistoryStepLen,
          prediction::kFeatureV2HistoryStepNum);

  // 2. Extract features.
  const auto features = ExtractActNetSpeedFeatureWithAvailableHistory(
      targets_map, *obj_sampler.get(), av_path);

  // 3. Infer.
  const auto objects_speed_pairs =
      act_net_speed_inferencer->PredictForObjects(absl::MakeSpan(features));

  // 4. Collect results.
  auto& st_boundaries_wd = *st_boundaries_with_decision;
  for (auto& st_bound_wd : st_boundaries_wd) {
    const auto& object_id_opt = st_bound_wd.object_id();
    const auto* speed_pairs_ptr =
        FindOrNull(objects_speed_pairs, object_id_opt.value_or(""));
    if (speed_pairs_ptr == nullptr) {
      continue;
    }
    const auto& speed_pairs = *speed_pairs_ptr;
    const auto& traj_id_opt = st_bound_wd.traj_id();
    if (!traj_id_opt.has_value()) continue;
    for (const auto& speed_pair : speed_pairs) {
      if (SpacetimeObjectTrajectory::MakeTrajectoryId(
              *object_id_opt, speed_pair.traj_index) == *traj_id_opt) {
        const auto& probs = speed_pair.relation_probs;
        st_bound_wd.set_decision_prob(probs.yield, probs.pass);
        const auto decision_type =
            ActNetSpeedDecisionLevel(
                FLAGS_planner_speed_use_act_net_speed_decision_level) ==
                    ActNetSpeedDecisionLevel::kUncertainDecision
                ? ToSpeedDecisionTypeWithUncertainty(probs)
                : ToSpeedDecisionType(probs);
        const auto model_decision_info =
            absl::StrFormat("ActNetSpeed: %s, Y(%.2f)/P(%.2f). ",
                            StBoundaryProto::DecisionType_Name(decision_type),
                            probs.yield * 100.0, probs.pass * 100.0);
        const auto decision_info =
            absl::StrCat(st_bound_wd.decision_info(), " ", model_decision_info);
        st_bound_wd.set_decision_info(decision_info);  // Append decision info.
        if (ActNetSpeedDecisionLevel(
                FLAGS_planner_speed_use_act_net_speed_decision_level) ==
            ActNetSpeedDecisionLevel::kNoDecision) {
          continue;
        }
        // Engage to make decision.
        if (decision_type != StBoundaryProto::UNKNOWN) {
          // Only modify decision when certain.
          st_bound_wd.set_decision_type(decision_type);
          st_bound_wd.set_decision_reason(
              StBoundaryProto::ACT_NET_SPEED_DECIDER);
        }
      }
    }
  }

  return absl::OkStatus();
}

void EvaluateActNetSpeedDecision(
    absl::Span<const StBoundaryWithDecision> st_bounds_wd) {
  for (const auto& st_bound_wd : st_bounds_wd) {
    const auto& type = st_bound_wd.decision_type();
    const auto& prob = st_bound_wd.decision_prob();
    if (HasValidDecisionProb(prob)) {
      const auto curr_time = Clock::Now();
      const auto prob_type = DecisionProbToDecisionType(prob);
      if (AreDifferentSpeedDecisions(type, prob_type)) {
        QEVENT_EVERY_N_SECONDS(
            "changqing", "planner_act_net_speed_decision_diff",
            /*every_n_seconds=*/5.0, [&](QEvent* qevent) {
              qevent->AddField("st boundary id", st_bound_wd.id())
                  .AddField("planner speed decision",
                            StBoundaryProto::DecisionType_Name(type))
                  .AddField("decision reason",
                            StBoundaryProto::DecisionReason_Name(
                                st_bound_wd.decision_reason()))
                  .AddField("yield prob", prob.yield())
                  .AddField("pass prob", prob.pass());
              qevent->SetTimestamp(ToUnixMicros(curr_time));
            });
      }
    }
  }
}

}  // namespace planner
}  // namespace qcraft
