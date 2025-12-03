#include "onboard/planner/ml/context_groundtruth_extractor/context_groundtruth_extractor.h"

#include <cmath>     // for fabs
#include <iterator>  // for move_iterator, make_move_iterator
#include <map>       // for map, operator==
#include <string>    // for string
#include <utility>   // for move
#include <vector>    // for vector

#include "onboard/math/geometry/proto/affine_transformation.pb.h"  // for Vec2dProto
#include "onboard/planner/ml/planner_ml_defs.h"  // for kTrajectoryGroundTruthDuration, kTrajecto...
#include "onboard/prediction/feature_extractor/act_net_feature.h"  // for ActNetFeature, ActNetObjectFeature
#include "onboard/prediction/feature_extractor/feature_extraction_util.h"  // for AlignPredictedTrajectoryPoints, ConvertTr...
#include "onboard/prediction/proto/prediction_common.pb.h"  // for PredictedTrajectoryPointProto
#include "onboard/proto/perception.pb.h"                    // for ObjectProto
#include "onboard/proto/prediction.pb.h"  // for ObjectPredictionProto, PredictedTrajector...
#include "onboard/utils/map_util.h"  // for FindOrNull

namespace qcraft {
namespace planner {
namespace ml {

namespace {
constexpr double kInconsistencyFactor = 3.0;
constexpr double kInconsistencySpeedDiff = 5.0;
constexpr double kMaxPossibleSpeed = 69.4;     // m/s.
constexpr double kMaxPossibleAcc = 9.8;        // m/s^2.
constexpr double kObjectStaticEpsilon = 1e-1;  // m/s.
constexpr double kAvStaticEpsilon = 1e-2;      // m/s.

TrajectoryGroundTruth ExtractTrajectoryGroundTruth(
    const PredictedTrajectoryProto& oracle_traj, const double oracle_ts,
    double cur_ts, const Vec2d& ref_pos, double rot_rad, bool is_av) {
  constexpr double kDesiredDuration = kTrajectoryGroundTruthDuration + 1;
  // Get traj point from oracle_traj with static state cut off.
  const double t_diff = cur_ts - oracle_ts;
  std::vector<PredictedTrajectoryPointProto> raw_traj;
  for (int i = 0; i < oracle_traj.points_size(); ++i) {
    const auto& raw_pt = oracle_traj.points(i);
    if (raw_pt.t() > t_diff + kDesiredDuration) {
      break;
    }
    auto pt = raw_pt;
    auto cur_pos = (Vec2d(pt.pos()) - ref_pos).Rotate(rot_rad);
    pt.mutable_pos()->set_x(cur_pos.x());
    pt.mutable_pos()->set_y(cur_pos.y());
    pt.set_theta(pt.theta() + rot_rad);
    // Add a non-smoothness (velocity inconsistency) checker.
    if (!is_av && !raw_traj.empty()) {
      const auto& last_pt = raw_traj.back();
      const double dis = Vec2d(last_pt.pos()).DistanceTo(Vec2d(pt.pos()));
      const double pseudo_speed = std::fabs(dis / (pt.t() - last_pt.t()));
      const double speed_diff = std::fabs(pt.v() - last_pt.v());
      const double pseudo_acc = std::fabs(speed_diff / (pt.t() - last_pt.t()));
      if ((pseudo_speed > std::fabs(pt.v()) * kInconsistencyFactor &&
           std::fabs(pseudo_speed - std::fabs(pt.v())) >
               kInconsistencySpeedDiff) ||
          pseudo_speed > kMaxPossibleSpeed || pt.v() > kMaxPossibleSpeed ||
          pseudo_acc > kMaxPossibleAcc) {
        break;
      }
    }
    raw_traj.push_back(std::move(pt));
    if (!is_av && raw_pt.v() < kObjectStaticEpsilon) {
      break;
    }
    if (is_av && raw_pt.v() < kAvStaticEpsilon) {
      break;
    }
  }
  double cur_t = raw_traj.back().t();
  while (((!is_av && raw_traj.back().v() < kObjectStaticEpsilon) ||
          (is_av && raw_traj.back().v() < kAvStaticEpsilon)) &&
         raw_traj.back().t() < t_diff + kDesiredDuration) {
    auto last_pt = raw_traj.back();
    cur_t += kTrajectoryGroundTruthStep;
    last_pt.set_t(cur_t);
    raw_traj.push_back(std::move(last_pt));
  }
  // Align and interpolate the raw_traj by the timeline of current_ts.
  auto aligned_traj = prediction::AlignPredictedTrajectoryPoints(
      raw_traj, oracle_ts, cur_ts, kTrajectoryGroundTruthDuration,
      kTrajectoryGroundTruthStep);

  TrajectoryGroundTruth traj_gt;
  for (int i = 1, size = aligned_traj.size(); i < size; ++i) {
    GroundTruthPointProto pt;
    *pt.mutable_pos() = aligned_traj[i].pos();
    pt.set_s(aligned_traj[i].s());
    pt.set_theta(aligned_traj[i].theta());
    pt.set_kappa(aligned_traj[i].kappa());
    pt.set_t(aligned_traj[i].t());
    pt.set_v(aligned_traj[i].v());
    pt.set_a(aligned_traj[i].a());
    *traj_gt.add_gt_points() = std::move(pt);
  }

  return traj_gt;
}
}  // namespace

TrajectoryGroundTruth ExtractObjectTrajectoryGroundTruth(
    const ObjectPredictionProto& object_prediction_proto, const double cur_ts,
    const Vec2d& ref_pos, const double rot_rad) {
  const auto oracle_ts =
      object_prediction_proto.perception_object().timestamp();
  const auto& oracle_traj = object_prediction_proto.trajectories(0);
  auto gt = ExtractTrajectoryGroundTruth(oracle_traj, oracle_ts, cur_ts,
                                         ref_pos, rot_rad, /*is_av=*/false);
  gt.set_object_id(object_prediction_proto.id());
  return gt;
}

TrajectoryGroundTruth ExtractAVTrajectoryGroundTruth(
    const TrajectoryProto& trajectory_proto,
    const VehicleGeometryParamsProto& veh_geom_params, const double cur_ts,
    const Vec2d& ref_pos, const double rot_rad) {
  const auto oracle_ts = trajectory_proto.trajectory_start_timestamp();
  // Note(Jinyun): It is shifted from vehicle center to geometric center.
  auto av_traj = prediction::ConvertTrajectoryProtoToPredictedTrajectoryPoints(
      trajectory_proto, veh_geom_params);
  PredictedTrajectoryProto oracle_traj;
  *oracle_traj.mutable_points() = {std::make_move_iterator(av_traj.begin()),
                                   std::make_move_iterator(av_traj.end())};
  auto gt = ExtractTrajectoryGroundTruth(oracle_traj, oracle_ts, cur_ts,
                                         ref_pos, rot_rad, /*is_av=*/true);
  gt.set_object_id("AV");
  return gt;
}

ContextGroundTruth ExtractContextGroundTruth(
    const ContextGroundTruthExtractionInput& input) {
  ContextGroundTruth gt;
  // Get current timestamp in seconds.
  const double current_ts = input.context_feature->current_ts;

  // Get reference coordinates.
  const auto& ref_position = input.context_feature->ref_position;
  const auto& rot_rad = input.context_feature->rot_rad;

  // Get objects' oracle trajectories.
  std::map<std::string, const ObjectPredictionProto*> objs_trajs_map;
  for (const auto& obj : input.log_prediction->objects()) {
    objs_trajs_map[obj.id()] = &obj;
  }

  // Get objects's future trajectories.
  for (const auto& obj :
       input.context_feature->act_net_feature.context_obj_features) {
    auto* obj_traj_ptr = FindOrNull(objs_trajs_map, obj.id);
    TrajectoryGroundTruth traj_gt;
    if (obj_traj_ptr == nullptr) {
      // Set empty traj ground truth to match the sequences of object of
      // features and gts.
      traj_gt.set_object_id(obj.id);
    } else {
      traj_gt = ExtractObjectTrajectoryGroundTruth(**obj_traj_ptr, current_ts,
                                                   ref_position, rot_rad);
    }
    *gt.add_objs_gt_traj() = std::move(traj_gt);
  }

  // Get av oracle trajectory.
  auto av_gt = ExtractAVTrajectoryGroundTruth(
      *input.log_av_trajectory, *input.veh_geom_params, current_ts,
      ref_position, rot_rad);
  *gt.mutable_av_gt_traj() = std::move(av_gt);

  return gt;
}

}  // namespace ml
}  // namespace planner
}  // namespace qcraft
