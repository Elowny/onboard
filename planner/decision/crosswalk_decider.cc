#include "onboard/planner/decision/crosswalk_decider.h"

#include <algorithm>
#include <cmath>
#include <ostream>
#include <string>
#include <utility>
#include <vector>

// IWYU pragma: no_include <google/protobuf/repeated_ptr_field.h>

#include "absl/container/flat_hash_set.h"
#include "absl/strings/str_cat.h"
#include "glog/logging.h"

#include "onboard/container/strong_int.h"
#include "onboard/eval/qevent.h"
#include "onboard/eval/qevent_base.h"
#include "onboard/global/logging.h"
#include "onboard/lite/check.h"
#include "onboard/maps/lane_path.h"
#include "onboard/maps/map_or_die_macros.h"
#include "onboard/maps/maps_common.h"
#include "onboard/maps/proto/semantic_map.pb.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/math/frenet_common.h"
#include "onboard/math/geometry/halfplane.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/math/geometry/segment2d.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/decision/decision_util.h"
#include "onboard/planner/decision/proto/constraint.pb.h"
#include "onboard/planner/object/object_vector.h"
#include "onboard/planner/object/planner_object.h"
#include "onboard/planner/second_order_trajectory_point.h"
#include "onboard/prediction/object_prediction.h"
#include "onboard/prediction/predicted_trajectory.h"
#include "onboard/proto/perception/fusion/object.pb.h"
#include "onboard/utils/source_location.h"
#include "onboard/utils/status_macros.h"

// IWYU pragma: no_include "Eigen/Core"

namespace qcraft {
namespace planner {
namespace {

struct CrosswalkManager {
  mapping::CrosswalkInfo crosswalk;
  double start_s_on_drive_passage;
  double end_s_on_drive_passage;
  bool tl_controlled;
  const CrosswalkStateProto* last_state = nullptr;
};

// Used to compare double values.
constexpr double kEpsilon = 1e-5;

// This variable depends on the accuracy of the prediction module. For now,
// let's set this value to 3.0s, which means that only predicted trajectories of
// less than 3.0s are considered.
constexpr double kStableObjectPredictedTrajectoryTime = 2.0;  // s

// This variable is used to determine the distance threshold of objects
// associated with crosswalk. If the minimum distance between the object and
// the crosswalk is greater than this value, ignore this object.
constexpr double kCrosswalkAssociationDistance = 1.3;  // m

// If the distance between the object and the vehicle boundary is greater than
// this value and the movement direction deviates from the lane center, ignore
// this object.
constexpr double kIgnoreLateralMoveOffBuffer = 1.0;  // m

// The stand off distance before the crosswalk, when AV decides to stop before
// this crosswalk.
constexpr double kCrossWalkStandoff = 2.0;  // m

// Ignore crosswalk which far away from AV current position
constexpr double kMaxLookAheadDistance = 150;  // m

// The comfortable starting acceleration when AV stop before crosswalk
constexpr double kComfortableAcceleration = 0.5;  // m/s^2

// The two variables are used to distinguish APPROACHING/STOP/PASSED state.
// kStopMinDistance may relative to kCrossWalkStandoff
constexpr double kStopMinDistance = 3.0;  // m
constexpr double kStopMaxVelocity = 0.3;  // m/s

// At the first 5s after av stop before crosswalk, av will yield both static and
// moving objects. 5s later, making stop decision depend on av or pedestrian who
// will reach the crosspoint first.
constexpr double kMaxYieldingTime = 8.0;  // s

constexpr double kEmergencyStopDeceleration = -3.5;  // m/s^2

// The function to calculate comfortable deceleration according vehicle speed
const PiecewiseLinearFunction kVehicleSpeedComfortableBrakePlf(
    std::vector<double>{0.0, 5.0, 10.0}, std::vector<double>{-3.5, -2.5, -1.5});

// The function to calculate time gain when object moving on the crosswalk.
const PiecewiseLinearFunction kObjectMovingTimeGainPlf(
    std::vector<double>{0.1, 0.5}, std::vector<double>{0.0, 4.0});

const CrosswalkStateProto* FindCrosswalkStateOrNull(
    mapping::ElementId crosswalk_id,
    const ::google::protobuf::RepeatedPtrField<CrosswalkStateProto>&
        crosswalk_states) {
  for (const auto& state : crosswalk_states) {
    if (state.crosswalk_id() == crosswalk_id.value()) return &state;
  }
  return nullptr;
}

std::vector<CrosswalkManager> FindCrosswalks(
    const PlannerSemanticMapManager& psmm, const DrivePassage& passage,
    const mapping::LanePath& lane_path_from_start, double s_offset,
    const ::google::protobuf::RepeatedPtrField<CrosswalkStateProto>&
        crosswalk_states) {
  std::vector<CrosswalkManager> crosswalk_managers;

  for (const auto& seg : lane_path_from_start) {
    SMM_ASSIGN_LANE_OR_CONTINUE(lane_info, psmm, seg.lane_id);

    for (const auto& [cw_id, cw_frac] : lane_info.Crosswalks()) {
      const auto cw_ptr = psmm.FindCrosswalkByIdOrNull(cw_id);
      if (cw_ptr == nullptr) continue;
      const auto& cw_info = *cw_ptr;

      const double start_s_on_ref_path =
          lane_path_from_start.LaneIndexPointToArclength(seg.lane_index,
                                                         cw_frac.x());
      const double end_s_on_ref_path =
          lane_path_from_start.LaneIndexPointToArclength(seg.lane_index,
                                                         cw_frac.y());

      // ignore crosswalk which behind AV
      if (end_s_on_ref_path < kEpsilon) continue;

      // ignore crosswalk which far away from AV
      if (end_s_on_ref_path > kMaxLookAheadDistance) continue;

      const bool tl_ctrl_cw =
          cw_info.proto->has_type()
              ? cw_info.proto->type() ==
                    mapping::CrosswalkProto::TRAFFIC_LIGHT_CONTROLLED
              : false;
      const bool tl_ctrl_path =
          IsTrafficLightControlledLane(*QCHECK_NOTNULL(lane_info.proto));
      const bool tl_controlled = tl_ctrl_cw && tl_ctrl_path;

      constexpr double kQeventSignCrosswalkDistance = 5.0;
      // Record un tl controlled crosswalk.
      if (start_s_on_ref_path < kQeventSignCrosswalkDistance &&
          tl_controlled == false) {
        QEVENT_EVERY_N_SECONDS("jiayukou", "crosswalk_un_tl_controlled",
                               /*seconds=*/60.0, [&](QEvent* qevent) {
                                 qevent->AddField("cw_id", cw_info.id.value())
                                     .AddField("cw_type", cw_info.proto->type())
                                     .AddField("start_s", start_s_on_ref_path)
                                     .AddField("end_s", end_s_on_ref_path)
                                     .AddField("tl_control", tl_controlled);
                               });
      }

      crosswalk_managers.emplace_back(CrosswalkManager{
          .crosswalk = cw_info,
          .start_s_on_drive_passage =
              start_s_on_ref_path + s_offset + passage.lane_path_start_s(),
          .end_s_on_drive_passage =
              end_s_on_ref_path + s_offset + passage.lane_path_start_s(),
          .tl_controlled = tl_controlled,
          .last_state =
              FindCrosswalkStateOrNull(cw_info.id, crosswalk_states)});
    }
  }

  return crosswalk_managers;
}

bool IsComfortableDecelerationForCrosswalk(double speed, double av_front_edge_s,
                                           double crosswalk_start_s) {
  // Yield to pedestrian only when AV not on the crosswalk
  if (crosswalk_start_s < av_front_edge_s) return false;

  const double comfortable_deceleration_dist =
      0.5 * speed * speed / std::fabs(kVehicleSpeedComfortableBrakePlf(speed));

  return crosswalk_start_s > av_front_edge_s + comfortable_deceleration_dist;
}

// This function return the time gain, When crosswalk type is UNTLCONTROLED.
// Allow more time for pedestrian to crosswalk. Because of the speed of cyclists
// is faster than pedestian ,so deal with this problem separately.
double UnTlControledCrosswalkTimeGain(ObjectType type) {
  switch (type) {
    case OT_PEDESTRIAN:
      return 4.0;
    case OT_CYCLIST:
    case OT_TRICYCLIST:
      return 0.6;
    case OT_UNKNOWN_STATIC:
    case OT_VEHICLE:
    case OT_LARGE_VEHICLE:
    case OT_MOTORCYCLIST:
    case OT_FOD:
    case OT_UNKNOWN_MOVABLE:
    case OT_VEGETATION:
    case OT_BARRIER:
    case OT_BARRIER_ANTI_COLLISION_BUCKET:
    case OT_BARRIER_ANTI_COLLISION_POST:
    case OT_CONE:
    case OT_WARNING_TRIANGLE:
      return 0.0;
  }
}
bool IsConsideredType(ObjectType type) {
  switch (type) {
    case OT_PEDESTRIAN:
    case OT_CYCLIST:
    case OT_TRICYCLIST:
      return true;
    case OT_UNKNOWN_STATIC:
    case OT_VEHICLE:
    case OT_LARGE_VEHICLE:
    case OT_MOTORCYCLIST:
    case OT_FOD:
    case OT_UNKNOWN_MOVABLE:
    case OT_VEGETATION:
    case OT_BARRIER:
    case OT_BARRIER_ANTI_COLLISION_BUCKET:
    case OT_BARRIER_ANTI_COLLISION_POST:
    case OT_CONE:
    case OT_WARNING_TRIANGLE:
      return false;
  }
}

// If the distance between the object and the vehicle boundary is greater than
// this value and the movement direction toward the lane center, ignore
// this object.
double GetIgnoreLateralTowardBuffer(ObjectType type) {
  switch (type) {
    case OT_PEDESTRIAN:
      return 5.0;
    case OT_CYCLIST:
    case OT_TRICYCLIST:
      return 2.0;
    case OT_UNKNOWN_STATIC:
    case OT_VEHICLE:
    case OT_LARGE_VEHICLE:
    case OT_MOTORCYCLIST:
    case OT_FOD:
    case OT_UNKNOWN_MOVABLE:
    case OT_VEGETATION:
    case OT_BARRIER:
    case OT_BARRIER_ANTI_COLLISION_BUCKET:
    case OT_BARRIER_ANTI_COLLISION_POST:
    case OT_CONE:
    case OT_WARNING_TRIANGLE:
      return 0.0;
  }
}

bool IsObjectTrajectoryOnCrosswalk(const CrosswalkManager& cw_manager,
                                   const PlannerObject& object) {
  const auto& cw_info = cw_manager.crosswalk;
  const auto& trajs = object.prediction().trajectories();
  for (const auto& pred_traj : trajs) {
    const auto& pred_traj_points = pred_traj.points();
    for (const auto& pt : pred_traj_points) {
      if (pt.t() > kStableObjectPredictedTrajectoryTime) break;

      if (cw_info.polygon_smooth.DistanceSquareTo(pt.pos()) < Sqr(kEpsilon))
        return true;
    }
  }
  return false;
}

bool IsObjectNearCrosswalk(const CrosswalkManager& cw_manager,
                           const PlannerObject& object, double distance) {
  const auto& cw_info = cw_manager.crosswalk;

  return cw_info.polygon_smooth.DistanceSquareTo(object.pose().pos()) <
         Sqr(distance);
}

absl::StatusOr<ConstraintProto::StopLineProto> GenerateCrosswalkConstraints(
    const CrosswalkManager& cw_manager, const DrivePassage& passage) {
  ASSIGN_OR_RETURN(const auto curbs, passage.QueryCurbPointAtS(
                                         cw_manager.start_s_on_drive_passage));

  ConstraintProto::StopLineProto stop_line;
  stop_line.set_s(cw_manager.start_s_on_drive_passage);
  stop_line.set_standoff(kCrossWalkStandoff);
  stop_line.set_time(0.0);
  HalfPlane halfplane(curbs.first, curbs.second);
  halfplane.ToProto(stop_line.mutable_half_plane());
  stop_line.set_id(absl::StrCat("crosswalk_", cw_manager.crosswalk.id));
  stop_line.mutable_source()->mutable_crosswalk()->set_crosswalk_id(
      cw_manager.crosswalk.id.value());
  return stop_line;
}

// AV yield to static object, only for must yied type crosswalk
bool CheckCrosswalkTypeValidToYieldObject(mapping::CrosswalkProto::Type type) {
  switch (type) {
    case mapping::CrosswalkProto::MUST_YEILD:
      return true;
    case mapping::CrosswalkProto::UNCONTROLLED:
    case mapping::CrosswalkProto::TRAFFIC_LIGHT_CONTROLLED:
      return false;
  }
}

double CalculateAccelerationTime(double dist, double vel_begin, double acc) {
  return (std::sqrt(2 * acc * dist + Sqr(vel_begin)) - vel_begin) / acc;
}

CrosswalkStateProto CreateApproachingState(mapping ::ElementId crosswalk_id) {
  CrosswalkStateProto new_state;
  new_state.set_crosswalk_id(crosswalk_id.value());

  new_state.set_stop_state(CrosswalkStateProto::APPROACHING);
  return new_state;
}

CrosswalkStateProto CreatePassedState(mapping ::ElementId crosswalk_id) {
  CrosswalkStateProto new_state;

  new_state.set_crosswalk_id(crosswalk_id.value());

  new_state.set_stop_state(CrosswalkStateProto::PASSED);
  return new_state;
}

CrosswalkStateProto CreateFirstStoppedState(mapping ::ElementId crosswalk_id,
                                            double now_in_seconds) {
  CrosswalkStateProto new_state;

  new_state.set_crosswalk_id(crosswalk_id.value());

  new_state.set_stop_time(now_in_seconds);
  new_state.set_stop_state(CrosswalkStateProto::STOPPED);
  return new_state;
}

// This function will return new state which not contains yield objects from
// last state.
CrosswalkStateProto UpdateCrosswalkState(double av_front_edge_s,
                                         double plan_start_v,
                                         double now_in_seconds,
                                         const CrosswalkManager& cw_manager) {
  // If make decision for this crosswalk for the first time , create approaching
  // state
  const auto& crosswalk_id = cw_manager.crosswalk.id;
  if (cw_manager.last_state == nullptr) {
    return CreateApproachingState(crosswalk_id);
  }

  const auto& last_state = *cw_manager.last_state;
  // If last state is PASSED, just return.
  if (last_state.stop_state() == CrosswalkStateProto::PASSED)
    return CreatePassedState(crosswalk_id);
  // If last state is APPROACHING, update state according to av position and
  // velocity
  if (last_state.stop_state() == CrosswalkStateProto::APPROACHING) {
    if (av_front_edge_s >
            cw_manager.start_s_on_drive_passage - kStopMinDistance &&
        plan_start_v < kStopMaxVelocity) {
      return CreateFirstStoppedState(crosswalk_id, now_in_seconds);
    } else if (av_front_edge_s > cw_manager.end_s_on_drive_passage) {
      return CreatePassedState(crosswalk_id);
    } else {
      return CreateApproachingState(crosswalk_id);
    }
  }

  // STOPPED ==> PASSING meet one of the following condition
  // 1. Exceed max yield time, which is 8.0s now
  // 2. AV has driven out of the crosswalk
  const bool exceed_yield_time =
      now_in_seconds - last_state.stop_time() > kMaxYieldingTime;
  if (exceed_yield_time ||
      av_front_edge_s > cw_manager.end_s_on_drive_passage) {
    return CreatePassedState(crosswalk_id);
  }

  CrosswalkStateProto new_state;
  new_state.set_crosswalk_id(crosswalk_id.value());

  new_state.set_stop_time(last_state.stop_time());
  new_state.set_stop_state(CrosswalkStateProto::STOPPED);
  return new_state;
}

double CalculateObjectAcrossRatio(const PlannerObject& object,
                                  const CrosswalkManager& cw_manager) {
  const auto& cw_segment = cw_manager.crosswalk.bone_axis_smooth;
  const auto& cw_unit_dir = cw_segment.unit_direction();

  const double object_signed_offset =
      (object.pose().pos() - cw_segment.start()).dot(cw_unit_dir);
  double across_ratio =
      std::clamp(object_signed_offset / cw_segment.length(), 0.0, 1.0);

  const Vec2d& object_vel = object.velocity();

  if (object_vel.dot(cw_unit_dir) < kEpsilon) {
    across_ratio = 1.0 - across_ratio;
  }
  return across_ratio;
}

bool IsValidCrosswalkType(
    absl::Span<const mapping::CrosswalkProto::Type> valid_cw_types,
    mapping::CrosswalkProto::Type type) {
  return std::find(valid_cw_types.begin(), valid_cw_types.end(), type) !=
         valid_cw_types.end();
}

}  // namespace

absl::StatusOr<CrosswalkDeciderOutput> BuildCrosswalkConstraints(
    const CrosswalkDeciderInput& input) {
  const auto& vehicle_geometry_params = *input.vehicle_geometry_params;
  const auto& psmm = *input.psmm;
  const auto& plan_start_point = *input.plan_start_point;
  const auto& passage = *input.passage;
  const auto& lane_path_from_start = *input.lane_path_from_start;
  const auto& obj_mgr = *input.obj_mgr;
  const auto& last_crosswalk_states = *input.last_crosswalk_states;
  const auto& valid_cw_types = input.valid_cw_types;
  const double now_in_seconds = input.now_in_seconds;
  const double s_offset = input.s_offset;

  std::vector<ConstraintProto::StopLineProto> cw_stop_lines;
  std::vector<ConstraintProto::SpeedRegionProto> cw_speed_regions;
  std::vector<CrosswalkStateProto> cw_states;
  // 1. Find nearest crosswalk
  const auto crosswalk_managers = FindCrosswalks(
      psmm, passage, lane_path_from_start, s_offset, last_crosswalk_states);

  // 2. Based on the trajectories of related objects, generate stop lines and
  // speed regions for each crosswalk

  const Vec2d plan_start_pos_xy(plan_start_point.path_point().x(),
                                plan_start_point.path_point().y());

  const auto plan_start_pos_sl =
      passage.QueryFrenetCoordinateAt(plan_start_pos_xy);
  if (!plan_start_pos_sl.ok()) {
    return plan_start_pos_sl.status();
  }

  const auto av_front_edge_s =
      vehicle_geometry_params.front_edge_to_center() + plan_start_pos_sl->s;

  // make sure plan_start_v greater than 0.0
  const double plan_start_v = std::max(plan_start_point.v(), kEpsilon);

  const auto av_front_edge_xy =
      passage.QueryPointXYAtSL(av_front_edge_s, plan_start_pos_sl->l);
  if (!av_front_edge_xy.ok()) return av_front_edge_xy.status();

  for (const auto& cw_manager : crosswalk_managers) {
    // Update crosswalk state
    auto cw_state = UpdateCrosswalkState(av_front_edge_s, plan_start_v,
                                         now_in_seconds, cw_manager);
    // Record last yield objects
    absl::flat_hash_set<std::string> last_must_yield_objects;
    if (cw_manager.last_state != nullptr) {
      const auto& last_state = *cw_manager.last_state;
      const int num_objects = last_state.must_yield_objects_size();
      for (int i = 0; i < num_objects; ++i) {
        last_must_yield_objects.insert(last_state.must_yield_objects(i));
      }
    }

    // Making yield decision for MUST_YEILD type crosswalk only.
    if (!IsValidCrosswalkType(valid_cw_types,
                              cw_manager.crosswalk.proto->type())) {
      continue;
    }

    const auto cw_start_xy = passage.QueryPointXYAtSL(
        cw_manager.start_s_on_drive_passage, plan_start_pos_sl->l);
    const auto cw_end_xy = passage.QueryPointXYAtSL(
        cw_manager.end_s_on_drive_passage, plan_start_pos_sl->l);
    if (!cw_start_xy.ok() || !cw_end_xy.ok()) continue;

    VLOG(2) << " * * * * *cw:\t" << cw_manager.crosswalk.id << " |start:\t"
            << cw_manager.start_s_on_drive_passage << " |end:\t"
            << cw_manager.end_s_on_drive_passage << " |tlc:\t"
            << cw_manager.tl_controlled;

    // Record must yield objects and output to planner state proto.
    std::vector<std::string> new_must_yield_objects;
    // Record all yield objects and output to planner debug proto.
    std::vector<std::string> all_yield_objects;

    for (const auto& object : obj_mgr.planner_objects()) {
      bool must_yield_to_object = false;
      // 2.1 whether filter out this object
      // 2.1.1 Check object type
      if (!IsConsideredType(object.type())) continue;

      const bool object_near_cw = IsObjectNearCrosswalk(
          cw_manager, object, kCrosswalkAssociationDistance);
      // 2.1.2 Check object's trajectory on crosswalk ,in the next 2.0s and
      // Check object's position
      if (!object_near_cw &&
          !IsObjectTrajectoryOnCrosswalk(cw_manager, object)) {
        continue;
      }

      // 2.1.3 Check object moving off the AV path
      const Vec2d cw_axis_unit_dir =
          cw_manager.crosswalk.bone_axis_smooth.unit_direction();

      const Vec2d object_velocity = object.velocity();
      const double object_signed_velocity_proj_on_cw =
          object_velocity.dot(cw_axis_unit_dir);

      const bool av_already_on_crosswalk =
          cw_manager.start_s_on_drive_passage < av_front_edge_s;

      const Vec2d origin =
          av_already_on_crosswalk ? *av_front_edge_xy : *cw_start_xy;
      const double object_signed_offset_to_path =
          (object.pose().pos() - origin).dot(cw_axis_unit_dir);

      const bool same_side =
          object_signed_offset_to_path * object_signed_velocity_proj_on_cw >
          0.0;

      const double av_half_width =
          (vehicle_geometry_params.left_edge_to_center() +
           vehicle_geometry_params.right_edge_to_center()) *
          0.5;

      if (same_side && std::abs(object_signed_offset_to_path) >
                           kIgnoreLateralMoveOffBuffer + av_half_width) {
        continue;
      }

      // 2.1.4 Check object moving parallel to AV path

      if (std::abs(object_velocity.CrossProd(cw_axis_unit_dir)) >
          std::abs(object_signed_velocity_proj_on_cw)) {
        continue;
      }

      // 2.1.5 Check object behind AV front edge,when AV already on the
      // crosswalk
      const Vec2d cw_axis_perp_unit_dir = (*cw_end_xy - *cw_start_xy).Unit();
      const double object_offset_to_cw =
          (object.pose().pos() - *cw_start_xy).dot(cw_axis_perp_unit_dir);
      const double av_offset_to_cw =
          (*av_front_edge_xy - *cw_start_xy).dot(cw_axis_perp_unit_dir);
      const double object_lead_av_dist = object_offset_to_cw - av_offset_to_cw;

      if (av_already_on_crosswalk && object_lead_av_dist < kEpsilon) {
        continue;
      }

      // Three condition needed to be check:
      // 1. Object near the crosswalk with the distance to the crosswalk less
      // than 1.0m.
      // 2. The type of crosswalk is MUST_YIELD.
      // 3. It is comfortable to stop before the crosswalk.

      const bool cw_type_valid_yield_object =
          CheckCrosswalkTypeValidToYieldObject(
              cw_manager.crosswalk.proto->type());
      const bool comfortable_deceleration =
          IsComfortableDecelerationForCrosswalk(
              plan_start_v, av_front_edge_s,
              cw_manager.start_s_on_drive_passage);

      if (object_near_cw && cw_type_valid_yield_object &&
          comfortable_deceleration) {
        switch (cw_state.stop_state()) {
          case CrosswalkStateProto::APPROACHING:
          case CrosswalkStateProto::STOPPED:
            // Refrash yield objects every time
            new_must_yield_objects.push_back(object.id());
            must_yield_to_object = true;
            break;
          case CrosswalkStateProto::PASSED:
            // Filter stationary object from last_must_yield_objects only in
            // PASSED state.
            if (!object.is_stationary() &&
                last_must_yield_objects.contains(object.id())) {
              new_must_yield_objects.push_back(object.id());
              must_yield_to_object = true;
            }
            break;
        }
      }

      // 2.2 it is necessary to generate stop line for this crosswalk
      // 2.2.1 check obs Ahead of AV cross centerline

      const double distance_to_crosswalk =
          std::max(object_lead_av_dist,
                   cw_manager.start_s_on_drive_passage - av_front_edge_s);

      double av_t_at_cw = distance_to_crosswalk / plan_start_v;
      double av_t_at_cw_acceleration = CalculateAccelerationTime(
          distance_to_crosswalk, plan_start_v, kComfortableAcceleration);

      // According crosswalk type and object type calculate object_time_gain.
      double object_time_gain = UnTlControledCrosswalkTimeGain(object.type());
      if (object.type() == OT_PEDESTRIAN) {
        object_time_gain += kObjectMovingTimeGainPlf(
            CalculateObjectAcrossRatio(object, cw_manager));
      }

      // Check whether object ahead of av cross the crosswalk with time gain.
      const double object_ahead_of_av_dist_with_time_gain =
          std::abs(object_signed_velocity_proj_on_cw) *
              (av_t_at_cw + object_time_gain) +
          av_half_width - std::abs(object_signed_offset_to_path);
      const bool object_ahead_of_av_with_time_gain =
          object_ahead_of_av_dist_with_time_gain > kEpsilon;

      // Check whether object near av when av accelerate cross the crosswalk.
      const double object_near_av_dist_without_time_gain =
          std::abs(object_signed_velocity_proj_on_cw) *
              av_t_at_cw_acceleration +
          GetIgnoreLateralTowardBuffer(object.type()) + av_half_width -
          std::abs(object_signed_offset_to_path);
      const bool object_near_av_without_time_gain =
          object_near_av_dist_without_time_gain > kEpsilon;

      // Check whether av stop comfortablely when yield to object.
      const double deceleration =
          -0.5 * Sqr(plan_start_v) / distance_to_crosswalk;
      const bool av_stop_comfortablely =
          deceleration > kEmergencyStopDeceleration;
      // 2.3 Generate stop lines for this crosswalk
      if ((object_ahead_of_av_with_time_gain &&
           object_near_av_without_time_gain && av_stop_comfortablely) ||
          must_yield_to_object) {
        all_yield_objects.push_back(object.id());
      }
    }

    for (auto& id : new_must_yield_objects) {
      cw_state.add_must_yield_objects(std::move(id));
    }
    cw_states.push_back(std::move(cw_state));

    if (!all_yield_objects.empty()) {
      ASSIGN_OR_CONTINUE(auto stop_line,
                         GenerateCrosswalkConstraints(cw_manager, passage));

      for (auto& id : all_yield_objects) {
        stop_line.mutable_source()->mutable_crosswalk()->add_object_id(
            std::move(id));
      }

      cw_stop_lines.push_back(std::move(stop_line));
    }
  }

  return CrosswalkDeciderOutput{.stop_lines = std::move(cw_stop_lines),
                                .speed_regions = std::move(cw_speed_regions),
                                .crosswalk_states = std::move(cw_states)};
}
}  // namespace planner
}  // namespace qcraft
