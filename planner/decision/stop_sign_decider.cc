#include "onboard/planner/decision/stop_sign_decider.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <utility>

#include "absl/algorithm/container.h"
#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/types/span.h"
#include "google/protobuf/repeated_ptr_field.h"

#include "onboard/container/strong_int.h"
#include "onboard/global/trace.h"
#include "onboard/maps/lane_path.h"
#include "onboard/maps/lane_point.h"
#include "onboard/maps/maps_common.h"
#include "onboard/maps/proto/semantic_map.pb.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/math/frenet_common.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/math/geometry/halfplane.h"
#include "onboard/math/vec.h"
#include "onboard/planner/object/planner_object.h"
#include "onboard/planner/object/spacetime_object_trajectory.h"
#include "onboard/planner/second_order_trajectory_point.h"
#include "onboard/planner/util/perception_util.h"
#include "onboard/planner/util/spatial_search_util.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/utils/status_macros.h"

namespace qcraft {
namespace planner {

namespace {

constexpr double kStopMinDistance = 1.0;  // Meters.
constexpr double kStopMaxVelocity = 0.3;  // Meters per second.

struct StopSignLaneInfo {
  // The lane associated with the stop sign. It is used as a unique identifier
  // of the stop sign.
  mapping::ElementId lane_id;
  // The stop line's position's cumulative distance on drive passage.
  double drive_passage_s;
  // All the associated lanes that we should yield to the cars on  these lanes
  // if they stopped earlier than us.
  std::vector<mapping::ElementId> other_lanes;

  const StopSignStateProto* state = nullptr;
};

// Create a passed stop sign state.
StopSignStateProto CreatePassedState(const mapping::ElementId lane_id) {
  StopSignStateProto state;
  state.set_stop_sign_lane_id(lane_id.value());
  state.set_stop_state(StopSignStateProto::PASSED);
  return state;
}

// Create an approaching stop sign state.
StopSignStateProto CreateApproachingState(const mapping::ElementId lane_id) {
  StopSignStateProto state;
  state.set_stop_sign_lane_id(lane_id.value());
  state.set_stop_state(StopSignStateProto::APPROACHING);
  return state;
}

bool IsStopSignObject(const PlannerObject& object) {
  if (!IsVehicle(object.type())) return false;
  if (object.object_proto().parked()) return false;
  if (object.object_proto().offroad()) return false;
  if (!object.is_stationary()) return false;
  return true;
}

// Create the stop sign state when AV first stopped for the stop sign.
StopSignStateProto CreateFirstStoppedState(
    const PlannerSemanticMapManager& psmm,
    const SpacetimeTrajectoryManager& traj_mgr, double now_in_seconds,
    const StopSignLaneInfo& stop_sign) {
  StopSignStateProto state;
  state.set_stop_sign_lane_id(stop_sign.lane_id.value());
  state.set_stop_time(now_in_seconds);
  state.set_stop_state(StopSignStateProto::STOPPED);

  // Stores the front-most object on each lane controlled by the stop sign.
  struct ObjectLaneInfo {
    const PlannerObject* object = nullptr;
    double lane_fraction = 0.0;
  };
  std::map<mapping::ElementId, ObjectLaneInfo> lane_objects;
  for (const auto lane : stop_sign.other_lanes) {
    lane_objects[lane];  // Initialize.
  }

  for (const auto& traj : traj_mgr.stationary_object_trajs()) {
    const auto& object = traj->planner_object();
    // Skip if this object is not included by stop sign logic.
    if (!IsStopSignObject(object)) continue;

    // Find the lane associated with the object.
    ASSIGN_OR_CONTINUE(
        const mapping::LanePoint lane_pt,
        FindClosestLanePointToSmoothPointWithHeadingBoundAtLevel(
            psmm.GetLevel(), psmm, object.bounding_box().center(),
            /*heading=*/object.pose().theta(), /*heading_penalty_weight=*/0.0,
            /*closest_point_on_lane=*/nullptr,
            /*cutoff_distance=*/object.bounding_box().width()));

    auto iter = lane_objects.find(lane_pt.lane_id());
    if (iter == lane_objects.end()) continue;  // Not on stop line lane.
    // Only keeps the object that has the largest lane fraction.
    if (iter->second.lane_fraction < lane_pt.fraction()) {
      iter->second = ObjectLaneInfo{.object = &object,
                                    .lane_fraction = lane_pt.fraction()};
    }
  }
  // Save to yield object list.
  for (const auto& [lane_id, object_info] : lane_objects) {
    if (object_info.object != nullptr) {
      auto* yield_object = state.add_yield_objects();
      yield_object->set_stop_lane_id(lane_id.value());
      yield_object->set_object_id(object_info.object->id());
    }
  }
  return state;
}

// Check if AV should keep stopped at the stop sign to wait for the object. If
// the object is still stopped on the waiting lane, or the object is still in
// the intersection, we should keep stopped.
bool ShouldStopForObject(const PlannerSemanticMapManager& psmm,
                         mapping::ElementId stop_lane_id,
                         const PlannerObject& object) {
  const auto object_lanes =
      psmm.GetLanesInfoAtLevel(psmm.GetLevel(), object.bounding_box().center(),
                               object.bounding_box().diagonal() * 0.5);
  for (const auto* lane : object_lanes) {
    // If the object is still on the waiting lane, we should stop.
    if (lane->id == stop_lane_id) {
      return true;
    }
    // If the object is still in the intersection, we should stop.
    // TODO(lidong): We should check if the intersection ID is associated
    // with the stop sign.
    if (lane->is_in_intersection) {
      return true;
    }
  }
  return false;
}

// Return whether the lane act like a stop sign. It returns true if the lane
// associates to a stop sign.
bool HasStopSign(const mapping::LanePath::LaneSegment& lane_segment,
                 const mapping::LaneInfo& lane_info) {
  // If the end point of the lane is not inside the drive passage, skip.
  if (lane_segment.end_fraction < 1.0) {
    return false;
  }
  if (lane_info.proto->has_endpoint_associated_stop_sign()) {
    return true;
  }
  return false;
}

const StopSignStateProto* FindStopSignStateOrNull(
    mapping::ElementId lane_id,
    const ::google::protobuf::RepeatedPtrField<StopSignStateProto>&
        stop_sign_states) {
  for (const auto& state : stop_sign_states) {
    if (state.stop_sign_lane_id() == lane_id.value()) return &state;
  }
  return nullptr;
}

// Create the states when AV sees the stop sign for the first time.
StopSignStateProto CreateInitialStopSignState(
    const PlannerSemanticMapManager& psmm,
    const SpacetimeTrajectoryManager& traj_mgr, double now_in_seconds,
    double av_front_s, double av_velocity, const StopSignLaneInfo& stop_sign) {
  if (av_front_s < stop_sign.drive_passage_s - kStopMinDistance) {
    // AV is approaching the stop sign.
    return CreateApproachingState(stop_sign.lane_id);
  } else if (av_front_s > stop_sign.drive_passage_s + kStopMinDistance) {
    // AV has passed the stop sign.
    return CreatePassedState(stop_sign.lane_id);
  } else if (std::abs(av_velocity) < kStopMaxVelocity) {
    // Not very certain, judge by speed.
    return CreateFirstStoppedState(psmm, traj_mgr, now_in_seconds, stop_sign);
  } else {
    if (av_front_s > stop_sign.drive_passage_s) {
      return CreatePassedState(stop_sign.lane_id);
    } else {
      return CreateApproachingState(stop_sign.lane_id);
    }
  }
}

// Check if AV is still waiting for the stop sign memorized in last iteration.
// It returns an updated stop sign state proto if the waiting state is still
// valid.
StopSignStateProto UpdateStopSignState(
    const PlannerSemanticMapManager& psmm,
    const SpacetimeTrajectoryManager& traj_mgr, double now_in_seconds,
    const ApolloTrajectoryPointProto& av_state, double av_front_s,
    const StopSignLaneInfo& stop_sign) {
  // If we see this stop sign for the first time.
  if (stop_sign.state == nullptr) {
    return CreateInitialStopSignState(psmm, traj_mgr, now_in_seconds,
                                      av_front_s, av_state.v(), stop_sign);
  }

  const auto& last_state = *stop_sign.state;
  // If we have passed stop sign, skip.
  if (last_state.stop_state() == StopSignStateProto::PASSED) {
    return last_state;
  }
  // See if the state is changed from approaching to stopped.
  if (last_state.stop_state() == StopSignStateProto::APPROACHING) {
    if (av_front_s > stop_sign.drive_passage_s - kStopMinDistance &&
        av_state.v() < kStopMaxVelocity) {
      // Change to stop state.
      return CreateFirstStoppedState(psmm, traj_mgr, now_in_seconds, stop_sign);
    } else if (av_front_s > stop_sign.drive_passage_s + kStopMinDistance) {
      // Change state from approaching to passed, unlikely to happen but let's
      // handle it.
      return CreatePassedState(stop_sign.lane_id);
    } else {  // Still in approaching state.
      return last_state;
    }
  }

  // Handle the case that AV is stopped for the stop sign.
  constexpr double kStopSignWaitingTime = 1.5;  // Seconds.
  int num_waiting_vehicles = 0;
  // Check if there are stationary objects still stopped.
  const int num_objects = last_state.yield_objects_size();
  // A bit map to filter objects that we don't have to stop for.
  std::vector<int> object_keep_bit(num_objects, 1);
  for (int i = 0; i < num_objects; ++i) {
    const auto& yield_object = last_state.yield_objects(i);
    const auto trajs =
        traj_mgr.FindTrajectoriesByObjectId(yield_object.object_id());
    // This object has disappeared.
    if (trajs.empty()) {
      object_keep_bit[i] = 0;
    }

    if (ShouldStopForObject(psmm,
                            mapping::ElementId(yield_object.stop_lane_id()),
                            trajs.front()->planner_object())) {
      ++num_waiting_vehicles;
    } else {
      object_keep_bit[i] = 0;
    }
  }
  // When there is no object to wait, and AV has waited enough time, and can be
  // considered passed stop sign.
  const bool passed_stop_sign =
      num_waiting_vehicles == 0 &&
      now_in_seconds > last_state.stop_time() + kStopSignWaitingTime;
  if (passed_stop_sign) return CreatePassedState(stop_sign.lane_id);

  // Create a stopped state.
  StopSignStateProto new_state;
  new_state.set_stop_sign_lane_id(stop_sign.lane_id.value());
  new_state.set_stop_time(last_state.stop_time());
  new_state.set_stop_state(StopSignStateProto::STOPPED);
  for (int i = 0; i < num_objects; ++i) {
    if (object_keep_bit[i] == 1) {
      *new_state.add_yield_objects() = last_state.yield_objects(i);
    }
  }
  return new_state;
}

// Returns the stop signs on drive passage sorted by cumulative distance along
// the drive passage.
std::vector<StopSignLaneInfo> FindStopSigns(
    const PlannerSemanticMapManager& psmm, const DrivePassage& drive_passage,
    const ::google::protobuf::RepeatedPtrField<StopSignStateProto>&
        stop_sign_states) {
  std::vector<StopSignLaneInfo> stop_sign_lanes;
  const auto& lane_path = drive_passage.lane_path();
  for (const auto& lane_segment : lane_path) {
    const mapping::ElementId lane_id = lane_segment.lane_id;
    const auto* lane_info_ptr = psmm.FindLaneInfoOrNull(lane_id);
    if (lane_info_ptr == nullptr) continue;
    if (!HasStopSign(lane_segment, *lane_info_ptr)) continue;

    std::vector<mapping::ElementId> other_lanes;
    for (const auto& lane_interaction : lane_info_ptr->proto->interactions()) {
      const auto rule = lane_interaction.reaction_rule();
      if (rule == mapping::LaneInteractionProto::BOTH_STOP) {
        other_lanes.push_back(
            mapping::ElementId(lane_interaction.other_lane_id()));
      }
    }

    stop_sign_lanes.push_back(
        {.lane_id = lane_id,
         .drive_passage_s =
             lane_segment.end_s + drive_passage.lane_path_start_s(),
         .other_lanes = std::move(other_lanes),
         .state = FindStopSignStateOrNull(lane_id, stop_sign_states)});
  }
  absl::c_sort(stop_sign_lanes, [](const auto& lhs, const auto& rhs) {
    return lhs.drive_passage_s < rhs.drive_passage_s;
  });
  return stop_sign_lanes;
}

absl::StatusOr<ConstraintProto::StopLineProto> CreateStopLine(
    const DrivePassage& drive_passage, const StopSignLaneInfo& stop_sign) {
  ConstraintProto::StopLineProto stop_line;
  const auto curbs = drive_passage.QueryCurbPointAtS(stop_sign.drive_passage_s);
  if (!curbs.ok()) return absl::NotFoundError("Curb boundaries not found.");
  const HalfPlane hp(curbs->first, curbs->second);
  hp.ToProto(stop_line.mutable_half_plane());
  stop_line.set_s(stop_sign.drive_passage_s);
  stop_line.set_standoff(0.0);
  stop_line.set_time(0.0);
  stop_line.set_id(absl::StrCat("stop_sign_", stop_sign.lane_id));
  stop_line.mutable_source()->mutable_stop_sign()->set_lane_id(
      stop_sign.lane_id.value());
  return stop_line;
}

}  // namespace

absl::StatusOr<StopSignDeciderOutput> BuildStopSignConstraints(
    const PlannerSemanticMapManager& psmm,
    const SpacetimeTrajectoryManager& traj_mgr,
    const VehicleGeometryParamsProto& vehicle_geom,
    const DrivePassage& drive_passage, double now_in_seconds,
    const ApolloTrajectoryPointProto& plan_start_point,
    const ::google::protobuf::RepeatedPtrField<StopSignStateProto>&
        last_stop_sign_states) {
  SCOPED_QTRACE("BuildStopSignConstraints");

  ASSIGN_OR_RETURN(const auto plan_start_pos_sl,
                   drive_passage.QueryFrenetCoordinateAt(
                       Vec2d(plan_start_point.path_point().x(),
                             plan_start_point.path_point().y())));

  const auto stop_signs =
      FindStopSigns(psmm, drive_passage, last_stop_sign_states);

  StopSignDeciderOutput output;
  if (stop_signs.empty()) return output;

  std::vector<ConstraintProto::StopLineProto> stop_lines;
  stop_lines.reserve(stop_signs.size());
  std::vector<StopSignStateProto> stop_sign_states;
  const double av_front_s =
      plan_start_pos_sl.s + vehicle_geom.front_edge_to_center();
  for (const auto& stop_sign : stop_signs) {
    auto state = UpdateStopSignState(psmm, traj_mgr, now_in_seconds,
                                     plan_start_point, av_front_s, stop_sign);
    if (state.stop_state() != StopSignStateProto::PASSED) {
      auto stop_line = CreateStopLine(drive_passage, stop_sign);
      if (stop_line.ok()) {
        stop_lines.push_back(*stop_line);
      }
    }
    stop_sign_states.push_back(std::move(state));
  }
  output.stop_lines = std::move(stop_lines);
  output.stop_sign_states = std::move(stop_sign_states);
  return output;
}

}  // namespace planner
}  // namespace qcraft
