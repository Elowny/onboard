#include "onboard/planner/decision/pedestrians_decider.h"

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type

#include <algorithm>
#include <cmath>
#include <ostream>
#include <string>
#include <string_view>
#include <utility>

#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/types/span.h"
#include "glog/logging.h"

#include "onboard/eval/qevent.h"
#include "onboard/eval/qevent_base.h"
#include "onboard/global/logging.h"
#include "onboard/global/trace.h"
#include "onboard/maps/lane_path.h"
#include "onboard/maps/map_or_die_macros.h"
#include "onboard/maps/maps_common.h"
#include "onboard/math/frenet_common.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/object/planner_object.h"
#include "onboard/planner/object/spacetime_object_trajectory.h"
#include "onboard/planner/second_order_trajectory_point.h"
#include "onboard/prediction/object_prediction.h"
#include "onboard/prediction/predicted_trajectory.h"
#include "onboard/proto/perception/fusion/object.pb.h"
#include "onboard/utils/source_location.h"
#include "onboard/utils/status_macros.h"

namespace qcraft {
namespace planner {

namespace {
// This value is equal to speed_region.end_s() subtract speed_region.start_s(),
// manually adjustable.
constexpr double kPedestrianSpeedRegionLength = 10.0;  // m

// For Pedestrian constraints, to avoid AV slowing down too much and too
// quickly,we define  this attenuation factor. manually adjustable.
constexpr double kVelocityAttenuationCoefficient = 0.9;  // rate

// Min speed limit for speed region ,prevent decelerate to zero
constexpr double kVelocityMinLimit = 2.5;  // m/s

// This value expands the range of path sl boundary, which means pedestrian is
// considered only if it is within this range.
constexpr double kPedestrianEnterPathBoundaryBuffer = 1.0;  // m

// This value defines the maximum comfortable deceleration, if AV slows down
// sharply because of this pedestrian, ignore it.
constexpr double kComfortableDeceleration = -1.5;  // m/s^2

// Using to judge object moving direction whether perpendicular to path forward
// direction
constexpr double kPerpendicularThresholdAngle = M_PI / 6.0;

bool IsPedestrianType(ObjectType type) {
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

bool IsPedestrianInFrontOfAVFrontEdge(
    const qcraft::VehicleGeometryParamsProto& vehicle_geometry_params,
    const FrenetCoordinate& frenet_plan_start_pos,
    const FrenetBox& frenet_box) {
  return frenet_plan_start_pos.s +
             vehicle_geometry_params.front_edge_to_center() <
         frenet_box.s_min;
}

bool IsPedestrianEnterPathBoundaryWithBuffer(const PathSlBoundary& sl_boundary,
                                             const FrenetBox& frenet_box) {
  const auto [right_l, left_l] =
      sl_boundary.QueryTargetBoundaryL(frenet_box.s_min);

  return frenet_box.l_max >= right_l - kPedestrianEnterPathBoundaryBuffer &&
         frenet_box.l_min <= left_l + kPedestrianEnterPathBoundaryBuffer;
}

bool IsPedestrianCausingAVBrakeComfortable(
    double ego_speed,
    const qcraft::VehicleGeometryParamsProto& vehicle_geometry_params,
    const FrenetCoordinate& frenet_plan_start_pos,
    const FrenetBox& frenet_box) {
  const double comfortable_stop_dist =
      0.5 * ego_speed * ego_speed / std::fabs(kComfortableDeceleration);
  return frenet_box.s_min > comfortable_stop_dist + frenet_plan_start_pos.s +
                                vehicle_geometry_params.front_edge_to_center();
}

bool IsPedestrianOnCrosswalk(const PlannerSemanticMapManager& psmm,
                             const DrivePassage& passage,
                             const mapping::LanePath& lane_path_from_start,
                             double s_offset, const FrenetBox& frenet_box) {
  for (const auto& seg : lane_path_from_start) {
    SMM_ASSIGN_LANE_OR_CONTINUE(lane_info, psmm, seg.lane_id);

    for (const auto& cw : lane_info.Crosswalks()) {
      // calculate crosswalk start_s & end_s
      const double cw_start_s = lane_path_from_start.LaneIndexPointToArclength(
                                    seg.lane_index, cw.second.x()) +
                                s_offset + passage.lane_path_start_s();
      const double cw_end_s = lane_path_from_start.LaneIndexPointToArclength(
                                  seg.lane_index, cw.second.y()) +
                              s_offset + passage.lane_path_start_s();

      if (frenet_box.s_max > cw_start_s && frenet_box.s_min < cw_end_s) {
        return true;
      }
    }
  }
  return false;
}
bool IsPedestrianInTheMiddleOfTheLane(
    const qcraft::VehicleGeometryParamsProto& vehicle_geometry_params,
    const FrenetCoordinate& frenet_plan_start_pos,
    const FrenetBox& frenet_box) {
  return frenet_box.l_max >
             frenet_plan_start_pos.l -
                 vehicle_geometry_params.right_edge_to_center() &&
         frenet_box.l_min < frenet_plan_start_pos.l +
                                vehicle_geometry_params.left_edge_to_center();
}

bool IsCrossCenterLine(const DrivePassage& passage,
                       const PlannerObject& object) {
  const auto& trajs = object.prediction().trajectories();

  const auto is_two_points_accross_center_line = [&passage](const Vec2d& p1,
                                                            const Vec2d& p2) {
    const auto& p1_sl = passage.QueryFrenetCoordinateAt(p1);
    const auto& p2_sl = passage.QueryFrenetCoordinateAt(p2);

    // whether two points cross the center line
    if (p1_sl.ok() && p2_sl.ok()) {
      return p1_sl.value().l * p2_sl.value().l < 0;
    }
    return false;
  };

  for (const auto& pred_traj : trajs) {
    const auto& pred_traj_points = pred_traj.points();
    int pred_traj_size = pred_traj_points.size();
    for (int i = 0; i < pred_traj_size - 1; ++i) {
      const Vec2d& p1 = pred_traj_points[i].pos();
      const Vec2d& p2 = pred_traj_points[i + 1].pos();
      if (is_two_points_accross_center_line(p1, p2)) {
        return true;
      }
    }
  }

  return false;
}

bool IsObjectMovingPerpendicularToPath(const DrivePassage& passage,
                                       const PlannerObject& object) {
  const Vec2d object_velocity = object.velocity();
  const Vec2d object_pos = object.pose().pos();
  const auto tangent_or = passage.QueryTangentAt(object_pos);
  if (!tangent_or.ok()) return false;

  const double path_perpendicular_angle = tangent_or.value().Perp().Angle();
  const double object_moving_angle = object_velocity.Angle();

  const double angle_diff =
      std::abs(NormalizeAngle(path_perpendicular_angle - object_moving_angle));

  return angle_diff < kPerpendicularThresholdAngle ||
         angle_diff > M_PI - kPerpendicularThresholdAngle;
}

std::vector<const SpacetimeObjectTrajectory*>
FindPedestriansAssociateWithCurrentLane(
    const qcraft::VehicleGeometryParamsProto& vehicle_geometry_params,
    const PlannerSemanticMapManager& psmm,
    const ApolloTrajectoryPointProto& plan_start_point,
    const DrivePassage& passage, const mapping::LanePath& lane_path_from_start,
    double s_offset, const PathSlBoundary& sl_boundary,
    absl::Span<const SpacetimeObjectTrajectory> st_trajs) {
  FUNC_QTRACE();
  std::vector<const SpacetimeObjectTrajectory*> pedestrians_related;

  const Vec2d plan_start_pos(plan_start_point.path_point().x(),
                             plan_start_point.path_point().y());

  const auto frenet_plan_start_pos =
      passage.QueryFrenetCoordinateAt(plan_start_pos);
  if (!frenet_plan_start_pos.ok()) {
    VLOG(2) << "!!! Calculate plan start point frenet coordinate failed, no "
               "pedestrian decision!!!";
    return pedestrians_related;
  }

  const double plan_start_v = plan_start_point.v();

  for (const auto& st_traj : st_trajs) {
    // calculate general usbale variable
    const auto& object = st_traj.planner_object();

    const auto frenet_box = passage.QueryFrenetBoxAtContour(object.contour());
    if (!frenet_box.ok()) {
      VLOG(2) << st_traj.traj_id() << ", " << frenet_box.status();
      continue;
    }

    // remove non-pedestrian objects
    if (!IsPedestrianType(object.type())) {
      VLOG(2) << st_traj.traj_id() << " type is "
              << ObjectType_Name(object.type()) << ", ignore this object";
      continue;
    }

    // remove pedestrian objects those on the crosswalk
    if (IsPedestrianOnCrosswalk(psmm, passage, lane_path_from_start, s_offset,
                                frenet_box.value())) {
      VLOG(2) << st_traj.traj_id() << " on crosswalk, ignore this object";
      continue;
    }

    // remove pedestrian objects those behind AV front edge
    if (!IsPedestrianInFrontOfAVFrontEdge(vehicle_geometry_params,
                                          frenet_plan_start_pos.value(),
                                          frenet_box.value())) {
      VLOG(2) << st_traj.traj_id()
              << " behind AV front edge, ignore this object";
      continue;
    }

    // remove pedestrian objects those far away from path boundary
    if (!IsPedestrianEnterPathBoundaryWithBuffer(sl_boundary,
                                                 frenet_box.value())) {
      VLOG(2) << st_traj.traj_id()
              << " far away from path boundary, ignore this object";
      continue;
    }

    // only for cyclist objects, check move direction
    if (object.type() == OT_CYCLIST &&
        !IsObjectMovingPerpendicularToPath(passage, object)) {
      VLOG(2) << st_traj.traj_id()
              << " moving parallel to path, ignore this object";
      continue;
    }

    // remove pedestrian objects those causing AV brake sharply
    if (!IsPedestrianCausingAVBrakeComfortable(
            plan_start_v, vehicle_geometry_params,
            frenet_plan_start_pos.value(), frenet_box.value())) {
      VLOG(2) << st_traj.traj_id()
              << " causing AV brake sharply, ignore this object";
      continue;
    }

    // remove pedestrian objects those are not crossing the center line or in
    // the middle of the lane
    if (!IsCrossCenterLine(passage, object) &&
        !IsPedestrianInTheMiddleOfTheLane(vehicle_geometry_params,
                                          frenet_plan_start_pos.value(),
                                          frenet_box.value())) {
      VLOG(2) << st_traj.traj_id()
              << " isn't across the center line or in the middle of the lane, "
                 "ignore this object";
      continue;
    }

    pedestrians_related.emplace_back(&st_traj);
  }

  return pedestrians_related;
}

absl::StatusOr<ConstraintProto::SpeedRegionProto> GeneratePedestrianConstraint(
    double v, const DrivePassage& passage, const PlannerObject& object) {
  ASSIGN_OR_RETURN(const auto frenet_box,
                   passage.QueryFrenetBoxAtContour(object.contour()));

  double start_s = frenet_box.s_min;
  double end_s = frenet_box.s_min + kPedestrianSpeedRegionLength;

  ASSIGN_OR_RETURN(const auto start_point, passage.QueryPointXYAtS(start_s));
  ASSIGN_OR_RETURN(const auto end_point, passage.QueryPointXYAtS(end_s));

  ConstraintProto::SpeedRegionProto speed_region;
  start_point.ToProto(speed_region.mutable_start_point());
  end_point.ToProto(speed_region.mutable_end_point());
  speed_region.set_start_s(start_s);
  speed_region.set_end_s(end_s);
  speed_region.set_max_speed(
      std::max(v * kVelocityAttenuationCoefficient, kVelocityMinLimit));
  speed_region.set_min_speed(0.0);
  speed_region.mutable_source()->mutable_pedestrian_object()->set_id(
      object.id());
  speed_region.set_id(absl::StrCat("Pedestrian ", object.id()));

  return speed_region;
}

}  // namespace

absl::StatusOr<std::vector<ConstraintProto::SpeedRegionProto>>
BuildPedestriansConstraints(
    const qcraft::VehicleGeometryParamsProto& vehicle_geometry_params,
    const PlannerSemanticMapManager& psmm,
    const ApolloTrajectoryPointProto& plan_start_point,
    const DrivePassage& passage, const mapping::LanePath& lane_path_from_start,
    double s_offset, const PathSlBoundary& sl_boundary,
    const SpacetimeTrajectoryManager& st_traj_mgr) {
  FUNC_QTRACE();
  std::vector<ConstraintProto::SpeedRegionProto> ped_speed_regions;

  // select pedestrian objects
  const auto pedestrians_related = FindPedestriansAssociateWithCurrentLane(
      vehicle_geometry_params, psmm, plan_start_point, passage,
      lane_path_from_start, s_offset, sl_boundary, st_traj_mgr.trajectories());

  // generate speed regions
  for (const auto& ped : pedestrians_related) {
    const auto speed_region = GeneratePedestrianConstraint(
        plan_start_point.v(), passage, ped->planner_object());

    if (speed_region.ok()) {
      QEVENT_EVERY_N_SECONDS(
          "jiayukou", "prebrake_for_pedestrian",
          /*seconds=*/5.0, [&](QEvent* qevent) {
            qevent->AddField("object_id",
                             speed_region->source().pedestrian_object().id());
          });
      ped_speed_regions.emplace_back(speed_region.value());
      VLOG(2) << " + + + Pedestrian:\t" << ped->traj_id()
              << ", generate speed region at:\t"
              << speed_region.value().start_s();
    } else {
      VLOG(2) << " - - - Pedestrian:\t" << ped->traj_id() << ", "
              << speed_region.status().ToString();
    }
  }

  return ped_speed_regions;
}

}  // namespace planner

}  // namespace qcraft
