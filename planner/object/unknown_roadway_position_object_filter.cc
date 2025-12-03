#include "onboard/planner/object/unknown_roadway_position_object_filter.h"

#include "onboard/perception/tracker/multi_camera_fusion/proto/multi_camera_fusion.pb.h"
#include "onboard/planner/planner_flags.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/proto/perception/fusion/object.pb.h"

namespace qcraft {
namespace planner {

FilterReason::Type UnknownRoadwayPositionObjectFilter::Filter(
    const PlannerObject& object,
    const prediction::PredictedTrajectory& /*traj*/) const {
  // Don't filter objects that does not satisfy reflection object type.
  if ((object.type() == ObjectType::OT_CONE ||
       object.type() == ObjectType::OT_WARNING_TRIANGLE ||
       object.type() == ObjectType::OT_BARRIER ||
       object.type() == ObjectType::OT_BARRIER_ANTI_COLLISION_BUCKET ||
       object.type() == ObjectType::OT_BARRIER_ANTI_COLLISION_POST) &&
      (object.object_proto().road_way_position() ==
           multi_camera_fusion::RoadWayPositionType::RWPT_UNKNOWN ||
       FLAGS_planner_filter_static_object)) {
    return FilterReason::UNKNOWN_ROADWAY_POSITION_OBJECT;
  }

  return FilterReason::NONE;
}

}  // namespace planner
}  // namespace qcraft
