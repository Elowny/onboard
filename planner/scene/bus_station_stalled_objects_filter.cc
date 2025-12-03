#include "onboard/planner/scene/bus_station_stalled_objects_filter.h"

#include <utility>

#include "absl/status/statusor.h"
#include "google/protobuf/repeated_ptr_field.h"

#include "onboard/container/strong_int.h"
#include "onboard/maps/lane_point.h"
#include "onboard/maps/proto/semantic_map.pb.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/planner/router/route_sections_util.h"
#include "onboard/planner/util/lane_point_util.h"
#include "onboard/planner/util/spatial_search_util.h"

namespace qcraft {
namespace planner {
namespace {
bool IsFilteredType(ObjectType type) {
  switch (type) {
    case OT_VEHICLE:
    case OT_LARGE_VEHICLE:
      return true;
    case OT_PEDESTRIAN:
    case OT_CYCLIST:
    case OT_TRICYCLIST:
    case OT_UNKNOWN_STATIC:
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
}  // namespace

BusStationStalledObjectsFilter::BusStationStalledObjectsFilter(
    const PlannerSemanticMapManager& psmm,
    const RouteSections& route_sections) {
  const auto destination = route_sections.destination();
  if (destination.lane_id() == mapping::kInvalidElementId) return;
  const auto* lane_info_ptr = psmm.FindLaneInfoOrNull(destination.lane_id());
  // Destination info has not been loaded yet.
  if (lane_info_ptr == nullptr) return;
  const auto end_smooth_pos = ComputeLanePointPos(psmm, destination);
  const auto* bus_station_stop_area =
      psmm.GetNearestBusStationStopAreaAtLevel(psmm.GetLevel(), end_smooth_pos);
  if (bus_station_stop_area == nullptr ||
      bus_station_stop_area->lane_paths().empty()) {
    return;
  }
  station_lane_paths_.reserve(bus_station_stop_area->lane_paths().size());
  constexpr double kBackwardExtendLength = 20.0;  // m
  for (const auto& proto : bus_station_stop_area->lane_paths()) {
    const auto raw_lane_path =
        mapping::LanePath(psmm.semantic_map_manager(), proto);
    auto backward_extend_lane_path_or = BackwardExtendLanePathOnRouteSections(
        psmm, route_sections, raw_lane_path, kBackwardExtendLength);
    if (!backward_extend_lane_path_or.ok()) continue;
    station_lane_paths_.push_back(
        std::move(backward_extend_lane_path_or).value());
  }
}

bool BusStationStalledObjectsFilter::IsFiltered(
    const PlannerSemanticMapManager& psmm, const Vec2d& obj_pos,
    ObjectType obj_type) {
  if (station_lane_paths_.empty()) return false;
  if (!IsFilteredType(obj_type)) return false;
  for (const auto& lane_path : station_lane_paths_) {
    double arc_len;
    const bool is_on_lane = IsPointOnLanePathAtLevel(
        psmm.GetLevel(), psmm, obj_pos, lane_path, &arc_len);
    if (is_on_lane) return true;
  }
  return false;
}

}  // namespace planner
}  // namespace qcraft
