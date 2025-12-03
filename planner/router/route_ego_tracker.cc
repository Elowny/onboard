#include "onboard/planner/router/route_ego_tracker.h"

#include <algorithm>
#include <ostream>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "absl/hash/hash.h"
#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/time/time.h"
#include "glog/logging.h"
#include "google/protobuf/repeated_ptr_field.h"

#include "onboard/container/strong_int.h"
#include "onboard/global/clock.h"
#include "onboard/lite/logging.h"
#include "onboard/maps/lane_point.h"
#include "onboard/maps/map_or_die_macros.h"
#include "onboard/maps/proto/lane_path.pb.h"
#include "onboard/maps/proto/semantic_map.pb.h"
#include "onboard/maps/spatial_search_util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/router/route_util.h"
#include "onboard/utils/source_location.h"

namespace qcraft {
namespace planner {
namespace {

// point to lane distance,same as scheduler.
constexpr double kProjectToLaneRadiusError = kDefaultLaneWidth * 1.5;
// The min reroute distance if reroute when map-match failed
constexpr double kRerouteDistanceThresholdMeters = 200.0;
}  // namespace

// TODO(xiang): change section_lane_ids into std::int64_t
absl::StatusOr<PointToCompositeLanePath> FindLaneFromLastCompositeIndex(
    const CompositeLanePathProto& composite_lane_path,
    const CompositeLanePath::CompositeIndex& last_composite_index,
    absl::Span<const int64_t> section_lane_ids,
    mapping::ElementId to_find_lane_id) {
  for (int i = last_composite_index.lane_path_index;
       i < composite_lane_path.lane_paths().size(); ++i) {
    const auto& lane_path = composite_lane_path.lane_paths(i);
    int j = (i == last_composite_index.lane_path_index)
                ? last_composite_index.lane_index
                : 0;
    for (; j < lane_path.lane_ids().size(); j++) {
      const auto lane_id = lane_path.lane_ids(j);
      for (const auto neighbour_lane_id : section_lane_ids) {
        if (neighbour_lane_id == lane_id) {
          PointToCompositeLanePath point_to_composite_lane_path;
          point_to_composite_lane_path.point_on_route_lane =
              (to_find_lane_id == mapping::ElementId(lane_id));
          point_to_composite_lane_path.composite_index = {i, j};
          // The vehicle is one the lane path.
          point_to_composite_lane_path.route_lane_id =
              mapping::ElementId(lane_id);
          return point_to_composite_lane_path;
        }
      }
    }
  }
  return absl::NotFoundError(
      absl::StrCat("Cannot find the lane id ", to_find_lane_id));
}

RouteTrackerDebugProto GetTrackerInfoOnRoute(
    const mapping::v2::SemanticMapManager& smm,
    const route::MapIndex* map_index, mapping::LevelId level_id,
    const RouteManagerState& rm_state, const Vec2d& global, double /*heading*/,
    double av_speed_x) {
  RouteTrackerDebugProto tracker_proto;
  tracker_proto.set_success(false);

  const std::vector<mapping::PointToLane> match_lanes = PointToNearLanesAtLevel(
      *map_index, level_id, global, kProjectToLaneRadiusError);
  absl::flat_hash_set<int64_t> uniq_sections;
  std::vector<mapping::PointToLane> filtered_match_lanes;
  for (const auto& point_to_lane : match_lanes) {
    const auto insert_it =
        uniq_sections.insert(point_to_lane.lane_proto->section_id());
    if (insert_it.second) {
      filtered_match_lanes.push_back(point_to_lane);
    }
  }

  for (const auto& point_to_lane : filtered_match_lanes) {
    const mapping::LaneProto& lane_proto = *point_to_lane.lane_proto;
    const mapping::LanePoint lane_point = {mapping::ElementId(lane_proto.id()),
                                           point_to_lane.fraction};
    SMM_SECTION_PROTO_OR_CONTINUE(section, smm, lane_proto.section_id());
    const auto& lane_ids = section->lanes();
    VLOG(3) << "pose:" << global.DebugStringFullPrecision()
            << ", match point:" << lane_point.DebugString()
            << ", lane_id:" << lane_proto.id()
            << ", distance:" << point_to_lane.dist
            << ",composite index:" << rm_state.composite_index.DebugString();

    RouteTrackerDebugProto::TrackerMatchProto* cur_tracker_match =
        tracker_proto.add_candidates();

    // Save state
    lane_point.ToProto(cur_tracker_match->mutable_lane_point());
    cur_tracker_match->set_project_dist(point_to_lane.dist);

    const auto point_to_composite_lane_path_or = FindLaneFromLastCompositeIndex(
        rm_state.route.lane_path(), rm_state.composite_index, lane_ids,
        lane_point.lane_id());
    if (point_to_composite_lane_path_or.ok()) {
      const auto& composite_index =
          point_to_composite_lane_path_or->composite_index;
      const auto cur_time_micros = absl::ToUnixMicros(Clock::Now());
      const double travel_distance =
          (rm_state.last_lane_point.lane_id() == mapping::kInvalidElementId)
              ? 0.0
              : TravelDistanceBetween(smm, rm_state.route.lane_path(),
                                      rm_state.composite_index,
                                      rm_state.last_lane_point.fraction(),
                                      composite_index, lane_point.fraction());
      VLOG(2) << "travel distance:" << travel_distance
              << ", max_distance:" << kRerouteDistanceThresholdMeters;
      cur_tracker_match->set_travel_distance(travel_distance);
      if (rm_state.rms_debug.last_update_time() != -1 &&
          !IsTravelMatchedValid(
              rm_state.composite_index, rm_state.last_lane_point.fraction(),
              composite_index, lane_point.fraction(),
              rm_state.rms_debug.last_update_time(), cur_time_micros,
              travel_distance, av_speed_x, kRerouteDistanceThresholdMeters,
              rm_state.vx_p90 * 1.1)) {
        // To estimate the upper threshold of travel distance more accurate, so
        // the estimate speed multiply a small factor.
        continue;
      }
      // Ego tracker failed if section matched but the lane restrict is obeyed
      if (!point_to_composite_lane_path_or->point_on_route_lane &&
          !IsTwoLaneConnectedWithBrokenWhites(
              smm, lane_point.lane_id(),
              point_to_composite_lane_path_or->route_lane_id)) {
        QLOG_EVERY_N_SEC(INFO, 5)
            << "Although the vehicle is on the route, but may (not "
               "sure) obey the traffic rule.";
      }

      // Set the selected match result.
      cur_tracker_match->set_selected(true);
      composite_index.ToProto(tracker_proto.mutable_composite_index());
      tracker_proto.set_travel_distance(travel_distance);
      tracker_proto.set_route_lane_id(
          point_to_composite_lane_path_or->route_lane_id.value());
      tracker_proto.set_point_on_route_lane(
          point_to_composite_lane_path_or->point_on_route_lane);
      tracker_proto.set_success(true);
      lane_point.ToProto(tracker_proto.mutable_lane_point());
      composite_index.ToProto(tracker_proto.mutable_composite_index());

      return tracker_proto;
    } else {
      VLOG(3) << "Car to path failed. route lane path:\n"
              << rm_state.route.lane_path().ShortDebugString() << "\n current:"
              << rm_state.route_from_current.lane_path().ShortDebugString()
              << "\n index:"
              << rm_state.composite_index.DebugString()
              // << "\n section ids:" << absl::StrJoin(section_ids, ",")
              << "\n lane_point:" << lane_point.DebugString()
              << "\n pose:" << global.DebugStringFullPrecision()
              << "\n:" << point_to_composite_lane_path_or.status().message();
      cur_tracker_match->set_selected(false);
      cur_tracker_match->clear_travel_distance();
    }
  }
  return tracker_proto;
}
}  // namespace planner
}  // namespace qcraft
