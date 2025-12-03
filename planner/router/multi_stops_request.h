#ifndef ONBOARD_PLANNER_ROUTER_MULTI_STOPS_REQUEST_H_
#define ONBOARD_PLANNER_ROUTER_MULTI_STOPS_REQUEST_H_

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type

#include <string>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"

#include "common/proto/drive_mission.pb.h"

#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/maps/lane_point.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/maps/v2/semantic_map_manager.h"
#include "onboard/planner/router/road_conditions_process.h"
#include "onboard/planner/router/util/map_index.h"

namespace qcraft {
namespace planner {

class MultipleStopsRequest;

absl::StatusOr<MultipleStopsRequest> BuildMultipleStopsRequest(
    const mapping::v2::SemanticMapManager& semantic_map_manager,

    const route::MapIndex* map_index,
    const RoutingRequestProto& routing_request_proto);

class MultipleStopsRequest {
 public:
  MultipleStopsRequest() = default;
  explicit MultipleStopsRequest(
      std::vector<std::vector<RoutingDestinationProto>> destinations,
      std::vector<std::vector<mapping::LanePoint>> lane_point_destinations,
      bool infinite_loop, bool skip_past_stops, int num_destinations,
      std::vector<std::string> stop_name_list,
      route::RouteRestrictDistrict route_restrict_district,
      MultipleStopsRequestProto multi_stops_request_proto,
      std::string_view request_id, bool is_from_routing_request,
      RoutingRequestProto route_request_proto);

 public:
  void ToRoutingRequestProto(RoutingRequestProto* proto) const {
    ToRoutingRequestFromMultipleStops(proto);
  }

  bool empty() const { return destinations_.empty(); }

  bool infinite_loop() const { return infinite_loop_; }

  bool skip_past_stops() const { return skip_past_stops_; }

  int stop_size() const { return destinations_.size(); }

  int destination_size() const { return num_destinations_; }

  absl::string_view request_id() const { return request_id_; }

  const std::vector<std::vector<RoutingDestinationProto>>& destinations_proto()
      const {
    return destinations_;
  }

  const std::vector<std::vector<mapping::LanePoint>>& lane_point_destinations()
      const {
    return lane_point_destinations_;
  }

  const absl::flat_hash_set<mapping::ElementId>& avoid_lanes() const {
    return route_restrict_district_.avoid_lanes;
  }

  const route::RouteRestrictDistrict& route_restrict_district() const {
    return route_restrict_district_;
  }

  const std::vector<std::string>& stop_name_list() const {
    return stop_name_list_;
  }

  const RoutingDestinationProto& first_destination() const {
    return destinations_.front().front();
  }

  const RoutingDestinationProto& destination(int index) const {
    const auto index_pair = GetIndexPair(index);
    return destination(index_pair.first, index_pair.second);
  }

  const RoutingDestinationProto& destination(int stop_index,
                                             int dest_index) const {
    QCHECK(CheckIndex(stop_index, dest_index));
    return destinations_[stop_index][dest_index];
  }

  absl::StatusOr<mapping::LanePoint> ComputeDestinationLanePointByTotalIndex(
      const mapping::v2::SemanticMapManager& semantic_map_manager,
      const route::MapIndex* map_index, int index) const;

  absl::StatusOr<mapping::LanePoint> ComputeDestinationLanePoint(
      const mapping::v2::SemanticMapManager& semantic_map_manager,
      const route::MapIndex* map_index, int stop_index, int dest_index) const;

  absl::StatusOr<RoutingRequestProto> GenerateRoutingRequestProtoToNextStop(
      const mapping::v2::SemanticMapManager& semantic_map_manager,
      int next_dest_index) const;

  int GetStopIndex(int dest_index) const;

  int GetStopSerialNumber(int index) const;

  bool CheckIndex(int stop_index, int dest_index) const;

  bool HasNextDestinationIndex(int now_index, int* next_index) const;

  absl::StatusOr<int> GetNextDestinationIndex(int now_index) const;

  const MultipleStopsRequestProto& multi_stops_request_proto() const {
    return multi_stops_request_;
  }

  std::pair<int, int> GetIndexPair(int overall_index) const;

  void Clear();

  void ToRoutingRequestFromMultipleStops(RoutingRequestProto* proto) const;

 private:
  bool is_from_routing_request() const { return is_from_routing_request_; }

  // Example: loop
  // No.1 stop : destination_1_1, destination_1_2, destination_1_3(stop_1)/
  // No.2 stop : destination_2_1(stop_2) /
  // No.3 stop : destination_3_1, destination_3_2(stop_3)/
  //
  //            d_1_1  --->  d_1_2 --->   d_1_3(stop)
  //             ^                            |
  //             |                            |
  //             |                            !
  //         d_3_2(stop)  <---  d_3_1  <---  d_2_1(stop)
  //
  std::vector<std::vector<RoutingDestinationProto>> destinations_;
  std::vector<std::vector<mapping::LanePoint>> lane_point_destinations_;
  bool infinite_loop_ = false;
  bool skip_past_stops_ = true;
  int num_destinations_ = 0;
  std::vector<std::string> stop_name_list_;
  route::RouteRestrictDistrict route_restrict_district_;
  MultipleStopsRequestProto multi_stops_request_;
  std::string request_id_;
  bool is_from_routing_request_ = false;
  RoutingRequestProto route_request_proto_;  // Should keep the origin request.
};

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_ROUTER_MULTI_STOPS_REQUEST_H_
