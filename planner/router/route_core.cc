#include "onboard/planner/router/route_core.h"

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type

#include <algorithm>
#include <cmath>
#include <limits>
#include <ostream>
#include <utility>

#include "absl/container/flat_hash_set.h"
#include "absl/status/status.h"
#include "absl/strings/str_format.h"
#include "absl/strings/str_join.h"
#include "glog/logging.h"

#include "onboard/container/strong_int.h"
#include "onboard/global/logging.h"
#include "onboard/global/timer.h"
#include "onboard/global/trace.h"
#include "onboard/maps/lane_path.h"
#include "onboard/maps/map_or_die_macros.h"
#include "onboard/maps/proto/semantic_map.pb.h"
#include "onboard/maps/semantic_map_util.h"
#include "onboard/math/util.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/router/plot_util.h"
#include "onboard/planner/router/road_conditions_process.h"
#include "onboard/planner/router/route_sections_util.h"
#include "onboard/planner/router/route_util.h"
#include "onboard/planner/router/router_flags.h"
#include "onboard/utils/map_util.h"
#include "onboard/utils/status_macros.h"

namespace qcraft::planner::route {

namespace {
constexpr double kInfiniteCost = std::numeric_limits<double>::infinity();

LaneSearchState ObtainLaneSearchStateFrom(
    const mapping::PointToLane& point_to_lane, const RouteParamProto& param,
    bool forward, bool use_time) {
  LaneSearchState lane_search_state;
  lane_search_state.id = point_to_lane.lane_proto;
  lane_search_state.fraction = point_to_lane.fraction;
  lane_search_state.time_from_start =
      use_time
          ? param.cost_param().match_point_coefficient() * point_to_lane.dist /
                Kph2Mps((point_to_lane.lane_proto->speed_limit_kph() + 1e-9))
          : 0.0;
  lane_search_state.dist_from_start =
      point_to_lane.dist * param.cost_param().match_point_coefficient();
  lane_search_state.parent = nullptr;
  lane_search_state.connect_type = ConnectTypeFromParent::kNoParent;
  lane_search_state.which = forward ? 0b0101 : 0b1001;
  return lane_search_state;
}

inline mapping::ElementId GetParentId(const LaneSearchState* lane_state) {
  return (lane_state->parent == nullptr)
             ? mapping::kInvalidElementId
             : mapping::ElementId(lane_state->parent->id->id());
}

inline const mapping::LaneProto& GetLaneProto(
    const LaneSearchState& lane_state) {
  return *lane_state.id;
}

inline double GetCurStateCost(const LaneSearchState& lane_search_state,
                              bool use_time) {
  return use_time ? lane_search_state.time_from_start
                  : lane_search_state.dist_from_start;
}

inline bool IsCurStateConnectWithParent(
    const LaneSearchState& lane_search_state) {
  return lane_search_state.connect_type == ConnectTypeFromParent::kNoParent ||
         lane_search_state.connect_type == ConnectTypeFromParent::kConnect;
}

inline bool AvoidCurLaneIfBusOnly(const mapping::LaneProto& lane_proto,
                                  bool is_bus) {
  // NOTE(zuowei): For time-sharing bus only lanes, we can not estimate
  // whether to avoid when arriving, so here we only consider all-day bus only
  // lanes and update avoid lanes to planner dynamicly.
  return !is_bus && lane_proto.type() == mapping::LaneProto::BUS_ONLY &&
         lane_proto.bus_only_day_intervals().empty();
}

bool CurLaneShouldAvoid(const RouteCorePrecondition& route_pre,
                        const mapping::LaneProto& lane_proto) {
  return route_pre.route_restrict_district.avoid_lanes.contains(
             mapping::ElementId(lane_proto.id())) ||
         AvoidCurLaneIfBusOnly(lane_proto, route_pre.is_bus);
}

double CalCurrentLaneCost(const mapping::v2::SemanticMapManager& smm,
                          const LaneSearchState& lane_state,
                          const RouteParamProto* param,
                          const RouteCorePrecondition& route_pre,
                          double start_frac, double end_frac) {
  const auto& route_restrict = route_pre.route_restrict_district;
  const auto& lane_proto = *lane_state.id;

  if (route_restrict.restrict_sections.contains(
          mapping::SectionId(lane_proto.section_id()))) {
    return kInfiniteCost;
  }

  if (mapping::IsPassengerVehicleAvoidLaneType(lane_proto.type())) {
    return param->cost_param().passenger_vehicle_avoid_lane_cost();
  }

  const double lane_length =
      GetLaneProtoLength(smm, lane_proto) * (end_frac - start_frac);
  const bool ignore_avoid_lane =
      (lane_state.parent == nullptr &&
       lane_length <= param->ignore_avoid_lane_length());

  double blacklist_cost = 0.0;
  if (route_restrict.avoid_lanes.contains(
          mapping::ElementId(lane_proto.id())) &&
      !ignore_avoid_lane) {
    blacklist_cost =
        param->cost_param().blacklist().base_cost() +
        param->cost_param().blacklist().cost_per_meters() * lane_length;
  }

  double bus_lane_cost = 0.0;
  if (AvoidCurLaneIfBusOnly(lane_proto, route_pre.is_bus) &&
      !ignore_avoid_lane) {
    bus_lane_cost =
        param->cost_param().blacklist().base_cost() +
        param->cost_param().blacklist().cost_per_meters() * lane_length;
  }

  double direction_cost = 0.0;
  if (lane_state.connect_type != ConnectTypeFromParent::kNoParent) {
    switch (lane_proto.direction()) {
      case mapping::LaneProto::STRAIGHT:
        break;
      case mapping::LaneProto::LEFT_TURN:
        direction_cost = param->cost_param().turn_cost().left_turn();
        break;
      case mapping::LaneProto::RIGHT_TURN:
        direction_cost = param->cost_param().turn_cost().right_turn();
        break;
      case mapping::LaneProto::UTURN:
        direction_cost = param->cost_param().turn_cost().u_turn();
        break;
    }
  }

  const auto it = route_restrict.extra_sections_cost.find(
      mapping::SectionId(lane_proto.section_id()));
  const double extra_section_cost =
      it == route_restrict.extra_sections_cost.end() ? 0.0 : it->second;

  const double length_cost = lane_length + blacklist_cost + bus_lane_cost;

  return (route_pre.use_time
              ? length_cost / Kph2Mps(lane_proto.speed_limit_kph())
              : length_cost) +
         direction_cost + extra_section_cost;
}

int CalcLaneChangeTimes(const LaneSearchState& lane_state) {
  int lc_times = 0;
  const auto* cur_lane_state = &lane_state;
  if (IsCurStateConnectWithParent(lane_state)) {
    return lc_times;
  }
  const auto link_from_parent = lane_state.connect_type;
  while (cur_lane_state != nullptr &&
         cur_lane_state->connect_type == link_from_parent) {
    ++lc_times;
    cur_lane_state = cur_lane_state->parent;
  }
  return lc_times;
}

// NOTE(zuowei): Linear superposition for lane change times.
inline bool IsValidBackwardLaneChange(
    const mapping::v2::SemanticMapManager& smm,
    const LaneSearchState& lane_state, int lc_times) {
  return GetLaneProtoLength(smm, GetLaneProto(lane_state)) *
             lane_state.fraction >=
         kMinLcLaneLength * lc_times;
}

bool IsValidForwardLaneChange(const mapping::v2::SemanticMapManager& smm,
                              const LaneSearchState& lane_state,
                              int lc_times_before, int lc_times_after,
                              double end_fraction, bool lc_left) {
  const auto* tmp_lane_state = &lane_state;

  for (int i = 0; i < lc_times_before; ++i) {
    tmp_lane_state = tmp_lane_state->parent;
  }

  double lc_length = 0.0;
  while (tmp_lane_state != nullptr &&
         IsCurStateConnectWithParent(*tmp_lane_state)) {
    const auto& neighbors = lc_left
                                ? tmp_lane_state->id->lane_neighbors_on_left()
                                : tmp_lane_state->id->lane_neighbors_on_right();
    if (neighbors.empty() || neighbors[0].opposite()) {
      break;
    }
    if (CrossSolidBoundary(GetLaneNeighborBoundary(neighbors[0]), lc_left)) {
      break;
    }
    const double end_frac =
        tmp_lane_state->id->section_id() == lane_state.id->section_id()
            ? end_fraction
            : 1.0;
    const double fraction = std::max(0.0, end_frac - tmp_lane_state->fraction);
    lc_length +=
        GetLaneProtoLength(smm, GetLaneProto(*tmp_lane_state)) * fraction;
    tmp_lane_state = tmp_lane_state->parent;
  }

  return lc_length >= kMinLcLaneLength * lc_times_after;
}

std::vector<std::pair<mapping::ElementId, double>> SearchForNeighbors(
    const mapping::v2::SemanticMapManager& smm,
    const LaneSearchState& lane_state, const RouteParamProto& route_param,
    bool lc_left, bool use_time, bool forward) {
  std::vector<std::pair<mapping::ElementId, double>> results;
  const auto& neighbors =
      lc_left ? GetLaneProto(lane_state).lane_neighbors_on_left()
              : GetLaneProto(lane_state).lane_neighbors_on_right();

  for (const auto& neighbor : neighbors) {
    if (neighbor.opposite()) continue;
    if (lane_state.parent != nullptr &&
        neighbor.other_id() == lane_state.parent->id->id()) {
      continue;
    }
    const int lc_times_before = CalcLaneChangeTimes(lane_state);
    const bool valid_lc =
        forward
            ? IsValidForwardLaneChange(smm, lane_state, lc_times_before,
                                       lc_times_before + 1,
                                       /*end_fraction=*/1.0, lc_left)
            : IsValidBackwardLaneChange(smm, lane_state, lc_times_before + 1);

    if (CrossSolidBoundary(GetLaneNeighborBoundary(neighbor), lc_left) ||
        !valid_lc) {
      results.push_back(
          std::make_pair(mapping::ElementId(neighbor.other_id()),
                         route_param.cost_param().invalid_lc_cost()));
      continue;
    }
    const double lc_cost =
        use_time ? route_param.cost_param().lc_cost()
                 : route_param.cost_param().lc_cost() *
                       Kph2Mps(GetLaneProto(lane_state).speed_limit_kph());
    results.push_back(
        std::make_pair(mapping::ElementId(neighbor.other_id()), lc_cost));
  }

  return results;
}

absl::StatusOr<std::vector<mapping::ElementId>> ExtractLaneIds(
    const SearchStateMap& state_map, mapping::ElementId link_lane_id,
    bool forward) {
  const auto* link_state_ptr = FindOrNull(state_map, link_lane_id.value());
  if (link_state_ptr == nullptr) {
    return absl::NotFoundError("Link lane id is not found.");
  }
  const auto* cur_search_state = *link_state_ptr;
  std::vector<mapping::ElementId> lane_ids;
  while (cur_search_state != nullptr) {
    lane_ids.push_back(mapping::ElementId(cur_search_state->id->id()));
    cur_search_state = cur_search_state->parent;
  }
  if (forward) {
    std::reverse(lane_ids.begin(), lane_ids.end());
  }
  VLOG(1) << "direction: " << forward
          << ", lane_ids: " << absl::StrJoin(lane_ids, ", ");
  return lane_ids;
}

absl::StatusOr<CompositeLanePath> ComposeCompositeLanePathInSameSection(
    const mapping::v2::SemanticMapManager& smm,
    const CompositeLanePath& source_clp, const CompositeLanePath& target_clp) {
  if (source_clp.IsEmpty()) {
    return absl::InvalidArgumentError("Source composite lane path is empty.");
  }
  if (target_clp.IsEmpty()) {
    return source_clp;
  }
  if (source_clp.IsConnectedTo(&smm, target_clp)) {
    return source_clp.Connect(&smm, target_clp);
  }

  SMM_LANE_PROTO_OR_RETURN(
      source_last_lane_proto, smm, source_clp.back().lane_id(),
      absl::NotFoundError(
          absl::StrCat("Failed to find lane: ", source_clp.back().lane_id())));
  SMM_LANE_PROTO_OR_RETURN(
      tar_first_lane_proto, smm, target_clp.front().lane_id(),
      absl::NotFoundError(
          absl::StrCat("Failed to find lane: ", target_clp.front().lane_id())));
  SMM_SECTION_PROTO_OR_RETURN(
      section_proto, smm, source_last_lane_proto->section_id(),
      absl::NotFoundError(absl::StrCat("Failed to find section: ",
                                       source_last_lane_proto->section_id())));
  constexpr double kErrorLength = 0.1;  // m.
  const double fraction = source_clp.back().fraction();
  if (source_last_lane_proto->section_id() !=
          tar_first_lane_proto->section_id() ||
      std::abs(fraction - target_clp.front().fraction()) *
              section_proto->average_length() >
          kErrorLength) {
    return absl::InvalidArgumentError(
        "Input two composite lane path can not be composed.");
  }
  int source_idx = 0, target_idx = 0;
  for (int i = 0; i < section_proto->lanes().size(); ++i) {
    if (section_proto->lanes()[i] == source_clp.back().lane_id().value()) {
      source_idx = i;
    }
    if (section_proto->lanes()[i] == target_clp.front().lane_id().value()) {
      target_idx = i;
    }
  }
  const bool lc_left = source_idx > target_idx;
  std::vector<mapping::LanePath> lane_paths = source_clp.lane_paths();
  std::vector<CompositeLanePath::TransitionInfo> transitions =
      source_clp.transitions();
  const auto tran = CompositeLanePath::TransitionInfo{
      .overlap_length = 0.0,
      .transition_point_fraction = fraction,
      .lc_left = lc_left,
      .lc_section_id = mapping::SectionId(section_proto->id()),
  };
  auto push_func = [&](int i) {
    transitions.push_back(tran);
    if (i != target_idx) {
      lane_paths.emplace_back(
          &smm,
          std::vector<mapping::ElementId>{
              mapping::ElementId(section_proto->lanes()[i])},
          fraction, fraction);
    }
  };
  if (lc_left) {
    for (int i = source_idx - 1; i >= target_idx; --i) {
      push_func(i);
    }
  } else {
    for (int i = source_idx + 1; i <= target_idx; ++i) {
      push_func(i);
    }
  }
  lane_paths.insert(lane_paths.end(), target_clp.lane_paths().begin(),
                    target_clp.lane_paths().end());
  transitions.insert(transitions.end(), target_clp.transitions().begin(),
                     target_clp.transitions().end());
  return CompositeLanePath(std::move(lane_paths), std::move(transitions));
}

}  // namespace

absl::StatusOr<std::pair<RouteSections, CompositeLanePath>>
RouteCore::SearchForRoutePathFromLanePoint(
    const RouteCorePrecondition& route_precondi,
    const std::vector<mapping::PointToLane>& origins,
    const std::vector<mapping::PointToLane>& dests) {
  SCOPED_QTRACE("RouteCore::SearchForRoutePathFromLanePoint");
  ScopedMultiTimer timer("route_core: search_for_route_path");
  if (origins.empty()) {
    return absl::InvalidArgumentError(
        "Alternative origins should not be empty.");
  }
  if (dests.empty()) {
    return absl::InvalidArgumentError(
        "Alternative destinations should not be empty.");
  }

  SearchStateComparator cmp{.use_time = route_precondi.use_time};
  std::vector<LaneSearchState*> forward_items;
  forward_items.reserve(4096);
  open_forward_ = OpenQueue(cmp, std::move(forward_items));
  std::vector<LaneSearchState*> backward_items;
  backward_items.reserve(4096);
  open_backward_ = OpenQueue(cmp, std::move(backward_items));

  forward_states_.clear();
  backward_states_.clear();
  state_ptrs_.clear();
  state_ptrs_.reserve(8192);

  for (const auto& point_to_lane : origins) {
    state_ptrs_.push_back(std::make_unique<LaneSearchState>(
        ObtainLaneSearchStateFrom(point_to_lane, *route_param_,
                                  /*forward=*/true, route_precondi.use_time)));
    forward_states_[point_to_lane.lane_proto->id()] = state_ptrs_.back().get();
    open_forward_.push(state_ptrs_.back().get());
  }

  for (const auto& point_to_lane : dests) {
    state_ptrs_.push_back(std::make_unique<LaneSearchState>(
        ObtainLaneSearchStateFrom(point_to_lane, *route_param_,
                                  /*forward=*/false, route_precondi.use_time)));
    backward_states_[point_to_lane.lane_proto->id()] = state_ptrs_.back().get();
    open_backward_.push(state_ptrs_.back().get());
  }
  timer.Mark("Initialize origins and destinations");

  mapping::ElementId link_lane_id = mapping::kInvalidElementId;
  // Hack, only for one lane in one map.
  if (origins.front().lane_proto->id() == dests.front().lane_proto->id() &&
      origins.front().fraction <= dests.front().fraction) {
    link_lane_id = mapping::ElementId(origins.front().lane_proto->id());
  } else {
    ASSIGN_OR_RETURN(link_lane_id, LaneLevelBiDijkstra(route_precondi));
  }
  timer.Mark("Lane level Bi-Dijkstra");

  auto result_or = GenerateRoutePath(link_lane_id, route_precondi);

  timer.Mark("Generate route path");
  state_code_ = SearchStateCode::kSearchSuccess;
  return result_or;
}

absl::StatusOr<mapping::ElementId> RouteCore::LaneLevelBiDijkstra(
    const RouteCorePrecondition& route_pre) {
  bool forward = true;
  double min_path_cost = std::numeric_limits<double>::infinity();
  mapping::ElementId link_id = mapping::kInvalidElementId;
  while (!open_forward_.empty() && !open_backward_.empty()) {
    auto* top_forward = open_forward_.top();
    auto* top_backward = open_backward_.top();

    const auto& cur_lane_proto =
        forward ? GetLaneProto(*top_forward) : GetLaneProto(*top_backward);
    if (forward) {
      open_forward_.pop();
      if (forward_states_[cur_lane_proto.id()]->IsClose()) {
        forward = !forward;
        continue;
      }
      top_forward->which ^= 0b0011;
    } else {
      open_backward_.pop();
      if (backward_states_[cur_lane_proto.id()]->IsClose()) {
        forward = !forward;
        continue;
      }
      top_backward->which ^= 0b0011;
    }

    const double top_cost = GetCurStateCost(*top_forward, route_pre.use_time) +
                            GetCurStateCost(*top_backward, route_pre.use_time);

    auto* cur_lane_state = forward ? top_forward : top_backward;

    const auto forward_iter = forward_states_.find(cur_lane_proto.id());
    const auto backward_iter = backward_states_.find(cur_lane_proto.id());
    if (forward_iter != forward_states_.end() &&
        backward_iter != backward_states_.end() &&
        forward_iter->second->fraction <= backward_iter->second->fraction) {
      double all_cost = 0.0;
      // Accumulate continuous LC.
      const int forward_lc_times = CalcLaneChangeTimes(*forward_iter->second);
      const int backward_lc_times = CalcLaneChangeTimes(*backward_iter->second);

      const bool opposite_lc = forward_lc_times != 0 &&
                               backward_lc_times != 0 &&
                               forward_iter->second->connect_type ==
                                   backward_iter->second->connect_type;
      if (!opposite_lc) {
        const bool lc_left = forward_lc_times != 0
                                 ? forward_iter->second->connect_type ==
                                       ConnectTypeFromParent::kLcLeft
                                 : backward_iter->second->connect_type ==
                                       ConnectTypeFromParent::kLcRight;
        if (!IsValidForwardLaneChange(
                *smm_, *forward_iter->second, forward_lc_times,
                forward_lc_times + backward_lc_times,
                backward_iter->second->fraction, lc_left)) {
          all_cost += route_param_->cost_param().invalid_lc_cost();
        }
      }

      all_cost += GetCurStateCost(*forward_iter->second, route_pre.use_time) +
                  GetCurStateCost(*backward_iter->second, route_pre.use_time) +
                  CalCurrentLaneCost(*smm_, *cur_lane_state, route_param_,
                                     route_pre, forward_iter->second->fraction,
                                     backward_iter->second->fraction);
      VLOG(2) << "forward: " << forward << " lane id: " << cur_lane_proto.id()
              << " top cost: " << top_cost << " all cost: " << all_cost
              << " min path cost: " << min_path_cost;

      if (all_cost < min_path_cost) {
        min_path_cost = all_cost;
        link_id = mapping::ElementId(cur_lane_proto.id());
      }

      if (min_path_cost <= top_cost) {
        VLOG(1) << "search link lane id: " << cur_lane_proto.id();
        break;
      }
    }
    ExpandState(route_pre, cur_lane_state, forward);

    forward = !forward;
  }

  if (link_id != mapping::kInvalidElementId) {
    return link_id;
  }

  if (open_forward_.empty()) {
    state_code_ = SearchStateCode::kForwardError;
  } else if (open_backward_.empty()) {
    state_code_ = SearchStateCode::kBackwardError;
  }
  return absl::NotFoundError("Can not find route lane path.");
}

void RouteCore::ExpandState(const RouteCorePrecondition& route_pre,
                            LaneSearchState* cur_state, bool forward) {
  auto& open_queue = forward ? open_forward_ : open_backward_;
  auto& states = forward ? forward_states_ : backward_states_;

  VLOG(2) << "direction: " << forward
          << ", current lane: " << cur_state->id->id()
          << ", prev lane: " << GetParentId(cur_state)
          << ", cum cost: " << GetCurStateCost(*cur_state, route_pre.use_time);
  VLOG(3) << "current open queue: " << OpenQueueDebugString(forward);

  if (!FLAGS_route_allow_express_way) {
    SMM_SECTION_PROTO_OR_RETURN(current_section_proto, *smm_,
                                cur_state->id->section_id(), (void)(0));
    if (IsCityExpressOrHighway(current_section_proto->road_class())) {
      VLOG(3) << "Ignore expend next FLAGS_route_allow_express_way:"
              << FLAGS_route_allow_express_way << ", "
              << current_section_proto->id();
      return;
    }
  }

  const double prev_cost = GetCurStateCost(*cur_state, route_pre.use_time);
  const auto* visited_lane_state_ptr = FindOrNull(states, cur_state->id->id());
  // visited multi-times.
  if (visited_lane_state_ptr != nullptr &&
      GetCurStateCost(**visited_lane_state_ptr, route_pre.use_time) <
          prev_cost) {
    return;
  }
  const double cost_on_lane =
      forward ? CalCurrentLaneCost(*smm_, *cur_state, route_param_, route_pre,
                                   cur_state->fraction, /*end_frac=*/1.0)
              : CalCurrentLaneCost(*smm_, *cur_state, route_param_, route_pre,
                                   /*start_frac=*/0.0, cur_state->fraction);

  if (cost_on_lane == std::numeric_limits<double>::infinity()) {
    return;
  }

  // Directly connected lanes.
  const auto& next_lane_ids = forward ? cur_state->id->outgoing_lanes()
                                      : cur_state->id->incoming_lanes();
  for (auto next_id : next_lane_ids) {
    SMM_LANE_PROTO_OR_CONTINUE(next_lane_proto, *smm_, next_id);
    const double new_cost = prev_cost + cost_on_lane;
    if (const auto* next_state = FindOrNull(states, next_id);
        next_state == nullptr ||
        new_cost < GetCurStateCost(**next_state, route_pre.use_time)) {
      state_ptrs_.push_back(std::make_unique<LaneSearchState>(LaneSearchState{
          .id = next_lane_proto,
          .fraction = forward ? 0.0 : 1.0,
          .time_from_start = (route_pre.use_time ? new_cost : 0.0),
          .dist_from_start = (route_pre.use_time ? 0.0 : new_cost),
          .parent = cur_state,
          .connect_type = ConnectTypeFromParent::kConnect,
          .which = forward ? 0b0101 : 0b1001}));

      states[next_id] = state_ptrs_.back().get();
      open_queue.push(state_ptrs_.back().get());
    }
  }

  // Lane change.
  const auto lc_fn = [&](const auto& neighbors, bool lc_left) {
    const auto link_parent_status = lc_left ? ConnectTypeFromParent::kLcLeft
                                            : ConnectTypeFromParent::kLcRight;
    for (const auto [neighbor_id, lc_cost] : neighbors) {
      SMM_LANE_PROTO_OR_CONTINUE(neighbor_lane_ptr, *smm_, neighbor_id);
      const double new_cost = prev_cost + lc_cost;
      if (const auto* neighbor_state =
              FindOrNull(states, neighbor_lane_ptr->id());
          neighbor_state == nullptr ||
          new_cost < GetCurStateCost(**neighbor_state, route_pre.use_time)) {
        state_ptrs_.push_back(std::make_unique<LaneSearchState>(LaneSearchState{
            .id = neighbor_lane_ptr,
            .fraction = cur_state->fraction,
            .time_from_start = (route_pre.use_time ? new_cost : 0.0),
            .dist_from_start = (route_pre.use_time ? 0.0 : new_cost),
            .parent = cur_state,
            .connect_type = link_parent_status,
            .which = forward ? 0b0101 : 0x1001}));
        states[neighbor_id.value()] = state_ptrs_.back().get();
        open_queue.push(state_ptrs_.back().get());
      }
    }
  };

  const auto left_neighbors =
      SearchForNeighbors(*smm_, *cur_state, *route_param_, /*lc_left=*/true,
                         route_pre.use_time, forward);
  lc_fn(left_neighbors, /*lc_left=*/true);
  const auto right_neighbors =
      SearchForNeighbors(*smm_, *cur_state, *route_param_, /*lc_left=*/false,
                         route_pre.use_time, forward);
  lc_fn(right_neighbors, /*lc_left=*/false);
}

absl::StatusOr<std::pair<RouteSections, CompositeLanePath>>
RouteCore::GenerateRoutePath(mapping::ElementId link_lane_id,
                             const RouteCorePrecondition& route_pre) const {
  ASSIGN_OR_RETURN(
      const auto forward_lane_ids,
      ExtractLaneIds(forward_states_, link_lane_id, /*forward=*/true));

  ASSIGN_OR_RETURN(
      const auto backward_lane_ids,
      ExtractLaneIds(backward_states_, link_lane_id, /*forward=*/false));

  std::vector<mapping::ElementId> lane_ids(forward_lane_ids.begin(),
                                           forward_lane_ids.end());
  lane_ids.insert(lane_ids.end(), backward_lane_ids.begin() + 1,
                  backward_lane_ids.end());

  const auto end_fraction_fn = [this, &lane_ids, &route_pre](
                                   const mapping::LaneProto* lane_proto,
                                   bool forward) {
    if (lane_proto == nullptr) return 0.0;
    if (forward && CurLaneShouldAvoid(route_pre, *lane_proto) &&
        lane_ids.front().value() != lane_proto->id()) {
      return 0.0;
    }
    return (lane_proto->section_id() ==
            backward_states_.at(lane_ids.back().value())->id->section_id())
               ? backward_states_.at(lane_ids.back().value())->fraction
               : 1.0;
  };

  const auto lc_left_fn = [this](mapping::ElementId source_id,
                                 mapping::ElementId target_id) {
    SMM_LANE_PROTO_OR_RETURN(source_proto, *smm_, source_id, false);
    SMM_SECTION_PROTO_OR_RETURN(section_proto, *smm_,
                                source_proto->section_id(), false);
    int source_idx = 0, target_idx = 0;
    for (int i = 0; i < section_proto->lanes().size(); ++i) {
      if (section_proto->lanes()[i] == source_id.value()) source_idx = i;
      if (section_proto->lanes()[i] == target_id.value()) target_idx = i;
    }
    return source_idx > target_idx;
  };
  std::vector<mapping::LanePath> lane_paths;
  std::vector<planner::CompositeLanePath::TransitionInfo> transitions;

  std::vector<mapping::ElementId> tmp_lane_ids;
  tmp_lane_ids.reserve(lane_ids.size());
  tmp_lane_ids.push_back(lane_ids.front());
  double start_fraction =
      forward_states_.at(lane_ids.front().value())->fraction;

  for (size_t i = 1; i < lane_ids.size(); ++i) {
    SMM_LANE_PROTO_OR_CONTINUE(prev_lane_proto, *smm_, lane_ids[i - 1]);
    if (mapping::IsOutgoingLane(*smm_, *prev_lane_proto, lane_ids[i])) {
      tmp_lane_ids.push_back(lane_ids[i]);
    } else {
      const bool forward = (i + 1 <= forward_lane_ids.size()) ? true : false;
      const double end_fraction = end_fraction_fn(prev_lane_proto, forward);
      lane_paths.emplace_back(smm_, tmp_lane_ids, start_fraction, end_fraction);
      tmp_lane_ids.clear();

      const bool lc_left = lc_left_fn(lane_ids[i - 1], lane_ids[i]);
      transitions.emplace_back(CompositeLanePath::TransitionInfo{
          .overlap_length = 0.0,
          .lc_left = lc_left,
          .lc_section_id = mapping::SectionId(prev_lane_proto->section_id())});

      tmp_lane_ids.push_back(lane_ids[i]);
      start_fraction = end_fraction;
    }
  }

  if (!tmp_lane_ids.empty()) {
    SMM_LANE_PROTO_OR_ERROR(lane_proto, *smm_, tmp_lane_ids.back());
    lane_paths.emplace_back(smm_, tmp_lane_ids, start_fraction,
                            end_fraction_fn(lane_proto,
                                            /*forward=*/false));
  }

  CompositeLanePath final_clp(std::move(lane_paths), std::move(transitions));
  return std::make_pair(RouteSectionsFromCompositeLanePath(*smm_, final_clp),
                        final_clp);
}

std::string RouteCore::OpenQueueDebugString(bool forward) const {
  std::string out;
  OpenQueue tmp_queue = forward ? open_forward_ : open_backward_;
  while (!tmp_queue.empty()) {
    const auto t = tmp_queue.top();
    const std::string str = forward ? "forward" : "backward";
    absl::StrAppendFormat(
        &out, "%s, lane id: %d, prev lane id: %d, time: %f, distance: %f\n",
        str, t->id->id(), GetParentId(t), t->time_from_start,
        t->dist_from_start);
    tmp_queue.pop();
  }
  return out;
}

absl::StatusOr<std::pair<RouteSections, CompositeLanePath>>
RouteCore::SearchForRoutePathAlongLanePoints(
    const RouteCorePrecondition& route_precondi,
    const std::vector<mapping::PointToLane>& origin,
    const std::vector<std::vector<mapping::PointToLane>>& dests,
    const CoordinateConverter* cc) {
  SCOPED_QTRACE("RouteCore::SearchForRoutePathAlongLanePoints");
  auto tmp_origin = origin;
  CompositeLanePath final_result;

  for (int i = 0; i < dests.size(); ++i) {
    ASSIGN_OR_RETURN(
        auto tmp_sections_lanes,
        SearchForRoutePathFromLanePoint(route_precondi, tmp_origin, dests[i]));
    if (FLAGS_router_send_route_core_state_to_canvas && cc != nullptr) {
      planner::SendRouteCoreStateToCanvas(*cc, state_ptrs_, std::to_string(i));
    }
    if (i == 0) {
      final_result = tmp_sections_lanes.second;
    } else {
      auto final_result_or = ComposeCompositeLanePathInSameSection(
          *smm_, final_result, tmp_sections_lanes.second);
      if (!final_result_or.ok()) {
        const auto end_lane_id = final_result.back().lane_id();
        mapping::PointToLane tmp_ptl;
        for (const auto& ptl : tmp_origin) {
          if (ptl.lane_proto->id() == end_lane_id.value()) {
            tmp_ptl = ptl;
            break;
          }
        }
        ASSIGN_OR_RETURN(auto try_sections_lanes,
                         SearchForRoutePathFromLanePoint(route_precondi,
                                                         {tmp_ptl}, dests[i]));
        final_result = final_result.Connect(smm_, try_sections_lanes.second);
        tmp_origin = dests[i];
        continue;
      }
      final_result = std::move(final_result_or).value();
    }
    tmp_origin = dests[i];
  }
  return std::make_pair(RouteSectionsFromCompositeLanePath(*smm_, final_result),
                        final_result);
}

}  // namespace qcraft::planner::route
