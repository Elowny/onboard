#ifndef ONBOARD_PLANNER_ROUTER_ROUTE_STRUCT_H_
#define ONBOARD_PLANNER_ROUTER_ROUTE_STRUCT_H_
#include <cstdint>

namespace qcraft::planner::route {
// default small heap, T[0] element is only used for sentinel.
enum SearchQueueCode : int {
  kNone = 0b0,
  kOpen = 0b1,
  kClose = 0b10,
  kForward = 0b100,
  kBackward = 0b1000
};

enum class ConnectTypeFromParent : int {
  kConnect = 0,
  kLcLeft = 1,
  kLcRight = 2,
  kNoParent = 3
};

// Must be a trivial type.
template <typename Key>
struct SearchStateT {
  Key id;
  double fraction = 0.0;
  double time_from_start = 0.0;
  double dist_from_start = 0.0;
  SearchStateT<Key>* parent = nullptr;
  ConnectTypeFromParent connect_type;
  int which = kNone;  // in which queue
  bool IsOpen() const { return (which & kOpen) == kOpen; }
  bool IsClose() const { return (which & kClose) == kClose; }
  bool IsForward() const { return (which & kForward) == kForward; }
  bool IsBackward() const { return (which & kBackward) == kBackward; }
};

using SearchState = SearchStateT<int64_t>;

}  // namespace qcraft::planner::route

#endif  // ONBOARD_PLANNER_ROUTER_ROUTE_STRUCT_H_
