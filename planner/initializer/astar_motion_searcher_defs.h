#ifndef ONBOARD_PLANNER_INITIALIZER_ASTAR_MOTION_SEARCHER_DEFS_H_
#define ONBOARD_PLANNER_INITIALIZER_ASTAR_MOTION_SEARCHER_DEFS_H_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/hash/hash.h"
#include "boost/heap/fibonacci_heap.hpp"

#include "onboard/math/util.h"
#include "onboard/planner/initializer/dp_motion_searcher_defs.h"
#include "onboard/planner/initializer/motion_graph.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/proto/trajectory_point.pb.h"
namespace qcraft::planner {

struct AstarNode {
  int key;
  int layer;
  int parent;
  GeometryNodeIndex geom_index;
  double cost_g;
  double cost_h;
  std::vector<double> feature_costs;
  MotionState state;
  std::shared_ptr<MotionForm> motion;
  IgnoreTrajMap ignored_trajs;
};

struct AstarPriorityComp {
  bool operator()(const std::shared_ptr<AstarNode>& a,
                  const std::shared_ptr<AstarNode>& b) const {
    return a->cost_g + a->cost_h > b->cost_g + b->cost_h;
  }
};

using OpenPQ =
    boost::heap::fibonacci_heap<std::shared_ptr<AstarNode>,
                                boost::heap::compare<AstarPriorityComp>>;
using OpenMap = absl::flat_hash_map<int, typename OpenPQ::handle_type>;
using CloseMap = absl::flat_hash_map<int, std::shared_ptr<AstarNode>>;

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_INITIALIZER_ASTAR_MOTION_SEARCHER_DEFS_H_
