#ifndef ONBOARD_PLANNER_INITIALIZER_MOTION_GRAPH_CACHE_H_
#define ONBOARD_PLANNER_INITIALIZER_MOTION_GRAPH_CACHE_H_

#include <memory>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/hash/hash.h"
#include "absl/status/statusor.h"

#include "onboard/async/async_util.h"
#include "onboard/math/util.h"
#include "onboard/planner/initializer/astar_motion_searcher_defs.h"
#include "onboard/planner/initializer/dp_motion_searcher_defs.h"
#include "onboard/planner/initializer/geometry/geometry_graph.h"
#include "onboard/planner/initializer/motion_form.h"
#include "onboard/planner/initializer/motion_graph.h"
#include "onboard/utils/source_location.h"

namespace qcraft::planner {
// motion graph cache is to collect the motion edges calculated as well as their
// related costs (except for the leading object costs);

// (v, acc, t, GeometryForm) -> costs (or summation oftthe calculated costs.)

// For one particular node, the motion form ended at this node should be
// specified by: acc, init_v, and the GeometryForm (which is specified by
// start_node, end_node, as well as the type of the connection: lateral quintic
// polynomials or quintic spirals.)

struct MotionEdgeCache {
  std::unique_ptr<MotionForm> ptr_motion_form;
  std::vector<double> costs;
  IgnoreTrajMap ignored_trajs;
};

// Discrete motion edge sample
struct MotionEdgeKey {
  // One specific motion edge sample can be uniquely defined as a geometry edge
  // (defined as the start node, end node as well as the geometry edge type)
  // plus it's initial velocity and acceleration. Then a MotionEdgeKey can be
  // created using DpMotionInfo directly during motion search.

  // NOTE(boqian): the following members are all internal key values and should
  // not be directly referenced from outside.
  // TODO(boqian): make the keys private.
  int a0_key;
  int v0_key;
  int t0_key;
  GeometryEdgeIndex geom_edge_index;

  double v0() const { return v0_key * 0.01; }
  double a0() const { return a0_key * 0.01; }

  MotionEdgeKey(double acc, double init_v, double t,
                GeometryEdgeIndex geom_edge_index)
      : a0_key(RoundToInt(acc * 100)),
        v0_key(RoundToInt(init_v * 100)),
        t0_key(RoundToInt(t * 100)),
        geom_edge_index(geom_edge_index) {}

  MotionEdgeKey()
      : a0_key(0),
        v0_key(0),
        t0_key(0),
        geom_edge_index(GeometryEdgeVector<GeometryEdge>::kInvalidIndex) {}

  friend bool operator==(const MotionEdgeKey& lhs, const MotionEdgeKey& rhs) {
    return lhs.a0_key == rhs.a0_key && lhs.v0_key == rhs.v0_key &&
           lhs.t0_key == rhs.t0_key &&
           lhs.geom_edge_index.value() == rhs.geom_edge_index.value();
  }

  template <typename H>
  friend H AbslHashValue(H h, const MotionEdgeKey& mek) {
    return H::combine(std::move(h), mek.a0_key, mek.v0_key, mek.t0_key,
                      mek.geom_edge_index.value());
  }
};

struct NewCacheInfo {
  MotionEdgeKey key;
  MotionEdgeCache cache;
};

// Main structure to hold information during dynamic programming.
struct DpMotionInfo {
  MotionEdgeKey key;
  double sum_cost = 0.0;
  std::vector<double> costs;
  double start_t = 0.0;
  MotionEdgeIndex prev_motion_edge_index;
  GeometryNodeIndex end_geometry_node_index;
  MotionForm* motion_form;
  GeometryEdgeIndex geometry_edge_index;
  IgnoreTrajMap ignored_trajs;
};

class MotionGraphCache {
 public:
  MotionGraphCache() {}

  void BatchGetOrFail(const std::vector<MotionEdgeKey>& samples,
                      std::vector<DpMotionInfo>* ptr_result,
                      std::vector<int>* failed_idx) const;

  void Insert(const MotionEdgeKey& key, std::vector<double> costs,
              IgnoreTrajMap ignored_trajs,
              std::unique_ptr<MotionForm> ptr_motion_form);

  void BatchInsert(std::vector<NewCacheInfo> new_motion_forms);

  absl::StatusOr<MotionForm*> GetMotionForm(const MotionEdgeKey& key) const;

  absl::StatusOr<std::vector<double>> GetCosts(const MotionEdgeKey& key) const;

  virtual inline int size() const { return cache_.size(); }

  virtual ~MotionGraphCache() {
    DestroyContainerAsyncMarkSource(std::move(cache_), (QCRAFT_LOC).ToString());
  }

 private:
  absl::flat_hash_map<MotionEdgeKey, MotionEdgeCache> cache_;
};

struct AstarCacheInfo {
  MotionEdgeKey key;
  std::shared_ptr<AstarNode> node;
};

class AstarGraphCache : public MotionGraphCache {
 public:
  AstarGraphCache() {}

  void BatchGetOrFail(const std::vector<MotionEdgeKey>& samples,
                      std::vector<AstarCacheInfo>* ptr_result,
                      std::vector<int>* failed_idx) const;

  void Insert(const MotionEdgeKey& key, const std::shared_ptr<AstarNode>& node);

  void BatchInsert(std::vector<AstarCacheInfo> new_motion_forms);

  absl::StatusOr<std::shared_ptr<AstarNode>> GetNode(
      const MotionEdgeKey& key) const;

  inline int size() const override { return astar_cache_.size(); }

  ~AstarGraphCache() override {
    DestroyContainerAsyncMarkSource(std::move(astar_cache_),
                                    (QCRAFT_LOC).ToString());
  }

 private:
  absl::flat_hash_map<MotionEdgeKey, std::shared_ptr<AstarNode>> astar_cache_;
};

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_INITIALIZER_MOTION_GRAPH_CACHE_
