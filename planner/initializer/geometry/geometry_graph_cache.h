#ifndef ONBOARD_PLANNER_INITIALIZER_GEOMETRY_GEOMETRY_GRAPH_CACHE_H_
#define ONBOARD_PLANNER_INITIALIZER_GEOMETRY_GEOMETRY_GRAPH_CACHE_H_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "absl/base/thread_annotations.h"
#include "absl/container/flat_hash_map.h"
#include "absl/hash/hash.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/synchronization/mutex.h"

#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/math/vec.h"
#include "onboard/planner/initializer/geometry/geometry_form.h"
#include "onboard/planner/initializer/geometry/geometry_graph.h"
#include "onboard/planner/initializer/geometry/geometry_graph_debug.h"
#include "onboard/planner/initializer/geometry/geometry_state.h"
#include "onboard/planner/initializer/proto/initializer.pb.h"
#include "onboard/utils/map_util.h"

namespace qcraft::planner {

struct GeometryGraphCacheKey {
  int start_x;
  int start_y;
  int start_k;
  int end_x;
  int end_y;

  GeometryGraphCacheKey(const GeometryNode& start_node,
                        const GeometryNode& end_node) {
    const double discretize_factor = 100.0;
    start_x = static_cast<int>(start_node.xy.x() * discretize_factor);
    start_y = static_cast<int>(start_node.xy.y() * discretize_factor);
    start_k = static_cast<int>(start_node.k * discretize_factor);
    end_x = static_cast<int>(end_node.xy.x() * discretize_factor);
    end_y = static_cast<int>(end_node.xy.y() * discretize_factor);
  }

  friend bool operator==(const GeometryGraphCacheKey& lhs,
                         const GeometryGraphCacheKey& rhs) {
    return lhs.start_x == rhs.start_x && lhs.end_x == rhs.end_x &&
           lhs.start_y == rhs.start_y && lhs.start_k == rhs.start_k &&
           lhs.end_y == rhs.end_y;
  }

  template <typename H>
  friend H AbslHashValue(H h, const GeometryGraphCacheKey& key) {
    return H::combine(std::move(h), key.start_k, key.start_x, key.start_y,
                      key.end_x, key.end_y);
  }
};

enum class ConnectionResult {
  SUCCESS = 1,
  OMIT_TOO_SHORT = 2,
  OMIT_LATERAL_OFFSET = 3,
  OMIT_NOT_CONVERGE_TO_CENTER = 4,
  FAIL_NO_POLY = 5,
  COLLIDE_TRUNCATE = 6,
  COLLIDE_NO_EDGE = 7,
  FAIL_CURB_COLLISION = 8,
  FAIL_INVALID_CURVATURE = 9,
  NOT_SET = 10
};

struct GeometryEdgeCache {
  std::unique_ptr<GeometryForm> ptr_geometry_form;  // full geometry form.
  std::unique_ptr<GeometryForm> ptr_geometry_form_truncated;
  ConnectionResult connection_result = ConnectionResult::NOT_SET;
  GeometryState final_state;  // Valid when truncated.
  std::vector<std::string> collision_ids;
  std::vector<double> collision_accum_s;
  EdgeDebugInfo debug_info;
};

class GeometryGraphCache {
 public:
  GeometryGraphCache()
      : revisited_time_(0), successful_connection_(0), try_to_connect_(0) {}
  absl::StatusOr<GeometryForm*> GetGeometryForm(
      const GeometryGraphCacheKey& key) const;
  absl::StatusOr<GeometryForm*> GetFullGeometryForm(
      const GeometryGraphCacheKey& key) const;
  absl::StatusOr<ConnectionResult> GetConnectionResult(
      const GeometryGraphCacheKey& key);
  absl::StatusOr<std::pair<double, std::string>> GetCollisionInfo(
      const GeometryGraphCacheKey& key) const;
  const GeometryState& GetFinalState(const GeometryGraphCacheKey& key) const {
    absl::ReaderMutexLock lock(&mutex_);
    const auto ptr_edge_info = FindOrNull(cache_, key);
    QCHECK(ptr_edge_info->connection_result ==
           ConnectionResult::COLLIDE_TRUNCATE);
    return ptr_edge_info->final_state;
  }
  bool AddEdge(const GeometryGraphCacheKey& key, GeometryEdgeCache edge_info);
  void UpdateCollisionInfo(const GeometryGraphCacheKey& key,
                           const std::string& collision_obj_id,
                           double collision_s);
  void UpdateConnectionResult(const GeometryGraphCacheKey& key,
                              const ConnectionResult result);
  void UpdateTruncatedGeometryForm(
      const GeometryGraphCacheKey& key, const GeometryState& final_state,
      std::unique_ptr<GeometryForm> ptr_geometry_form_truncated,
      const std::string& collision_obj_id, double collision_s);

  void reset_debug() {
    absl::WriterMutexLock lock(&mutex_debug_);
    revisited_time_ = 0;
    try_to_connect_ = 0;
  }

  void AddResampleReasons(const std::vector<ResampleReason>& reasons) {
    absl::WriterMutexLock lock(&mutex_debug_);
    resample_reasons_ = std::move(reasons);
  }

  void ParseResampleReasonToProto(
      GeometryGraphDebugProto* graph_debug_proto) const;
  void ParseConnectionProcessDebugInfoToProto(
      GeometryGraphDebugProto* graph_debug_proto) const;
  void ParseCollisionInfoToProto(GeometryGraphDebugProto* proto) const;

  std::string DebugString() const {
    absl::ReaderMutexLock lock(&mutex_);
    absl::ReaderMutexLock lock_debug(&mutex_debug_);
    return absl::StrCat("Cache debug: total size: ", cache_.size(),
                        ", revisited time: ", revisited_time_,
                        ", try to connect: ", try_to_connect_,
                        ", successful connection: ", successful_connection_);
  }

 private:
  mutable absl::Mutex mutex_;
  mutable absl::Mutex mutex_debug_;
  absl::flat_hash_map<GeometryGraphCacheKey, GeometryEdgeCache> cache_
      ABSL_GUARDED_BY(mutex_);
  absl::flat_hash_map<std::string, int> collision_counter_
      ABSL_GUARDED_BY(mutex_);  // object id->number of collision.

  // For debug.
  std::vector<ResampleReason> resample_reasons_ ABSL_GUARDED_BY(mutex_debug_);
  int revisited_time_ ABSL_GUARDED_BY(mutex_debug_);
  int successful_connection_ ABSL_GUARDED_BY(mutex_debug_);
  int try_to_connect_ ABSL_GUARDED_BY(mutex_debug_);
};
}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_INITIALIZER_GEOMETRY_GEOMETRY_GRAPH_CACHE_H_
