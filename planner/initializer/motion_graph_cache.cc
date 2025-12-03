#include "onboard/planner/initializer/motion_graph_cache.h"

#include <algorithm>

#include "absl/status/status.h"

#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/utils/map_util.h"

namespace qcraft::planner {
void MotionGraphCache::BatchGetOrFail(
    const std::vector<MotionEdgeKey>& keys,
    std::vector<DpMotionInfo>* ptr_dp_motion_infos,
    std::vector<int>* ptr_failed_idx) const {
  auto& dp_motion_infos = *ptr_dp_motion_infos;
  auto& failed_idx = *ptr_failed_idx;
  for (int i = 0; i < keys.size(); i++) {
    const auto& key = keys[i];
    const auto ptr_edge_info = FindOrNull(cache_, key);
    if (ptr_edge_info != nullptr) {
      dp_motion_infos[i].costs.resize(ptr_edge_info->costs.size());
      dp_motion_infos[i].costs = ptr_edge_info->costs;
      dp_motion_infos[i].motion_form = ptr_edge_info->ptr_motion_form.get();
      dp_motion_infos[i].ignored_trajs = ptr_edge_info->ignored_trajs;

    } else {
      failed_idx.push_back(i);
    }
  }
}

void MotionGraphCache::Insert(const MotionEdgeKey& key,
                              std::vector<double> costs,
                              IgnoreTrajMap ignored_trajs,
                              std::unique_ptr<MotionForm> ptr_motion_form) {
  if (!cache_.contains(key)) {
    MotionEdgeCache edge_cache = {.ptr_motion_form = std::move(ptr_motion_form),
                                  .costs = std::move(costs),
                                  .ignored_trajs = std::move(ignored_trajs)};
    auto [it, success] =
        cache_.emplace(std::make_pair(key, std::move(edge_cache)));
    QCHECK(success);
  }
}

void MotionGraphCache::BatchInsert(std::vector<NewCacheInfo> new_motion_forms) {
  for (auto it = new_motion_forms.begin(); it != new_motion_forms.end(); ++it) {
    if (!cache_.contains(it->key)) {
      auto [iter, success] =
          cache_.emplace(std::make_pair(it->key, std::move(it->cache)));
      QCHECK(success);
    }
  }
}

absl::StatusOr<MotionForm*> MotionGraphCache::GetMotionForm(
    const MotionEdgeKey& key) const {
  const auto ptr_edge_info = FindOrNull(cache_, key);
  if (ptr_edge_info != nullptr) {
    return ptr_edge_info->ptr_motion_form.get();
  } else {
    return absl::NotFoundError("Queried motion form not in cache.");
  }
}

absl::StatusOr<std::vector<double>> MotionGraphCache::GetCosts(
    const MotionEdgeKey& key) const {
  const auto ptr_edge_info = FindOrNull(cache_, key);
  if (ptr_edge_info != nullptr) {
    return ptr_edge_info->costs;
  } else {
    return absl::NotFoundError("Queried motion costs not in cache.");
  }
}

// ----------------Astar Graph Cache-----------------
void AstarGraphCache::BatchGetOrFail(
    const std::vector<MotionEdgeKey>& keys,
    std::vector<AstarCacheInfo>* ptr_cache_infos,
    std::vector<int>* ptr_failed_idx) const {
  auto& node_infos = *ptr_cache_infos;
  auto& failed_idx = *ptr_failed_idx;
  for (int i = 0; i < keys.size(); i++) {
    const auto& key = keys[i];
    const auto ptr_node_info = FindOrNull(astar_cache_, key);
    if (ptr_node_info != nullptr) {
      node_infos[i].node = *ptr_node_info;
    } else {
      failed_idx.push_back(i);
    }
  }
}

void AstarGraphCache::Insert(const MotionEdgeKey& key,
                             const std::shared_ptr<AstarNode>& node) {
  if (!astar_cache_.contains(key)) {
    auto [it, success] = astar_cache_.emplace(std::make_pair(key, node));
    QCHECK(success);
  }
}

void AstarGraphCache::BatchInsert(std::vector<AstarCacheInfo> new_caches) {
  for (auto it = new_caches.begin(); it != new_caches.end(); ++it) {
    if (!astar_cache_.contains(it->key)) {
      auto [iter, success] =
          astar_cache_.emplace(std::make_pair(it->key, std::move(it->node)));
      QCHECK(success);
    }
  }
}

absl::StatusOr<std::shared_ptr<AstarNode>> AstarGraphCache::GetNode(
    const MotionEdgeKey& key) const {
  const auto ptr_node_info = FindOrNull(astar_cache_, key);
  if (ptr_node_info != nullptr) {
    return *ptr_node_info;
  } else {
    return absl::NotFoundError("Queried motion form not in cache.");
  }
}

}  // namespace qcraft::planner
