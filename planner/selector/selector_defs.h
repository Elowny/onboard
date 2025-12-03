#ifndef ONBOARD_PLANNER_SELECTOR_DEFS_H_
#define ONBOARD_PLANNER_SELECTOR_DEFS_H_

#include <memory>
#include <string>
#include <vector>

#include "absl/container/flat_hash_map.h"

#include "onboard/planner/selector/cost_feature_base.h"

namespace qcraft::planner {

typedef std::vector<std::unique_ptr<CostFeatureBase>> CostFeatures;
typedef absl::flat_hash_map<std::string, CostFeatureBase::CostVec> WeightTable;

struct FeatureCostSum {
  double cost_common;
  double cost_same_start;

  inline double cost_sum() const { return cost_common + cost_same_start; }
  bool operator<(const FeatureCostSum& other) const {
    return cost_sum() < other.cost_sum();
  }
  void add(double val, bool is_common) {
    (is_common ? cost_common : cost_same_start) += val;
  }
};

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_SELECTOR_DEFS_H_
