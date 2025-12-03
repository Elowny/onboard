#ifndef ONBOARD_PREDICTION_POST_PROCESS_SVT_COST_PROVIDER_H_
#define ONBOARD_PREDICTION_POST_PROCESS_SVT_COST_PROVIDER_H_

#include <memory>
#include <string>
#include <vector>

#include "absl/types/span.h"

#include "onboard/planner/speed/speed_vector.h"
#include "onboard/prediction/post_process/object_svt_sample.h"
#include "onboard/prediction/post_process/svt_feature_cost.h"
#include "onboard/prediction/proto/conflict_resolver.pb.h"

namespace qcraft {

namespace prediction {
class SvtCostProviderBase {
 public:
  absl::Span<const std::string> cost_names() const { return cost_names_; }

  absl::Span<const double> weights() const { return weights_; }

  void ComputeCost(const std::vector<SvtNode>& nodes, double prev_edge_sum_cost,
                   double sampling_ds, DpEdgeInfo* edge_info) const;

 protected:
  template <typename Config>
  void BuildWeightTable(const Config& cost_config);

  std::vector<std::unique_ptr<SvtFeatureCost>> features_;

 private:
  std::vector<std::string> cost_names_;
  std::vector<double> weights_;
  std::vector<int> feature_size_;
};

// TODO(changqing): Move to object conflict resolver input.
struct SvtCostProviderInput {
  const std::vector<double>* stationary_objects = nullptr;
  const std::vector<double>* stoplines = nullptr;
  const planner::SpeedVector* ref_speed = nullptr;
  const ConflictResolutionConfigProto::SpeedFeatureCostConfig* cost_config =
      nullptr;
  double stationary_follow_distance = 1e-6;  // Avoid Nan inv.
};

class SvtCostProvider : public SvtCostProviderBase {
 public:
  explicit SvtCostProvider(const SvtCostProviderInput& input);
};
}  // namespace prediction
}  // namespace qcraft

#endif  // ONBOARD_PREDICTION_POST_PROCESS_SVT_COST_PROVIDER_H_
