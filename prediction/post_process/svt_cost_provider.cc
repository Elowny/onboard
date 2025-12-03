#include "onboard/prediction/post_process/svt_cost_provider.h"

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type
#include <algorithm>

#include "absl/algorithm/container.h"
#include "absl/strings/str_cat.h"
#include "google/protobuf/descriptor.h"

#include "onboard/lite/check.h"

namespace qcraft {
namespace prediction {
template <typename Config>
void SvtCostProviderBase::BuildWeightTable(const Config& cost_config) {
  const auto* reflection = cost_config.GetReflection();
  const auto* descriptor = cost_config.GetDescriptor();
  for (const auto& feature : features_) {
    auto* feature_desc =
        descriptor->FindFieldByName(std::string(feature->name()));
    const auto& feature_conf =
        reflection->GetMessage(cost_config, feature_desc);
    const auto* feature_ref = feature_conf.GetReflection();
    std::vector<const google::protobuf::FieldDescriptor*> feature_weights_desc;
    feature_ref->ListFields(feature_conf, &feature_weights_desc);
    int feature_size = 0;
    for (const auto* desc : feature_weights_desc) {
      if (desc->type() != google::protobuf::FieldDescriptor::TYPE_DOUBLE) {
        continue;
      }
      ++feature_size;
      cost_names_.push_back(absl::StrCat(feature->name(), ".", desc->name()));
      weights_.push_back(feature_ref->GetDouble(feature_conf, desc));
    }
    feature_size_.push_back(feature_size);
  }
}

void SvtCostProviderBase::ComputeCost(const std::vector<SvtNode>& nodes,
                                      double prev_edge_sum_cost,
                                      double sampling_ds,
                                      DpEdgeInfo* edge_info) const {
  const auto states = SampleDp(nodes, *edge_info, sampling_ds);
  auto& feature_costs = (edge_info->cost).feature_costs;
  feature_costs.resize(weights_.size());
  for (int i = 0, index = 0, n = features_.size(); i < n; ++i) {
    features_[i]->ComputeFeatureCost(
        states, absl::MakeSpan(&feature_costs[index], feature_size_[i]));
    index += feature_size_[i];
  }
  for (int i = 0; i < feature_costs.size(); ++i) {
    feature_costs[i] = feature_costs[i] * weights_[i];
  }
  edge_info->final_t = states.back().t;
  edge_info->final_v = states.back().v;
  edge_info->states = std::make_unique<std::vector<SvtState>>(states);
  edge_info->cost.sum_cost =
      prev_edge_sum_cost + absl::c_accumulate(feature_costs, 0.0);
}

// ---------------- SvtCostProvider ------------------
SvtCostProvider::SvtCostProvider(const SvtCostProviderInput& input) {
  // TODO(changqing): Add cost on/off in config proto.
  QCHECK_NOTNULL(input.ref_speed);
  QCHECK_NOTNULL(input.stoplines);
  QCHECK_NOTNULL(input.stationary_objects);

  const auto& stoplines = *input.stoplines;
  const auto& stationary_objects = *input.stationary_objects;
  const auto& ref_speed = *input.ref_speed;
  const auto& cost_config = *input.cost_config;

  if (cost_config.has_svt_object_collision()) {
    features_.push_back(std::make_unique<SvtObjectCollisionFeatureCost>(
        stationary_objects, input.stationary_follow_distance));
  }

  if (cost_config.has_svt_stopline()) {
    features_.push_back(std::make_unique<SvtStoplineFeatureCost>(stoplines));
  }

  if (cost_config.has_svt_reference_speed()) {
    features_.push_back(
        std::make_unique<SvtReferenceSpeedFeatureCost>(&ref_speed));
  }

  BuildWeightTable<ConflictResolutionConfigProto::SpeedFeatureCostConfig>(
      cost_config);
}
}  // namespace prediction
}  // namespace qcraft
