#ifndef ONBOARD_PLANNER_INITIALIZER_COST_FEATURE_H_
#define ONBOARD_PLANNER_INITIALIZER_COST_FEATURE_H_

#include <algorithm>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "onboard/planner/decision/constraint_manager.h"
#include "onboard/planner/initializer/geometry/geometry_graph.h"
#include "onboard/planner/initializer/motion_graph.h"
#include "onboard/planner/initializer/proto/initializer_config.pb.h"
#include "onboard/planner/initializer/ref_speed_table.h"

namespace qcraft::planner {

struct GeometryEdgeInfo {
  const GeometryForm* geometry_form = nullptr;
  std::vector<GeometryState> states;
  bool terminating = false;
};

struct MotionEdgeInfo {
  double start_t = 0.0;  // Start time of the motion.
  const MotionForm* motion_form = nullptr;
  std::vector<MotionState> const_interval_states;
  std::vector<MotionState> equal_interval_states;
};

class FeatureCost {
 public:
  explicit FeatureCost(std::string name) : name_(std::move(name)) {}

  virtual void ComputeCost(const MotionEdgeInfo& /*edge_info*/,
                           absl::Span<double> /*cost*/) const {}

  virtual void ComputeCost(const GeometryEdgeInfo& /*edge_info*/,
                           absl::Span<double> /*cost*/) const {}

  absl::string_view name() const { return name_; }

  virtual ~FeatureCost() {}

 private:
  std::string name_;
};

}  // namespace qcraft::planner
#endif  // ONBOARD_PLANNER_INITIALIZER_COST_FEATURE_H_
