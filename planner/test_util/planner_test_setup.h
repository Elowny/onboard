#ifndef ONBOARD_PLANNER_TEST_UTIL_PLANNER_TEST_SETUP_H_
#define ONBOARD_PLANNER_TEST_UTIL_PLANNER_TEST_SETUP_H_

#include <string>
#include <string_view>

#include "gtest/gtest.h"

#include "onboard/lite/lite_module.h"
#include "onboard/maps/lane_point.h"
#include "onboard/maps/semantic_map_manager.h"
#include "onboard/math/vec.h"
#include "onboard/planner/composite_lane_path.h"
#include "onboard/planner/planner_input.h"
#include "onboard/planner/proto/planner_output.pb.h"
#include "onboard/planner/test_util/planner_object_builder.h"
#include "onboard/proto/positioning.pb.h"

namespace qcraft {
namespace planner {

class PlannerTestSetup : public LiteModule, public testing::Test {
 public:
  void set_map(std::string_view map_name);
  void set_sdc_pose(const PoseProto& sdc_pose) { sdc_pose_ = sdc_pose; }
  void set_route(const PoseProto& /*pose*/, const Vec2d& /*global_point*/) {}
  void set_route(const PoseProto& /*pose*/,
                 const mapping::LanePoint& /*lane_point*/) {}
  PlannerObjectBuilder* add_object() { return &object_builder_; }

  // The main task of this function is to recreate `planner_input_` and call
  // RunMainLoop() of PlannerModule.
  void UpdateState() {}

  // Similar to PlannerModule's PublishOutput function. Send the data to
  // Vantage for visualization.
  void PublishState() {}

  const PlannerInput& planner_input() const { return planner_input_; }
  const PlannerOutput& planner_output() const { return planner_output_; }

 private:
  // Lite Module functions.
  void OnInit() override {}
  void OnSubscribeChannels() override {}
  void OnSetUpTimers() override {}

  // Construct planner input from set.
  PlannerInput planner_input_;
  // Recreate planner output based on input_.
  PlannerOutput planner_output_;

  PoseProto sdc_pose_;
  PlannerObjectBuilder object_builder_;
  CompositeLanePath route_path_;
  std::string map_name_;
  SemanticMapManager map_;
};

}  // namespace planner
}  // namespace qcraft
#endif  // ONBOARD_PLANNER_TEST_UTIL_PLANNER_TEST_SETUP_H_
