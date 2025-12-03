#include "onboard/prediction/container/prediction_context.h"

#include <string>
#include <utility>

#include "gtest/gtest.h"

#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/prediction/container/objects_history.h"
#include "onboard/prediction/test_util/prediction_context_builder.h"
#include "onboard/proto/assist_state.pb.h"
#include "onboard/proto/route.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {
namespace prediction {
namespace {
TEST(PredictionContextTest, CheckPredictionContext) {
  const auto& psmm = planner::CreateDojoTestPSMM();
  VehicleGeometryParamsProto vparams;
  ConflictResolverParams cparams;
  std::string object_id("1");
  auto input = BuildPredictionInput(psmm, vparams, cparams);
  auto context = BuildOneObjectPredictionContext(object_id, &input);

  const ObjectsHistory& object_long_term_history_manager =
      context.object_long_term_history_manager();
  const ObjectsHistory& object_history_manager =
      context.object_history_manager();
  const AvContext& av_context = context.av_context();
  const TrafficLightManager& traffic_light_manager =
      context.traffic_light_manager();
  const planner::DrivePassage* drive_passage = context.av_drive_passage();
  bool has_objects = context.HasObjects();
  EXPECT_EQ(av_context.GetAvObjectHistory().size(), 1);
  EXPECT_EQ(object_history_manager.Contains("1"), true);
  EXPECT_EQ(object_long_term_history_manager.Contains("1"), true);
  EXPECT_EQ(traffic_light_manager.GetInferedTlStateMap().size(), 0);
  EXPECT_NE(drive_passage, nullptr);
  EXPECT_TRUE(has_objects);

  auto autonomy_state =
      std::make_shared<AutonomyStateProto>(AutonomyStateProto());
  autonomy_state->mutable_assist_state()->set_assist_drive_system_state(
      AssistStateProto::ASSIST_NOA_ACTIVE);
  auto route_manager_output =
      std::make_shared<planner::RouteManagerOutputProto>(
          planner::RouteManagerOutputProto());

  route_manager_output->set_route_status(
      planner::RouteManagerOutputProto::NORMAL);
  route_manager_output->mutable_route_sections_from_current()
      ->set_start_fraction(0.0);
  route_manager_output->mutable_route_sections_from_current()->set_end_fraction(
      1.0);
  route_manager_output->mutable_route_sections_from_current()->add_section_id(
      12401);
  route_manager_output->mutable_route_sections_from_current()->add_section_id(
      12400);
  route_manager_output->mutable_route_sections_from_current()->add_section_id(
      12408);
  route_manager_output->mutable_route_sections_from_current()->add_section_id(
      12412);
  route_manager_output->mutable_route_sections_from_current()->add_section_id(
      12414);
  input.autonomy_state = std::move(autonomy_state);
  input.route_manager_output = std::move(route_manager_output);
  context = BuildOneObjectPredictionContext(object_id, &input);
  EXPECT_NE(drive_passage, nullptr);
}
}  // namespace
}  // namespace prediction
}  // namespace qcraft
