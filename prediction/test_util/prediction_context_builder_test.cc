#include "onboard/prediction/test_util/prediction_context_builder.h"

#include <memory>  // for operator!=, unique_ptr
#include <string>  // for allocator, string

#include "gtest/gtest.h"  // for Test, AssertionResult, TestInfo, EXPECT_TRUE, Message, TEST

#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/prediction/container/objects_history.h"  // for ObjectsHistory

namespace qcraft {
namespace prediction {
namespace {
TEST(PredictionInputBuildTest, BuildInput) {
  const auto& psmm = planner::CreateDojoTestPSMM();
  VehicleGeometryParamsProto vparams;
  ConflictResolverParams cparams;
  const auto input = BuildPredictionInput(psmm, vparams, cparams);
  EXPECT_TRUE(input.objects_history != nullptr);
}
TEST(PredictionContextBuilderTest, BuildOneObject) {
  const auto& psmm = planner::CreateDojoTestPSMM();
  VehicleGeometryParamsProto vparams;
  ConflictResolverParams cparams;
  std::string object_id("object");
  auto input = BuildPredictionInput(psmm, vparams, cparams);
  const auto context = BuildOneObjectPredictionContext(object_id, &input);
  EXPECT_TRUE(context.object_history_manager().Contains("object"));
}
TEST(PredictionContextBuilderTest, BuildMultiObjects) {
  const auto& psmm = planner::CreateDojoTestPSMM();
  VehicleGeometryParamsProto vparams;
  ConflictResolverParams cparams;
  auto input = BuildPredictionInput(psmm, vparams, cparams);

  const int objects_num = 64;
  const auto context = BuildMultiObjectsPredictionContext(objects_num, &input);
  for (int i = 0; i < objects_num; ++i) {
    EXPECT_TRUE(context.object_history_manager().Contains(std::to_string(i)));
  }
}
}  // namespace
}  // namespace prediction
}  // namespace qcraft
