#include "onboard/prediction/prediction_module.h"

#include "gtest/gtest.h"

#include "onboard/lite/proto/lite_common.pb.h"
#include "onboard/lite/unittest/lite2_unittest_helper.h"

namespace qcraft {
namespace prediction {
namespace {
class PredictionModuleTest : public ::testing::Test {
 protected:
  void SetUp() override { module_.Init("PredictionModule", PREDICTION_MODULE); }

  void TearDown() override { module_.Destroy(); }

  PredictionModule* prediction_module() {
    return dynamic_cast<PredictionModule*>(module_.GetLiteModule());
  }

 private:
  Lite2UnitTestHelper module_;
};
TEST_F(PredictionModuleTest, ModuleInitTest) {
  auto* prediction = prediction_module();
  prediction->OnInit();
  const auto* input = prediction->prediction_input();
  EXPECT_TRUE(input->veh_geom_params != nullptr);
  EXPECT_TRUE(input->objects_history != nullptr);
  EXPECT_TRUE(input->av_context != nullptr);
  EXPECT_TRUE(input->long_term_objects_history != nullptr);
  EXPECT_TRUE(input->conflict_resolver_params != nullptr);
}
}  // namespace
}  // namespace prediction
}  // namespace qcraft
