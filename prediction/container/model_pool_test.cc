#include "onboard/prediction/container/model_pool.h"

#include "gtest/gtest.h"

#include "onboard/global/buffered_logger.h"
#include "onboard/global/run_context.h"
#include "onboard/lite/check.h"

namespace qcraft {
namespace prediction {

namespace {
TEST(ModelPoolTest, GetInferencersForNoa) {
  auto param_manager = CreateParamManagerFromCarId("Q8001");
  QCHECK(param_manager != nullptr);
  auto param_finder = CreateParamFinderWithCarId("Q8001");
  QCHECK(param_finder != nullptr);
  FLAGS_run_mode_type = 1;
  ModelPool model_pool(*param_manager, *param_finder, {});
  EXPECT_NE(model_pool.GetActNetJ5Inferencer(), nullptr);
  EXPECT_EQ(model_pool.GetActNetInferencer(), nullptr);
  EXPECT_EQ(model_pool.GetActNetLocalJ5Inferencer(), nullptr);
  EXPECT_NE(model_pool.GetCutinSLNetJ5Inferencer(), nullptr);
  EXPECT_NE(model_pool.GetLaneSelectionNetJ5Inferencer(), nullptr);
}

TEST(ModelPoolTest, GetInferencersForFsd) {
  auto param_manager = CreateParamManagerFromCarId("Q8001");
  QCHECK(param_manager != nullptr);
  auto param_finder = CreateParamFinderWithCarId("Q8001");
  QCHECK(param_finder != nullptr);
  FLAGS_run_mode_type = 0;
  ModelPool model_pool(*param_manager, *param_finder, {});
  EXPECT_NE(model_pool.GetActNetInferencer(), nullptr);
  EXPECT_NE(model_pool.GetCutinSLNetInferencer(), nullptr);
}

}  // namespace
}  // namespace prediction
}  // namespace qcraft
