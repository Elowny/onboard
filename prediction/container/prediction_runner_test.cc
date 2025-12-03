#include "onboard/prediction/container/prediction_runner.h"

#include <string>

#include "google/protobuf/repeated_ptr_field.h"

#include "gtest/gtest.h"

#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/params/param_finder.h"
#include "onboard/params/param_manager.h"
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/prediction/post_process/conflict_resolver_params.h"
#include "onboard/prediction/test_util/prediction_context_builder.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {
namespace prediction {
namespace {
TEST(PredictionRunnerTest, ComputePredictionTest) {
  const auto& psmm = planner::CreateDojoTestPSMM();
  VehicleGeometryParamsProto vparams;
  ConflictResolverParams cparams;
  std::string object_id("1");
  auto input = BuildPredictionInput(psmm, vparams, cparams);
  const auto context = BuildOneObjectPredictionContext(object_id, &input);
  auto param_manager = CreateParamManagerFromCarId("Q8001");
  QCHECK(param_manager != nullptr);
  auto param_finder = CreateParamFinderWithCarId("Q8001");
  QCHECK(param_finder != nullptr);
  ModelPool model_pool(*param_manager, *param_finder, {});
  PredictionDebugProto debug_proto;
  auto res = ComputePrediction(input, &model_pool,
                               /*thread_pool=*/nullptr, &debug_proto);
  EXPECT_TRUE(res.ok());
  EXPECT_EQ((*res)->objects().size(), 1);
}
}  // namespace
}  // namespace prediction
}  // namespace qcraft
