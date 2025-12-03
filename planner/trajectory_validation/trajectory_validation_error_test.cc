#include "onboard/planner/trajectory_validation/trajectory_validation_error.h"

#include "absl/strings/str_format.h"
#include "glog/logging.h"

#include "gtest/gtest.h"

namespace qcraft {
namespace planner {

TEST(TrajValidErrTest, TrajValidErrTest) {
  {
    TrajectoryValidationError error;
    EXPECT_EQ(error.GetErrorCode(),
              TrajectoryValidationResultProto::UNKNOWN_ERROR);
    EXPECT_EQ(error.GetErrorMessage(), "unknown error code");
  }
  {
    TrajectoryValidationError error(
        TrajectoryValidationResultProto::ACCELERATION_OVER_LIMIT,
        absl::StrFormat("acceleration over limit: a[%d] = %f", 0, 0.0));
    EXPECT_EQ(error.GetErrorCode(),
              TrajectoryValidationResultProto::ACCELERATION_OVER_LIMIT);
    EXPECT_EQ(error.GetErrorMessage(),
              "acceleration over limit: a[0] = 0.000000");
  }
}

TEST(TrajValidErrTest, TrajValidErrLogTest) {
  TrajectoryValidationError error(
      TrajectoryValidationResultProto::ACCELERATION_OVER_LIMIT,
      absl::StrFormat("acceleration over limit: a[%d] = %f", 0, 0.0));
  EXPECT_EQ(error.GetErrorCode(),
            TrajectoryValidationResultProto::ACCELERATION_OVER_LIMIT);
  EXPECT_EQ(error.GetErrorMessage(),
            "acceleration over limit: a[0] = 0.000000");
  LOG(INFO) << error.GetErrorCodeName();
  LOG(INFO) << error.GetErrorMessage();
}
}  // namespace planner
}  // namespace qcraft
