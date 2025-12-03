#include "onboard/planner/plan/acc/test_util.h"

#include "absl/status/status.h"
#include "google/protobuf/descriptor.h"
#include "google/protobuf/message.h"

#include "gtest/gtest.h"

#include "onboard/utils/status_macros.h"

namespace qcraft {
namespace planner {
namespace {

absl::Status ValidateParams(const google::protobuf::Message& params) {
  const google::protobuf::Descriptor* descriptor = params.GetDescriptor();
  const google::protobuf::Reflection* reflection = params.GetReflection();
  for (int i = 0; i < descriptor->field_count(); ++i) {
    const google::protobuf::FieldDescriptor* field = descriptor->field(i);
    if (!field->is_optional()) continue;
    if (!reflection->HasField(params, field)) {
      return absl::NotFoundError(
          absl::StrCat("Missing field: ", field->full_name()));
    }
    if (field->type() == google::protobuf::FieldDescriptor::TYPE_MESSAGE) {
      RETURN_IF_ERROR(ValidateParams(reflection->GetMessage(params, field)));
    }
  }
  return absl::OkStatus();
}

TEST(CreateDefaultPlannerParamOnlyFillAccParams, Works) {
  const auto planner_params = CreateDefaultPlannerParamOnlyFillAccParams();
  const auto status = ValidateParams(planner_params.acc_params());
  EXPECT_OK(status);
}
}  // namespace
}  // namespace planner
}  // namespace qcraft
