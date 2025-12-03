#include "onboard/prediction/post_process/conflict_resolver_params.h"

#include <ostream>
#include <string>

#include "google/protobuf/descriptor.h"
#include "google/protobuf/message.h"

#include "onboard/lite/logging.h"
#include "onboard/utils/file_util.h"

namespace qcraft {
namespace prediction {

namespace {
void ValidateParams(const google::protobuf::Message& params) {
  const google::protobuf::Descriptor* descriptor = params.GetDescriptor();
  const google::protobuf::Reflection* reflection = params.GetReflection();
  for (int i = 0; i < descriptor->field_count(); ++i) {
    const google::protobuf::FieldDescriptor* field = descriptor->field(i);
    if (!field->is_optional()) {
      continue;
    }
    QCHECK(reflection->HasField(params, field))
        << "Missing field: " << field->name();
    if (field->type() == google::protobuf::FieldDescriptor::TYPE_MESSAGE) {
      ValidateParams(reflection->GetMessage(params, field));
    }
  }
}
}  // namespace

void ConflictResolverParams::LoadParams() {
  file_util::FileToProto(
      "onboard/prediction/post_process/"
      "prediction_post_process_default_params.pb.txt",
      &config_params_);
  ValidateParams(config_params_);
  QLOG(INFO)
      << "Load conflict resolution params from file and validation success.";
}
}  // namespace prediction
}  // namespace qcraft
