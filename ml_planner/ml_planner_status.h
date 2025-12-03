#ifndef ONBOARD_ML_PLANNER_ML_PLANNER_STATUS_H_
#define ONBOARD_ML_PLANNER_ML_PLANNER_STATUS_H_

#include <string>

#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"

#include "onboard/ml_planner/proto/ml_planner_status.pb.h"

namespace qcraft::mlplanner {

class MLPlannerStatus {
 public:
  MLPlannerStatus() {}

  MLPlannerStatus(MLPlannerStatusProto::MLPlannerStatusCode status,
                  absl::string_view str)
      : status_(status), message_(str) {}

  bool ok() const { return status_ == MLPlannerStatusProto::OK; }
  MLPlannerStatusProto::MLPlannerStatusCode status_code() const {
    return status_;
  }
  absl::string_view message() const { return message_; }

  std::string ToString() const {
    return absl::StrCat(MLPlannerStatusProto::MLPlannerStatusCode_Name(status_),
                        ": ", message_);
  }

  void ToProto(MLPlannerStatusProto* proto) const {
    proto->Clear();
    proto->set_status(status_);
    *proto->mutable_message() = message_;
  }

  void FromProto(const MLPlannerStatusProto& proto) {
    if (proto.has_status()) {
      status_ = proto.status();
    }
    if (proto.has_message()) {
      message_ = proto.message();
    }
  }

 private:
  MLPlannerStatusProto::MLPlannerStatusCode status_ =
      MLPlannerStatusProto::UNINITIALIZED;
  std::string message_;
};

inline MLPlannerStatus OkMLPlannerStatus() {
  return MLPlannerStatus(MLPlannerStatusProto::OK, "ok");
}

}  // namespace qcraft::mlplanner

#endif  // ONBOARD_ML_PLANNER_ML_PLANNER_STATUS_H_
