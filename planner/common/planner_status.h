#ifndef ONBOARD_PLANNER_COMMON_PLANNER_STATUS_H_
#define ONBOARD_PLANNER_COMMON_PLANNER_STATUS_H_

#include <string>

#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"

#include "onboard/planner/common/proto/planner_status.pb.h"

namespace qcraft::planner {

class PlannerStatus {
 public:
  PlannerStatus() {}

  PlannerStatus(PlannerStatusProto::PlannerStatusCode status,
                absl::string_view str)
      : status_(status), message_(str) {}

  bool ok() const { return status_ == PlannerStatusProto::OK; }
  PlannerStatusProto::PlannerStatusCode status_code() const { return status_; }
  absl::string_view message() const { return message_; }

  std::string ToString() const {
    return absl::StrCat(PlannerStatusProto::PlannerStatusCode_Name(status_),
                        ": ", message_);
  }

  void ToProto(PlannerStatusProto* proto) const {
    proto->Clear();
    proto->set_status(status_);
    *proto->mutable_message() = message_;
  }

  void FromProto(const PlannerStatusProto& proto) {
    if (proto.has_status()) {
      status_ = proto.status();
    }
    if (proto.has_message()) {
      message_ = proto.message();
    }
  }

 private:
  PlannerStatusProto::PlannerStatusCode status_ = PlannerStatusProto::OK;
  std::string message_;
};

PlannerStatus OkPlannerStatus();

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_COMMON_PLANNER_STATUS_H_
