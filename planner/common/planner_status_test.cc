#include "onboard/planner/common/planner_status.h"

#include "gtest/gtest.h"

#include "onboard/planner/common/proto/planner_status.pb.h"

namespace qcraft::planner {

namespace {

TEST(PlannerStatus, ToFromProto) {
  const PlannerStatus status(PlannerStatusProto::INPUT_INCORRECT,
                             "bla bla.bla");
  PlannerStatusProto proto;
  status.ToProto(&proto);

  PlannerStatus status2;
  status2.FromProto(proto);

  EXPECT_EQ(status.ToString(), status2.ToString());
}

}  // namespace

}  // namespace qcraft::planner
