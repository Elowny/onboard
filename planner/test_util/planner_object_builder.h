#ifndef ONBOARD_PLANNER_TEST_UTIL_PLANNER_OBJECT_BUILDER_H_
#define ONBOARD_PLANNER_TEST_UTIL_PLANNER_OBJECT_BUILDER_H_

#include <optional>

#include "onboard/math/vec.h"
#include "onboard/planner/object/planner_object.h"
#include "onboard/planner/test_util/object_prediction_builder.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/proto/perception/fusion/object.pb.h"

namespace qcraft {
namespace planner {

class PlannerObjectBuilder {
 public:
  // TODO(lidong) Delete all the pose functions, as pose is a derived field in
  // PlannerObject.
  PlannerObjectBuilder& set_type(ObjectType type);

  PlannerObjectBuilder& set_object(const ObjectProto& object);

  PlannerObjectBuilder& set_pos(const Vec2d& pos);

  PlannerObjectBuilder& set_v(double v);
  PlannerObjectBuilder& set_theta(double theta);

  PlannerObjectBuilder& set_stationary(bool stationary);

  ObjectPredictionBuilder* get_object_prediction_builder();

  PlannerObject Build();

 private:
  // TODO(lidong): Delete this function in the future.
  void FromObjectProto(const ObjectProto& object_proto);

  ObjectPredictionBuilder object_prediction_builder_;
  std::optional<ObjectProto> object_proto_;
  PlannerObject object_;
};

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_TEST_UTIL_PLANNER_OBJECT_BUILDER_H_
