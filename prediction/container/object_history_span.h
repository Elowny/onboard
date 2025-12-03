#ifndef ONBOARD_PREDICTION_CONTAINER_OBJECT_HISTORY_SPAN_H_
#define ONBOARD_PREDICTION_CONTAINER_OBJECT_HISTORY_SPAN_H_
#include <stddef.h>

#include "boost/circular_buffer.hpp"

#include "onboard/math/geometry/box2d.h"
#include "onboard/math/vec.h"
#include "onboard/prediction/container/prediction_object.h"
#include "onboard/prediction/prediction_defs.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/utils/elements_history.h"

namespace qcraft {
namespace prediction {
class ObjectHistorySpan
    : public elements_history::CircularSpan<
          elements_history::Node<double, PredictionObject>> {
 public:
  using CircularSpan<
      elements_history::Node<double, PredictionObject>>::CircularSpan;
  const ObjectIDType& id() const {
    return CircularSpan::back().val.object_proto().id();
  }
  const ObjectProto& object_proto() const {
    return CircularSpan::back().val.object_proto();
  }
  const Vec2d& pos() const { return CircularSpan::back().val.pos(); }
  double heading() const { return CircularSpan::back().val.heading(); }
  double v() const { return CircularSpan::back().val.v(); }

  ObjectType type() const {
    return CircularSpan::back().val.object_proto().type();
  }
  double timestamp() const {
    return CircularSpan::back().val.object_proto().timestamp();
  }
  const Box2d& bounding_box() const {
    return CircularSpan::back().val.bounding_box();
  }
  ObjectHistorySpan GetHistoryFrom(double t) const;
};
}  // namespace prediction
}  // namespace qcraft

#endif  // ONBOARD_PREDICTION_CONTAINER_OBJECT_HISTORY_SPAN_H_
