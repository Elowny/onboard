#ifndef ONBOARD_PLANNER_OBJECT_PLANNER_OBJECT_H_
#define ONBOARD_PLANNER_OBJECT_PLANNER_OBJECT_H_

#include <optional>
#include <string>
#include <vector>

#include "onboard/math/geometry/aabox2d.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/math/vec.h"
#include "onboard/planner/object/proto/planner_object.pb.h"
#include "onboard/planner/second_order_trajectory_point.h"
#include "onboard/prediction/object_prediction.h"
#include "onboard/prediction/predicted_trajectory.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/proto/perception/fusion/object.pb.h"
#include "onboard/proto/prediction.pb.h"

namespace qcraft {
namespace planner {

class PlannerObject {
 public:
  PlannerObject() = default;

  explicit PlannerObject(prediction::ObjectPrediction object_prediction);

  void ToPlannerObjectProto(PlannerObjectProto* planner_object_proto) const;

  const std::string& id() const { return object_proto_.id(); }

  int num_trajs() const { return prediction_.trajectories().size(); }

  const prediction::PredictedTrajectory& traj(int i) const {
    return prediction_.trajectories()[i];
  }

  const ObjectStopTimeProto& stop_time_info() const {
    return prediction_.stop_time();
  }

  const prediction::ObjectLongTermBehavior& long_term_behavior() const {
    return prediction_.long_term_behavior();
  }

  // Returns the most likely trajectory. Returns std::nullopt if the object has
  // no trajectory.
  std::optional<int> MostLikelyTrajectory() const;

  ObjectType type() const { return object_proto_.type(); }
  void set_type(ObjectType otype) { object_proto_.set_type(otype); }

  const ObjectProto& object_proto() const { return object_proto_; }

  // TODO(lidong): Delete this API.
  // For object type OT_VEHICLE, OT_MOTORCYCLIST, OT_CYCLIST, and OT_TRICYCLIST,
  // its pose v and a stand for longtiduinal quantities (may be negative); for
  // other types, its pose v and a stand for absoluate values.
  const SecondOrderTrajectoryPoint& pose() const { return pose_; }

  Vec2d velocity() const { return velocity_; }

  const Polygon2d& contour() const { return contour_; }

  // Contour without time shift.
  const Polygon2d& raw_contour() const { return raw_contour_; }

  const AABox2d& aabox() const { return aabox_; }

  // TODO(lidong): Remove this mutable function.
  void set_stationary(bool stationary) { is_stationary_ = stationary; }

  bool is_stationary() const { return is_stationary_; }

  const prediction::ObjectPrediction& prediction() const { return prediction_; }
  prediction::ObjectPrediction* mutable_prediction() { return &prediction_; }

  const Box2d& bounding_box() const { return bounding_box_; }
  // The bounding box returned by perception. If not exist, it returns
  // bounding_box().
  const Box2d& perception_bbox() const { return perception_bbox_; }

  double proto_timestamp() const { return object_proto_.timestamp(); }

  bool is_sim_agent() const { return is_sim_agent_; }

  bool is_large_vehicle() const { return is_large_vehicle_; }
  // The id of the base object controlled by a simulation agent.
  const std::string& base_id() const { return base_id_; }

 private:
  void FromPrediction(const prediction::ObjectPrediction& prediction);

  bool is_stationary_ = false;
  SecondOrderTrajectoryPoint pose_;
  Vec2d velocity_;
  Polygon2d contour_;
  Polygon2d raw_contour_;
  Box2d bounding_box_;
  Box2d perception_bbox_;
  AABox2d aabox_;
  prediction::ObjectPrediction prediction_;

  ObjectProto object_proto_;

  bool is_sim_agent_ = false;
  bool is_large_vehicle_ = false;
  std::string base_id_;

  friend class PlannerObjectBuilder;
};

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_OBJECT_PLANNER_OBJECT_H_
