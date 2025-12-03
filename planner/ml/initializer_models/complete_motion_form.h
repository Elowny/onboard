#ifndef ONBOARD_PLANNER_ML_INITIALIZER_MODELS_COMPLETE_MOTION_FORM_H_
#define ONBOARD_PLANNER_ML_INITIALIZER_MODELS_COMPLETE_MOTION_FORM_H_

#include <vector>

#include "onboard/planner/initializer/geometry/geometry_form.h"
#include "onboard/planner/initializer/geometry/geometry_form_builder.h"
#include "onboard/planner/initializer/motion_form.h"
#include "onboard/planner/initializer/motion_state.h"
#include "onboard/proto/trajectory_point.pb.h"

namespace qcraft::planner {

class CompleteMotion final : public MotionForm {
 public:
  explicit CompleteMotion(const GeometryFormBuilder* form_builder,
                          const std::vector<ApolloTrajectoryPointProto>& traj);

  double duration() const override { return duration_; }
  MotionState GetStartMotionState() const override;
  MotionState GetEndMotionState() const override;

  /**
  @brief: Use relative time to get a motion state
  **/
  MotionState State(double t) const override;

  const GeometryForm* geometry() const override { return nullptr; }

  MotionFormType type() const override {
    return MotionFormType::COMPLETE_MOTION;
  }
  SampledMotionFormStates SampleStates() const override;
  std::vector<MotionState> SampleEqualIntervalStates() const override;

 private:
  std::vector<MotionState> Sample(double d_t) const;
  double duration_ = 0.0;
  std::vector<ApolloTrajectoryPointProto> traj_;
  std::vector<double> relative_stations_;
  const GeometryFormBuilder* form_builder_;
};

}  // namespace qcraft::planner
#endif  // ONBOARD_PLANNER_ML_INITIALIZER_MODELS_COMPLETE_MOTION_FORM_H_
