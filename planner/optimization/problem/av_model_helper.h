#ifndef ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_AV_MODEL_HELPER_H_
#define ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_AV_MODEL_HELPER_H_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "glog/logging.h"

#include "onboard/math/eigen.h"
#include "onboard/math/frenet_frame.h"
#include "onboard/planner/optimization/problem/cost_helper.h"

namespace qcraft {
namespace planner {

template <typename PROB>
class AvModelHelper : public CostHelper<PROB> {
 public:
  using StatesType = typename PROB::StatesType;
  using ControlsType = typename PROB::ControlsType;

  AvModelHelper(int horizon, std::vector<double> dists_to_rac,
                std::vector<double> angles_to_axis, std::string name)
      : CostHelper<PROB>(horizon, std::move(name)),
        dist_to_rac_(std::move(dists_to_rac)),
        angle_to_axis_(std::move(angles_to_axis)),
        circle_num_(dist_to_rac_.size()) {
    QCHECK_EQ(dist_to_rac_.size(), angle_to_axis_.size());
    rotation_.reserve(circle_num_);
    for (const double angle : angle_to_axis_) {
      rotation_.push_back(Vec2d::FastUnitFromAngle(angle));
    }
    centers_.resize(CostHelper<PROB>::horizon());
    tangents_.resize(CostHelper<PROB>::horizon());
    normals_.resize(CostHelper<PROB>::horizon());
    for (int k = 0; k < CostHelper<PROB>::horizon(); ++k) {
      centers_[k].resize(circle_num_, Vec2d(0.0, 0.0));
      tangents_[k].resize(circle_num_, Vec2d(0.0, 0.0));
      normals_[k].resize(circle_num_, Vec2d(0.0, 0.0));
    }
  }

  const std::vector<std::vector<Vec2d>>& centers() const { return centers_; }
  const std::vector<std::vector<Vec2d>>& tangents() const { return tangents_; }
  const std::vector<std::vector<Vec2d>>& normals() const { return normals_; }

  const std::vector<double>& dist_to_rac() const { return dist_to_rac_; }
  const std::vector<double>& angle_to_axis() const { return angle_to_axis_; }

  void Update(const StatesType& xs, const ControlsType& /*us*/) override {
    for (int k = 0; k < CostHelper<PROB>::horizon(); ++k) {
      auto& centers_k = centers_[k];
      auto& tangents_k = tangents_[k];
      auto& normals_k = normals_[k];
      const Vec2d pos = PROB::pos(xs, k);
      const Vec2d tangent = Vec2d::FastUnitFromAngleN12(PROB::theta(xs, k));
      for (int i = 0; i < circle_num_; ++i) {
        const Vec2d corner_tangent = tangent.Rotate(rotation_[i]);
        tangents_k[i] = corner_tangent;
        normals_k[i] = corner_tangent.Perp();
        centers_k[i] = pos + dist_to_rac_[i] * corner_tangent;
      }
    }
  }

 private:
  std::string name_ = "";
  std::vector<double> dist_to_rac_;
  std::vector<double> angle_to_axis_;
  std::vector<Vec2d> rotation_;
  int circle_num_;

  std::vector<std::vector<Vec2d>> centers_;
  std::vector<std::vector<Vec2d>> tangents_;
  std::vector<std::vector<Vec2d>> normals_;
};

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_AV_MODEL_HELPER_H_
