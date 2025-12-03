#ifndef ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_CENTER_LINE_QUERY_HELPER_H_
#define ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_CENTER_LINE_QUERY_HELPER_H_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "glog/logging.h"

#include "onboard/lite/logging.h"
#include "onboard/math/eigen.h"
#include "onboard/math/frenet_frame.h"
#include "onboard/planner/optimization/problem/cost_helper.h"
#include "onboard/planner/proto/planner_params.pb.h"

namespace qcraft {
namespace planner {

template <typename PROB>
class CenterLineQueryHelper : public CostHelper<PROB> {
 public:
  using StatesType = typename PROB::StatesType;
  using ControlsType = typename PROB::ControlsType;
  CenterLineQueryHelper(
      int horizon,
      std::vector<VehicleCircleModelParamsProto::CircleParams> av_circle_model,
      const FrenetFrame* path, int last_real_point_index, std::string name)
      : CostHelper<PROB>(horizon, std::move(name)),
        path_(path),
        last_real_point_index_(last_real_point_index),
        av_circle_model_(std::move(av_circle_model)),
        circle_num_(av_circle_model_.size()) {
    QCHECK_NOTNULL(path_);

    for (int i = 0; i < circle_num_; ++i) {
      if (av_circle_model_[i].type() ==
          VehicleCircleModelParamsProto::REAR_AXIS_CENTER) {
        rac_index_ = i;
      }
      if (av_circle_model_[i].type() !=
              VehicleCircleModelParamsProto::REAR_AXIS_CENTER &&
          av_circle_model_[i].type() !=
              VehicleCircleModelParamsProto::MID_AXIS_CENTER &&
          av_circle_model_[i].type() !=
              VehicleCircleModelParamsProto::FRONT_AXIS_CENTER) {
        QLOG(FATAL) << "Cicle type "
                    << VehicleCircleModelParamsProto::Type_Name(
                           av_circle_model_[i].type())
                    << " is not allowed!";
      }
    }
    QCHECK_NE(rac_index_, -1);

    // Resize queried info.
    av_tangents_.resize(CostHelper<PROB>::horizon());
    s_l_list_.resize(circle_num_);
    normals_.resize(circle_num_);
    indices_.resize(circle_num_);
    index_pairs_.resize(circle_num_);
    alphas_.resize(circle_num_);
    for (int i = 0; i < circle_num_; ++i) {
      s_l_list_[i].resize(CostHelper<PROB>::horizon());
      normals_[i].resize(CostHelper<PROB>::horizon());
      indices_[i].resize(CostHelper<PROB>::horizon());
      index_pairs_[i].resize(CostHelper<PROB>::horizon());
      alphas_[i].resize(CostHelper<PROB>::horizon());
    }
  }

  const std::vector<VehicleCircleModelParamsProto::CircleParams>&
  av_circle_model() const {
    return av_circle_model_;
  }
  int rac_index() const { return rac_index_; }

  const std::vector<Vec2d>& av_tangents() const { return av_tangents_; }

  // Returns result of Rac.
  const std::vector<FrenetCoordinate>& s_l_list() const {
    return s_l_list_[rac_index_];
  }
  const std::vector<Vec2d>& normals() const { return normals_[rac_index_]; }
  const std::vector<int>& indices() const { return indices_[rac_index_]; }
  const std::vector<std::pair<int, int>>& index_pairs() const {
    return index_pairs_[rac_index_];
  }
  const std::vector<double>& alphas() const { return alphas_[rac_index_]; }

  // Returns resutls of all control points at axis, currently include Rac, Fac
  // and Mids for path boundary cost.
  const std::vector<std::vector<FrenetCoordinate>>& all_s_l_list() const {
    return s_l_list_;
  }
  const std::vector<std::vector<Vec2d>>& all_normals() const {
    return normals_;
  }
  const std::vector<std::vector<int>>& all_indices() const { return indices_; }
  const std::vector<std::vector<std::pair<int, int>>>& all_index_pairs() const {
    return index_pairs_;
  }
  const std::vector<std::vector<double>>& all_alphas() const { return alphas_; }

  // Path info.
  const std::vector<double>& s_knots() const { return path_->s_knots(); }
  const std::vector<Vec2d>& tangents() const { return path_->tangents(); }
  const std::vector<Vec2d>& points() const { return path_->points(); }
  int last_real_point_index() const { return last_real_point_index_; }

  void Update(const StatesType& xs, const ControlsType& /*us*/) override {
    const auto& raw_indices = path_->raw_indices();
    // Update Rac firstly.
    for (int k = 0; k < CostHelper<PROB>::horizon(); ++k) {
      const Vec2d pos = PROB::pos(xs, k);
      av_tangents_[k] = Vec2d::FastUnitFromAngleN12(PROB::theta(xs, k));
      int* index_ptr = &indices_[rac_index_][k];
      path_->XYToSL(pos, &s_l_list_[rac_index_][k], &normals_[rac_index_][k],
                    index_ptr, &alphas_[rac_index_][k]);
      auto& index_pair = index_pairs_[rac_index_][k];
      index_pair.first = raw_indices[*index_ptr];
      index_pair.second = raw_indices[(*index_ptr) + 1];
    }
    // Update other control points.
    // TODO(zhuang): Maybe we can search from Rac sequentially.
    for (int k = 0; k < CostHelper<PROB>::horizon(); ++k) {
      const Vec2d rac_pos = PROB::StateGetPos(PROB::GetStateAtStep(xs, k));
      for (int i = 0; i < circle_num_; ++i) {
        if (i == rac_index_) continue;
        const Vec2d pos =
            rac_pos + av_tangents_[k] * av_circle_model_[i].dist_to_rac();
        int* index_ptr = &indices_[i][k];
        path_->XYToSL(pos, &s_l_list_[i][k], &normals_[i][k], index_ptr,
                      &alphas_[i][k]);
        auto& index_pair = index_pairs_[i][k];
        index_pair.first = raw_indices[*index_ptr];
        index_pair.second = raw_indices[(*index_ptr) + 1];
      }
    }
  }

  const FrenetFrame& path() const {
    QCHECK_NOTNULL(path_);
    return *path_;
  }

 private:
  std::string name_ = "";
  const FrenetFrame* path_;  // Not owned.
  int last_real_point_index_;

  std::vector<VehicleCircleModelParamsProto::CircleParams> av_circle_model_;
  int circle_num_;

  // Frenet xy to sl query results.
  int rac_index_ = -1;
  std::vector<Vec2d> av_tangents_;
  std::vector<std::vector<FrenetCoordinate>> s_l_list_;
  std::vector<std::vector<Vec2d>> normals_;
  std::vector<std::vector<int>> indices_;
  std::vector<std::vector<std::pair<int, int>>> index_pairs_;
  std::vector<std::vector<double>> alphas_;
};

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_CENTER_LINE_QUERY_HELPER_H_
