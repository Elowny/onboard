#ifndef ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_STATIC_BOUNDARY_COST_H_
#define ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_STATIC_BOUNDARY_COST_H_

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type

#include <algorithm>
#include <iterator>
#include <memory>
#include <optional>
#include <ostream>
#include <string>
#include <utility>
#include <vector>

#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "glog/logging.h"

#include "onboard/base/macros.h"
#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/math/frenet_common.h"
#include "onboard/math/frenet_frame.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/optimization/problem/center_line_query_helper.h"
#include "onboard/planner/optimization/problem/cost.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {
namespace planner {

// A static boundary represented by a sequence of path points, and the
// associated boundary distances to those path points.
template <typename PROB>
class StaticBoundaryCost : public Cost<PROB> {
 public:
  struct BreakPoint {
    bool has_break_point = false;  // if true, then N > 2.
    std::vector<double> alphas;    // [0, a1, ... , 1] in order, size = N.
    std::vector<double> alpha_interval_inv;  // size = N - 1.
    std::vector<double> dists;               // size = N.

    // Input alpha must be in [0,1].
    std::pair<int, int> GetIndexPair(double alpha) const {
      const auto iter = std::lower_bound(alphas.begin(), alphas.end(), alpha);
      const int index = std::distance(alphas.begin(), iter);
      if (UNLIKELY(index == 0)) {
        return std::make_pair(0, 1);
      }
      if (UNLIKELY(index == alphas.size())) {
        return std::make_pair(alphas.size() - 2, alphas.size() - 1);
      }
      return std::make_pair(index - 1, index);
    }

    double ComputeBoundaryDist(const std::pair<int, int>& index_pair,
                               double alpha) const {
      const double inner_alpha = (alpha - alphas[index_pair.first]) *
                                 alpha_interval_inv[index_pair.first];
      return Lerp(dists[index_pair.first], dists[index_pair.second],
                  inner_alpha);
    }
  };

  using StateType = typename PROB::StateType;
  using ControlType = typename PROB::ControlType;
  using StatesType = typename PROB::StatesType;
  using ControlsType = typename PROB::ControlsType;

  using GType = typename PROB::GType;
  using DGDxType = typename PROB::DGDxType;
  using DGDuType = typename PROB::DGDuType;
  using DDGDxDxType = typename PROB::DDGDxDxType;
  using DDGDxDuType = typename PROB::DDGDxDuType;
  using DDGDuDxType = typename PROB::DDGDuDxType;
  using DDGDuDuType = typename PROB::DDGDuDuType;

  using DividedG = typename Cost<PROB>::DividedG;

  using CostType = typename Cost<PROB>::CostType;

  static constexpr double kNormalizedScale = 10.0;
  StaticBoundaryCost(
      int horizon, VehicleGeometryParamsProto vehicle_geometry_params,
      const std::vector<Vec2d>& path_points,
      const CenterLineQueryHelper<PROB>* center_line_helper,
      std::vector<double> static_boundary_dists,
      std::vector<BreakPoint> break_points, std::vector<double> ref_gains,
      bool left, const std::vector<double>& dist_to_rac,
      const std::optional<VehicleCircleModelParamsProto::CircleParams>&
          corner_info,
      bool use_qtfm, std::vector<std::string> sub_names,
      std::vector<double> cascade_gains = {1.0},
      std::vector<double> cascade_buffers = {0.0},
      bool using_hessian_approximate = false,
      std::string name = absl::StrCat(PROB::kProblemPrefix,
                                      "StaticBoundaryCost"),
      double scale = 1.0, CostType cost_type = Cost<PROB>::CostType::MUST_HAVE)
      : Cost<PROB>(std::move(name), scale * kNormalizedScale, cost_type),
        horizon_(horizon),
        vehicle_geometry_params_(std::move(vehicle_geometry_params)),
        path_points_(path_points),
        static_boundary_dists_(std::move(static_boundary_dists)),
        path_boundary_size_(static_boundary_dists_.size()),
        break_points_(std::move(break_points)),
        ref_gains_(std::move(ref_gains)),
        left_(left),
        using_hessian_approximate_(using_hessian_approximate),
        dist_to_rac_(dist_to_rac),
        circle_num_(dist_to_rac.size()),
        has_corner_(corner_info.has_value()),
        cascade_gains_(std::move(cascade_gains)),
        cascade_buffers_(std::move(cascade_buffers)),
        cascade_num_(cascade_gains_.size()),
        sub_names_(std::move(sub_names)),
        center_line_helper_(center_line_helper) {
    QCHECK_GT(horizon_, 0);
    QCHECK_GT(path_points_.size(), 1);
    QCHECK_EQ(cascade_num_, sub_names_.size());
    QCHECK_EQ(cascade_num_, cascade_gains_.size());
    QCHECK_EQ(path_boundary_size_ - 1, break_points_.size());
    QCHECK_EQ(path_boundary_size_, path_points_.size());
    QCHECK_EQ(path_boundary_size_, ref_gains_.size());
    if (corner_info.has_value()) {
      corner_dist_to_rac_ = corner_info->dist_to_rac();
      corner_radius_ = corner_info->radius();
      corner_rotation_ = Vec2d::FastUnitFromAngle(corner_info->angle_to_axis());
    }
    path_lengths_sqr_inv_.reserve(path_points_.size() - 1);
    for (int i = 0; i + 1 < path_points_.size(); ++i) {
      path_lengths_sqr_inv_.push_back(
          1.0 / (path_points_[i + 1] - path_points_[i]).squaredNorm());
    }

    if (center_line_helper_ != nullptr) {
      const auto& circle_model = center_line_helper_->av_circle_model();
      QCHECK_EQ(circle_model.size(), dist_to_rac_.size());
      for (int i = 0; i < circle_model.size(); ++i) {
        QCHECK_NEAR(dist_to_rac_[i], circle_model[i].dist_to_rac(), 1e-9);
      }
    } else {
      if (use_qtfm) {
        path_ = std::make_unique<QtfmEnhancedKdTreeFrenetFrame>(
            BuildQtfmEnhancedKdTreeFrenetFrame(path_points,
                                               /*down_sample_raw_points=*/true)
                .value());
      } else {
        path_ = std::make_unique<KdTreeFrenetFrame>(
            BuildKdTreeFrenetFrame(path_points, /*down_sample_raw_points=*/true)
                .value());
      }
    }

    av_tangents_.resize(horizon_);
    // Resize to [circle_num_][horizon_].
    circle_dists_to_static_boundary_.resize(circle_num_);
    normals_.resize(circle_num_);
    index_pairs_.resize(circle_num_);
    break_point_index_pairs_.resize(circle_num_);
    alphas_.resize(circle_num_);
    gains_.resize(circle_num_);
    effective_index_.resize(circle_num_);
    for (int i = 0; i < circle_num_; ++i) {
      circle_dists_to_static_boundary_[i].resize(horizon_);
      normals_[i].resize(horizon_);
      index_pairs_[i].resize(horizon_);
      break_point_index_pairs_[i].resize(horizon_);
      alphas_[i].resize(horizon_);
      gains_[i].resize(horizon_);
    }
    // Resize to [horizon_].
    if (has_corner_) {
      corner_tangents_.resize(horizon_);
      corner_circle_dists_to_static_boundary_.resize(horizon_);
      corner_normals_.resize(horizon_);
      corner_index_pairs_.resize(horizon_);
      corner_break_point_index_pairs_.resize(horizon_);
      corner_alphas_.resize(horizon_);
      corner_gains_.resize(horizon_);
    }
  }

  DividedG SumGForAllSteps(const StatesType& /*xs*/, const ControlsType& /*us*/,
                           int horizon) const override {
    QCHECK_EQ(horizon, horizon_);
    DividedG res(sub_names_.size());
    // Points at axis(Rac, Fac and Mids).
    for (int i = 0; i < circle_num_; ++i) {
      for (int k = 0; k < horizon_; ++k) {
        if (k >= effective_index_[i]) break;
        const double d = circle_dists_to_static_boundary_[i][k];
        for (int j = 0; j < cascade_num_; ++j) {
          if (d < cascade_buffers_[j]) {
            res.AddSubG(j, 0.5 * Cost<PROB>::scale() * gains_[i][k] *
                               cascade_gains_[j] *
                               Sqr(cascade_buffers_[j] - d));
          }
        }
      }
    }
    // Corner.
    if (has_corner_) {
      for (int k = 0; k < horizon_; ++k) {
        if (k >= corner_effective_index_) break;
        const double d = corner_circle_dists_to_static_boundary_[k];
        for (int j = 0; j < cascade_num_; ++j) {
          if (d < cascade_buffers_[j]) {
            res.AddSubG(j, 0.5 * Cost<PROB>::scale() * corner_gains_[k] *
                               cascade_gains_[j] *
                               Sqr(cascade_buffers_[j] - d));
          }
        }
      }
    }
    for (int i = 0; i < sub_names_.size(); ++i) {
      res.SetSubName(i, sub_names_[i] + Cost<PROB>::name());
    }
    return res;
  }

  // g.
  DividedG EvaluateGWithDebugInfo(int k, const StateType& /*x*/,
                                  const ControlType& /*u*/,
                                  bool using_scale) const override {
    DividedG res(sub_names_.size());
    for (int i = 0; i < sub_names_.size(); ++i) {
      res.SetSubName(i, sub_names_[i] + Cost<PROB>::name());
    }
    // Points at axis(Rac, Fac and Mids).
    for (int i = 0; i < circle_num_; ++i) {
      if (k >= effective_index_[i]) continue;
      const double d = circle_dists_to_static_boundary_[i][k];
      for (int j = 0; j < cascade_num_; ++j) {
        if (d < cascade_buffers_[j]) {
          res.AddSubG(j, 0.5 * Cost<PROB>::scale() * gains_[i][k] *
                             cascade_gains_[j] * Sqr(cascade_buffers_[j] - d));
        }
      }
    }
    // Corner.
    if (has_corner_ && k < corner_effective_index_) {
      const double d = corner_circle_dists_to_static_boundary_[k];
      for (int j = 0; j < cascade_num_; ++j) {
        if (d < cascade_buffers_[j]) {
          res.AddSubG(j, 0.5 * Cost<PROB>::scale() * corner_gains_[k] *
                             cascade_gains_[j] * Sqr(cascade_buffers_[j] - d));
        }
      }
    }
    if (!using_scale) {
      res.VecDiv(cascade_gains_);
    }
    return res;
  }

  // g.
  double EvaluateG(int k, const StateType& /*x*/,
                   const ControlType& /*u*/) const override {
    double g = 0.0;
    // Points at axis(Rac, Fac and Mids).
    for (int i = 0; i < circle_num_; ++i) {
      if (k >= effective_index_[i]) continue;
      const double d = circle_dists_to_static_boundary_[i][k];
      for (int j = 0; j < cascade_num_; ++j) {
        if (d < cascade_buffers_[j]) {
          g += 0.5 * Cost<PROB>::scale() * gains_[i][k] * cascade_gains_[j] *
               Sqr(cascade_buffers_[j] - d);
        }
      }
    }
    // Corner.
    if (has_corner_ && k < corner_effective_index_) {
      const double d = corner_circle_dists_to_static_boundary_[k];
      for (int j = 0; j < cascade_num_; ++j) {
        if (d < cascade_buffers_[j]) {
          g += 0.5 * Cost<PROB>::scale() * corner_gains_[k] *
               cascade_gains_[j] * Sqr(cascade_buffers_[j] - d);
        }
      }
    }
    return g;
  }

  // Gradients with superposition.
  void AddDGDx(int k, const StateType& /*x*/, const ControlType& /*u*/,
               DGDxType* dgdx) const override {
    // Points at axis(Rac, Fac and Mids).
    for (int i = 0; i < circle_num_; ++i) {
      if (k < effective_index_[i]) {
        AddCircleDGDx(dgdx, circle_dists_to_static_boundary_[i][k],
                      dist_to_rac_[i], av_tangents_[k], index_pairs_[i][k],
                      break_point_index_pairs_[i][k], alphas_[i][k],
                      normals_[i][k], gains_[i][k]);
      }
    }
    // Corner.
    if (has_corner_ && k < corner_effective_index_) {
      AddCircleDGDx(dgdx, corner_circle_dists_to_static_boundary_[k],
                    corner_dist_to_rac_, corner_tangents_[k],
                    corner_index_pairs_[k], corner_break_point_index_pairs_[k],
                    corner_alphas_[k], corner_normals_[k], corner_gains_[k]);
    }
  }

  void AddDGDu(int /*k*/, const StateType& /*x*/, const ControlType& /*u*/,
               DGDuType* /*dgdu*/) const override {}

  // Hessians with superposition.
  void AddDDGDxDx(int k, const StateType& /*x*/, const ControlType& /*u*/,
                  DDGDxDxType* ddgdxdx) const override {
    // Points at axis(Rac, Fac and Mids).
    for (int i = 0; i < circle_num_; ++i) {
      if (k < effective_index_[i]) {
        for (int j = 0; j < cascade_num_; ++j) {
          AddCircleDDGDxDx(ddgdxdx, circle_dists_to_static_boundary_[i][k],
                           dist_to_rac_[i], av_tangents_[k], index_pairs_[i][k],
                           break_point_index_pairs_[i][k], alphas_[i][k],
                           normals_[i][k], gains_[i][k]);
        }
      }
    }
    // Corner.
    if (has_corner_ && k < corner_effective_index_) {
      AddCircleDDGDxDx(ddgdxdx, corner_circle_dists_to_static_boundary_[k],
                       corner_dist_to_rac_, corner_tangents_[k],
                       corner_index_pairs_[k],
                       corner_break_point_index_pairs_[k], corner_alphas_[k],
                       corner_normals_[k], corner_gains_[k]);
    }
  }

  void AddDDGDuDx(int /*k*/, const StateType& /*x*/, const ControlType& /*u*/,
                  DDGDuDxType* /*ddgdudx*/) const override {}
  void AddDDGDuDu(int /*k*/, const StateType& /*x*/, const ControlType& /*u*/,
                  DDGDuDuType* /*ddgdudu*/) const override {}

  void Update(const StatesType& xs, const ControlsType& /*us*/,
              int horizon) override {
    QCHECK_EQ(horizon, horizon_);
    if (left_) {
      VLOG(4) << "Update for left boundary.";
    } else {
      VLOG(4) << "Update for right boundary.";
    }

    if (center_line_helper_ == nullptr) {
      // Update tangents.
      for (int k = 0; k < horizon_; ++k) {
        av_tangents_[k] = Vec2d::FastUnitFromAngleN12(PROB::theta(xs, k));
      }
      // Update points at axis(Rac, Fac and Mids).
      const double half_width = vehicle_geometry_params_.width() * 0.5;
      for (int i = 0; i < circle_num_; ++i) {
        Update(av_tangents_, dist_to_rac_[i], half_width, xs, &normals_[i],
               &circle_dists_to_static_boundary_[i], &gains_[i],
               &index_pairs_[i], &break_point_index_pairs_[i], &alphas_[i],
               &effective_index_[i]);
      }
      // Update corner.
      if (has_corner_) {
        for (int k = 0; k < horizon_; ++k) {
          corner_tangents_[k] = av_tangents_[k].Rotate(corner_rotation_);
        }
        Update(corner_tangents_, corner_dist_to_rac_, corner_radius_, xs,
               &corner_normals_, &corner_circle_dists_to_static_boundary_,
               &corner_gains_, &corner_index_pairs_,
               &corner_break_point_index_pairs_, &corner_alphas_,
               &corner_effective_index_);
      }
    } else {
      // Update points at axis(Rac, Fac and Mids).
      UpdateWithCenterLineHelper();
      // Update corner.
      if (has_corner_) {
        for (int k = 0; k < horizon_; ++k) {
          corner_tangents_[k] = av_tangents_[k].Rotate(corner_rotation_);
        }
        Update(corner_tangents_, corner_dist_to_rac_, corner_radius_, xs,
               &corner_normals_, &corner_circle_dists_to_static_boundary_,
               &corner_gains_, &corner_index_pairs_,
               &corner_break_point_index_pairs_, &corner_alphas_,
               &corner_effective_index_);
      }
    }
  }

 private:
  void UpdateWithCenterLineHelper() {
    const double half_width = vehicle_geometry_params_.width() * 0.5;
    const double sign = left_ ? -1.0 : 1.0;

    av_tangents_ = center_line_helper_->av_tangents();
    normals_ = center_line_helper_->all_normals();
    index_pairs_ = center_line_helper_->all_index_pairs();
    alphas_ = center_line_helper_->all_alphas();

    const auto& all_s_l_list = center_line_helper_->all_s_l_list();
    for (int i = 0; i < circle_num_; ++i) {
      effective_index_[i] = horizon_;
      for (int k = 0; k < horizon_; ++k) {
        const FrenetCoordinate& sl = all_s_l_list[i][k];
        const std::pair<int, int>& index_pair = index_pairs_[i][k];
        const double alpha = alphas_[i][k];
        if (index_pair.second >= path_boundary_size_) {
          effective_index_[i] = k;
          break;
        }
        if (alpha < 0.0) {
          circle_dists_to_static_boundary_[i][k] =
              static_boundary_dists_[index_pair.first] + sign * sl.l -
              half_width;
          gains_[i][k] = ref_gains_[index_pair.first];
        } else if (alpha > 1.0) {
          circle_dists_to_static_boundary_[i][k] =
              static_boundary_dists_[index_pair.second] + sign * sl.l -
              half_width;
          gains_[i][k] = ref_gains_[index_pair.second];
        } else {
          double boundary_dist;
          if (break_points_[index_pair.first].has_break_point) {
            break_point_index_pairs_[i][k] =
                break_points_[index_pair.first].GetIndexPair(alpha);
            boundary_dist = break_points_[index_pair.first].ComputeBoundaryDist(
                break_point_index_pairs_[i][k], alpha);
          } else {
            boundary_dist =
                Lerp(static_boundary_dists_[index_pair.first],
                     static_boundary_dists_[index_pair.second], alpha);
          }
          circle_dists_to_static_boundary_[i][k] =
              boundary_dist + sign * sl.l - half_width;
          gains_[i][k] = ref_gains_[index_pair.first];
        }

        if (UNLIKELY(VLOG_IS_ON(4))) {
          VLOG(4) << "Dists [" << i << "][" << k << "]: " << sl.l;
          VLOG(4) << "Normal [" << i << "][" << k << "]: " << normals_[i][k].x()
                  << " " << normals_[i][k].y();
          VLOG(4) << "Index pair [" << i << "][" << k
                  << "]: " << index_pair.first << " " << index_pair.second;
          VLOG(4) << "Alpha [" << i << "][" << k << "]: " << alpha;
          VLOG(4) << "To boundary dists [" << i << "][" << k
                  << "]: " << circle_dists_to_static_boundary_[i][k];
        }
      }
    }
  }

  const FrenetFrame& GetPath() const {
    if (center_line_helper_ != nullptr) {
      return center_line_helper_->path();
    } else {
      QCHECK_NOTNULL(path_);
      return *path_;
    }
  }

  void Update(const std::vector<Vec2d>& tangents, double dist_to_rac,
              double radius, const StatesType& xs, std::vector<Vec2d>* normals,
              std::vector<double>* to_static_boundary_dists,
              std::vector<double>* gains,
              std::vector<std::pair<int, int>>* index_pairs,
              std::vector<std::pair<int, int>>* break_point_index_pairs,
              std::vector<double>* alphas, int* effective_index) const {
    const double sign = left_ ? -1.0 : 1.0;
    *effective_index = horizon_;
    for (int k = 0; k < horizon_; ++k) {
      const Vec2d xy = PROB::pos(xs, k) + dist_to_rac * tangents[k];
      FrenetCoordinate sl;
      Vec2d normal;
      std::pair<int, int> index_pair;
      double alpha = 0.0;

      GetPath().XYToSL(xy, &sl, &normal, &index_pair, &alpha);

      if (index_pair.second >= path_boundary_size_) {
        *effective_index = k;
        break;
      }

      (*normals)[k] = normal;
      (*index_pairs)[k] = index_pair;
      (*alphas)[k] = alpha;
      if (alpha < 0.0) {
        (*to_static_boundary_dists)[k] =
            static_boundary_dists_[index_pair.first] + sign * sl.l - radius;
        (*gains)[k] = ref_gains_[index_pair.first];
      } else if (alpha > 1.0) {
        (*to_static_boundary_dists)[k] =
            static_boundary_dists_[index_pair.second] + sign * sl.l - radius;
        (*gains)[k] = ref_gains_[index_pair.second];
      } else {
        double boundary_dist;
        if (break_points_[index_pair.first].has_break_point) {
          (*break_point_index_pairs)[k] =
              break_points_[index_pair.first].GetIndexPair(alpha);
          boundary_dist = break_points_[index_pair.first].ComputeBoundaryDist(
              (*break_point_index_pairs)[k], alpha);
        } else {
          boundary_dist =
              Lerp(static_boundary_dists_[index_pair.first],
                   static_boundary_dists_[index_pair.second], alpha);
        }
        (*to_static_boundary_dists)[k] = boundary_dist + sign * sl.l - radius;
        (*gains)[k] = ref_gains_[index_pair.first];
      }

      if (UNLIKELY(VLOG_IS_ON(4))) {
        VLOG(4) << "Dists [" << k << "]: " << sl.l;
        VLOG(4) << "Normal [" << k << "]: " << normals->at(k).x() << " "
                << normals->at(k).y();
        VLOG(4) << "Index pair [" << k << "]: " << index_pairs->at(k).first
                << " " << index_pairs->at(k).second;
        VLOG(4) << "Alpha [" << k << "]: " << alphas->at(k);
        VLOG(4) << "To boundary dists [" << k
                << "]: " << (*to_static_boundary_dists)[k];
      }
    }
  }

  void AddCircleDGDx(DGDxType* dgdx, double d_corner, double dist_to_rac,
                     const Vec2d& tangent,
                     const std::pair<int, int>& corner_index_pair,
                     const std::pair<int, int>& break_point_index_pair,
                     double corner_alpha, const Vec2d& corner_normal,
                     double point_gain) const {
    const Vec2d d_delta(-dist_to_rac * tangent.y(), dist_to_rac * tangent.x());
    for (int j = 0; j < cascade_num_; ++j) {
      if (d_corner < cascade_buffers_[j]) {
        const double d_penetration = d_corner - cascade_buffers_[j];
        const double sign = left_ ? -1.0 : 1.0;
        const double final_gain =
            Cost<PROB>::scale() * cascade_gains_[j] * point_gain;
        if (corner_alpha < 0.0 || corner_alpha > 1.0) {
          (*dgdx).template segment<2>(0) +=
              final_gain * d_penetration * sign * corner_normal.transpose();
          (*dgdx)[2] +=
              final_gain * d_penetration * sign * corner_normal.dot(d_delta);
        } else {
          Vec2d segment = path_points_[corner_index_pair.second] -
                          path_points_[corner_index_pair.first];
          double segment_sqr_norm_inv =
              path_lengths_sqr_inv_[corner_index_pair.first];
          double d0, d1;
          const auto& break_point = break_points_[corner_index_pair.first];
          if (break_point.has_break_point) {
            segment *= break_point.alphas[break_point_index_pair.second] -
                       break_point.alphas[break_point_index_pair.first];
            segment_sqr_norm_inv *= Sqr(
                break_point.alpha_interval_inv[break_point_index_pair.first]);
            d0 = break_point.dists[break_point_index_pair.first];
            d1 = break_point.dists[break_point_index_pair.second];
          } else {
            d0 = static_boundary_dists_[corner_index_pair.first];
            d1 = static_boundary_dists_[corner_index_pair.second];
          }
          const Vec2d vec =
              (d1 - d0) * segment * segment_sqr_norm_inv + sign * corner_normal;
          (*dgdx).template segment<2>(0) +=
              final_gain * d_penetration *
              ((d1 - d0) * segment.transpose() * segment_sqr_norm_inv +
               sign * corner_normal.transpose());
          (*dgdx)[2] += final_gain * d_penetration * vec.dot(d_delta);
        }
      }
    }
  }

  void AddCircleDDGDxDx(DDGDxDxType* ddgdxdx, double d_corner,
                        double dist_to_rac, const Vec2d& tangent,
                        const std::pair<int, int>& corner_index_pair,
                        const std::pair<int, int>& break_point_index_pair,
                        double corner_alpha, const Vec2d& corner_normal,
                        double point_gain) const {
    const Vec2d d_delta(-dist_to_rac * tangent.y(), dist_to_rac * tangent.x());
    const Vec2d dd_delta = -dist_to_rac * tangent;
    for (int j = 0; j < cascade_num_; ++j) {
      if (d_corner < cascade_buffers_[j]) {
        const double d_penetration = d_corner - cascade_buffers_[j];
        Vec2d segment = path_points_[corner_index_pair.second] -
                        path_points_[corner_index_pair.first];
        double segment_sqr_norm_inv =
            path_lengths_sqr_inv_[corner_index_pair.first];
        double d0, d1;
        const auto& break_point = break_points_[corner_index_pair.first];
        if (break_point.has_break_point) {
          segment *= break_point.alphas[break_point_index_pair.second] -
                     break_point.alphas[break_point_index_pair.first];
          segment_sqr_norm_inv *=
              Sqr(break_point.alpha_interval_inv[break_point_index_pair.first]);
          d0 = break_point.dists[break_point_index_pair.first];
          d1 = break_point.dists[break_point_index_pair.second];
        } else {
          d0 = static_boundary_dists_[corner_index_pair.first];
          d1 = static_boundary_dists_[corner_index_pair.second];
        }
        const double sign = left_ ? -1.0 : 1.0;
        const double final_gain =
            Cost<PROB>::scale() * cascade_gains_[j] * point_gain;
        if (using_hessian_approximate_) {
          if (corner_alpha < 0.0 || corner_alpha > 1.0) {
            (*ddgdxdx).template block<2, 2>(PROB::kStateXIndex,
                                            PROB::kStateXIndex) +=
                final_gain * corner_normal * corner_normal.transpose();
            (*ddgdxdx).template block<2, 1>(PROB::kStateXIndex,
                                            PROB::kStateThetaIndex) +=
                final_gain * corner_normal * corner_normal.dot(d_delta);
            (*ddgdxdx).template block<1, 2>(PROB::kStateThetaIndex,
                                            PROB::kStateXIndex) +=
                final_gain * corner_normal.transpose() *
                corner_normal.dot(d_delta);
            (*ddgdxdx)(PROB::kStateThetaIndex, PROB::kStateThetaIndex) +=
                final_gain * (Sqr(corner_normal.dot(d_delta)));
          } else {
            const Vec2d vec = (d1 - d0) * segment * segment_sqr_norm_inv +
                              sign * corner_normal;
            (*ddgdxdx).template block<2, 2>(PROB::kStateXIndex,
                                            PROB::kStateXIndex) +=
                final_gain * vec * vec.transpose();
            (*ddgdxdx).template block<2, 1>(PROB::kStateXIndex,
                                            PROB::kStateThetaIndex) +=
                final_gain * vec * vec.dot(d_delta);
            (*ddgdxdx).template block<1, 2>(PROB::kStateThetaIndex,
                                            PROB::kStateXIndex) +=
                final_gain * vec.transpose() * vec.dot(d_delta);
            (*ddgdxdx)(PROB::kStateThetaIndex, PROB::kStateThetaIndex) +=
                final_gain * (Sqr(vec.dot(d_delta)));
          }
        } else {
          if (corner_alpha < 0.0 || corner_alpha > 1.0) {
            (*ddgdxdx).template block<2, 2>(PROB::kStateXIndex,
                                            PROB::kStateXIndex) +=
                final_gain * corner_normal * corner_normal.transpose();
            (*ddgdxdx).template block<2, 1>(PROB::kStateXIndex,
                                            PROB::kStateThetaIndex) +=
                final_gain * corner_normal * corner_normal.dot(d_delta);
            (*ddgdxdx).template block<1, 2>(PROB::kStateThetaIndex,
                                            PROB::kStateXIndex) +=
                final_gain * corner_normal.transpose() *
                corner_normal.dot(d_delta);
            (*ddgdxdx)(PROB::kStateThetaIndex, PROB::kStateThetaIndex) +=
                final_gain *
                (Sqr(corner_normal.dot(d_delta)) +
                 d_penetration * sign * corner_normal.dot(dd_delta));
          } else {
            const Vec2d vec = (d1 - d0) * segment * segment_sqr_norm_inv +
                              sign * corner_normal;
            (*ddgdxdx).template block<2, 2>(PROB::kStateXIndex,
                                            PROB::kStateXIndex) +=
                final_gain * vec * vec.transpose();
            (*ddgdxdx).template block<2, 1>(PROB::kStateXIndex,
                                            PROB::kStateThetaIndex) +=
                final_gain * vec * vec.dot(d_delta);
            (*ddgdxdx).template block<1, 2>(PROB::kStateThetaIndex,
                                            PROB::kStateXIndex) +=
                final_gain * vec.transpose() * vec.dot(d_delta);
            (*ddgdxdx)(PROB::kStateThetaIndex, PROB::kStateThetaIndex) +=
                final_gain *
                (Sqr(vec.dot(d_delta)) + d_penetration * vec.dot(dd_delta));
          }
        }
      }
    }
  }

  int horizon_ = 0;
  VehicleGeometryParamsProto vehicle_geometry_params_;
  std::vector<Vec2d> path_points_;            // size = n.
  std::vector<double> path_lengths_sqr_inv_;  // size = n-1.
  // Inner vectors are distances, size = n, outer vector is multi layer path
  // boundaries.
  std::vector<double> static_boundary_dists_;
  int path_boundary_size_;
  std::vector<BreakPoint> break_points_;
  std::vector<double> ref_gains_;
  // Whether the boundary is on the left of the path.
  bool left_ = false;
  bool using_hessian_approximate_;
  // Vehicle model info.
  std::vector<double> dist_to_rac_;
  int circle_num_;
  bool has_corner_ = false;
  double corner_dist_to_rac_;
  double corner_radius_;
  Vec2d corner_rotation_;

  std::vector<Vec2d> av_tangents_;
  std::vector<Vec2d> corner_tangents_;

  std::vector<double> cascade_gains_;
  std::vector<double> cascade_buffers_;
  int cascade_num_;
  std::vector<std::string> sub_names_;

  std::unique_ptr<FrenetFrame> path_;
  const CenterLineQueryHelper<PROB>* center_line_helper_;

  // States of all control points at axis(Rac, Fac and Mids).
  // Get the following with [circle_index][horizon_index].
  std::vector<std::vector<double>> circle_dists_to_static_boundary_;
  std::vector<std::vector<Vec2d>> normals_;
  std::vector<std::vector<std::pair<int, int>>> index_pairs_;
  std::vector<std::vector<std::pair<int, int>>> break_point_index_pairs_;
  std::vector<std::vector<double>> alphas_;
  std::vector<std::vector<double>> gains_;
  // Get effective_index_ with [circle_index].
  std::vector<int> effective_index_;

  // States of corner.
  std::vector<double> corner_circle_dists_to_static_boundary_;
  std::vector<Vec2d> corner_normals_;
  std::vector<std::pair<int, int>> corner_index_pairs_;
  std::vector<std::pair<int, int>> corner_break_point_index_pairs_;
  std::vector<double> corner_alphas_;
  std::vector<double> corner_gains_;
  int corner_effective_index_;
};

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_STATIC_BOUNDARY_COST_H_
