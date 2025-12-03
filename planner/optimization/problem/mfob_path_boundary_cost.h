#ifndef ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_MFOB_PATH_BOUNDARY_COST_H_
#define ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_MFOB_PATH_BOUNDARY_COST_H_

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type

#include <algorithm>
#include <functional>
#include <memory>
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
#include "onboard/planner/optimization/problem/mixed_fourth_order_bicycle.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {
namespace planner {

// A path boundary represented by a sequence of path points, and the
// associated boundary distances to those path points.
template <typename PROB>
class MfobPathBoundaryCost : public Cost<PROB> {
 public:
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
  MfobPathBoundaryCost(
      int horizon, VehicleGeometryParamsProto vehicle_geometry_params,
      const std::vector<Vec2d>& path_points,
      const CenterLineQueryHelper<PROB>* center_line_helper,
      std::vector<double> l_offsets,
      std::vector<std::vector<double>> path_boundary_dists,
      std::vector<std::vector<double>> dists_to_clamp_buffers, bool left,
      bool using_hessian_approximate, std::vector<double> dist_to_rac,
      int rac_index, std::vector<std::vector<double>> ref_gains,
      std::vector<std::string> sub_names, bool use_qtfm,
      std::vector<double> buffers_min = {-1.0},
      const std::vector<double>& rear_buffers_max = {1.0},
      const std::vector<double>& front_buffers_max = {0.6},
      const std::vector<double>& clamped_buffer_offset = {0.0},
      std::vector<double> cascade_gains = {1.0},
      std::vector<double> rear_gain = {1.0},
      std::vector<double> front_gain = {1.0},
      std::string name = absl::StrCat(PROB::kProblemPrefix, "PathBoundaryCost"),
      double scale = 1.0, CostType cost_type = Cost<PROB>::CostType::MUST_HAVE)
      : Cost<PROB>(std::move(name), scale * kNormalizedScale, cost_type),
        horizon_(horizon),
        vehicle_geometry_params_(std::move(vehicle_geometry_params)),
        path_points_(path_points),
        l_offsets_(std::move(l_offsets)),
        path_boundary_dists_(std::move(path_boundary_dists)),
        path_boundary_size_(l_offsets_.size()),
        left_(left),
        using_hessian_approximate_(using_hessian_approximate),
        dist_to_rac_(std::move(dist_to_rac)),
        rac_index_(rac_index),
        circle_num_(dist_to_rac_.size()),
        ref_gains_(std::move(ref_gains)),
        cascade_gains_(std::move(cascade_gains)),
        sub_names_(std::move(sub_names)),
        cascade_num_(cascade_gains_.size()),
        center_line_helper_(center_line_helper),
        rear_gain_(std::move(rear_gain)),
        front_gain_(std::move(front_gain)) {
    QCHECK_GT(horizon_, 0);
    QCHECK_GT(path_points_.size(), 1);
    QCHECK_EQ(cascade_num_, sub_names_.size());
    QCHECK_EQ(cascade_num_, cascade_gains_.size());
    QCHECK_EQ(cascade_num_, rear_gain_.size());
    QCHECK_EQ(cascade_num_, front_gain_.size());
    QCHECK_EQ(cascade_num_, ref_gains_.size());
    QCHECK_EQ(cascade_num_, buffers_min.size());
    QCHECK_EQ(cascade_num_, rear_buffers_max.size());
    QCHECK_EQ(cascade_num_, front_buffers_max.size());
    QCHECK_EQ(cascade_num_, clamped_buffer_offset.size());
    QCHECK_GE(path_points_.size(), l_offsets_.size());
    QCHECK_GE(path_points_.size(), path_boundary_dists_.front().size());
    path_lengths_sqr_inv_.reserve(path_points_.size() - 1);
    for (int i = 0; i + 1 < path_points_.size(); ++i) {
      path_lengths_sqr_inv_.push_back(
          1.0 / (path_points_[i + 1] - path_points_[i]).squaredNorm());
    }
    if (center_line_helper_ != nullptr) {
      const auto& circle_model = center_line_helper_->av_circle_model();
      QCHECK_EQ(circle_model.size(), dist_to_rac_.size());
      QCHECK_EQ(rac_index_, center_line_helper_->rac_index());
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
    // Compute clamped buffers.
    const double half_width = vehicle_geometry_params_.width() * 0.5;
    clamped_path_boundary_buffers_.resize(circle_num_);
    for (int i = 0; i < circle_num_; ++i) {
      const auto& front_rear_buffer_max =
          (i == rac_index_ ? rear_buffers_max : front_buffers_max);
      clamped_path_boundary_buffers_[i].resize(cascade_num_);
      for (int j = 0; j < cascade_num_; ++j) {
        clamped_path_boundary_buffers_[i][j].reserve(path_boundary_size_);
        for (int m = 0; m < path_boundary_size_; ++m) {
          double center_to_boundary_dist = path_boundary_dists_[j][m];
          double clamp_buffer = front_rear_buffer_max[j];
          if (left_) {
            center_to_boundary_dist -= l_offsets_[m];
            clamp_buffer =
                center_to_boundary_dist - dists_to_clamp_buffers[j][m];
          } else {
            center_to_boundary_dist += l_offsets_[m];
            clamp_buffer =
                center_to_boundary_dist + dists_to_clamp_buffers[j][m];
          }
          clamped_path_boundary_buffers_[i][j].push_back(std::clamp(
              center_to_boundary_dist - half_width + clamped_buffer_offset[j],
              buffers_min[j],
              std::min(clamp_buffer, front_rear_buffer_max[j])));
        }
      }
    }

    // Reize to [horizon_num].
    av_tangents_.resize(horizon_);
    // Resize to [circle_num][cascade_num][horizon_num].
    circle_dists_to_path_boundary_.resize(circle_num_);
    gains_.resize(circle_num_);
    dynamic_buffers_.resize(circle_num_);
    for (int i = 0; i < circle_num_; ++i) {
      circle_dists_to_path_boundary_[i].resize(cascade_num_);
      gains_[i].resize(cascade_num_);
      dynamic_buffers_[i].resize(cascade_num_);
      for (int j = 0; j < cascade_num_; ++j) {
        circle_dists_to_path_boundary_[i][j].resize(horizon_);
        gains_[i][j].resize(horizon_);
        dynamic_buffers_[i][j].resize(horizon_);
      }
    }
    // Resize to [circle_num][horizon_num].
    normals_.resize(circle_num_);
    index_pairs_.resize(circle_num_);
    alphas_.resize(circle_num_);
    for (int i = 0; i < circle_num_; ++i) {
      normals_[i].resize(horizon_);
      index_pairs_[i].resize(horizon_);
      alphas_[i].resize(horizon_);
    }
    // Resize to [circle_num].
    effective_index_.resize(circle_num_);
  }

  DividedG SumGForAllSteps(const StatesType& /*xs*/, const ControlsType& /*us*/,
                           int horizon) const override {
    QCHECK_EQ(horizon, horizon_);
    DividedG res(sub_names_.size());
    for (int i = 0; i < circle_num_; ++i) {
      const auto& rear_front_gain =
          (i == rac_index_ ? rear_gain_ : front_gain_);
      for (int k = 0; k < horizon_; ++k) {
        if (k >= effective_index_[i]) break;
        for (int j = 0; j < cascade_num_; ++j) {
          const double d = circle_dists_to_path_boundary_[i][j][k];
          if (d < dynamic_buffers_[i][j][k]) {
            res.AddSubG(j, 0.5 * Cost<PROB>::scale() * gains_[i][j][k] *
                               rear_front_gain[j] *
                               Sqr(dynamic_buffers_[i][j][k] - d));
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
    for (int i = 0; i < circle_num_; ++i) {
      if (k >= effective_index_[i]) continue;
      const auto& rear_front_gain =
          (i == rac_index_ ? rear_gain_ : front_gain_);
      for (int j = 0; j < cascade_num_; ++j) {
        const double d = circle_dists_to_path_boundary_[i][j][k];
        if (d < dynamic_buffers_[i][j][k]) {
          res.AddSubG(j, 0.5 * Cost<PROB>::scale() * gains_[i][j][k] *
                             rear_front_gain[j] *
                             Sqr(dynamic_buffers_[i][j][k] - d));
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
    for (int i = 0; i < circle_num_; ++i) {
      if (k >= effective_index_[i]) continue;
      const auto& rear_front_gain =
          (i == rac_index_ ? rear_gain_ : front_gain_);
      for (int j = 0; j < cascade_num_; ++j) {
        const double d = circle_dists_to_path_boundary_[i][j][k];
        if (d < dynamic_buffers_[i][j][k]) {
          g += 0.5 * Cost<PROB>::scale() * gains_[i][j][k] *
               rear_front_gain[j] * Sqr(dynamic_buffers_[i][j][k] - d);
        }
      }
    }
    return g;
  }

  // Gradients with superposition.
  void AddDGDx(int k, const StateType& /*x*/, const ControlType& /*u*/,
               DGDxType* dgdx) const override {
    for (int i = 0; i < circle_num_; ++i) {
      const auto& rear_front_gain =
          (i == rac_index_ ? rear_gain_ : front_gain_);
      if (k < effective_index_[i]) {
        for (int j = 0; j < cascade_num_; ++j) {
          AddCircleDGDx(dgdx, circle_dists_to_path_boundary_[i][j][k],
                        dist_to_rac_[i], av_tangents_[k],
                        dynamic_buffers_[i][j][k], path_boundary_dists_[j],
                        clamped_path_boundary_buffers_[i][j],
                        index_pairs_[i][k], alphas_[i][k], normals_[i][k],
                        gains_[i][j][k], rear_front_gain[j]);
        }
      }
    }
  }

  void AddDGDu(int /*k*/, const StateType& /*x*/, const ControlType& /*u*/,
               DGDuType* /*dgdu*/) const override {}

  // Hessians with superposition.
  void AddDDGDxDx(int k, const StateType& /*x*/, const ControlType& /*u*/,
                  DDGDxDxType* ddgdxdx) const override {
    for (int i = 0; i < circle_num_; ++i) {
      const auto& rear_front_gain =
          (i == rac_index_ ? rear_gain_ : front_gain_);
      if (k < effective_index_[i]) {
        for (int j = 0; j < cascade_num_; ++j) {
          AddCircleDDGDxDx(ddgdxdx, circle_dists_to_path_boundary_[i][j][k],
                           dist_to_rac_[i], av_tangents_[k],
                           dynamic_buffers_[i][j][k], path_boundary_dists_[j],
                           clamped_path_boundary_buffers_[i][j],
                           index_pairs_[i][k], alphas_[i][k], normals_[i][k],
                           gains_[i][j][k], rear_front_gain[j]);
        }
      }
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
      VLOG(4) << "Update rac for left boundary.";
    } else {
      VLOG(4) << "Update rac for right boundary.";
    }

    const double half_width = vehicle_geometry_params_.width() * 0.5;
    const double left = left_;
    const auto get_distance_to_path_boundary =
        [&half_width, &left](double boundary_dist,
                             double state_dist) -> double {
      return left ? boundary_dist - state_dist - half_width
                  : boundary_dist + state_dist - half_width;
    };

    if (center_line_helper_ == nullptr) {
      for (int k = 0; k < horizon; ++k) {
        av_tangents_[k] = Vec2d::FastUnitFromAngleN12(PROB::theta(xs, k));
      }
      for (int i = 0; i < circle_num_; ++i) {
        const double dist_to_rac = dist_to_rac_[i];
        const auto get_state_pos_at_step =
            [&av_tangents = std::as_const(av_tangents_), &dist_to_rac](
                const StatesType& xs, int k) {
              return PROB::pos(xs, k) + dist_to_rac * av_tangents[k];
            };
        Update(get_state_pos_at_step, get_distance_to_path_boundary,
               clamped_path_boundary_buffers_[i], xs, &normals_[i],
               &dynamic_buffers_[i], &circle_dists_to_path_boundary_[i],
               &gains_[i], &index_pairs_[i], &alphas_[i], &effective_index_[i]);
      }
    } else {
      UpdateWithCenterLineHelper(clamped_path_boundary_buffers_, xs);
    }
  }

 private:
  void UpdateWithCenterLineHelper(
      const std::vector<std::vector<std::vector<double>>>&
          clamped_path_boundary_buffers,
      const StatesType& /*xs*/) {
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
        for (int j = 0; j < cascade_num_; ++j) {
          if (alpha < 0.0) {
            dynamic_buffers_[i][j][k] =
                clamped_path_boundary_buffers[i][j][index_pair.first];
            circle_dists_to_path_boundary_[i][j][k] =
                path_boundary_dists_[j][index_pair.first] - half_width +
                sign * sl.l;
            gains_[i][j][k] =
                ref_gains_[j][index_pair.first] * cascade_gains_[j];
          } else if (alpha > 1.0) {
            dynamic_buffers_[i][j][k] =
                clamped_path_boundary_buffers[i][j][index_pair.second];
            circle_dists_to_path_boundary_[i][j][k] =
                path_boundary_dists_[j][index_pair.second] - half_width +
                sign * sl.l;
            gains_[i][j][k] =
                ref_gains_[j][index_pair.second] * cascade_gains_[j];
          } else {
            dynamic_buffers_[i][j][k] = Lerp(
                clamped_path_boundary_buffers[i][j][index_pair.first],
                clamped_path_boundary_buffers[i][j][index_pair.second], alpha);
            const double boundary_dist =
                Lerp(path_boundary_dists_[j][index_pair.first],
                     path_boundary_dists_[j][index_pair.second], alpha);
            circle_dists_to_path_boundary_[i][j][k] =
                boundary_dist - half_width + sign * sl.l;
            gains_[i][j][k] =
                ref_gains_[j][index_pair.first] * cascade_gains_[j];
          }
        }
        if (UNLIKELY(VLOG_IS_ON(4))) {
          VLOG(4) << "-----------------circle index: " << i
                  << " -----------------";
          VLOG(4) << "Dists [" << k << "]: " << sl.l;
          VLOG(4) << "Normal [" << k << "]: " << normals_[i][k].x() << " "
                  << normals_[i][k].y();
          VLOG(4) << "Index pair [" << k << "]: " << index_pairs_[i][k].first
                  << " " << index_pairs_[i][k].second;
          VLOG(4) << "Alpha [" << k << "]: " << alphas_[i][k];
          for (int j = 0; j < cascade_num_; ++j) {
            VLOG(4) << "To boundary dists [" << j << "][" << k
                    << "]: " << circle_dists_to_path_boundary_[i][j][k];
            VLOG(4) << "Dynamic buffer [" << j << "][" << k
                    << "]: " << dynamic_buffers_[i][j][k];
          }
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

  void Update(
      const std::function<Vec2d(const StatesType&, int)>& get_pos_at_step,
      const std::function<double(double, double)>&
          get_distance_to_path_boundary,
      const std::vector<std::vector<double>>& clamped_path_boundary_buffers,
      const StatesType& xs, std::vector<Vec2d>* normals,
      std::vector<std::vector<double>>* dynamic_buffer,
      std::vector<std::vector<double>>* to_path_boundary_dists,
      std::vector<std::vector<double>>* gains,
      std::vector<std::pair<int, int>>* index_pairs,
      std::vector<double>* alphas, int* effective_index) const {
    *effective_index = horizon_;
    for (int k = 0; k < horizon_; ++k) {
      const Vec2d xy = get_pos_at_step(xs, k);
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
      for (int i = 0; i < cascade_num_; ++i) {
        if (alpha < 0.0) {
          (*dynamic_buffer)[i][k] =
              clamped_path_boundary_buffers[i][index_pair.first];
          (*to_path_boundary_dists)[i][k] = get_distance_to_path_boundary(
              path_boundary_dists_[i][index_pair.first], sl.l);
          (*gains)[i][k] = ref_gains_[i][index_pair.first] * cascade_gains_[i];

        } else if (alpha > 1.0) {
          (*dynamic_buffer)[i][k] =
              clamped_path_boundary_buffers[i][index_pair.second];
          (*to_path_boundary_dists)[i][k] = get_distance_to_path_boundary(
              path_boundary_dists_[i][index_pair.second], sl.l);
          (*gains)[i][k] = ref_gains_[i][index_pair.second] * cascade_gains_[i];
        } else {
          (*dynamic_buffer)[i][k] =
              Lerp(clamped_path_boundary_buffers[i][index_pair.first],
                   clamped_path_boundary_buffers[i][index_pair.second], alpha);
          const double boundary_dist =
              Lerp(path_boundary_dists_[i][index_pair.first],
                   path_boundary_dists_[i][index_pair.second], alpha);
          (*to_path_boundary_dists)[i][k] =
              get_distance_to_path_boundary(boundary_dist, sl.l);
          (*gains)[i][k] = ref_gains_[i][index_pair.first] * cascade_gains_[i];
        }
      }
      if (UNLIKELY(VLOG_IS_ON(4))) {
        VLOG(4) << "Dists [" << k << "]: " << sl.l;
        VLOG(4) << "Normal [" << k << "]: " << normals->at(k).x() << " "
                << normals->at(k).y();
        VLOG(4) << "Index pair [" << k << "]: " << index_pairs->at(k).first
                << " " << index_pairs->at(k).second;
        VLOG(4) << "Alpha [" << k << "]: " << alphas->at(k);
        for (int i = 0; i < cascade_num_; ++i) {
          VLOG(4) << "To boundary dists [" << i << "][" << k
                  << "]: " << (*to_path_boundary_dists)[i][k];
          VLOG(4) << "Dynamic buffer [" << i << "][" << k
                  << "]: " << (*dynamic_buffer)[i][k];
        }
      }
    }
  }

  void AddCircleDGDx(DGDxType* dgdx, double d_corner, double dist_to_rac,
                     const Vec2d& tangent, double corner_dynamic_buffer,
                     const std::vector<double>& path_boundary_dists,
                     const std::vector<double>& clamped_path_boundary_buffers,
                     const std::pair<int, int>& corner_index_pair,
                     double corner_alpha, const Vec2d& corner_normal,
                     double gain, double point_gain) const {
    if (d_corner < corner_dynamic_buffer) {
      const double d_penetration = d_corner - corner_dynamic_buffer;
      const Vec2d segment = path_points_[corner_index_pair.second] -
                            path_points_[corner_index_pair.first];
      const double d0 = path_boundary_dists[corner_index_pair.first];
      const double d1 = path_boundary_dists[corner_index_pair.second];
      double d_buffer = 0.0;
      if (corner_alpha >= 0.0 && corner_alpha <= 1.0) {
        d_buffer = clamped_path_boundary_buffers[corner_index_pair.second] -
                   clamped_path_boundary_buffers[corner_index_pair.first];
      }
      const Vec2d d_delta(-dist_to_rac * tangent.y(),
                          dist_to_rac * tangent.x());
      const double sign = left_ ? -1.0 : 1.0;
      const double final_gain = Cost<PROB>::scale() * gain * point_gain;
      if (corner_alpha < 0.0 || corner_alpha > 1.0) {
        (*dgdx).template segment<2>(0) +=
            final_gain * d_penetration * sign * corner_normal.transpose();
        (*dgdx)[2] +=
            final_gain * d_penetration * sign * corner_normal.dot(d_delta);
      } else {
        const Vec2d vec = (d1 - d0 - d_buffer) * segment *
                              path_lengths_sqr_inv_[corner_index_pair.first] +
                          sign * corner_normal;
        (*dgdx).template segment<2>(0) +=
            final_gain * d_penetration *
            ((d1 - d0 - d_buffer) * segment.transpose() *
                 path_lengths_sqr_inv_[corner_index_pair.first] +
             sign * corner_normal.transpose());
        (*dgdx)[2] += final_gain * d_penetration * vec.dot(d_delta);
      }
    }
  }

  void AddCircleDDGDxDx(
      DDGDxDxType* ddgdxdx, double d_corner, double dist_to_rac,
      const Vec2d& tangent, double corner_dynamic_buffer,
      const std::vector<double>& path_boundary_dists,
      const std::vector<double>& clamped_path_boundary_buffers,
      const std::pair<int, int>& corner_index_pair, double corner_alpha,
      const Vec2d& corner_normal, double gain, double point_gain) const {
    if (d_corner < corner_dynamic_buffer) {
      const double d_penetration = d_corner - corner_dynamic_buffer;
      const Vec2d segment = path_points_[corner_index_pair.second] -
                            path_points_[corner_index_pair.first];
      const double d0 = path_boundary_dists[corner_index_pair.first];
      const double d1 = path_boundary_dists[corner_index_pair.second];
      double d_buffer = 0.0;
      if (corner_alpha >= 0.0 && corner_alpha <= 1.0) {
        d_buffer = clamped_path_boundary_buffers[corner_index_pair.second] -
                   clamped_path_boundary_buffers[corner_index_pair.first];
      }
      const Vec2d d_delta(-dist_to_rac * tangent.y(),
                          dist_to_rac * tangent.x());
      const double sign = left_ ? -1.0 : 1.0;
      const double final_gain = Cost<PROB>::scale() * gain * point_gain;
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
          const Vec2d vec = (d1 - d0 - d_buffer) * segment *
                                path_lengths_sqr_inv_[corner_index_pair.first] +
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
        const Vec2d dd_delta = -dist_to_rac * tangent;
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
              final_gain * (Sqr(corner_normal.dot(d_delta)) +
                            d_penetration * sign * corner_normal.dot(dd_delta));
        } else {
          const Vec2d vec = (d1 - d0 - d_buffer) * segment *
                                path_lengths_sqr_inv_[corner_index_pair.first] +
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

  int horizon_ = 0;
  VehicleGeometryParamsProto vehicle_geometry_params_;
  std::vector<Vec2d> path_points_;            // size = n.
  std::vector<double> path_lengths_sqr_inv_;  // size = n-1.
  // Inner vectors are distances, size = n, outer vector is multi layer path
  // boundaries.
  std::vector<double> l_offsets_;
  std::vector<std::vector<double>> path_boundary_dists_;
  int path_boundary_size_;
  // Whether the boundary is on the left of the path.
  bool left_ = false;
  bool using_hessian_approximate_;
  // Vehicle model info.
  std::vector<double> dist_to_rac_;
  int rac_index_ = -1;
  int circle_num_;

  // Gains for each station of every path boundary.
  std::vector<std::vector<double>> ref_gains_;

  std::vector<double> cascade_gains_;
  std::vector<std::string> sub_names_;
  int cascade_num_;

  std::unique_ptr<FrenetFrame> path_;
  const CenterLineQueryHelper<PROB>* center_line_helper_;

  std::vector<double> rear_gain_;
  std::vector<double> front_gain_;

  // States of all control points.
  // Get av_tangents_ with [horizon_index].
  std::vector<Vec2d> av_tangents_;
  // Get the following with [circle_index][cascade_index][path_boundary_index].
  std::vector<std::vector<std::vector<double>>> clamped_path_boundary_buffers_;
  // Get the following with [circle_index][cascade_index][horizon_index].
  std::vector<std::vector<std::vector<double>>> circle_dists_to_path_boundary_;
  std::vector<std::vector<std::vector<double>>> gains_;
  std::vector<std::vector<std::vector<double>>> dynamic_buffers_;
  // Get the following with [circle_index][horizon_index].
  std::vector<std::vector<Vec2d>> normals_;
  std::vector<std::vector<std::pair<int, int>>> index_pairs_;
  std::vector<std::vector<double>> alphas_;
  // Get effective_index_ with [circle_index].
  std::vector<int> effective_index_;
};

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_MFOB_PATH_BOUNDARY_COST_H_
