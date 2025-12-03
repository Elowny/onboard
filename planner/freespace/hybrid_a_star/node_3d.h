#ifndef ONBOARD_PLANNER_FREESPACE_HYBRID_A_STAR_NODE_3D_H
#define ONBOARD_PLANNER_FREESPACE_HYBRID_A_STAR_NODE_3D_H

#include <limits>
#include <memory>
#include <string>

#include "onboard/math/fast_math.h"
#include "onboard/math/geometry/aabox2d.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {
namespace planner {

class Node3d {
 public:
  Node3d(double x, double y, double theta) : x_(x), y_(y), theta_(theta) {
    double theta_cos_sin[2];
    fast_math::CosAndSin<7>(theta_, theta_cos_sin);
    sin_theta_ = theta_cos_sin[1];
    cos_theta_ = theta_cos_sin[0];
  }
  virtual ~Node3d() = default;

  double x() const { return x_; }
  double y() const { return y_; }
  double theta() const { return theta_; }
  double sin_theta() const { return sin_theta_; }
  double cos_theta() const { return cos_theta_; }
  int x_grid() const { return x_grid_; }
  int y_grid() const { return y_grid_; }
  int theta_grid() const { return theta_grid_; }
  double steer() const { return steer_; }
  double s() const { return s_; }
  unsigned int id() const { return id_; }
  double dis_to_obs() const { return dis_to_obs_; }
  double traj_cost() const { return traj_cost_; }
  double heuristic_cost() const { return heuristic_cost_; }
  double total_cost() const { return total_cost_; }
  bool forward() const { return forward_; }
  std::shared_ptr<Node3d> prev_node() const { return prev_node_; }

  Box2d GetVehicleBoundingBoxWithBuffer(
      const VehicleGeometryParamsProto& veh_geo_params, double lateral_buffer,
      double longitudinal_buffer) const;
  std::string ToString() const;

  void set_steer(double steer) { steer_ = steer; }
  void set_s(double s) { s_ = s; }
  void set_forward(bool dir) { forward_ = dir; }
  void set_prev_node(const std::shared_ptr<Node3d>& prev_node) {
    prev_node_ = prev_node;
  }
  void set_dis_to_obs(double dis_to_obs) { dis_to_obs_ = dis_to_obs; }
  void set_traj_cost(double traj_cost) { traj_cost_ = traj_cost; }
  void set_heuristic_cost(double heuristic_cost) {
    heuristic_cost_ = heuristic_cost;
  }
  void set_total_cost(double total_cost) { total_cost_ = total_cost; }

  void SetGridsAndId(const AABox2d& region, double xy_resolution,
                     double theta_resolution);

 private:
  double x_;
  double y_;
  double theta_;  // Vehicle heading.
  double sin_theta_;
  double cos_theta_;
  int x_grid_ = 0;
  int y_grid_ = 0;
  int theta_grid_ = 0;
  // Left positive, right negative. This value has been normalized by kappa, for
  // example if steer = 0.2, then the path curvature is also 0.2.
  double steer_ = 0.0;
  double s_ = 0.0;

  unsigned int id_ = 0;

  double dis_to_obs_ = std::numeric_limits<double>::infinity();
  double traj_cost_ = 0.0;
  double heuristic_cost_ = 0.0;
  double total_cost_ = 0.0;

  bool forward_ = true;

  std::shared_ptr<Node3d> prev_node_ = nullptr;
};
}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_FREESPACE_HYBRID_A_STAR_NODE_3D_H
