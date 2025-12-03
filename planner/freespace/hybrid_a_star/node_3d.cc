#include "onboard/planner/freespace/hybrid_a_star/node_3d.h"

#include <cmath>

#include "absl/strings/str_format.h"

#include "onboard/math/util.h"
#include "onboard/math/vec.h"
namespace qcraft {
namespace planner {

void Node3d::SetGridsAndId(const AABox2d& region, double xy_resolution,
                           double theta_resolution) {
  x_grid_ = FloorToInt((x_ - region.min_x()) / xy_resolution);
  y_grid_ = FloorToInt((y_ - region.min_y()) / xy_resolution);
  theta_grid_ = FloorToInt((NormalizeAngle(theta_) + M_PI) / theta_resolution);
  id_ = static_cast<unsigned int>(x_grid_) +
        (static_cast<unsigned int>(y_grid_) << 10) +
        (static_cast<unsigned int>(theta_grid_) << 20);
}

Box2d Node3d::GetVehicleBoundingBoxWithBuffer(
    const VehicleGeometryParamsProto& veh_geo_params, double lateral_buffer,
    double longitudinal_buffer) const {
  const double offset = 0.5 * (veh_geo_params.front_edge_to_center() -
                               veh_geo_params.back_edge_to_center());
  const double center_x = x_ + cos_theta_ * offset;
  const double center_y = y_ + sin_theta_ * offset;
  return Box2d(0.5 * veh_geo_params.length() + longitudinal_buffer,
               0.5 * veh_geo_params.width() + lateral_buffer,
               Vec2d(center_x, center_y), theta_,
               Vec2d(cos_theta_, sin_theta_));
}

std::string Node3d::ToString() const {
  return absl::StrFormat("x: %f, y: %f, theta: %f, dir: %d", x_, y_, theta_,
                         forward_);
}

}  // namespace planner
}  // namespace qcraft
