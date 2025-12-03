#ifndef ONBOARD_PLANNER_COMMON_GEOMETRY_PATH_UTIL_H_
#define ONBOARD_PLANNER_COMMON_GEOMETRY_PATH_UTIL_H_

#include <optional>

#include "onboard/math/geometry/box2d.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/math/geometry/segment2d.h"
#include "onboard/math/vec.h"

namespace qcraft {
namespace planner {

/**
 * @brief Compute the first overlap distance with an obstacle segment for ego
 * box driving at a constant kappa.
 * @param pos Current rear-axle center of ego vehicle.
 * @param ego_box Current bounding box of ego vehicle.
 * @param forward Whether ego vehicle is driving forward.
 * @param kappa The current kappa of ego vehicle.
 * @param segment The obstacle segment.
 * @return First overlap distance or std::nullopt if there is no overlap.
 */
std::optional<double> ComputeFirstOverlapDistanceWithSegment(
    const Vec2d& pos, const Box2d& ego_box, bool forward, double kappa,
    const Segment2d& segment);

/**
 * @brief Compute the first overlap distance with an obstacle box for ego box
 * driving at a constant kappa.
 * @param pos Current rear-axle center of ego vehicle.
 * @param ego_box Current bounding box of ego vehicle.
 * @param forward Whether ego vehicle is driving forward.
 * @param kappa The current kappa of ego vehicle.
 * @param box The obstacle box.
 * @return First overlap distance or std::nullopt if there is no overlap.
 */
std::optional<double> ComputeFirstOverlapDistanceWithBox(const Vec2d& pos,
                                                         const Box2d& ego_box,
                                                         bool forward,
                                                         double kappa,
                                                         const Box2d& box);

/**
 * @brief Compute the first overlap distance with an obstacle polygon for ego
 * box driving at a constant kappa.
 * @param pos Current rear-axle center of ego vehicle.
 * @param ego_box Current bounding box of ego vehicle.
 * @param forward Whether ego vehicle is driving forward.
 * @param kappa The current kappa of ego vehicle.
 * @param polygon The obstacle polygon.
 * @return First overlap distance or std::nullopt if there is no overlap.
 */
std::optional<double> ComputeFirstOverlapDistanceWithPolygon(
    const Vec2d& pos, const Box2d& ego_box, bool forward, double kappa,
    const Polygon2d& polygon);

/**
 * @brief Compute the first overlap distance with an obstacle segment for ego
 * polygon driving at a constant kappa.
 * @param pos Current rear-axle center of ego vehicle.
 *  @param tangent Unit heading vector of ego vehicle.
 * @param ego_polygon Current polygon of ego vehicle.
 * @param forward Whether ego vehicle is driving forward.
 * @param kappa The current kappa of ego vehicle.
 * @param segment The obstacle segment.
 * @return First overlap distance or std::nullopt if there is no overlap.
 */
std::optional<double> ComputeFirstOverlapDistanceWithSegment(
    const Vec2d& pos, const Vec2d& tangent, const Polygon2d& ego_polygon,
    bool forward, double kappa, const Segment2d& segment);

/**
 * @brief Compute the first overlap distance with an obstacle box for ego
 * polygon driving at a constant kappa.
 * @param pos Current rear-axle center of ego vehicle.
 *  @param tangent Unit heading vector of ego vehicle.
 * @param ego_polygon Current polygon of ego vehicle.
 * @param forward Whether ego vehicle is driving forward.
 * @param kappa The current kappa of ego vehicle.
 * @param box The obstacle box.
 * @return First overlap distance or std::nullopt if there is no overlap.
 */
std::optional<double> ComputeFirstOverlapDistanceWithBox(
    const Vec2d& pos, const Vec2d& tangent, const Polygon2d& ego_polygon,
    bool forward, double kappa, const Box2d& box);

/**
 * @brief Compute the first overlap distance with an obstacle polygon for ego
 * polygon driving at a constant kappa.
 * @param pos Current rear-axle center of ego vehicle.
 * @param tangent Unit heading vector of ego vehicle.
 * @param ego_polygon Current polygon of ego vehicle.
 * @param forward Whether ego vehicle is driving forward.
 * @param kappa The current kappa of ego vehicle.
 * @param polygon The obstacle polygon.
 * @return First overlap distance or std::nullopt if there is no overlap.
 */
std::optional<double> ComputeFirstOverlapDistanceWithPolygon(
    const Vec2d& pos, const Vec2d& tangent, const Polygon2d& ego_polygon,
    bool forward, double kappa, const Polygon2d& polygon);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_COMMON_GEOMETRY_PATH_UTIL_H_
