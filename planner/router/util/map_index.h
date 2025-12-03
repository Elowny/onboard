#ifndef ONBOARD_PLANNER_ROUTER_UTIL_MAP_INDEX_H_
#define ONBOARD_PLANNER_ROUTER_UTIL_MAP_INDEX_H_

// IWYU pragma: no_include <boost/geometry/index/parameters.hpp>
// IWYU pragma: no_include <boost/iterator/iterator_facade.hpp>

#include <functional>
#include <utility>
#include <vector>

#include "boost/geometry/core/cs.hpp"
#include "boost/geometry/geometries/box.hpp"
#include "boost/geometry/geometries/point_xy.hpp"
#include "boost/geometry/index/rtree.hpp"

#include "onboard/maps/proto/semantic_map.pb.h"
#include "onboard/maps/semantic_map_manager.h"
#include "onboard/maps/spatial_search_util.h"
#include "onboard/maps/v2/semantic_map_manager.h"
#include "onboard/math/vec.h"

namespace qcraft::planner::route {

template <typename T>
struct MBR {
  T x1, y1, x2, y2;
};

class MapIndex {
  typedef boost::geometry::model::d2::point_xy<double,
                                               boost::geometry::cs::cartesian>
      DPoint;
  typedef boost::geometry::model::box<DPoint> DBox;

  using LaneProtoValue = std::pair<DBox, const mapping::LaneProto*>;
  using Rtree =
      boost::geometry::index::rtree<LaneProtoValue,
                                    boost::geometry::index::quadratic<16>>;

 public:
  MapIndex() = default;

 public:
  void InitIndex(const mapping::v2::SemanticMapManager& semantic_map_manager);
  void InitIndex(const mapping::SemanticMapManager& semantic_map_manager);

  std::vector<mapping::PointToLane> PointToNearLanes(
      Vec2d global, double dist,
      const std::function<bool(const mapping::LaneProto*)>& predicate_fn,
      const std::function<bool(const Vec2d& p1, const Vec2d& p2)>&
          segment2d_fn = [](const auto&, const auto&) { return true; }) const;

 private:
  std::vector<LaneProtoValue> QueryLanes(const DBox& box) const;

 private:
  Rtree tree_;
};
}  // namespace qcraft::planner::route

#endif  // ONBOARD_PLANNER_ROUTER_UTIL_MAP_INDEX_H_
