#ifndef ONBOARD_PLANNER_ROUTER_ROUTE_TEST_UTIL_H_
#define ONBOARD_PLANNER_ROUTER_ROUTE_TEST_UTIL_H_

#include "onboard/maps/semantic_map_manager.h"
#include "onboard/maps/v2/semantic_map_manager.h"
#include "onboard/planner/composite_lane_path.h"
#include "onboard/planner/router/route_sections.h"
#include "onboard/planner/router/util/map_index.h"
#include "onboard/proto/positioning.pb.h"

namespace qcraft::planner {

struct TestRouteResult {
  const mapping::v2::SemanticMapManager* smm;
  CompositeLanePath route_lane_path;
  RouteSections route_sections;
  PoseProto pose;
};

// Refer to
// https://drive.google.com/drive/u/0/folders/1do80OvVwM2wuAk-FaQ-6RwpcHxZDzhu_
TestRouteResult CreateAStraightForwardRouteInUrbanDojo();

TestRouteResult CreateAStraightForwardRouteWithSolidInUrbanDojo();

// Refer to
// https://drive.google.com/drive/u/0/folders/1do80OvVwM2wuAk-FaQ-6RwpcHxZDzhu_
TestRouteResult CreateAUturnRouteInDojo();

// Refer to
// https://drive.google.com/drive/u/0/folders/1do80OvVwM2wuAk-FaQ-6RwpcHxZDzhu_
TestRouteResult CreateALeftTurnRouteInDojo();

TestRouteResult CreateALeftTurnWithDirectionInfoRouteInDojo();

TestRouteResult CreateARightTurnWithDirectionInfoRouteInDojo();

// Refer to
// https://drive.google.com/drive/u/0/folders/1do80OvVwM2wuAk-FaQ-6RwpcHxZDzhu_
TestRouteResult CreateAForkLaneRouteInDojo();

TestRouteResult CreateASingleLaneRouteInDojo();

TestRouteResult CreateAContinuousLaneChangeRouteInDojo();

TestRouteResult CreateAContinuousLaneChangeRouteWithSolidInDojo();

const mapping::v2::SemanticMapManager& LoadDojoMap();
const mapping::SemanticMapManager& LoadDojoMapV1();
const route::MapIndex* LoadDojoIndex();

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_ROUTER_ROUTE_TEST_UTIL_H_
