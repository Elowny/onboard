#ifndef ONBOARD_PLANNER_ROUTER_THIRDPARTY_TENCENT_ACTION_FORMAT_H_
#define ONBOARD_PLANNER_ROUTER_THIRDPARTY_TENCENT_ACTION_FORMAT_H_

#include <optional>
#include <string>

#include "onboard/proto/route_navigation.pb.h"

namespace qcraft::planner::route {

std::optional<NaviActionProto::NaviActionType> ConvertMainAction(
    const std::string& main_action);

std::optional<NaviActionProto::SubNaviActionType> ConvertSubAction(
    const std::string& sub_action);
}  // namespace qcraft::planner::route

#endif  // ONBOARD_PLANNER_ROUTER_THIRDPARTY_TENCENT_ACTION_FORMAT_H_
