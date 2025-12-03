#include "onboard/planner/router/thirdparty/tencent_action_format.h"

#include <string>
#include <unordered_map>
namespace qcraft::planner::route {
namespace tencent {
// main action
// 左转
// 右转
// 偏左转
// 偏右转
// 左后转
// 右后转
// 左转掉头
// 直行
// 靠左
// 靠右
// 进入环岛
// 空
// see
// !link[https://lbs.qq.com/service/webService/webServiceGuide/webServiceRoute]
std::unordered_map<std::string, NaviActionProto::NaviActionType>
    main_action_map = {
        {"直行", NaviActionProto_NaviActionType_STRAIGHT},
        {"靠左", NaviActionProto_NaviActionType_LEFT_SIDE},
        {"靠右", NaviActionProto_NaviActionType_RIGHT_SIDE},

        {"偏右转", NaviActionProto_NaviActionType_TURN_RIGHT},
        {"右转", NaviActionProto_NaviActionType_TURN_RIGHT},
        {"右后转", NaviActionProto_NaviActionType_TURN_RIGHT},

        {"偏左转", NaviActionProto_NaviActionType_TURN_LEFT},
        {"左转", NaviActionProto_NaviActionType_TURN_LEFT},
        {"左后转", NaviActionProto_NaviActionType_TURN_LEFT},

        {"左转调头", NaviActionProto_NaviActionType_U_TURN},
        {"左转掉头", NaviActionProto_NaviActionType_U_TURN},
        {"进入环岛", NaviActionProto_NaviActionType_ROUNDABOUT},
};
std::unordered_map<std::string, NaviActionProto::SubNaviActionType>
    sub_action_map = {
        {"进入主路", NaviActionProto_SubNaviActionType_ENTER_MAIN_ROAD},
        {"进入辅路", NaviActionProto_SubNaviActionType_ENTER_SIDE_ROAD},
        {"进高速", NaviActionProto_SubNaviActionType_ENTER_FREE_WAY},
        {"进入匝道", NaviActionProto_SubNaviActionType_ENTER_RAMP},
        {"进入隧道", NaviActionProto_SubNaviActionType_ENTER_TUNEL},
        {"驶出高速", NaviActionProto_SubNaviActionType_EXIT_FREE_WAY},
        {"驶出当前高速", NaviActionProto_SubNaviActionType_EXIT_FREE_WAY},
};
}  // namespace tencent

std::optional<NaviActionProto::NaviActionType> ConvertMainAction(
    const std::string& main_action) {
  if (tencent::main_action_map.count(main_action) == 0) {
    return std::nullopt;
  }
  return tencent::main_action_map[main_action];
}

std::optional<NaviActionProto::SubNaviActionType> ConvertSubAction(
    const std::string& sub_action) {
  if (tencent::sub_action_map.count(sub_action) == 0) {
    return std::nullopt;
  }
  return tencent::sub_action_map[sub_action];
}

}  // namespace qcraft::planner::route
