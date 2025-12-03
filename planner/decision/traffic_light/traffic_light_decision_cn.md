# Traffic Light Decision

## 红绿灯的Proto定义

红绿灯的proto定义在：[onboard/maps/proto/semantic_map.proto](https://gitlab.qcraft.ai/root/qcraft/-/blob/618cf1cf6950b2dbddad01a1d6a62401fb46e695/onboard/maps/proto/semantic_map.proto#L135)

```protobuf
message LaneProto {
...
 repeated sfixed64 startpoint_associated_traffic_lights = 31;
 
// Only used at left waiting turn lane in intersection
  message MultiTrafficLightControlPoint {
    optional double lane_fraction = 1
        [default = 0];  // fraction position at this lane
    repeated sfixed64 left_tls =
        2;  // left traffic lights which can affect this point
    repeated sfixed64 straight_tls = 3;
    repeated sfixed64 right_tls = 4;
  }
  repeated MultiTrafficLightControlPoint multi_traffic_light_control_points =
      34;
...
}
```

红绿灯一般定义在路口中央的`LaneProto`上，其中有两种方式：

* 使用`startpoint_associated_traffic_lights`定义受单个红绿灯颜色控制的lane。
* 使用`multi_traffic_light_control_points`定义受多个红绿灯颜色共同控制的lane，其中`lane_fraction`表示受红绿灯控制的lane的具体位置。

其实以上两种方式可以由后者统一（TODO）。

## 红绿灯感知结果投票方式

![image-20210602114956016](./img/image-20210602114956016.png)

该投票过程在：[onboard/planner/route_tl_info.cc](https://gitlab.qcraft.ai/root/qcraft/-/blob/618cf1cf6950b2dbddad01a1d6a62401fb46e695/onboard/planner/route_tl_info.cc#L56)

### 红绿灯感知结果

红绿灯的感知结果定义在：[onboard/proto/perception.proto](https://gitlab.qcraft.ai/root/qcraft/-/blob/618cf1cf6950b2dbddad01a1d6a62401fb46e695/onboard/proto/perception.proto#L262)

```protobuf
enum TrafficLightColor {
  TL_UNKNOWN = 0;
  TL_RED = 1;
  TL_YELLOW = 2;
  TL_GREEN = 3;
}
...
message TrafficLightStateProto {
  ...
  optional TrafficLightColor color = 3;
  ...
  optional bool flashing = 7 [default = false];
  ...
}
```

决策这边常用的属性为红绿灯的`color`和`flashing`这两个属性，分表表示红绿灯的颜色和是否闪烁。

### 对同一个灯箱感知结果投票

红绿灯检测会使用多个相机独立检测，因此对于同一个红绿灯灯箱，多个相机的独立检测结果可能会不同，因此首先对同一个灯箱的检测结果进行投票，决定出该灯箱最终的颜色结果，参与该投票过程的有多个相机的感知结果和planner模块内部维护的历史感知结果，在函数`getTrafficLightDetectResult`中。

```c++
	std::map<std::pair<TrafficLightColor, bool>, std::pair<double, bool>> 
        camera_votes;  // {{color, flashing},{blief, get_from_history}}   

	for (const auto &state_i : camera_indices) {
      VLOG(2) << "  Traffic light " << tl_id << " state: "
              << ((tl_states.states(state_i).flashing() ? "flashing " : "") +
                  TrafficLightColor_Name(tl_states.states(state_i).color()));
      TrafficLightColor color = tl_states.states(state_i).color();
      bool flashing = tl_states.states(state_i).flashing();
      bool is_from_history = false;
      if (color == TL_UNKNOWN) {
        const auto history_result = getHistoryTrafficLightResult(tl_id);
        color = history_result.first;
        flashing = history_result.second;
        is_from_history = true;
      }
      camera_votes[{color, flashing}].first += 1.0 / camera_indices_size;
      if (is_from_history)
        camera_votes[{color, flashing}].second = is_from_history;
    }
    return camera_votes
```

会维护一个`camera_votes`用于统计每个相机的检测结果，每有一个相机检测到一个结果，该结果的置信度将上升，若该相机的感知结果为`TL_UNKNOWN`，则将从该红绿灯的历史检测结果中获取作为该相机的感知结果。

### 多灯箱决定单方向控制红绿灯投票方式

例如直行车道只受直行信号灯控制，但是可以表示直行信号灯的灯箱有多个，此时需要对多个灯箱检测结果进行投票，决定最终直行信号灯的颜色，首先获取每个灯箱的感知投票结果，然后利用每个灯箱的感知投票结果重新统计多个灯箱的感知投票，选择投票结果最大的本次单信号灯的决策结果，其中可能会出现平票的情况，因此设定以下规则：

1. 无论`TL_UNKNOWN`的票数高，都会以非`TL_UNKNOWN`的感知结果为准。
2. 平票时按照感知结果优先级选择优先级较高的结果作为决策结果，优先级顺序为：红灯>黄灯>绿闪>绿灯。
3. 被标记为从历史结果中得到的感知结果的优先级<非历史感知结果。

### 红绿灯状态转移滤波

状态转移逻辑代码位于：[onboard/planner/route_tl_info.cc: compare_with_voting_history()](https://gitlab.qcraft.ai/root/qcraft/-/blob/618cf1cf6950b2dbddad01a1d6a62401fb46e695/onboard/planner/route_tl_info.cc#L373)

在返回红绿灯投票最终结果前，将会校验投票结果的状态转移合理性，红绿灯投票结果会维护一个滑窗，包含过去1s内的红绿灯状态，若是当前红绿灯与滑窗内最近的红绿灯结果满足转移逻辑，则将当前红绿灯状态更新进该滑窗，若不满足状态转移结果，则不会将当前红绿灯状态加入该滑窗，因此若持续1s不符合状态转移逻辑，该滑窗就会为空，将会强制将当前红绿灯状态加入滑窗。

状态转移逻辑为：红灯->绿灯->绿闪或黄灯->黄灯->红灯，以及它们的自转移（即自己转换到自己）

## 红绿灯决策逻辑

### 单方向信号灯控制逻辑

红绿灯决策逻辑代码位于：[onboard/planner/decision/traffic_light_decider.cc：MakeTrafficLightDecision()](https://gitlab.qcraft.ai/root/qcraft/-/blob/618cf1cf6950b2dbddad01a1d6a62401fb46e695/onboard/planner/decision/traffic_light_decider.cc#L17)

* 若车辆还未到路口停止线
  * 若此时为红灯或`TL_UNKNOWN`则停车。
  * 若此时为绿闪或黄灯，则会估计红绿灯变红前的剩余时间和车辆到达停止线的时间，同时会计算以-3.5的减速度刹停的距离
    * 若到达停止线时间 < 红绿灯变红的剩余时间，或刹停距离在停止线之后，则通过该停止线。
    * 如果上一时刻的决策为停在停止线之前，则本次决策也为停在停止线前。
  * 若此时为绿灯，则正常通过。
* 若车辆已经通过通过停止线且未超过30m距离
  * 若此时为绿灯，则通过路口。
  * 若此时不为绿灯，则与上一时刻的决策相同。

### 左待转区信号灯控制逻辑

![image-20210602165930729](./img/image-20210602165930729.png)

左待转区决策逻辑代码位于：[onboard/planner/decision/empty_road_decider.cc：MakeDecisionForTrafficLights()](https://gitlab.qcraft.ai/root/qcraft/-/blob/618cf1cf6950b2dbddad01a1d6a62401fb46e695/onboard/planner/decision/empty_road_decider.cc#L402)

* 若车辆在待转区第一个停止线前
  * 若左转灯不为绿灯 &&  （直行灯为绿闪或黄灯），则判断能否在其变红前进入待转区。
  * 若左转灯为绿闪，则判断能否在其变红前进入待转区。
  * 若左转和直行灯都不为绿灯，则停车。
* 若车辆超过待转区第一个停止线10m内
  * 如果左转灯为绿灯或黄灯，则记录车辆为**左转绿灯进入状态。**
  * 若左转灯不为绿灯 && （直行不为红灯或`TL_UNKNOWN`），则与上一时刻决策相同。
* 若车辆进入待转区，在第二个停止线前
  * 如果左转灯为绿灯或黄灯，则将其记录为**左转绿灯进入状态。**
  * 如果左转灯不为绿灯，也不是黄灯，也不是**左转绿灯进入状态**，则停在第二个停止线前。