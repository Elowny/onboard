# 红绿灯路口决策

整个对于红绿灯决策的过程主要包括两大模块：红绿灯状态投票和红绿灯决策逻辑。

文档将从红绿灯的Proto定义、红绿灯状态投票和红绿灯决策逻辑三个部分进行介绍。



## 红绿灯的proto定义

红绿灯的proto定义包括地图的车道红绿灯信息proto和感知的红绿灯状态proto。前者用来获知路口的lane绑定的相应红绿灯信息，后者用于获知感知的红绿灯检测结果，两者共同作为红绿灯感知结果投票的输入。



### 车道红绿灯信息proto

车道红绿灯信息的proto定义在：[onboard/maps/proto/semantic_map.proto](https://gitlab.qcraft.ai/root/qcraft/-/blob/618cf1cf6950b2dbddad01a1d6a62401fb46e695/onboard/maps/proto/semantic_map.proto#L135)

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

* 使用`startpoint_associated_traffic_lights`定义受单个红绿灯状态控制的lane。
* 使用`multi_traffic_light_control_points`定义受多个红绿灯状态共同控制的lane，其中`lane_fraction`表示受红绿灯控制的lane的具体位置。左转待转区属于这种情况，由`left_tls`和`straight_tls`共同控制，受控制的lane的具体位置通常定义为`lane_fraction = 0.0`和`lane_fraction = 1.0`两处，分别表示第一根停止线和左转待转区的第二根停止线。

![image-20210602165930729](./img/image-20210602165930729.png)



### 感知的红绿灯状态proto

感知的红绿灯状态proto定义在：[onboard/proto/perception.proto](https://gitlab.qcraft.ai/root/qcraft/-/blob/618cf1cf6950b2dbddad01a1d6a62401fb46e695/onboard/proto/perception.proto#L211)

```c++
enum TrafficLightColor {
  TL_UNKNOWN = 0;
  TL_RED = 1;
  TL_YELLOW = 2;
  TL_GREEN = 3;
}
...
message TrafficLightStateProto {
  optional double timestamp = 1;
  optional int64 traffic_light_id = 2;
  optional TrafficLightColor color = 3;
...
  optional bool flashing = 7 [default = false];
...
}

message TrafficLightStatesProto {
  optional LiteHeader header = 1;
  repeated TrafficLightStateProto states = 2;
}
```

投票用到的红绿灯感知状态主要用到的信息包括：检测的时间戳`timestamp`、红绿灯id`traffic_light_id`、红绿灯颜色`color`和是否闪烁`flashing`。



## 红绿灯状态投票

对于某一条受红绿灯控制的车道，感知的红绿灯状态proto会给出多个结果，包括不同相机对同一个id的红绿灯灯箱的检测结果，和控制同一条车道的不同红绿灯灯箱的检测结果。在对路口的车道进行红绿灯决策之前，需要对感知给出的结果进行投票，最终得到该车道用于决策的红绿灯信息。在投票过程中会参考并维护相应的历史状态，对感知结果进行修正，以保证在感知不稳定的情况下，最终也可以得到最为可靠的红绿灯状态作为决策逻辑的输入。

**因此，红绿灯状态投票的目的是：在对路口的红绿灯进行决策之前，给出一个可用于决策的可信度较高的红绿灯状态结果**。

![image-20210602114956016](./img/image-20210602114956016.png)



### 红绿灯状态投票结果储存方式

红绿灯状态的投票结果储存在`class TlInfo `中：onboard/planner/decision/traffic_light/tl_info.h

```C++
enum class TlControlType {
  SINGLE_DIRECTION = 0,
  LEFT_WAITING_AREA = 1,
};

enum class TlState {
  TL_STATE_UNKNOWN = 0,
  TL_STATE_RED = 1,
  TL_STATE_YELLOW = 2,
  TL_STATE_GREEN_FLASHING = 3,
  TL_STATE_GREEN = 4,
  TL_STATE_YELLOW_FLASHING = 5,
};

enum class TlDirection {
  UNMARKED = 0,
  STRAIGHT = 1,
  LEFT = 2,
  RIGHT = 3,
  UTURN = 4,
};
...
struct SingleTlInfo {
  mapping::ElementId tl_id;
  TlState tl_state;
  double estimated_turn_red_time_left;
};

class TlInfo {
 public:
  TlInfo(mapping::ElementId lane_id,
         const std::vector<double>& control_point_route_s,
         const std::vector<double>& control_point_relative_s,
         bool can_go_on_red,
         const absl::flat_hash_map<TlDirection, SingleTlInfo>& tls,
         bool is_fresh, const std::string& last_error_msg);
...
 private:
  // lane properties
  mapping::ElementId lane_id_;
  // TODO(zixuan): route_s maybe abandoned in the future, delete it then.
  std::vector<double> control_point_route_s_;
  std::vector<double> control_point_relative_s_;
  bool can_go_on_red_;
  // associated tl properties
  absl::flat_hash_map<TlDirection, SingleTlInfo> tls_;
  TlControlType tl_control_type_ = TlControlType::SINGLE_DIRECTION;
  // freshness properties
  bool is_fresh_ = false;
  std::string last_error_msg_;
};
```

`class TlInfo`是红绿灯决策逻辑的输入，储存前方第一个路口的红绿灯信息。用于决策的主要信息包括：受红绿灯控制的车道id`lane_id_`、升序排列的停止线控制点的s坐标`control_point_relative_s_`/`control_point_route_s_`、该车道的控制类型`tl_control_type_`和控制该车道的红绿灯状态`tls_`。

车道的控制类型现在分为两种：

- 车道的控制类型为单方向信号灯控制时`TlControlType::SINGLE_DIRECTION`，车道的停止线控制点只有一个：`control_point_relative_s_.size() == 1`/`control_point_route_s_.size() == 1`，红绿灯状态`tls_.size() == 1`，只有一个单一方向的红绿灯信息。
- 车道的控制类型为左转待转区时`TlControlType::LEFT_WAITING_AREA`，车道的停止线控制点有两个：`control_point_relative_s_.size() == 2`/`control_point_route_s_.size() == 2`，红绿灯状态`tls_.size() == 2`，包括左转`TlDirection::LEFT`和直行`TlDirection::STRAIGHT`两个指示方向的的红绿灯信息。

未来如果有新的车道控制类型可以继续添加在`enum class TlControlType`中并完善相应的红绿灯投票和决策逻辑。



红绿灯状态的投票结果由`CreateTlInfo`函数创建。



### 创建红绿灯状态投票结果

红绿灯的投票过程在onboard/planner/decision/traffic_light_state_voter.h和onboard/planner/decision/traffic_light_state_voter.cc中。创建`TlInfo`中某一个方向的红绿灯信息由函数`CreateSingleTlInfo`实现，主要分为四个阶段：

1. 对同一个id的红绿灯灯箱感知结果投票
2. 对车道绑定的控制同一方向的多个灯箱感知结果投票
3. 对车道某一方向的红绿灯投票结果进行状态转移滤波
4. 更新信号灯变红的估计剩余时间

投票过程中需要参考和维护的历史状态定义在onboard/planner/decision/traffic_light_state_voter.h中，包括：

- 不同id的红绿灯的历史感知结果
- 车道的红绿灯状态历史投票结果
- 不同id的红绿灯的变红倒计时信息

```c++
struct YellowLightObservation {
  absl::Time first_flashing_green_observation = absl::InfiniteFuture();
  absl::Time latest_flashing_green_observation = absl::InfiniteFuture();
  absl::Time first_yellow_observation = absl::InfiniteFuture();
  absl::Time latest_yellow_observation = absl::InfiniteFuture();

  bool operator==(const YellowLightObservation& o) const;
};

using TrafficLightPerceptionStateHistory =
    absl::flat_hash_map<mapping::ElementId,  // traffic light id
                        HistoryBufferAbslTime<TrafficLightStateProto>>;
using TrafficLightVoteStateHistory = absl::flat_hash_map<
    mapping::ElementId,  // lane id
    absl::flat_hash_map<TlDirection, HistoryBufferAbslTime<TlState>>>;
using YellowLightObservations =
    absl::flat_hash_map<mapping::ElementId,  // traffic light id
                        YellowLightObservation>;

struct TrafficLightHistoryManager {
  TrafficLightPerceptionStateHistory tl_perception_history;
  TrafficLightVoteStateHistory tl_vote_history;
  YellowLightObservations yellow_light_observations;
};
```



#### 对同一个id的红绿灯灯箱感知结果投票

红绿灯检测会使用多个相机独立检测，因此对于同一个红绿灯灯箱，多个相机的独立检测结果可能会不同，因此首先对同一个灯箱的检测结果进行投票。参与该投票过程的有多个相机的感知结果和planner模块内部维护的历史感知结果`TrafficLightPerceptionStateHistory tl_perception_history`。具体实现函数`get_tl_perception_votes`在onboard/planner/decision/traffic_light_state_voter.cc中，最终返回该灯箱所有可能状态的投票结果`std::vector<TlVote> tl_votes`。`struct TlVote`储存投票信息，定义在onboard/planner/decision/traffic_light_state_voter.h中。

```c++
struct TlVote {
  TlState tl_state;
  double belief;
  bool from_history;

  bool operator>(const TlVote& rhs_tl_vote) const;
};
```

函数内部会维护一个`absl::flat_hash_map<TlState, std::pair<double, bool>> camera_votes`用于统计相机对每种状态的检测结果，每有一个相机检测到这种状态，该状态的置信度将上升，若该相机的感知结果为`TL_UNKNOWN`，则将从该红绿灯的历史检测结果中获取作为该相机的感知结果直接返回。



#### 对车道绑定的控制同一方向的多个灯箱感知结果投票

具体实现函数在onboard/planner/decision/traffic_light_state_voter.cc的`VoteForLaneAssociatedTlResult`中。例如某车道只受直行信号灯控制，但是指示直行的信号灯灯箱可能有多个，此时需要对多个灯箱检测结果进行投票，决定最终直行信号灯的颜色，首先获取每个灯箱的感知投票结果，并遵循以下规则进行投票：

1. 优先选择非历史感知结果
2. 选择置信度较大的作为本次单信号灯的投票结果
3. 平票时按照感知结果优先级选择优先级较高的结果作为投票结果，优先级顺序为：红灯>黄灯>绿闪>绿灯>黄闪

该投票规则由`struct TlVote`的重载运算符`>`定义。



#### 对车道某一方向的红绿灯投票结果进行状态转移滤波

状态转移滤波在onboard/planner/decision/traffic_light_state_voter.cc的`FilterTlState`函数中实现：在返回红绿灯投票最终结果前，将会校验投票结果的状态转移合理性，红绿灯投票结果会维护一个滑窗`TrafficLightVoteStateHistory tl_vote_history`，包含过去1.0s内的红绿灯状态，若是当前红绿灯与滑窗内最近的红绿灯结果满足转移逻辑，则将当前红绿灯状态更新进该滑窗，若不满足状态转移结果，则不会将当前红绿灯状态加入该滑窗，同时添加相应的QEvent，若因此持续1.0s不符合状态转移逻辑，该滑窗就会为空，将会强制将当前红绿灯状态加入滑窗。

合理的状态转移逻辑为：红灯->绿灯->绿闪或黄灯->黄灯->红灯，以及它们的自转移（即自己转换到自己），该逻辑在函数`TlStateTransitionIsCorrect`中实现，未来也可以向其中添加新的状态转移逻辑判断。



#### 更新信号灯变红的估计剩余时间

具体实现函数在onboard/planner/decision/traffic_light_state_voter.cc的`UpdateTurnRedTimeLeft`中。当经过状态转移滤波的红绿灯状态为`TlState::TL_STATE_YELLOW`或`TlState::TL_STATE_GREEN_FLASHING`时，函数会返回估计的信号灯变红剩余时间。同时函数还会维护不同id红绿灯的变红倒计时信息的历史状态`YellowLightObservations yellow_light_observations`。

对信号灯变红剩余时间的估计方法为：根据代码注释中的相关标准，得到不同限速下的绿闪和黄灯倒计时的总时长，再根据第一次和当前观察到绿闪/黄灯的时间进行估计。



#### 创建`TlInfo`

由`CreateTlInfo`函数实现：首先遍历当前的`lane_path`，找到自车当前位置最近的绑定红绿灯的车道，当该车道为单方向信号灯控制时，直接创建调用`CreateSingleTlInfo`创建该方向的红绿灯信息；当车道为左转待转区车道时（多方向信号灯控制），分别对左转方向和直行方向的信号灯调用`CreateSingleTlInfo`创建的红绿灯信息。如果在`kPathLength`的范围内没有绑定红绿灯的车道，则不会创建`TlInfo`。`TlInfo`将作为红绿灯决策逻辑的输入。



## 红绿灯决策逻辑



### 单方向信号灯控制逻辑

单方向信号灯控制逻辑通过函数`MakeSingleTrafficLightStopDecision`实现。具体逻辑如下：

* 若车辆还未到路口停止线
  * 若此时为红灯或`TL_STATE_UNKNOWN`则停车。
  * 若此时为绿闪或黄灯，则会估计红绿灯变红前的剩余时间和车辆到达停止线的时间，同时会计算以-1.5m/ss的减速度（以较为舒适的减速度提前刹车）和以-3.5m/ss的减速度（急刹）刹停的距离
    * 若提前刹车距离 < 越过停止线的距离，则做出刹停决策
    * 若越过停止线时间 < 红绿灯变红的剩余时间，或急刹刹停距离在停止线之后，则通过该停止线。
    * 如果上一时刻的决策为停在停止线之前，则本次决策也为停在停止线前。
  * 若此时为黄闪，则正常通过。目前和绿灯一样没有加任何速度限制，未来如果有相关需求可以修改。
  * 若此时为绿灯，则正常通过。
* 若车辆已经通过通过停止线且未超过10m距离
  * 若此时为绿灯，则通过路口。
  * 若此时不为绿灯，则与上一时刻的决策相同。



### 左转待转区信号灯控制逻辑

左转待转区灯控制逻辑通过函数`MakeMultiTrafficLightStopDecision`实现

左转待转区有两根停止线可以激活，分别为路口的第一根停止线和左转待转区的第二根停止线，左转待转区的红绿灯决策要根据两个方向的红绿灯状态共同完成。除此之外，还需要维护一个状态量`bool entry_with_left_light_not_red`来指示车辆进入左转待转区时是否为红灯（不为红灯则可以继续通行，否则需要在待转区停车等待）。具体逻辑如下：

* 左转灯为绿灯
  * 可以通过。
  * 若此时车辆已经越过第一根停止线，则标记`entry_with_left_light_not_red = true`。
* 左转灯为绿闪或黄灯
  * 若此时车辆没有越过第一根停止线，则会估计红绿灯变红前的剩余时间和车辆到达停止线的时间，同时会计算以-1.5m/ss的减速度（以较为舒适的减速度提前刹车）和以-3.5m/ss的减速度（急刹）刹停的距离
    * 若提前刹车距离 < 越过第一根停止线的距离，则做出刹停决策，停在第一根停止线前。
    * 若越过停止线时间 < 红绿灯变红的剩余时间，或急刹刹停距离在第一根停止线之后，则通过。
    * 如果上一时刻的决策为停在第一根停止线之前，则本次决策也为停在第一根停止线前。
  * 若此时车辆已经越过第一根停止线，在待转区内
    * 若`entry_with_left_light_not_red = true`，说明左转灯是在车辆越过第一根停止线后才变为绿闪或黄灯，之前不为红灯，这种情况继续通行。
    * 若`entry_with_left_light_not_red = false`，说明左转灯在车辆进入待转区时为红灯，这种情况应停车等待，停在第二根停止线之前，等待左转灯变绿再继续通行。
* 左转灯为红灯或`TL_STATE_UNKNOWN`
  * 若此时车辆没有越过第一根停止线
    * 若直行灯为绿灯，则车辆应该进入待转区等待，停在第二根停止线之前。
    * 若直行灯不为绿灯，则车辆应该停在第一根停止线之前。
  * 若此时车辆已经越过第一根停止线，在待转区内
    * 若`entry_with_left_light_not_red = true`，说明左转灯是在车辆越过第一根停止线后才变为红灯的，之前不为红灯，这种情况继续通行。
    * 若`entry_with_left_light_not_red = false`，说明左转灯在车辆进入待转区时为红灯，这种情况应停车等待，停在第二根停止线之前，等待左转灯变绿再继续通行。



