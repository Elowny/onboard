## 功能说明
- 目标：实现车辆线控系统特性开环测试
  - 针对不同的线控接口，包含以下3种测试工况：
  - 油门刹车接口-标定表生成2.0、特性验证（包括延迟、死区等）；
  - 加速度接口-线控响应特性验证（包括延迟、死区等）；
  - 转角接口-线控响应特性验证（包括延迟、死区、零偏等）；

## 功能流程
- 启动检查：当激活此功能时，首先检查当前车轮是否归零;

- 配置参数：example：calibration.pb.txt;

- 启动程序：./run_autonomy.sh control_openloop_enable=true  control_openloop_path=XXX/calibration.pb.txt

- 详细说明：https://qcraft.feishu.cn/docs/doccn1m9SK5t9ldgwDymTJ4YxPc
