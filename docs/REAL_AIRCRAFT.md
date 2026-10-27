# 实机、外部定位与任务操作

本包实飞证据仅来自 log_19。日志记录了完整 BPPSMC 两圈任务、落地和普通上锁。本次打包没有连接实体飞控，不能把重新构建的固件称为本轮新实飞通过；历史飞行二进制的逐字节身份也未认证。

## 已知配置与仍需提供的物理信息

| 项目 | 从样例可确认的配置 |
|---|---|
| 飞控 | PX4_FMU_V6C / Pixhawk 6C |
| PX4 基线 | v1.15.4，99c40407ffd7ac184e2d7b4b293f36f10fe561ef |
| 机型参数 | SYS_AUTOSTART=4050，Generic 250 Racer；SYS_HITL=0 |
| 控制器 | MPC_CTRL_METHOD=1，ET-BPPSMC |
| 参考轨迹 | 飞控内部伯努利双纽线，0.8 m 高、a=1.6 m、25 s/圈、2 圈 |
| 定位配置 | EKF2_EV_CTRL=11、EKF2_GPS_CTRL=0、EKF2_HGT_REF=3；融合外部位置/高度/航向，不融合外部速度 |
| 外部位置接口 | vehicle_visual_odometry；消息名不能确定是视觉还是动捕设备 |
| 动力与机械 | 电机型号/KV、桨叶型号、ESC/协议、电池串数/容量、质量/载荷未提供 |
| 定位系统硬件与发布程序 | 未提供设备型号、外部软件源码和真实采样/传输延迟 |
| 地面站、接收机 | QGC 版本及接收机实物型号未由日志确认 |

上述未提供项不妨碍源码构建、日志复算或包内 SITL；但阻止将物理实飞条件宣称为完整可复制。请用实际设备记录补充它们，不从 Generic 250 名称推断动力配置。机载轨迹无需主机设定值发布器，外部定位仍可能需要主机及其原有桥接程序。

## 固件和配置顺序

1. 接线、传感器校准、执行器测试期间拆下全部桨叶。备份现有参数。按实际 Pixhawk 6C 和动力系统说明接线，官方参考：[Pixhawk 6C 接线](https://docs.px4.io/v1.15/en/assembly/quick_start_pixhawk6c)。
2. 在 QGC 自定义固件入口选择本包 `artifacts/px4_fmu-v6c_onboard/px4_fmu-v6c_onboard.px4`。它是实机板级目标，不是 HITL 模拟输出固件。核对 SYS_AUTOSTART=4050、SYS_HITL=0。
3. 保留适用于实际机体的传感器校准、动力、输出、电池和接收机配置。不得把 `all_parameters_REFERENCE_ONLY.params` 当作另一架飞机的完整初始化文件。
4. 导入前先查看 `params/flight_profile.params`：其中仅含从 log_19 提取的 BPP/F8、控制器选择和悬停映射。F8_ENABLE=1 在重启后启用模块，MPC_CTRL_METHOD=1 选择任务内 BPPSMC；原生内环和实机参数仍需要匹配实际机体。
5. 复现样例定位条件时，恢复实际外部定位接口，核对 NED/FRD 坐标、姿态方向、外参、尺度、有效时间戳和更新率。日志中的 EV 配置和 0 延迟参数是记录值，不是未知定位系统的通用设置。参考：[PX4 v1.15 外部定位](https://docs.px4.io/v1.15/en/ros/external_position_estimation)。
6. 核对 RC、模式选择、解锁/上锁、Kill、RC 丢失和 OFFBOARD 丢失的实际配置及效果。具体开关通道由接收机实测决定，不能直接套用 SITL 的模拟通道映射。

## 机载任务流程

落地且上锁时应用参数，重启后运行：

```text
figure8_trajectory status
mc_pos_control status
```

等待任务 ready，先在已经验证的手动模式人工解锁，再从地面切入 OFFBOARD。出现 `Figure8 started: BPPSMC` 后任务依次完成起飞、悬停、两圈、返回悬停和下降。模块不负责人工解锁，也不需要主机重复请求 OFFBOARD。避免同时运行另一个轨迹发布器。

正常结束以 `Figure8 complete: landed and disarmed`、飞控上锁和电机停止共同确认。随后把解锁开关恢复上锁位并退出 OFFBOARD。任务启用锁存策略下，人工退出 OFFBOARD 会中止任务，并按源码使用原生控制链路；不会在空中自动重新开始任务。

Kill 是紧急停桨，不能作为普通降落方式。接管模式的油门含义要按实际模式确认。此处只说明包内实现和已记录样例，不替代机主既有操作规程或对未知机体的调参。

## 样例中必须保留的记录

log_19 在约 46.655 s 有一次 `Avionics Power low: 4.79 Volt` 文本。稀疏 system_power 记录未捕获同一低值，最低约 4.817 V；二者采样频率不同，不能删除告警或声称电压全程正常。此后任务完成且无 failsafe，日志未证明该告警造成跟踪误差。

battery_status 的连接/电压记录不能当作可靠动力电池测量。新的实飞记录应补充可信电池、电源和实验条件；这些已知信息边界也写入样例报告。

SITL 启动脚本、模拟 RC 映射和 X500 的仿真专用悬停映射均不能导入真实飞机。
