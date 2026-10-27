# 参数文件与 log_19 配置

本包全部样例参数均从唯一 log_19 提取，没有混用其他架次或未实飞候选值。JSON 与 17 位有效数字 QGC 文本保留解码后的 float32 精确值；下表仅为便于阅读的四舍五入显示。

| 参数 | 样例值 | 含义 |
|---|---:|---|
| `BPP_ACC_FLOOR` | 0.05 | 事件所需最小校正加速度 |
| `BPP_ACC_XY` | 6 | 每水平轴加速度输出限幅，m/s² |
| `BPP_ACC_Z` | 10 | 垂直加速度输出限幅，m/s² |
| `BPP_AMAX_XY` | 0.72 | 水平性能中心加速度尺度，m/s²；不是输出上限 |
| `BPP_AMAX_Z` | 0.9 | 垂直性能中心加速度尺度，m/s² |
| `BPP_BAND_XY` | 0.75 | 水平漏斗最小半宽，m |
| `BPP_BAND_Z` | 0.75 | 垂直漏斗最小半宽，m |
| `BPP_DELTA` | 0.1 | 锚点误差对应的漏斗半宽比例 |
| `BPP_E0_MIN` | 0.05 | 锚点误差幅值下限，m |
| `BPP_EPS_PRE` | 0.1 | 趋近时变增益容差 |
| `BPP_EPS_SLIDE` | 0.1 | 滑动时变增益容差 |
| `BPP_K0_XY` | 1.3 | 水平线性趋近基础增益 |
| `BPP_K0_Z` | 1.9 | 垂直线性趋近基础增益 |
| `BPP_K1_XY` | 0.25 | 水平 tanh 切换项增益 |
| `BPP_K1_Z` | 0.4 | 垂直 tanh 切换项增益 |
| `BPP_KS_XY` | 1.7 | 水平滑模面基础增益 |
| `BPP_KS_Z` | 2 | 垂直滑模面基础增益 |
| `BPP_PHI_XY` | 1.5 | 水平 tanh 平滑边界层尺度 |
| `BPP_PHI_Z` | 1.2 | 垂直 tanh 平滑边界层尺度 |
| `BPP_TRIG_RAMP` | 0.3 | 起飞首次垂直事件阈值比例 |
| `BPP_TRIG_XY` | 0.76 | 水平事件阈值/半宽比例 |
| `BPP_TRIG_Z` | 0.95 | 垂直事件阈值/半宽比例 |
| `BPP_T_PRE` | 2 | 趋近时间参数，s |
| `BPP_T_SLIDE` | 2 | 滑动时间参数，s |
| `F8_ENABLE` | 1 | 机载任务启用；修改后重启 |
| `F8_ERR_MAX` | 3 | 三维误差中止阈值，m |
| `F8_HEIGHT` | 0.8 | 相对地面准备点高度，m |
| `F8_HOV_TIME` | 5 | 任务前后最少悬停时间，s |
| `F8_LAPS` | 2 | 圈数 |
| `F8_LND_DEPTH` | 0.3 | 降落参考越过起飞平面的深度，m |
| `F8_LND_TIME` | 18 | 下降参考加速时长上限，s；受最低终端速度逻辑影响 |
| `F8_PERIOD` | 25 | 每圈时长，s |
| `F8_RADIUS` | 1.6 | 伯努利双纽线尺度 a，m |
| `F8_TKO_TIME` | 12 | 平滑起飞参考时长，s |
| `MPC_CTRL_METHOD` | 1 | 0 原生 PID；1 本文位置控制器 |
| `MPC_THR_HOVER` | 0.21 | 实机归一化悬停推力映射 |

## 生效与配置边界

- `MPC_CTRL_METHOD`、`BPP_*` 在未解锁时应用；F8 任务参数在地面准备时读取，飞行中修改不改变本次任务。`F8_ENABLE` 修改后重启。
- 固件默认值仍来自冻结源码，通常是 PID、任务关闭和原始 BPP 初值；上表是样例运行参数，两者不能混为一谈。
- `flight_profile.params` 仅 36 项；完整 1153 项见 `all_parameters.json` 和 `all_parameters_REFERENCE_ONLY.params`。后者包含校准、板型、电源和模式设置，仅作来源记录。
- 原始日志没有飞行中参数修改，所以不存在一架次内需要拼接的多组 BPP 配置。
- 坐标为本地 NED，位置误差为实际减参考；Z 正误差表示低于参考。
- 仿真沿用相同 BPP/F8。X500 的悬停映射只写在仿真启动配置中，记录见 `validation/sitl_result.json`，不写入实机参数文件，也不修改冻结控制器源码。

## 其他配置的来源

外部位置/高度/航向融合配置为 EKF2_EV_CTRL=11、EKF2_GPS_CTRL=0、EKF2_HGT_REF=3，外部速度未融合。SYS_AUTOSTART=4050、SYS_HITL=0。机体的原生内环、校准、接收机等其他数值完整保留在 all_parameters.json，未作为通用可移植配置自动应用。
