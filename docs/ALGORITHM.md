# 算法与冻结源码

控制器入口：`firmware/overlay/src/modules/mc_pos_control/PositionControl/PositionControl.cpp` 的 `_applyBppsmcOverride()`；接入与参数处理位于相邻 `MulticopterPositionControl.cpp/.hpp`，参数定义为 `bppsmc_params.c`。

## 控制链路

```text
机载位置/速度/加速度参考 → 保存纯参考 → 原生位置/速度 PID
  → 满足 BPPSMC 激活条件时覆盖对应轴加速度 → PX4 推力映射
  → 原生姿态/角速度环 → 控制分配 → 执行器
```

NED 坐标中 e1=p-p_ref、e2=v-v_ref，Z 向下为正。BPPSMC 替换激活轴的外环输出，不是与原生 PID 积分持续叠加。MPC_CTRL_METHOD=1 选择当前外环；不替换 Stabilized/Acro 的原生内环。

## 性能函数、障碍变换和滑模面

重锚后中心以有符号 e0 乘 cos² 衰减：

```text
Ts = pi*sqrt(e0/(2*a_max))
rho(tau) = direction*e0*cos²(pi*tau/(2*Ts)), tau<Ts
rho = 0, tau>=Ts
b = max(BPP_DELTA*e0, BPP_BAND_axis)
zeta = clip((e1-rho+b)/(2*b), 1e-6, 1-1e-6)
epsilon = log(zeta/(1-zeta))
J = 1/(2*b*zeta*(1-zeta))
epsilon_dot = J*(e2-rho_dot)
S = epsilon_dot + lambda*epsilon
```

初始化时中心保持为零；首次接受事件后才进入中心衰减。a_max 是性能函数加速度尺度，不是最终输出限幅。

有界时变项：

```text
gamma = tau/(eps_pre*T_pre² + tau²)
tau_s = max(0, tau-T_pre)
eta = tau_s/(eps_slide*T_slide² + tau_s²)
lambda = KS + eta
kappa = K0 + gamma
```

控制律包含参考前馈、J 的导数、性能中心导数、滑模面补偿、线性趋近和 K1*tanh(S/PHI)。最终加速度按 BPP_ACC_XY/Z 限幅，推力映射沿用 PX4 `_accelerationControl()`。

## 事件与数值实现

当中心相对误差超过触发比例乘半宽，并且未限幅修正加速度有限、幅值足够且方向朝内时，独立轴接受一次重锚；误差回到触发边界内后重新允许触发。E0_MIN 对锚点幅值设下限。起飞首次垂直触发另有 TRIG_RAMP。

日志调试消息在重锚后发布，但 rho/S/输出是本周期计算结果，e0/t0/counter 可能已经更新。不能在事件样本中将这两套时刻混用。本包样例三轴没有重锚；附带滑模项分解仅适用于样例的零中心、无重锚窗口，对其他事件工况会明确拒绝。

论文理想 sign 控制律与本包的 tanh、加速度限幅、障碍钳位和锚点幅值下限不同；不将有限时间/预设性能的理想条件无条件套用到饱和、有噪声的实机实现。包内保留原源码的对比控制器编译依赖，但默认 BASELINE_CONTROLLER=0，构建检查还验证 `PositionControl::update()` 中确实调用 BPPSMC。

## 机载任务

`figure8_trajectory` 经 uORB 以 50 Hz 生成伯努利双纽线的位置、速度、加速度和固定航向参考，使用 OFFBOARD 通道。相位使用五次多项式，速度和加速度包含链式求导项，每圈起止为零：

```text
u = elapsed/period
s = 10*u³ - 15*u⁴ + 6*u⁵
theta = 2*pi*s - pi/2
north = n0 + a*cos(theta)/(1+sin²(theta))
east = e0 + a*sin(theta)*cos(theta)/(1+sin²(theta))
```

尺度 a 对应 F8_RADIUS，不是两个方向独立半径。阶段为起飞、悬停、八字、返回悬停、降落、接地和完成。任务状态机原样收录，不包含已撤销的 BPP_SAFE_EN/BPP_JERK 等工程扩展。

机载任务启用策略会锁存；任务外、人工接管或状态超时后，满足对应源码条件时禁用 BPPSMC，使用原生链路。任务外且从未启用机载模块时仍遵循一般控制器选择逻辑，不应笼统理解为所有 OFFBOARD 客户端都有同样的回退策略。
