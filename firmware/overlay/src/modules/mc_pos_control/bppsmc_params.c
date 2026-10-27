/**
 * 多旋翼位置控制器类型
 *
 * 选择 PX4 原生串级位置 P / 速度 PID 控制器，或 ET-BPPSMC 加速度控制器。
 * 仅在解除武装后应用。姿态和角速度控制器仍为 PX4 原生控制器。
 *
 * @value 0 PX4 原生 PID
 * @value 1 ET-BPPSMC
 * @min 0
 * @max 1
 * @group BPPSMC
 */
PARAM_DEFINE_INT32(MPC_CTRL_METHOD, 0);

/**
 * 水平预设性能函数加速度参数
 *
 * 用于设置最速性能函数形状，不是控制器输出限幅。仅在解除武装后应用，X/Y 轴共用此值。
 * @unit m/s^2
 * @min 0.05
 * @max 5
 * @decimal 2
 * @group BPPSMC
 */
PARAM_DEFINE_FLOAT(BPP_AMAX_XY, 0.72f);

/**
 * 垂直预设性能函数加速度参数
 *
 * 仅在解除武装后应用。
 * @unit m/s^2
 * @min 0.05
 * @max 5
 * @decimal 2
 * @group BPPSMC
 */
PARAM_DEFINE_FLOAT(BPP_AMAX_Z, 0.90f);

/**
 * 水平误差漏斗最小半宽
 *
 * 实际半宽为 max(BPP_DELTA * 初始误差, 此参数值)。仅在解除武装后应用，X/Y 轴共用此值。
 * @unit m
 * @min 0.05
 * @max 10
 * @decimal 2
 * @group BPPSMC
 */
PARAM_DEFINE_FLOAT(BPP_BAND_XY, 0.75f);

/**
 * 垂直误差漏斗最小半宽
 *
 * 仅在解除武装后应用。
 * @unit m
 * @min 0.05
 * @max 10
 * @decimal 2
 * @group BPPSMC
 */
PARAM_DEFINE_FLOAT(BPP_BAND_Z, 0.75f);

/**
 * 初始误差漏斗半宽比例
 *
 * 仅在解除武装后应用。
 * @min 0.01
 * @max 1
 * @decimal 2
 * @group BPPSMC
 */
PARAM_DEFINE_FLOAT(BPP_DELTA, 0.10f);

/**
 * 水平事件触发阈值比例
 *
 * 触发阈值为此参数乘以漏斗半宽。仅在解除武装后应用，X/Y 轴共用此值。
 * @min 0.05
 * @max 0.99
 * @decimal 2
 * @group BPPSMC
 */
PARAM_DEFINE_FLOAT(BPP_TRIG_XY, 0.76f);

/**
 * 垂直事件触发阈值比例
 *
 * 仅在解除武装后应用。
 * @min 0.05
 * @max 0.99
 * @decimal 2
 * @group BPPSMC
 */
PARAM_DEFINE_FLOAT(BPP_TRIG_Z, 0.95f);

/**
 * 水平滑模面增益
 *
 * 仅在解除武装后应用，X/Y 轴共用此值。
 * @min 0.05
 * @max 10
 * @decimal 2
 * @group BPPSMC
 */
PARAM_DEFINE_FLOAT(BPP_KS_XY, 1.40f);

/**
 * 垂直滑模面增益
 *
 * 仅在解除武装后应用。
 * @min 0.05
 * @max 10
 * @decimal 2
 * @group BPPSMC
 */
PARAM_DEFINE_FLOAT(BPP_KS_Z, 1.75f);

/**
 * 水平线性趋近增益
 *
 * 仅在解除武装后应用，X/Y 轴共用此值。
 * @min 0.05
 * @max 10
 * @decimal 2
 * @group BPPSMC
 */
PARAM_DEFINE_FLOAT(BPP_K0_XY, 1.20f);

/**
 * 垂直线性趋近增益
 *
 * 仅在解除武装后应用。
 * @min 0.05
 * @max 10
 * @decimal 2
 * @group BPPSMC
 */
PARAM_DEFINE_FLOAT(BPP_K0_Z, 1.80f);

/**
 * 水平切换增益
 *
 * 乘以 tanh(S / BPP_PHI_XY)。仅在解除武装后应用，X/Y 轴共用此值。
 * @min 0
 * @max 5
 * @decimal 2
 * @group BPPSMC
 */
PARAM_DEFINE_FLOAT(BPP_K1_XY, 0.20f);

/**
 * 垂直切换增益
 *
 * 仅在解除武装后应用。
 * @min 0
 * @max 5
 * @decimal 2
 * @group BPPSMC
 */
PARAM_DEFINE_FLOAT(BPP_K1_Z, 0.35f);

/**
 * 水平 tanh 边界层宽度
 *
 * 仅在解除武装后应用，X/Y 轴共用此值。
 * @min 0.05
 * @max 10
 * @decimal 2
 * @group BPPSMC
 */
PARAM_DEFINE_FLOAT(BPP_PHI_XY, 1.50f);

/**
 * 垂直 tanh 边界层宽度
 *
 * 仅在解除武装后应用。
 * @min 0.05
 * @max 10
 * @decimal 2
 * @group BPPSMC
 */
PARAM_DEFINE_FLOAT(BPP_PHI_Z, 1.20f);

/**
 * 预设趋近时间
 *
 * 仅在解除武装后应用。
 * @unit s
 * @min 0.1
 * @max 20
 * @decimal 2
 * @group BPPSMC
 */
PARAM_DEFINE_FLOAT(BPP_T_PRE, 2.f);

/**
 * 趋近阶段容差
 *
 * 仅在解除武装后应用。
 * @min 0.001
 * @max 1
 * @decimal 3
 * @group BPPSMC
 */
PARAM_DEFINE_FLOAT(BPP_EPS_PRE, 0.1f);

/**
 * 预设滑动时间
 *
 * 仅在解除武装后应用。
 * @unit s
 * @min 0.1
 * @max 20
 * @decimal 2
 * @group BPPSMC
 */
PARAM_DEFINE_FLOAT(BPP_T_SLIDE, 2.f);

/**
 * 滑动阶段容差
 *
 * 仅在解除武装后应用。
 * @min 0.001
 * @max 1
 * @decimal 3
 * @group BPPSMC
 */
PARAM_DEFINE_FLOAT(BPP_EPS_SLIDE, 0.1f);

/**
 * 事件锚点误差最小值
 *
 * 仅在解除武装后应用。
 * @unit m
 * @min 0.001
 * @max 1
 * @decimal 3
 * @group BPPSMC
 */
PARAM_DEFINE_FLOAT(BPP_E0_MIN, 0.05f);

/**
 * 触发事件所需的最小校正加速度
 *
 * 仅在解除武装后应用。
 * @unit m/s^2
 * @min 0.001
 * @max 1
 * @decimal 3
 * @group BPPSMC
 */
PARAM_DEFINE_FLOAT(BPP_ACC_FLOOR, 0.05f);

/**
 * 起飞阶段首次垂直事件触发阈值
 *
 * 此值为漏斗半宽的比例。仅在解除武装后应用。
 * @min 0.05
 * @max 0.99
 * @decimal 2
 * @group BPPSMC
 */
PARAM_DEFINE_FLOAT(BPP_TRIG_RAMP, 0.30f);

/**
 * BPPSMC 水平输出加速度限幅
 *
 * 进入 PX4 原生推力映射前，北东地坐标系各水平轴的绝对加速度限值。
 * 仅在解除武装后应用，X/Y 轴共用此值。
 * @unit m/s^2
 * @min 0.1
 * @max 15
 * @decimal 2
 * @group BPPSMC
 */
PARAM_DEFINE_FLOAT(BPP_ACC_XY, 6.f);

/**
 * BPPSMC 垂直输出加速度限幅
 *
 * 进入 PX4 原生推力映射前，北东地坐标系垂直轴的绝对加速度限值。
 * 仅在解除武装后应用。
 * @unit m/s^2
 * @min 0.1
 * @max 15
 * @decimal 2
 * @group BPPSMC
 */
PARAM_DEFINE_FLOAT(BPP_ACC_Z, 10.f);
