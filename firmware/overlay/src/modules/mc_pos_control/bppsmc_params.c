/**
 * Multicopter position-controller selection
 *
 * Select the native PX4 cascaded position-P / velocity-PID controller,
 * or the CIPPSMC acceleration controller.
 * Applied only while disarmed. Attitude and rate controllers stay native PX4.
 *
 * @value 0 Native PX4 PID
 * @value 1 CIPPSMC
 * @min 0
 * @max 1
 * @group CIPPSMC
 */
PARAM_DEFINE_INT32(MPC_CTRL_METHOD, 0);

/**
 * Horizontal prescribed-performance acceleration scale
 *
 * Sets the shape of the performance function. It is not the output limit.
 * Applied only while disarmed. Shared by the x and y axes.
 * @unit m/s^2
 * @min 0.05
 * @max 5
 * @decimal 2
 * @group CIPPSMC
 */
PARAM_DEFINE_FLOAT(BPP_AMAX_XY, 0.72f);

/**
 * Vertical prescribed-performance acceleration scale
 *
 * Applied only while disarmed.
 * @unit m/s^2
 * @min 0.05
 * @max 5
 * @decimal 2
 * @group CIPPSMC
 */
PARAM_DEFINE_FLOAT(BPP_AMAX_Z, 0.90f);

/**
 * Minimum horizontal corridor half-width
 *
 * The half-width used in flight is max(BPP_DELTA * initial error, this value).
 * Applied only while disarmed. Shared by the x and y axes.
 * @unit m
 * @min 0.05
 * @max 10
 * @decimal 2
 * @group CIPPSMC
 */
PARAM_DEFINE_FLOAT(BPP_BAND_XY, 0.75f);

/**
 * Minimum vertical corridor half-width
 *
 * Applied only while disarmed.
 * @unit m
 * @min 0.05
 * @max 10
 * @decimal 2
 * @group CIPPSMC
 */
PARAM_DEFINE_FLOAT(BPP_BAND_Z, 0.75f);

/**
 * Corridor half-width as a fraction of the initial error
 *
 * Applied only while disarmed.
 * @min 0.01
 * @max 1
 * @decimal 2
 * @group CIPPSMC
 */
PARAM_DEFINE_FLOAT(BPP_DELTA, 0.10f);

/**
 * Horizontal event-trigger fraction
 *
 * The trigger distance is this value times the corridor half-width.
 * Applied only while disarmed. Shared by the x and y axes.
 * @min 0.05
 * @max 0.99
 * @decimal 2
 * @group CIPPSMC
 */
PARAM_DEFINE_FLOAT(BPP_TRIG_XY, 0.76f);

/**
 * Vertical event-trigger fraction
 *
 * Applied only while disarmed.
 * @min 0.05
 * @max 0.99
 * @decimal 2
 * @group CIPPSMC
 */
PARAM_DEFINE_FLOAT(BPP_TRIG_Z, 0.95f);

/**
 * Horizontal sliding-surface gain
 *
 * Applied only while disarmed. Shared by the x and y axes.
 * @min 0.05
 * @max 10
 * @decimal 2
 * @group CIPPSMC
 */
PARAM_DEFINE_FLOAT(BPP_KS_XY, 1.40f);

/**
 * Vertical sliding-surface gain
 *
 * Applied only while disarmed.
 * @min 0.05
 * @max 10
 * @decimal 2
 * @group CIPPSMC
 */
PARAM_DEFINE_FLOAT(BPP_KS_Z, 1.75f);

/**
 * Horizontal linear reaching gain
 *
 * Applied only while disarmed. Shared by the x and y axes.
 * @min 0.05
 * @max 10
 * @decimal 2
 * @group CIPPSMC
 */
PARAM_DEFINE_FLOAT(BPP_K0_XY, 1.20f);

/**
 * Vertical linear reaching gain
 *
 * Applied only while disarmed.
 * @min 0.05
 * @max 10
 * @decimal 2
 * @group CIPPSMC
 */
PARAM_DEFINE_FLOAT(BPP_K0_Z, 1.80f);

/**
 * Horizontal switching gain
 *
 * Multiplies tanh(S / BPP_PHI_XY). Applied only while disarmed.
 * Shared by the x and y axes.
 * @min 0
 * @max 5
 * @decimal 2
 * @group CIPPSMC
 */
PARAM_DEFINE_FLOAT(BPP_K1_XY, 0.20f);

/**
 * Vertical switching gain
 *
 * Applied only while disarmed.
 * @min 0
 * @max 5
 * @decimal 2
 * @group CIPPSMC
 */
PARAM_DEFINE_FLOAT(BPP_K1_Z, 0.35f);

/**
 * Horizontal tanh boundary-layer width
 *
 * Applied only while disarmed. Shared by the x and y axes.
 * @min 0.05
 * @max 10
 * @decimal 2
 * @group CIPPSMC
 */
PARAM_DEFINE_FLOAT(BPP_PHI_XY, 1.50f);

/**
 * Vertical tanh boundary-layer width
 *
 * Applied only while disarmed.
 * @min 0.05
 * @max 10
 * @decimal 2
 * @group CIPPSMC
 */
PARAM_DEFINE_FLOAT(BPP_PHI_Z, 1.20f);

/**
 * Prescribed reaching time
 *
 * Applied only while disarmed.
 * @unit s
 * @min 0.1
 * @max 20
 * @decimal 2
 * @group CIPPSMC
 */
PARAM_DEFINE_FLOAT(BPP_T_PRE, 2.f);

/**
 * Reaching-phase tolerance
 *
 * Applied only while disarmed.
 * @min 0.001
 * @max 1
 * @decimal 3
 * @group CIPPSMC
 */
PARAM_DEFINE_FLOAT(BPP_EPS_PRE, 0.1f);

/**
 * Prescribed sliding time
 *
 * Applied only while disarmed.
 * @unit s
 * @min 0.1
 * @max 20
 * @decimal 2
 * @group CIPPSMC
 */
PARAM_DEFINE_FLOAT(BPP_T_SLIDE, 2.f);

/**
 * Sliding-phase tolerance
 *
 * Applied only while disarmed.
 * @min 0.001
 * @max 1
 * @decimal 3
 * @group CIPPSMC
 */
PARAM_DEFINE_FLOAT(BPP_EPS_SLIDE, 0.1f);

/**
 * Minimum anchor-error magnitude
 *
 * Applied only while disarmed.
 * @unit m
 * @min 0.001
 * @max 1
 * @decimal 3
 * @group CIPPSMC
 */
PARAM_DEFINE_FLOAT(BPP_E0_MIN, 0.05f);

/**
 * Minimum corrective acceleration required to accept an event
 *
 * Applied only while disarmed.
 * @unit m/s^2
 * @min 0.001
 * @max 1
 * @decimal 3
 * @group CIPPSMC
 */
PARAM_DEFINE_FLOAT(BPP_ACC_FLOOR, 0.05f);

/**
 * First vertical trigger fraction during takeoff
 *
 * The value is a fraction of the corridor half-width.
 * Applied only while disarmed.
 * @min 0.05
 * @max 0.99
 * @decimal 2
 * @group CIPPSMC
 */
PARAM_DEFINE_FLOAT(BPP_TRIG_RAMP, 0.30f);

/**
 * Horizontal acceleration command limit
 *
 * Absolute acceleration limit on each horizontal NED axis before
 * the native PX4 thrust mapping. Applied only while disarmed.
 * Shared by the x and y axes.
 * @unit m/s^2
 * @min 0.1
 * @max 15
 * @decimal 2
 * @group CIPPSMC
 */
PARAM_DEFINE_FLOAT(BPP_ACC_XY, 6.f);

/**
 * Vertical acceleration command limit
 *
 * Absolute acceleration limit on the vertical NED axis before
 * the native PX4 thrust mapping. Applied only while disarmed.
 * @unit m/s^2
 * @min 0.1
 * @max 15
 * @decimal 2
 * @group CIPPSMC
 */
PARAM_DEFINE_FLOAT(BPP_ACC_Z, 10.f);
