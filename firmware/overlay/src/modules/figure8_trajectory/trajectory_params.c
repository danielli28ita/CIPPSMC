/**
 * Enable onboard figure-eight experiment
 *
 * Reboot after changing. Requires manual input and a valid local position.
 * Arm in a manual control mode, then select OFFBOARD on the ground.
 * While enabled, native PID is used outside an active onboard task.
 * Do not run external OFFBOARD setpoint publishers at the same time.
 *
 * @boolean
 * @reboot_required true
 * @group Onboard Figure8
 */
PARAM_DEFINE_INT32(F8_ENABLE, 0);

/**
 * Takeoff height above the starting ground
 *
 * @unit m
 * @min 0.5
 * @max 10
 * @decimal 1
 * @group Onboard Figure8
 */
PARAM_DEFINE_FLOAT(F8_HEIGHT, 3.0f);

/**
 * Bernoulli lemniscate scale a
 *
 * North extent is +/-a; east extent is +/-a/(2*sqrt(2)).
 * Matches the unscaled curve in BPPSMC-reproducible.
 *
 * @unit m
 * @min 0.2
 * @max 10
 * @decimal 1
 * @group Onboard Figure8
 */
PARAM_DEFINE_FLOAT(F8_RADIUS, 3.0f);

/**
 * Duration of each figure-eight lap
 *
 * Smooth quintic phase, with zero velocity/acceleration at each lap boundary.
 *
 * @unit s
 * @min 20
 * @max 300
 * @decimal 1
 * @group Onboard Figure8
 */
PARAM_DEFINE_FLOAT(F8_PERIOD, 60.0f);

/**
 * Number of figure-eight laps
 *
 * @min 1
 * @max 20
 * @group Onboard Figure8
 */
PARAM_DEFINE_INT32(F8_LAPS, 1);

/**
 * Smooth takeoff reference duration
 *
 * @unit s
 * @min 5
 * @max 60
 * @decimal 1
 * @group Onboard Figure8
 */
PARAM_DEFINE_FLOAT(F8_TKO_TIME, 12.0f);

/**
 * Minimum hover before and after the figure-eight
 *
 * Also waits for position error <= 0.5 m and speed <= 0.4 m/s for 2 s.
 * Aborts if not settled within another 30 s.
 *
 * @unit s
 * @min 2
 * @max 60
 * @decimal 1
 * @group Onboard Figure8
 */
PARAM_DEFINE_FLOAT(F8_HOV_TIME, 5.0f);

/**
 * Landing ramp duration limit
 *
 * Descending reference continues through contact until landing is confirmed.
 * Aborts after this duration plus 30 s if landing is not confirmed.
 *
 * @unit s
 * @min 8
 * @max 60
 * @decimal 1
 * @group Onboard Figure8
 */
PARAM_DEFINE_FLOAT(F8_LND_TIME, 18.0f);

/**
 * Landing reference depth below starting ground
 *
 * For a level takeoff/landing surface. Not a terrain clearance measurement.
 *
 * @unit m
 * @min 0.05
 * @max 0.5
 * @decimal 2
 * @group Onboard Figure8
 */
PARAM_DEFINE_FLOAT(F8_LND_DEPTH, 0.3f);

/**
 * Maximum three-dimensional tracking error
 *
 * Exceeding this value aborts the onboard task and requests PX4 Land.
 * Native PID is used for the abort/RC takeover path.
 *
 * @unit m
 * @min 0.5
 * @max 10
 * @decimal 1
 * @group Onboard Figure8
 */
PARAM_DEFINE_FLOAT(F8_ERR_MAX, 3.0f);
