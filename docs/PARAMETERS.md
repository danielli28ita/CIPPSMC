# Parameters and the Flight A configuration

Every sample parameter in this table was taken from Flight A (`log_1.ulg`). No other flight and no unflown candidate value is mixed in. The JSON and the 17-significant-digit QGroundControl text keep the decoded float32 values. The table below is rounded for reading.

| Parameter | Sample | Meaning |
|---|---:|---|
| `BPP_ACC_FLOOR` | 0.05 | Minimum corrective acceleration required for an event |
| `BPP_ACC_XY` | 6 | Per-horizontal-axis acceleration command limit, m/s² |
| `BPP_ACC_Z` | 10 | Vertical acceleration command limit, m/s² |
| `BPP_AMAX_XY` | 0.72 | Horizontal performance-center acceleration scale, m/s²; not the output limit |
| `BPP_AMAX_Z` | 0.9 | Vertical performance-center acceleration scale, m/s² |
| `BPP_BAND_XY` | 0.75 | Minimum horizontal corridor half-width, m |
| `BPP_BAND_Z` | 0.75 | Minimum vertical corridor half-width, m |
| `BPP_DELTA` | 0.1 | Corridor half-width as a fraction of the anchor error |
| `BPP_E0_MIN` | 0.05 | Floor on the anchor-error magnitude, m |
| `BPP_EPS_PRE` | 0.1 | Reaching-phase tolerance of the time-varying gain |
| `BPP_EPS_SLIDE` | 0.1 | Sliding-phase tolerance of the time-varying gain |
| `BPP_K0_XY` | 1.3 | Horizontal linear reaching gain |
| `BPP_K0_Z` | 1.9 | Vertical linear reaching gain |
| `BPP_K1_XY` | 0.25 | Horizontal tanh switching gain |
| `BPP_K1_Z` | 0.4 | Vertical tanh switching gain |
| `BPP_KS_XY` | 1.7 | Horizontal sliding-surface gain |
| `BPP_KS_Z` | 2 | Vertical sliding-surface gain |
| `BPP_PHI_XY` | 1.5 | Horizontal tanh boundary-layer scale |
| `BPP_PHI_Z` | 1.2 | Vertical tanh boundary-layer scale |
| `BPP_TRIG_RAMP` | 0.3 | Trigger fraction for the first vertical event during takeoff |
| `BPP_TRIG_XY` | 0.76 | Horizontal trigger as a fraction of the half-width |
| `BPP_TRIG_Z` | 0.95 | Vertical trigger as a fraction of the half-width |
| `BPP_T_PRE` | 2 | Reaching-time parameter, s |
| `BPP_T_SLIDE` | 2 | Sliding-time parameter, s |
| `F8_ENABLE` | 1 | Enable the onboard task; a change takes effect after reboot |
| `F8_ERR_MAX` | 3 | Three-dimensional error at which the task aborts, m |
| `F8_HEIGHT` | 0.8 | Height relative to the ground-preparation point, m |
| `F8_HOV_TIME` | 5 | Minimum hover before and after the task, s |
| `F8_LAPS` | 2 | Number of laps |
| `F8_LND_DEPTH` | 0.3 | How far the landing reference crosses the takeoff plane, m |
| `F8_LND_TIME` | 18 | Upper bound on the descent-reference ramp, s; the minimum terminal-speed logic also applies |
| `F8_PERIOD` | 25 | Duration of one lap, s |
| `F8_RADIUS` | 1.6 | Bernoulli-lemniscate scale a, m |
| `F8_TKO_TIME` | 12 | Smooth takeoff-reference duration, s |
| `MPC_CTRL_METHOD` | 1 | 0 native PID; 1 this position controller |
| `MPC_THR_HOVER` | 0.21 | Normalized hover-thrust mapping on the aircraft |

## When the values take effect

- `MPC_CTRL_METHOD` and `BPP_*` are applied while disarmed. Figure-eight task parameters are read during ground preparation; changing them in flight does not change the current task. `F8_ENABLE` takes effect after reboot.
- Firmware defaults still come from the frozen sources, usually PID, task disabled, and the original `BPP` initial values. The table is the sample that flew. The two sets are not interchangeable.
- `flight_profile.params` has 36 entries. All 1153 entries are in `all_parameters.json` and `all_parameters_REFERENCE_ONLY.params`. The latter includes calibration, board, power, and mode settings and is a source record only.
- The original log has no in-flight parameter changes, so one flight does not contain several `BPP` configurations to splice.
- Coordinates are local NED. Position error is measured minus reference. A positive z error is below the reference.
- A simulator run can reuse the same `BPP` and `F8` values. The X500 hover mapping is written only in the simulator start configuration and is recorded in `validation/sitl_result.json`. It is not written into the aircraft parameter file and it does not modify the frozen controller sources.

## Other recorded configuration

External position, height, and yaw fusion are `EKF2_EV_CTRL=11`, `EKF2_GPS_CTRL=0`, and `EKF2_HGT_REF=3`. External velocity is not fused. `SYS_AUTOSTART=4050` and `SYS_HITL=0`. The native inner-loop, calibration, and receiver values remain in `all_parameters.json`. They are not applied automatically as a portable configuration.
