# Real-flight setup and parameters

This note records the platform, onboard task, controller selection, and the 36 algorithm/task values that flew on Flight A (`data/log_1.ulg`) and Flight B (`data/log_2.ulg`). Parameter names `BPP_*` and the topic `bppsmc_debug` are the names stored in the ULogs; they are kept as logged.

Every numeric value below cites a source key or ULog field. Quantities that are not in the logs are marked **unknown**. This is a single completed real-flight record (two logs of the same task on one airframe). It is not a multi-flight statistical claim and not an independent position truth: position error is estimator minus reference.

Offline recompute for Flight A:

```bash
python3 scripts/analyze_sample.py --verify
```

Tolerances are relative `1e-6` and absolute `1e-8` ([data/expected_metrics.json](../data/expected_metrics.json)).

## 1. Platform

| Item | Value | Source |
|---|---|---|
| Airframe class | Generic 250 Racer class quadrotor | ULog boot line `Loading airframe: /etc/init.d/airframes/4050_generic_250`; `SYS_AUTOSTART` in `params/all_parameters.json` |
| Airframe parameter | `SYS_AUTOSTART=4050`, `SYS_HITL=0` | `params/all_parameters.json` keys `SYS_AUTOSTART`, `SYS_HITL` |
| Flight controller | Pixhawk 6C (`PX4_FMU_V6C`) | ULog info `ver_hw`; also `data/PROVENANCE.json` → `firmware_metadata.ver_hw` |
| Autopilot | PX4 v1.15.4, baseline `99c40407ffd7ac184e2d7b4b293f36f10fe561ef` | ULog info `ver_sw`; boot line `PX4 version: 1.15.4`; `data/PROVENANCE.json` → `firmware_metadata.ver_sw` |
| Battery configuration | 4S (`BAT1_N_CELLS=4`) | `params/all_parameters.json` key `BAT1_N_CELLS`. Propulsion-battery voltage/current in `battery_status` are not a trusted measurement in these logs |
| Position sensing | External vision fused; no GPS fusion | `EKF2_EV_CTRL=11`, `EKF2_HGT_REF=3`, `EKF2_GPS_CTRL=0` in `params/all_parameters.json`; topic `vehicle_visual_odometry` present |
| External pose device / publisher | **unknown** | Message name `vehicle_visual_odometry` does not identify the camera or motion-capture stack |
| Motor, propeller, ESC, mass, payload | **unknown** | Not present in the ULogs |
| Board identity (both flights) | Same `sys_uuid` | ULog info `sys_uuid` = `000600000000383236313533511200350027` on Flight A and Flight B |

## 2. Onboard task

| Item | Value | Source |
|---|---|---|
| Takeoff reference ramp | `F8_TKO_TIME=12` s | `params/flight_profile.params` / `params/all_parameters.json` key `F8_TKO_TIME` |
| Pre- and post-task hover | `F8_HOV_TIME=5` s | key `F8_HOV_TIME` |
| Path | Bernoulli lemniscate (figure-eight), 2 laps | keys `F8_LAPS=2`; module messages and `figure8_status.stage` / `.lap` |
| Path scale | `F8_RADIUS=1.6` m (parameter *a*) | key `F8_RADIUS` |
| Lap period | `F8_PERIOD=25` s | key `F8_PERIOD` |
| Height above preparation point | `F8_HEIGHT=0.8` m | key `F8_HEIGHT` |
| Sequence | Takeoff ramp → hover → two laps → return hover → descent | `figure8_status.stage` and logged `[figure8_trajectory]` messages; see [data/REPORT.md](../data/REPORT.md) for Flight A stage times |
| Flight A log duration | 83.565173 s | ULog `(last_timestamp − start_timestamp)`; `data/expected_metrics.json` → `metrics.duration_s` |
| Flight B log duration | 82.843241 s (≈82.8 s) | ULog `(last_timestamp − start_timestamp)` on `data/log_2.ulg` |
| Same vehicle | Identical `sys_uuid` on A and B | ULog info `sys_uuid` (both logs) |

## 3. Controller configuration

| Item | Value | Source |
|---|---|---|
| Position-controller select | `MPC_CTRL_METHOD=1` | `params/flight_profile.params` / `params/all_parameters.json`; also `figure8_status.controller` unique value `1` |
| Sliding-surface gains | `BPP_KS_XY=1.70`, `BPP_KS_Z=2.00` (report shorthand KS_XY / KS_Z) | keys `BPP_KS_XY`, `BPP_KS_Z` |
| Hover thrust mapping | `MPC_THR_HOVER=0.21` | key `MPC_THR_HOVER` |
| Debug topic | `bppsmc_debug` | ULog topic name (fields include `acceleration_sp[*]`, `rho_center[*]`, `e0[*]`, `sliding_surface[*]`) |
| Parameter name prefix | `BPP_*` | Logged parameter names; not renamed in this package |

## 4. Parameter table (Flight A profile, 36 entries)

Values are the Flight A initial set. Display digits match the rounded table in [docs/PARAMETERS.md](PARAMETERS.md); the QGroundControl text and JSON keep the exact float32 bit patterns. Every row cites the parameter key in `params/flight_profile.params` and the same key in `params/all_parameters.json`.

### 4.1 Control gains

| Name | Value | Unit | Role | Source |
|---|---:|---|---|---|
| `BPP_KS_XY` | 1.7 | 1/s | Horizontal sliding-surface gain | `params/flight_profile.params` / `all_parameters.json` → `BPP_KS_XY` |
| `BPP_KS_Z` | 2 | 1/s | Vertical sliding-surface gain | → `BPP_KS_Z` |
| `BPP_K0_XY` | 1.3 | 1/s | Horizontal linear reaching gain | → `BPP_K0_XY` |
| `BPP_K0_Z` | 1.9 | 1/s | Vertical linear reaching gain | → `BPP_K0_Z` |
| `BPP_K1_XY` | 0.25 | 1/s² | Horizontal tanh switching gain | → `BPP_K1_XY` |
| `BPP_K1_Z` | 0.4 | 1/s² | Vertical tanh switching gain | → `BPP_K1_Z` |
| `BPP_PHI_XY` | 1.5 | — | Horizontal tanh boundary-layer scale | → `BPP_PHI_XY` |
| `BPP_PHI_Z` | 1.2 | — | Vertical tanh boundary-layer scale | → `BPP_PHI_Z` |
| `BPP_EPS_PRE` | 0.1 | — | Reaching-phase tolerance of the time-varying gain | → `BPP_EPS_PRE` |
| `BPP_EPS_SLIDE` | 0.1 | — | Sliding-phase tolerance of the time-varying gain | → `BPP_EPS_SLIDE` |
| `BPP_T_PRE` | 2 | s | Reaching-time parameter | → `BPP_T_PRE` |
| `BPP_T_SLIDE` | 2 | s | Sliding-time parameter | → `BPP_T_SLIDE` |
| `BPP_TRIG_RAMP` | 0.3 | — | Trigger fraction for the first vertical event during takeoff | → `BPP_TRIG_RAMP` |
| `BPP_TRIG_XY` | 0.76 | — | Horizontal trigger as a fraction of the half-width | → `BPP_TRIG_XY` |
| `BPP_TRIG_Z` | 0.95 | — | Vertical trigger as a fraction of the half-width | → `BPP_TRIG_Z` |
| `BPP_DELTA` | 0.1 | — | Corridor half-width as a fraction of the anchor error | → `BPP_DELTA` |
| `BPP_E0_MIN` | 0.05 | m | Floor on the anchor-error magnitude | → `BPP_E0_MIN` |
| `BPP_ACC_FLOOR` | 0.05 | m/s² | Minimum corrective acceleration required for an event | → `BPP_ACC_FLOOR` |
| `BPP_AMAX_XY` | 0.72 | m/s² | Horizontal performance-center acceleration scale (not the output limit) | → `BPP_AMAX_XY` |
| `BPP_AMAX_Z` | 0.9 | m/s² | Vertical performance-center acceleration scale | → `BPP_AMAX_Z` |

### 4.2 Limits

| Name | Value | Unit | Role | Source |
|---|---:|---|---|---|
| `BPP_ACC_XY` | 6 | m/s² | Per-horizontal-axis acceleration command limit | → `BPP_ACC_XY` |
| `BPP_ACC_Z` | 10 | m/s² | Vertical acceleration command limit | → `BPP_ACC_Z` |
| `BPP_BAND_XY` | 0.75 | m | Minimum horizontal corridor half-width | → `BPP_BAND_XY` |
| `BPP_BAND_Z` | 0.75 | m | Minimum vertical corridor half-width | → `BPP_BAND_Z` |

### 4.3 Task and reference

| Name | Value | Unit | Role | Source |
|---|---:|---|---|---|
| `MPC_CTRL_METHOD` | 1 | — | `0` native PID; `1` this position controller | → `MPC_CTRL_METHOD` |
| `MPC_THR_HOVER` | 0.21 | — | Normalized hover-thrust mapping on the aircraft | → `MPC_THR_HOVER` |
| `F8_ENABLE` | 1 | — | Enable the onboard task (takes effect after reboot) | → `F8_ENABLE` |
| `F8_ERR_MAX` | 3 | m | Three-dimensional error at which the task aborts | → `F8_ERR_MAX` |
| `F8_HEIGHT` | 0.8 | m | Height relative to the ground-preparation point | → `F8_HEIGHT` |
| `F8_HOV_TIME` | 5 | s | Minimum hover before and after the figure-eight | → `F8_HOV_TIME` |
| `F8_LAPS` | 2 | — | Number of laps | → `F8_LAPS` |
| `F8_LND_DEPTH` | 0.3 | m | How far the landing reference crosses the takeoff plane | → `F8_LND_DEPTH` |
| `F8_LND_TIME` | 18 | s | Upper bound on the descent-reference ramp | → `F8_LND_TIME` |
| `F8_PERIOD` | 25 | s | Duration of one lap | → `F8_PERIOD` |
| `F8_RADIUS` | 1.6 | m | Bernoulli-lemniscate scale *a* | → `F8_RADIUS` |
| `F8_TKO_TIME` | 12 | s | Smooth takeoff-reference duration | → `F8_TKO_TIME` |

Simulator-only hover mapping and STEP/SITL airframe values are not mixed into this table. See [VALIDATION.md](../VALIDATION.md) and [docs/REAL_AIRCRAFT.md](REAL_AIRCRAFT.md) for the aircraft procedure and reproduction boundary.
