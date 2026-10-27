# Reproduction checks

Date: 2026-10-26, Ubuntu 22.04 amd64. The environment is `environments/versions.json`. Python dependencies were installed in a new virtual environment, not through the author's NumPy compatibility path.

## What was run

| Check | Result | Evidence |
|---|---|---|
| Frozen sources | Exact v1.15.4 baseline, 32 dependency commits, 28 payload hashes | `firmware/BASELINE.json` |
| Sample and parameters | Flight A only for the metric file; raw SHA256 matches; 1153 parameters and 36 profile values exported | `data/PROVENANCE.json`, `params/` |
| Offline trajectory and state machine | C++ reference geometry, numerical derivatives, bounds, full stages, multiple laps, abort, and EKF shift | `validation/offline_tests.log` |
| Provenance and parameter tests | 5 tests passed, including JSON and QGroundControl text round trip and payload hashes | `validation/offline_tests.log` |
| Sample recomputation | 14 metrics within rtol 1e-6 and atol 1e-8 | `validation/sample_recompute.log`, `data/expected_metrics.json` |
| Aircraft target | `px4_fmu-v6c_onboard` built; default PID, task off; controller call and trajectory module present | `validation/px4_fmu-v6c_onboard.build.*` |
| Simulator target | `px4_sitl_default` built; branch check passed | `validation/px4_sitl_default.build.*` |
| Local simulated task | After the model hover mapping was set, two laps, landing, and a normal disarm completed | `validation/sitl_result.json`, `sitl_lifecycle.json` |

The build used a local Git object mirror and restored the pinned commit and payload in a new checkout. It did not copy build products or dirty dependency trees from the original development directory. The network fetch and the mirror fetch share the same baseline, dependency, and file checks. apt installation and a full network download were not repeated on a fresh Ubuntu machine.

## Simulator hover mapping

The first simulator run kept the original X500 start value `MPC_THR_HOVER=0.60`. The task sequence completed. The figure-eight RMSE was about **0.5086 m** and the maximum about **0.7647 m**, outside this package's engineering gate. That result is kept in `validation/sitl_native_mapping_result.json`.

After the model mismatch was identified, a second run set an X500-only `MPC_THR_HOVER=0.73` in the simulator start script and kept the Flight A `BPP` and `F8` parameters:

- Figure-eight RMSE **0.05983 m**, maximum **0.13236 m**.
- Two full laps, **9901** controller debug samples, landing, and a normal disarm.
- Inside this package's gate: RMSE below 0.15 m and maximum below 0.30 m.
- The simulator airframe is `SYS_AUTOSTART=4001` with GPS and simulated sensors, and `EV_CTRL=0`.
- Some `F8` defaults are constructed after the logger starts and are absent from the ULog initial set. The checker records which values were filled from the compiled parameter metadata. Those fills are not independent log observations.

The simulator-only value 0.73 is not written into the aircraft parameter file and does not modify the frozen controller sources. The two simulator ULogs from this round stay in the excluded workspace. The flight logs shipped here are Flight A and Flight B.

## Aircraft firmware image

Board id 56. Flash use 1,900,252 / 1,966,080 bytes (about 96.65%). The image includes the `.px4`, ELF, `parameters.xml`, `airframes.xml`, and the build manifest. Firmware defaults are `MPC_CTRL_METHOD=0` and `F8_ENABLE=0`. The sample parameters are a separate profile.

Flight A records a completed real task. This rebuild did not connect to or flash a flight controller. The rebuilt firmware is not proven byte-identical to the historical flight binary, and this round does not claim a new flight test.

## Not covered

- A new hardware flash, physical HITL, or another flight with different propulsion, battery, or payload.
- A device-level rebuild of the ROS, motion-capture, or external-pose publisher. Its model and source were not provided.
- A live RC, kill, and link-loss regression under `--extra-checks`. This round ran the default full simulator task. Parts of that state machine have offline tests.
- Multi-run disturbance statistics, the full paper experiment, performance tables for the other controllers, or byte-level identity of the firmware build.
- The physical cause of the Flight A 5 V low-voltage message. The record is kept and stated.

The candidate archive was unpacked in a separate directory, given a new Python virtual environment, and checked for integrity, offline tests, and sample recomputation. All of those checks passed; see `validation/release_checks.json`. The validation records were then added and the archive integrity was checked again.
