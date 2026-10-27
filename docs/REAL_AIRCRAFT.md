# Aircraft, external pose, and task operation

The flight evidence in this package is Flight A (`data/log_1.ulg`) and Flight B (`data/log_2.ulg`). Flight A records a complete two-lap task, landing, and a normal disarm. Building this package did not connect to a flight controller. The rebuilt firmware is not a new flight acceptance, and the historical flight binary is not certified byte for byte.

## Known configuration and physical items still missing

| Item | What the Flight A log establishes |
|---|---|
| Flight controller | PX4_FMU_V6C / Pixhawk 6C |
| PX4 baseline | v1.15.4, `99c40407ffd7ac184e2d7b4b293f36f10fe561ef` |
| Airframe parameter | `SYS_AUTOSTART=4050`, Generic 250 Racer; `SYS_HITL=0` |
| Controller | `MPC_CTRL_METHOD=1` |
| Reference | Onboard Bernoulli lemniscate, 0.8 m high, a = 1.6 m, 25 s/lap, 2 laps |
| Pose configuration | `EKF2_EV_CTRL=11`, `EKF2_GPS_CTRL=0`, `EKF2_HGT_REF=3`; external position, height, and yaw are fused, external velocity is not |
| External position interface | `vehicle_visual_odometry`; the message name does not identify a camera or a motion-capture device |
| Propulsion and mechanics | Motor model and KV, propeller, ESC and protocol, battery series and capacity, mass, and payload are not in the log |
| Pose hardware and publisher | Device model, external software, and the true sample and transport delay are not provided |
| Ground station and receiver | QGroundControl version and the physical receiver are not identified by the log |

Missing items do not block a source build, log recomputation, or the packaged simulator check. They do block a claim that the physical flight condition is fully reproducible. Record them from the actual equipment. Do not infer propulsion from the name Generic 250. The onboard trajectory does not need a companion trajectory publisher. External pose may still need its existing bridge.

## Firmware and configuration order

1. Remove all propellers during wiring, sensor calibration, and actuator tests. Back up the current parameters. Wire the Pixhawk 6C to the actual power system. Official reference: [Pixhawk 6C wiring](https://docs.px4.io/v1.15/en/assembly/quick_start_pixhawk6c).
2. In QGroundControl, select `artifacts/px4_fmu-v6c_onboard/px4_fmu-v6c_onboard.px4` as a custom firmware. It is the aircraft board target, not a HITL simulator image. Check `SYS_AUTOSTART=4050` and `SYS_HITL=0`.
3. Keep the sensor calibration, propulsion, output, battery, and receiver settings that belong to the actual vehicle. Do not use `all_parameters_REFERENCE_ONLY.params` as a full initialization file for another aircraft.
4. Read `params/flight_profile.params` before importing it. It contains only the `BPP` and `F8` values, the controller selection, and the hover mapping extracted from Flight A. `F8_ENABLE=1` enables the module after reboot. `MPC_CTRL_METHOD=1` selects this controller inside the task. The native inner loops and the remaining aircraft parameters still have to match the vehicle.
5. To repeat the sample's pose condition, restore the actual external-pose interface and check the NED/FRD frames, attitude sign, extrinsics, scale, valid timestamps, and update rate. The logged external-vision settings and the zero-delay parameters are recorded values, not a generic setup for an unknown pose system. Reference: [PX4 v1.15 external position](https://docs.px4.io/v1.15/en/ros/external_position_estimation).
6. Check the real RC, mode selection, arm and disarm, kill, RC-loss, and OFFBOARD-loss behavior. Switch channels come from the receiver. Do not copy the simulator's virtual channel map.

## Onboard task

Apply parameters while landed and disarmed, then reboot:

```text
figure8_trajectory status
mc_pos_control status
```

Wait until the task is ready. Arm by hand in a manual mode that has already been checked, then switch from the ground into OFFBOARD. After `Figure8 started: BPPSMC`, the task runs takeoff, hover, two laps, return hover, and descent. The module does not arm the vehicle and does not need the companion computer to request OFFBOARD again. Do not run a second trajectory publisher at the same time.

A normal ending is `Figure8 complete: landed and disarmed`, together with the controller disarmed and the motors stopped. Then return the arm switch to the disarmed position and leave OFFBOARD. With the enable policy latched, leaving OFFBOARD by hand aborts the task and returns to the native control path. The task does not restart in the air.

Kill stops the motors immediately. It is not a normal landing. Confirm what the throttle does in the takeover mode. This note describes the packaged implementation and the recorded samples. It does not replace the owner's procedure or a tuning session on an unknown vehicle.

## A record that stays with Flight A

At about 46.655 s, Flight A contains `Avionics Power low: 4.79 Volt`. The sparse `system_power` topic does not contain that same sample; its lowest value is about 4.817 V. The two topics are sampled at different rates. The warning is not deleted, and the log does not support a claim that the voltage was nominal for the whole flight. The task finishes afterward without a failsafe, and the log does not show that the warning caused the tracking error.

`battery_status` connection and voltage fields are not a reliable propulsion-battery measurement. A new flight record should add a trusted battery, power, and test-condition log. The same limits are stated in the sample report.

The simulator start script, its virtual RC map, and the X500 hover mapping used only in simulation must not be loaded onto the aircraft.
