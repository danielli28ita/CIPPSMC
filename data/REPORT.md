# Flight A: log_1

Source file: `log_1.ulg`. The bytes are unchanged. The SHA256 is in [PROVENANCE.json](PROVENANCE.json). The file name and the log's internal time label are not a certified wall clock. Times below are seconds from the ULog start.

The log lasts about 83.565 s, with `MPC_CTRL_METHOD=1`, `KS_XY=1.70`, and `KS_Z=2.00`. The path is 0.8 m high, with scale a = 1.6 m, 25 s per lap, and two laps. All 1153 initial parameters were exported. No parameter changes during the flight.

Flight B is a second real flight of the same onboard task. It is included in the README figures and is not part of the numbers in this report.

## Task and metrics

The task starts at 2.010 s, the figure-eight at 19.049 s, the return hover at 69.069 s, the descent at 74.069 s, touchdown detection at 81.614 s, the internal disarm command at 82.616 s, and task completion at 82.630 s. The log has no dropouts and no corridor-center resets.

| Figure-eight metric | Value |
|---|---:|
| Three-dimensional position RMSE | 0.10262372 m |
| Three-dimensional maximum error | 0.13943065 m |
| Horizontal position RMSE | 0.09590732 m |
| Mean error, x / y / z | -0.079265 / -0.024416 / -0.032456 m |
| Lap 1 / lap 2 three-dimensional RMSE | 0.102245 / 0.103001 m |

A negative mean z error means the vehicle is, on average, slightly above the reference. Stage, lap, attitude, and output statistics are in [analysis/summary.json](analysis/summary.json).

## Definitions

- Position is logged near 10 Hz and the reference near 5 Hz. The reference is linearly interpolated onto the position timestamps. Samples that were not simultaneous are not treated as exact pairs.
- The figure-eight window is `figure8_status` stage 5, using the previous status sample. Lap counts come from that message. Error is the EKF position minus the reference. RMSE is the square root of the mean squared error.
- The three-dimensional error is the Euclidean norm of the three axis errors. Descent-reference error after touchdown is not included in the figure-eight metrics.
- The errors are not an independent position truth. One flight does not support a statistical ranking.
- The sliding-term split applies only to this sample's zero-center window with no reset. It interpolates across topics and reconstructs the velocity error from the logged surface. It is not a system identification and not a closed-loop test of a candidate parameter set.

## Records that stay in the report

At about 46.655 s the log contains one 5 V avionics-low message, 4.79 V. The sparse power topic reaches about 4.817 V and does not contain that same sample. The task still finishes without a failsafe. The warning is not removed, and the log does not show that every power check was nominal. The `battery_status` connection and voltage fields are not a reliable propulsion-battery measurement. The external pose device, propulsion, and payload model are not in the log.

The Git hash in the log identifies the upstream baseline. It does not prove that the firmware rebuilt in this package is byte-identical to the binary that flew.

[Tracking plot](analysis/tracking.png) · [Horizontal path](analysis/trajectory.png) · [Sliding terms](analysis/sliding_terms.png) · [Expected metrics](expected_metrics.json)
