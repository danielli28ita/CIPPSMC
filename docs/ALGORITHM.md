# Algorithm and frozen sources

The controller entry point is `_applyBppsmcOverride()` in `firmware/overlay/src/modules/mc_pos_control/PositionControl/PositionControl.cpp`. The call site and parameter handling are in the neighboring `MulticopterPositionControl.cpp` and `.hpp`. Parameter definitions are in `bppsmc_params.c`.

## Signal path

```text
Onboard position, velocity, and acceleration reference
  -> stored pure reference
  -> native position and velocity PID
  -> acceleration override on each axis where the controller is active
  -> PX4 thrust mapping
  -> native attitude and rate loops
  -> control allocation
  -> actuators
```

In NED coordinates, e1 = p - p_ref and e2 = v - v_ref, with z positive down. The override replaces the outer-loop output on an active axis. It is not added on top of a continuing native PID integral. `MPC_CTRL_METHOD=1` selects this outer loop. Stabilized and Acro keep the native inner loops.

## Performance function, barrier, and sliding surface

After a reset, the center decays as a signed e0 times a squared cosine:

```text
Ts = pi*sqrt(e0/(2*a_max))
rho(tau) = direction*e0*cos^2(pi*tau/(2*Ts)), tau<Ts
rho = 0, tau>=Ts
b = max(BPP_DELTA*e0, BPP_BAND_axis)
zeta = clip((e1-rho+b)/(2*b), 1e-6, 1-1e-6)
epsilon = log(zeta/(1-zeta))
J = 1/(2*b*zeta*(1-zeta))
epsilon_dot = J*(e2-rho_dot)
S = epsilon_dot + lambda*epsilon
```

At initialization the center stays at zero. Center decay starts after the first accepted event. `a_max` scales the performance function. It is not the final output limit.

Bounded time-varying terms:

```text
gamma = tau/(eps_pre*T_pre^2 + tau^2)
tau_s = max(0, tau-T_pre)
eta = tau_s/(eps_slide*T_slide^2 + tau_s^2)
lambda = KS + eta
kappa = K0 + gamma
```

The law contains reference feedforward, the derivative of J, the derivative of the performance center, sliding-surface compensation, linear reaching, and K1*tanh(S/PHI). The acceleration is then limited by `BPP_ACC_XY` and `BPP_ACC_Z`. Thrust mapping remains PX4 `_accelerationControl()`.

## Events and numerics

An axis accepts a reset when the distance from the center exceeds the trigger fraction times the half-width, and the unsaturated corrective acceleration is finite, large enough, and directed inward. A new event is allowed only after the error has returned inside the trigger boundary. `E0_MIN` floors the anchor magnitude. The first vertical trigger during takeoff uses `TRIG_RAMP`.

The debug message is published after a reset. `rho`, `S`, and the output are the values computed on that cycle. `e0`, `t0`, and the counter may already have been updated. Those two times must not be mixed on an event sample. The packaged Flight A sample has no reset on any axis. The accompanying sliding-term split applies only to that zero-center window and rejects other event cases.

The ideal signum law in the paper differs from the tanh, acceleration limit, barrier clamp, and anchor floor in this package. The ideal finite-time and prescribed-performance conditions are not applied unchanged to the saturated, noisy flight implementation. Comparison-controller build dependencies from the original tree are retained. The default is `BASELINE_CONTROLLER=0`, and the build check verifies that `PositionControl::update()` calls this controller.

## Onboard task

`figure8_trajectory` publishes position, velocity, acceleration, and a fixed yaw reference for a Bernoulli lemniscate at 50 Hz over uORB, on the OFFBOARD path. The phase uses a quintic polynomial. Velocity and acceleration include the chain-rule terms, and each lap starts and ends at rest:

```text
u = elapsed/period
s = 10*u^3 - 15*u^4 + 6*u^5
theta = 2*pi*s - pi/2
north = n0 + a*cos(theta)/(1+sin^2(theta))
east = e0 + a*sin(theta)*cos(theta)/(1+sin^2(theta))
```

The scale `a` is `F8_RADIUS`. It is not a separate radius on each axis. The stages are takeoff, hover, figure-eight, return hover, landing, touchdown, and complete. The task state machine is included as frozen. Removed engineering extensions such as `BPP_SAFE_EN` and `BPP_JERK` are not included.

The onboard enable policy latches. Outside the task, after a manual takeover, or after a state timeout, the controller is disabled when the corresponding source condition holds, and the native path is used. When the onboard module has never been enabled, ordinary controller selection still applies. That fallback is not a claim about every OFFBOARD client.
