/****************************************************************************
 *
 *   Copyright (c) 2018 - 2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file PositionControl.hpp
 *
 * A cascaded position controller for position/velocity control only.
 */

#pragma once

#include <lib/mathlib/mathlib.h>
#include <matrix/matrix/math.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/bppsmc_debug.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include "etimr_gains.h"

struct PositionControlStates {
	matrix::Vector3f position;
	matrix::Vector3f velocity;
	matrix::Vector3f acceleration;
	float yaw;
};

struct BppsmcParameters {
	matrix::Vector3f ppf_acceleration{0.72f, 0.72f, 0.90f};
	matrix::Vector3f funnel_width{0.75f, 0.75f, 0.75f};
	matrix::Vector3f trigger_fraction{0.76f, 0.76f, 0.95f};
	matrix::Vector3f surface_gain{1.40f, 1.40f, 1.75f};
	matrix::Vector3f reaching_gain{1.20f, 1.20f, 1.80f};
	matrix::Vector3f switching_gain{0.20f, 0.20f, 0.35f};
	matrix::Vector3f boundary_layer{1.50f, 1.50f, 1.20f};
	matrix::Vector3f acceleration_limit{6.f, 6.f, 10.f};
	float funnel_fraction{0.10f};
	float reaching_time{2.f};
	float reaching_tolerance{0.1f};
	float sliding_time{2.f};
	float sliding_tolerance{0.1f};
	float minimum_error{0.05f};
	float authority_threshold{0.05f};
	float rampup_trigger{0.30f};
};

/**
 * 	Core Position-Control for MC.
 * 	This class contains P-controller for position and
 * 	PID-controller for velocity.
 * 	Inputs:
 * 		vehicle position/velocity/yaw
 * 		desired set-point position/velocity/thrust/yaw/yaw-speed
 * 		constraints that are stricter than global limits
 * 	Output
 * 		thrust vector and a yaw-setpoint
 *
 * 	If there is a position and a velocity set-point present, then
 * 	the velocity set-point is used as feed-forward. If feed-forward is
 * 	active, then the velocity component of the P-controller output has
 * 	priority over the feed-forward component.
 *
 * 	A setpoint that is NAN is considered as not set.
 * 	If there is a position/velocity- and thrust-setpoint present, then
 *  the thrust-setpoint is ommitted and recomputed from position-velocity-PID-loop.
 */
class PositionControl
{
public:

	PositionControl() = default;
	~PositionControl() = default;

	/**
	 * Set the position control gains
	 * @param P 3D vector of proportional gains for x,y,z axis
	 */
	void setPositionGains(const matrix::Vector3f &P) { _gain_pos_p = P; }

	/**
	 * Set the velocity control gains
	 * @param P 3D vector of proportional gains for x,y,z axis
	 * @param I 3D vector of integral gains
	 * @param D 3D vector of derivative gains
	 */
	void setVelocityGains(const matrix::Vector3f &P, const matrix::Vector3f &I, const matrix::Vector3f &D);

	/**
	 * Set the maximum velocity to execute with feed forward and position control
	 * @param vel_horizontal horizontal velocity limit
	 * @param vel_up upwards velocity limit
	 * @param vel_down downwards velocity limit
	 */
	void setVelocityLimits(const float vel_horizontal, const float vel_up, float vel_down);

	/**
	 * Set the minimum and maximum collective normalized thrust [0,1] that can be output by the controller
	 * @param min minimum thrust e.g. 0.1 or 0
	 * @param max maximum thrust e.g. 0.9 or 1
	 */
	void setThrustLimits(const float min, const float max);

	/**
	 * Set margin that is kept for horizontal control when prioritizing vertical thrust
	 * @param margin of normalized thrust that is kept for horizontal control e.g. 0.3
	 */
	void setHorizontalThrustMargin(const float margin);

	/**
	 * Set the maximum tilt angle in radians the output attitude is allowed to have
	 * @param tilt angle in radians from level orientation
	 */
	void setTiltLimit(const float tilt) { _lim_tilt = tilt; }

	/**
	 * Set the normalized hover thrust
	 * @param hover_thrust [HOVER_THRUST_MIN, HOVER_THRUST_MAX] with which the vehicle hovers not accelerating down or up with level orientation
	 */
	void setHoverThrust(const float hover_thrust) { _hover_thrust = math::constrain(hover_thrust, HOVER_THRUST_MIN, HOVER_THRUST_MAX); }

	/**
	 * Update the hover thrust without immediately affecting the output
	 * by adjusting the integrator. This prevents propagating the dynamics
	 * of the hover thrust signal directly to the output of the controller.
	 */
	void updateHoverThrust(const float hover_thrust_new);

	/**
	 * Pass the current vehicle state to the controller
	 * @param PositionControlStates structure
	 */
	void setState(const PositionControlStates &states);

	/**
	 * Pass the desired setpoints
	 * Note: NAN value means no feed forward/leave state uncontrolled if there's no higher order setpoint.
	 * @param setpoint setpoints including feed-forwards to execute in update()
	 */
	void setInputSetpoint(const trajectory_setpoint_s &setpoint);

	/**
	 * Apply P-position and PID-velocity controller that updates the member
	 * thrust, yaw- and yawspeed-setpoints.
	 * @see _thr_sp
	 * @see _yaw_sp
	 * @see _yawspeed_sp
	 * @param dt time in seconds since last iteration
	 * @return true if update succeeded and output setpoint is executable, false if not
	 */
	bool update(const float dt);

	/**
	 * Set the integral term in xy to 0.
	 * @see _vel_int
	 */
	void resetIntegral() { _vel_int.setZero(); }

	/**
	 * Enable/disable ET-BPPSMC acceleration override.
	 */
	void setBppsmcEnabled(bool enabled) { _bppsmc_enabled = enabled; }

	/** Apply validated parameters and reset the event memory while disarmed. */
	void setBppsmcParameters(const BppsmcParameters &parameters);

	/** True while takeoff rampup is active (Z uses tighter ET threshold). */
	void setBppsmcRampupActive(bool rampup) { _bppsmc_rampup_active = rampup; }

	/** Latch arm rising-edge time for ET segment alignment (since_arm plots). */
	void setBppsmcArmReferenceUs(uint64_t t_us) { _arm_ref_us = t_us; }

	/**
	 * Reset ET-BPPSMC internal state (pure setpoints + event-trigger memory).
	 * Call when leaving flight phase or after PX4 ground-hold failsafe setpoints.
	 */
	void resetBppsmcState();

	/**
	 * Set absolute acceleration limits (safety clamp) for BPPSMC output.
	 * Units: m/s^2 in NED frame.
	 */
	void setBppsmcAccelerationLimits(const matrix::Vector3f &acc_max) { _bppsmc_acc_max = acc_max; }

	/**
	 * If set, the tilt setpoint is computed by assuming no vertical acceleration
	 */
	void decoupleHorizontalAndVecticalAcceleration(bool val) { _decouple_horizontal_and_vertical_acceleration = val; }

	/**
	 * Get the controllers output local position setpoint
	 * These setpoints are the ones which were executed on including PID output and feed-forward.
	 * The acceleration or thrust setpoints can be used for attitude control.
	 * @param local_position_setpoint reference to struct to fill up
	 */
	void getLocalPositionSetpoint(vehicle_local_position_setpoint_s &local_position_setpoint) const;

	/**
	 * Get the controllers output attitude setpoint
	 * This attitude setpoint was generated from the resulting acceleration setpoint after position and velocity control.
	 * It needs to be executed by the attitude controller to achieve velocity and position tracking.
	 * @param attitude_setpoint reference to struct to fill up
	 */
	void getAttitudeSetpoint(vehicle_attitude_setpoint_s &attitude_setpoint) const;

	/**
	 * All setpoints are set to NAN (uncontrolled). Timestampt zero.
	 */
	static const trajectory_setpoint_s empty_trajectory_setpoint;

private:
	// The range limits of the hover thrust configuration/estimate
	static constexpr float HOVER_THRUST_MIN = 0.05f;
	static constexpr float HOVER_THRUST_MAX = 0.9f;

	bool _inputValid();

	/** True when at least one axis is eligible for ET-BPPSMC. */
	bool _bppsmcReady() const;

	bool _bppsmcAxisReady(int axis) const;
	bool _bppsmcAxisActive(int axis) const;

	void _positionControl(); ///< Position proportional control
	void _velocityControl(const float dt); ///< Velocity PID control
	void _accelerationControl(); ///< Acceleration setpoint processing

	/** Capture pure external setpoints before native P-loop contamination. */
	void _capturePureSetpoints();

	/** ET-BPPSMC: overwrite _acc_sp and re-run thrust mapping. */
	void _applyBppsmcOverride(const matrix::Vector3f &acc_sp_pid);
	void _applyEtAdrc(const matrix::Vector3f &acc_sp_pid, float dt);
	void _applyEtImr(const matrix::Vector3f &acc_sp_pid, float dt);
	void _applyFxtOf(const matrix::Vector3f &acc_sp_pid, float dt);
	void _resetBaselineState();

	/** Clear ET-BPPSMC persistent state (e.g. leaving valid tracking window). */
	void _resetBppsmcEtState();

	/** Clear captured pure setpoints (after failsafe / pre-takeoff sentinels). */
	void _resetBppsmcPureSetpoints();

	// Gains
	matrix::Vector3f _gain_pos_p; ///< Position control proportional gain
	matrix::Vector3f _gain_vel_p; ///< Velocity control proportional gain
	matrix::Vector3f _gain_vel_i; ///< Velocity control integral gain
	matrix::Vector3f _gain_vel_d; ///< Velocity control derivative gain

	// Limits
	float _lim_vel_horizontal{}; ///< Horizontal velocity limit with feed forward and position control
	float _lim_vel_up{}; ///< Upwards velocity limit with feed forward and position control
	float _lim_vel_down{}; ///< Downwards velocity limit with feed forward and position control
	float _lim_thr_min{}; ///< Minimum collective thrust allowed as output [-1,0] e.g. -0.9
	float _lim_thr_max{}; ///< Maximum collective thrust allowed as output [-1,0] e.g. -0.1
	float _lim_thr_xy_margin{}; ///< Margin to keep for horizontal control when saturating prioritized vertical thrust
	float _lim_tilt{}; ///< Maximum tilt from level the output attitude is allowed to have

	float _hover_thrust{}; ///< Thrust [HOVER_THRUST_MIN, HOVER_THRUST_MAX] with which the vehicle hovers not accelerating down or up with level orientation
	bool _decouple_horizontal_and_vertical_acceleration{true}; ///< Ignore vertical acceleration setpoint to remove its effect on the tilt setpoint

	// States
	matrix::Vector3f _pos; /**< current position */
	matrix::Vector3f _vel; /**< current velocity */
	matrix::Vector3f _vel_dot; /**< velocity derivative (replacement for acceleration estimate) */
	matrix::Vector3f _vel_int; /**< integral term of the velocity controller */
	float _yaw{}; /**< current heading */

	// Setpoints
	matrix::Vector3f _pos_sp; /**< desired position */
	matrix::Vector3f _vel_sp; /**< desired velocity */
	matrix::Vector3f _acc_sp; /**< desired acceleration */
	matrix::Vector3f _thr_sp; /**< desired thrust */
	float _yaw_sp{}; /**< desired heading */
	float _yawspeed_sp{}; /** desired yaw-speed */

	// --- ET-BPPSMC (firmware-embedded) ---
	bool _bppsmc_enabled{false};
	BppsmcParameters _bppsmc_parameters{};
	bool _bppsmc_rampup_active{false};
	uint64_t _arm_ref_us{0};
	bool _z_rampup_first_trigger_done{false};

	matrix::Vector3f _pd_pure{NAN, NAN, NAN};
	matrix::Vector3f _pd_pure_prev{NAN, NAN, NAN};
	matrix::Vector3f _vd_pure{NAN, NAN, NAN};
	matrix::Vector3f _ad_pure{NAN, NAN, NAN};

	matrix::Vector3f _bppsmc_acc_max{6.f, 6.f, 10.f};

	bool _et_init[3] {false, false, false};

	// ---- baseline variant states (ET-ADRC / ET-IMR / FxT-OF), reset via _resetBaselineState() ----
	bool _bl_init[3] {false, false, false};
	// ET-ADRC (Xu et al., AMC 449, 2023): cascade ESO + event-triggered sampling
	float _ea_alpha_hat[3][2] {};      // position-loop ESO states
	float _ea_beta_hat[3][2] {};       // velocity-loop ESO states
	float _ea_zeta[3] {};              // ET-held position error (pos loop)
	float _ea_eta[3] {};               // ET-held velocity error (vel loop)
	float _ea_theta_hat[3][5] {};      // FLS weights (5 Gaussian bases)
	float _ea_hbar_prev[3] {};         // previous pseudo-control (for hbar_dot)
	float _ea_hbar_dot[3] {};          // filtered derivative of pseudo-control
	uint32_t _ea_trig_pos[3] {};       // pos-loop transmission count
	uint32_t _ea_trig_vel[3] {};       // vel-loop transmission count
	// ET-IMR v2 (Zhao IM framework, PX4-adapted): measured (pe,ve) + xi[3]
	// u = -Kp pe - Kv ve - Kxi·xi; xi <- Phi_d xi + N_d y_held  (etimr_gains.h)
	float _imr_xi[3][3] {};
	float _imr_W[3] {};
	float _imr_y_held[3] {};
	uint32_t _imr_trig[3] {};
	bool _imr_sat[3] {false, false, false};
	// FxT-OF (Shao et al., ISA Trans. 110, 2021): FTESO + fixed-time controller
	float _fx_xp_hat[3] {};            // position estimate
	float _fx_xv_hat[3] {};            // velocity estimate (output feedback: no EKF vel used)
	float _fx_phi_hat[3] {};           // lumped disturbance estimate
	float _fx_f_prev[3] {};            // previous acceleration command (observer input)
	uint64_t _bl_t0_us[3] {};          // controller segment start (theta(t) switch)
	bool _ppf_hold_zero[3] {false, false, false};
	bool _trigger_armed[3] {true, true, true}; ///< ET re-arm latch (true = next event allowed)
	float _dir[3] {0.f, 0.f, 0.f};
	float _e0[3] {0.f, 0.f, 0.f};
	uint64_t _t0[3] {0, 0, 0};
	uint32_t _trigger_count[3] {0, 0, 0};
	bool _bppsmc_was_ready{false};

	uORB::Publication<bppsmc_debug_s> _bppsmc_debug_pub{ORB_ID(bppsmc_debug)};
};
