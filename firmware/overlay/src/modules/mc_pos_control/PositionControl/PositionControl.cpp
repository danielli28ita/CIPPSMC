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
 * @file PositionControl.cpp
 */

#include "PositionControl.hpp"
#include "ControlMath.hpp"
#include <float.h>
#include <mathlib/mathlib.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/time.h>
#include <geo/geo.h>

using namespace matrix;

const trajectory_setpoint_s PositionControl::empty_trajectory_setpoint = {0, {NAN, NAN, NAN}, {NAN, NAN, NAN}, {NAN, NAN, NAN}, {NAN, NAN, NAN}, NAN, NAN};

void PositionControl::setVelocityGains(const Vector3f &P, const Vector3f &I, const Vector3f &D)
{
	_gain_vel_p = P;
	_gain_vel_i = I;
	_gain_vel_d = D;
}

void PositionControl::setVelocityLimits(const float vel_horizontal, const float vel_up, const float vel_down)
{
	_lim_vel_horizontal = vel_horizontal;
	_lim_vel_up = vel_up;
	_lim_vel_down = vel_down;
}

void PositionControl::setThrustLimits(const float min, const float max)
{
	// make sure there's always enough thrust vector length to infer the attitude
	_lim_thr_min = math::max(min, 10e-4f);
	_lim_thr_max = max;
}

void PositionControl::setHorizontalThrustMargin(const float margin)
{
	_lim_thr_xy_margin = margin;
}

// BASELINE_CONTROLLER: 0 = ET-BPPSMC (proposed), 1 = ET-ADRC (Xu et al., AMC 449:127967, 2023),
// 2 = FxT-OF (Shao, Tian & Yang, ISA Trans. 110:213-224, 2021),
// 3 = ET-IMR (Zhao et al., IJRNC 35:4669-4685, 2025; internal-model output regulator).
// Patched by flights/baselines/apply_baseline_variant.py; all variants share setpoint capture,
// ready gating, acceleration limits and the hover-thrust mapping - only the acc_sp law differs.
#ifndef BASELINE_CONTROLLER
#define BASELINE_CONTROLLER 0
#endif

void PositionControl::updateHoverThrust(const float hover_thrust_new)
{
	// Given that the equation for thrust is T = a_sp * Th / g - Th
	// with a_sp = desired acceleration, Th = hover thrust and g = gravity constant,
	// we want to find the acceleration that needs to be added to the integrator in order obtain
	// the same thrust after replacing the current hover thrust by the new one.
	// T' = T => a_sp' * Th' / g - Th' = a_sp * Th / g - Th
	// so a_sp' = (a_sp - g) * Th / Th' + g
	// we can then add a_sp' - a_sp to the current integrator to absorb the effect of changing Th by Th'
	const float previous_hover_thrust = _hover_thrust;
	setHoverThrust(hover_thrust_new);

	_vel_int(2) += (_acc_sp(2) - CONSTANTS_ONE_G) * previous_hover_thrust / _hover_thrust
		       + CONSTANTS_ONE_G - _acc_sp(2);
}

void PositionControl::setState(const PositionControlStates &states)
{
	_pos = states.position;
	_vel = states.velocity;
	_yaw = states.yaw;
	_vel_dot = states.acceleration;
}

void PositionControl::setInputSetpoint(const trajectory_setpoint_s &setpoint)
{
	_pos_sp = Vector3f(setpoint.position);
	_vel_sp = Vector3f(setpoint.velocity);
	_acc_sp = Vector3f(setpoint.acceleration);
	_yaw_sp = setpoint.yaw;
	_yawspeed_sp = setpoint.yawspeed;
}

void PositionControl::_capturePureSetpoints()
{
	// PX4 pre-takeoff / ground-hold uses acc_z = 100 m/s^2 as a downward sentinel — never
	// treat it as trajectory feedforward (would saturate BPPSMC z output at +10 and kill thrust).
	static constexpr float ACC_FF_ABS_MAX = 15.f;

	for (int i = 0; i < 3; i++) {
		if (PX4_ISFINITE(_pos_sp(i))) {
			_pd_pure(i) = _pos_sp(i);

		} else if (!PX4_ISFINITE(_pd_pure(i))) {
			_pd_pure(i) = PX4_ISFINITE(_pos(i)) ? _pos(i) : 0.f;
		}

		if (PX4_ISFINITE(_vel_sp(i))) {
			_vd_pure(i) = _vel_sp(i);

		} else if (!PX4_ISFINITE(_vd_pure(i))) {
			_vd_pure(i) = 0.f;
		}

		if (PX4_ISFINITE(_acc_sp(i)) && fabsf(_acc_sp(i)) <= ACC_FF_ABS_MAX) {
			_ad_pure(i) = _acc_sp(i);

		} else {
			_ad_pure(i) = 0.f;
		}
	}
}

void PositionControl::_resetBppsmcEtState()
{
	for (int i = 0; i < 3; i++) {
		_et_init[i] = false;
		_ppf_hold_zero[i] = false;
		_dir[i] = 0.f;
		_e0[i] = 0.f;
		_t0[i] = 0;
		_trigger_count[i] = 0;
		_trigger_armed[i] = true;
	}
	_z_rampup_first_trigger_done = false;
}

void PositionControl::_resetBppsmcPureSetpoints()
{
	_pd_pure.setNaN();
	_vd_pure.setNaN();
	_ad_pure.setNaN();
}

void PositionControl::resetBppsmcState()
{
	_resetBppsmcEtState();
	_resetBaselineState();
	_resetBppsmcPureSetpoints();
	_bppsmc_was_ready = false;
	_pd_pure_prev.setNaN();
}

void PositionControl::setBppsmcParameters(const BppsmcParameters &parameters)
{
	const BppsmcParameters defaults{};
	_bppsmc_parameters = parameters;
	auto constrain_parameter = [](float value, float fallback, float minimum, float maximum) {
		return PX4_ISFINITE(value) ? math::constrain(value, minimum, maximum) : fallback;
	};

	for (int i = 0; i < 3; i++) {
		_bppsmc_parameters.ppf_acceleration(i) = constrain_parameter(parameters.ppf_acceleration(i), defaults.ppf_acceleration(i), 0.05f, 5.f);
		_bppsmc_parameters.funnel_width(i) = constrain_parameter(parameters.funnel_width(i), defaults.funnel_width(i), 0.05f, 10.f);
		_bppsmc_parameters.trigger_fraction(i) = constrain_parameter(parameters.trigger_fraction(i), defaults.trigger_fraction(i), 0.05f, 0.99f);
		_bppsmc_parameters.surface_gain(i) = constrain_parameter(parameters.surface_gain(i), defaults.surface_gain(i), 0.05f, 10.f);
		_bppsmc_parameters.reaching_gain(i) = constrain_parameter(parameters.reaching_gain(i), defaults.reaching_gain(i), 0.05f, 10.f);
		_bppsmc_parameters.switching_gain(i) = constrain_parameter(parameters.switching_gain(i), defaults.switching_gain(i), 0.f, 5.f);
		_bppsmc_parameters.boundary_layer(i) = constrain_parameter(parameters.boundary_layer(i), defaults.boundary_layer(i), 0.05f, 10.f);
		_bppsmc_parameters.acceleration_limit(i) = constrain_parameter(parameters.acceleration_limit(i), defaults.acceleration_limit(i), 0.1f, 15.f);
	}

	_bppsmc_parameters.funnel_fraction = constrain_parameter(parameters.funnel_fraction, defaults.funnel_fraction, 0.01f, 1.f);
	_bppsmc_parameters.reaching_time = constrain_parameter(parameters.reaching_time, defaults.reaching_time, 0.1f, 20.f);
	_bppsmc_parameters.reaching_tolerance = constrain_parameter(parameters.reaching_tolerance, defaults.reaching_tolerance, 0.001f, 1.f);
	_bppsmc_parameters.sliding_time = constrain_parameter(parameters.sliding_time, defaults.sliding_time, 0.1f, 20.f);
	_bppsmc_parameters.sliding_tolerance = constrain_parameter(parameters.sliding_tolerance, defaults.sliding_tolerance, 0.001f, 1.f);
	_bppsmc_parameters.minimum_error = constrain_parameter(parameters.minimum_error, defaults.minimum_error, 0.001f, 1.f);
	_bppsmc_parameters.authority_threshold = constrain_parameter(parameters.authority_threshold, defaults.authority_threshold, 0.001f, 1.f);
	_bppsmc_parameters.rampup_trigger = constrain_parameter(parameters.rampup_trigger, defaults.rampup_trigger, 0.05f, 0.99f);
	_bppsmc_acc_max = _bppsmc_parameters.acceleration_limit;
	resetBppsmcState();
}

bool PositionControl::_bppsmcAxisReady(int axis) const
{
	return PX4_ISFINITE(_pos(axis)) && PX4_ISFINITE(_vel(axis)) && PX4_ISFINITE(_vel_dot(axis))
	       && (PX4_ISFINITE(_pos_sp(axis)) || PX4_ISFINITE(_vel_sp(axis)));
}

bool PositionControl::_bppsmcAxisActive(int axis) const
{
	static constexpr float VEL_EPS = 0.05f;

	if (!_bppsmcAxisReady(axis)) {
		return false;
	}

	if (PX4_ISFINITE(_pos_sp(axis))) {
		return true;
	}

	return PX4_ISFINITE(_vd_pure(axis)) && (fabsf(_vd_pure(axis)) > VEL_EPS);
}

bool PositionControl::_bppsmcReady() const
{
	if (!_bppsmc_enabled) {
		return false;
	}

	for (int i = 0; i < 3; i++) {
		if (_bppsmcAxisActive(i)) {
			return true;
		}
	}

	return false;
}

void PositionControl::_applyBppsmcOverride(const Vector3f &acc_sp_pid)
{
	const uint64_t now_us = hrt_absolute_time();
	const Vector3f e1 = _pos - _pd_pure;
	const Vector3f e2 = _vel - _vd_pure;
	Vector3f acc_sp_bppsmc{};

	bppsmc_debug_s debug_msg{};
	debug_msg.timestamp = now_us;

	// ET-BPPSMC parameters (aligned with the paper's controller definition).
	//  - PPF: Brachistochrone cos², T_s = pi*sqrt(e0/(2*a_max))
	//  - Surface: S = eps_dot + lambda_i(tau_s)*eps, lambda = k_s + eta(tau_s)
	//  - Reaching: kappa = k0 + gamma(tau), gamma = tau/(eps_tol*T_preset^2 + tau^2)
	//  - Switching: k1*tanh(S/phi) (tex Remark: practical boundary-layer form)
	const Vector3f &a_max = _bppsmc_parameters.ppf_acceleration;
	// Funnel half-width band = max(delta_e0_frac*e0, delta_abs). The delta_e0_frac*e0 term is the
	// paper's Delta_rho = 2*delta*|e1(0)|; delta_abs is an absolute floor so the barrier never
	// collapses below the residual figure-8 tracking error (regulation funnel -> tracking funnel).
	const float delta_e0_frac = _bppsmc_parameters.funnel_fraction;
	const Vector3f &delta_abs = _bppsmc_parameters.funnel_width;
	// Event fires when |e1-rho| > trig_frac*band (a fraction of the funnel BOUNDARY = band).
	// NOTE: tested trig_frac=0.95 ("fire only at the boundary") and it BACKFIRED — delaying the
	// funnel re-centering lets the figure-8 error drift off the stale center, so tracking got
	// worse (rms_xy 0.18 -> 0.68) AND triggers increased. 0.76/0.90 keeps the funnel centered on
	// the moving target (tight tracking) while still firing near the boundary. 3 channels independent.
	const Vector3f &trig_frac = _bppsmc_parameters.trigger_fraction;
	const Vector3f &k_s = _bppsmc_parameters.surface_gain;
	const Vector3f &k_0 = _bppsmc_parameters.reaching_gain;
	const Vector3f &k_1 = _bppsmc_parameters.switching_gain;
	const Vector3f &phi_param = _bppsmc_parameters.boundary_layer;
	const float T_preset = _bppsmc_parameters.reaching_time;
	const float epsilon_tol = _bppsmc_parameters.reaching_tolerance;
	const float T_slide = _bppsmc_parameters.sliding_time;
	const float epsilon_s = _bppsmc_parameters.sliding_tolerance;
	const float E0_min = _bppsmc_parameters.minimum_error;
	const float acc_floor = _bppsmc_parameters.authority_threshold;
	const float trig_frac_z_rampup = _bppsmc_parameters.rampup_trigger;

	for (int i = 0; i < 3; i++) {
		if (!_bppsmcAxisActive(i)) {
			acc_sp_bppsmc(i) = acc_sp_pid(i);
			debug_msg.sliding_surface[i] = NAN;
			debug_msg.rho_center[i] = NAN;
			debug_msg.acceleration_sp[i] = acc_sp_bppsmc(i);
			debug_msg.e0[i] = _e0[i];
			debug_msg.t0[i] = _t0[i];
			debug_msg.trigger_count[i] = _trigger_count[i];
			continue;
		}

		if (!PX4_ISFINITE(e1(i)) || !PX4_ISFINITE(e2(i))) {
			acc_sp_bppsmc(i) = _ad_pure(i);
			debug_msg.sliding_surface[i] = NAN;
			debug_msg.rho_center[i] = NAN;
			debug_msg.acceleration_sp[i] = acc_sp_bppsmc(i);
			debug_msg.e0[i] = _e0[i];
			debug_msg.t0[i] = _t0[i];
			debug_msg.trigger_count[i] = _trigger_count[i];
			continue;
		}

		if (!_et_init[i]) {
			_et_init[i] = true;
			_dir[i] = (e1(i) >= 0.f) ? 1.f : -1.f;
			_e0[i] = fmaxf(fabsf(e1(i)), E0_min);
			_t0[i] = (_arm_ref_us > 0) ? _arm_ref_us : now_us;
			_ppf_hold_zero[i] = true;
		}

		const float tau = (now_us - _t0[i]) * 1e-6f;

		float rho_center = 0.f;
		float rho_center_dot = 0.f;
		float rho_center_ddot = 0.f;

		if (!_ppf_hold_zero[i]) {
			// PPF_ANALYTIC_BEGIN
		const float Ts = M_PI_F * sqrtf(_e0[i] / (2.f * a_max(i)));
		float rho_norm = 0.f;
		float rho_norm_dot = 0.f;
		float rho_norm_ddot = 0.f;

		if (tau < Ts) {
			const float phase = M_PI_F * tau / (2.f * Ts);
			const float c_val = cosf(phase);
			const float s_val = sinf(phase);
			rho_norm = c_val * c_val;
			rho_norm_dot = -(M_PI_F / Ts) * c_val * s_val;
			rho_norm_ddot = -(M_PI_F * M_PI_F / (2.f * Ts * Ts)) * (c_val * c_val - s_val * s_val);
		}
		// PPF_ANALYTIC_END

			rho_center = _dir[i] * _e0[i] * rho_norm;
			rho_center_dot = _dir[i] * _e0[i] * rho_norm_dot;
			rho_center_ddot = _dir[i] * _e0[i] * rho_norm_ddot;
		}

		// Funnel half-width = max(delta*e0, delta_abs); Delta_rho = 2*band. Transform eps = ln(Z/(1-Z)).
		const float band = fmaxf(delta_e0_frac * _e0[i], delta_abs(i));
		float Z = (e1(i) - rho_center + band) / (2.f * band);
		Z = math::constrain(Z, 1e-6f, 1.f - 1e-6f);
		const float epsilon = logf(Z / (1.f - Z));

		const float Jp = 1.f / (2.f * band * Z * (1.f - Z));
		const float Z_dot = (e2(i) - rho_center_dot) / (2.f * band);
		const float Jp_dot = -Jp * Jp * 2.f * band * Z_dot * (1.f - 2.f * Z);
		const float eps_dot = Jp * (e2(i) - rho_center_dot);

		// Time-varying sliding surface (tex eq:surface, eq:lambda_tv, eq:tau_s)
		const float tau_s = fmaxf(0.f, tau - T_preset);
		const float tau_s_active = (tau > T_preset) ? 1.f : 0.f;
		const float slide_denom = epsilon_s * T_slide * T_slide + tau_s * tau_s;
		const float eta = (slide_denom > FLT_EPSILON) ? (tau_s / slide_denom) : 0.f;
		const float lambda_tv = k_s(i) + eta;
		const float eta_dot = (slide_denom > FLT_EPSILON)
					? ((epsilon_s * T_slide * T_slide - tau_s * tau_s) / (slide_denom * slide_denom))
					: 0.f;
		const float lambda_dot_ts_eps = eta_dot * tau_s_active * epsilon;

		const float S = eps_dot + lambda_tv * epsilon;

		if (!PX4_ISFINITE(S) || !PX4_ISFINITE(Jp) || fabsf(Jp) < FLT_EPSILON) {
			acc_sp_bppsmc(i) = _ad_pure(i);
			debug_msg.sliding_surface[i] = NAN;
			debug_msg.rho_center[i] = rho_center;
			debug_msg.acceleration_sp[i] = acc_sp_bppsmc(i);
			debug_msg.e0[i] = _e0[i];
			debug_msg.t0[i] = _t0[i];
			debug_msg.trigger_count[i] = _trigger_count[i];
			continue;
		}

		const float gamma_denom = epsilon_tol * T_preset * T_preset + tau * tau;
		const float gamma_t = (gamma_denom > FLT_EPSILON) ? (tau / gamma_denom) : 0.f;
		const float kappa = gamma_t + k_0(i);
		const float sat_S = tanhf(S / phi_param(i));

		const float nu_full = Jp_dot * (e2(i) - rho_center_dot) - Jp * rho_center_ddot
				      + lambda_tv * eps_dot + lambda_dot_ts_eps
				      + kappa * S + k_1(i) * sat_S;

		float raw_acc = _ad_pure(i) - nu_full / Jp;
		const float acc_limit = _bppsmc_acc_max(i);
		acc_sp_bppsmc(i) = math::constrain(raw_acc, -acc_limit, acc_limit);

		if (!PX4_ISFINITE(acc_sp_bppsmc(i))) {
			acc_sp_bppsmc(i) = 0.f;
		}

		// --- Event trigger: error leaves the funnel boundary AND a correct control input exists. ---
		// "Correct control input" = a finite corrective acceleration (raw_acc - a_ff) that opposes the
		// boundary error, i.e. genuine authority pushing the error back inside the tube. Without it the
		// event is suppressed, which removes the need for any minimum inter-event time (MIET).
		const float err_diff = fabsf(e1(i) - rho_center);
		float trig_frac_eff = trig_frac(i);

		if (_bppsmc_rampup_active && i == 2 && !_z_rampup_first_trigger_done) {
			trig_frac_eff = trig_frac_z_rampup;
		}

		const float trig_thresh = trig_frac_eff * band;
		const float corr_acc = raw_acc - _ad_pure(i);
		const float err_center = e1(i) - rho_center;
		const bool ctrl_correct = PX4_ISFINITE(raw_acc)
					  && (fabsf(corr_acc) > acc_floor)
					  && (corr_acc * err_center < 0.f);

		// Re-arm only after the error has returned inside the boundary (replaces MIET debounce).
		if (err_diff <= trig_thresh) {
			_trigger_armed[i] = true;
		}

		if (err_diff > trig_thresh && ctrl_correct && _trigger_armed[i]) {
			_trigger_count[i]++;
			_t0[i] = now_us;
			_dir[i] = (e1(i) >= 0.f) ? 1.f : -1.f;
			_e0[i] = fmaxf(fabsf(e1(i)), E0_min);
			_ppf_hold_zero[i] = false;
			_trigger_armed[i] = false;

			if (i == 2 && _bppsmc_rampup_active) {
				_z_rampup_first_trigger_done = true;
			}
		}

		debug_msg.rho_center[i] = rho_center;
		debug_msg.sliding_surface[i] = S;
		debug_msg.trigger_count[i] = _trigger_count[i];
		debug_msg.acceleration_sp[i] = acc_sp_bppsmc(i);
		debug_msg.e0[i] = _e0[i];
		debug_msg.t0[i] = _t0[i];
	}

	_acc_sp = acc_sp_bppsmc;
	// Maps acc_sp to thrust; hover_thrust is applied here (acc_sp.z=0 -> hover).
	_accelerationControl();
	_bppsmc_debug_pub.publish(debug_msg);
	_pd_pure_prev = _pd_pure;
}


void PositionControl::_resetBaselineState()
{
	for (int i = 0; i < 3; i++) {
		_bl_init[i] = false;
		_ea_alpha_hat[i][0] = _ea_alpha_hat[i][1] = 0.f;
		_ea_beta_hat[i][0] = _ea_beta_hat[i][1] = 0.f;
		_ea_zeta[i] = _ea_eta[i] = 0.f;

		for (int k = 0; k < 5; k++) { _ea_theta_hat[i][k] = 0.f; }

		_ea_hbar_prev[i] = _ea_hbar_dot[i] = 0.f;
		_ea_trig_pos[i] = _ea_trig_vel[i] = 0;

		for (int k = 0; k < 3; k++) { _imr_xi[i][k] = 0.f; }

		_imr_W[i] = 0.f;
		_imr_y_held[i] = 0.f;
		_imr_trig[i] = 0;
		_imr_sat[i] = false;
		_fx_xp_hat[i] = _fx_xv_hat[i] = _fx_phi_hat[i] = _fx_f_prev[i] = 0.f;
		_bl_t0_us[i] = 0;
	}
}

// ET-ADRC baseline - Xu, Wang, Wang & Long, "Event-triggered active disturbance rejection
// trajectory tracking control for a quadrotor UAV", Appl. Math. Comput. 449 (2023) 127967.
// Per-axis cascade: position ESO (eq.13) + ET (eq.16) -> pseudo-control hbar (eq.17);
// velocity ESO (eq.20) + ET (eq.22) + FLS adaptation (eq.23) -> u (eq.24) = acc_sp.
// Implementation decisions (disclosed in the ledger): tau fixed at TAU (paper Remark 2:
// converged tau <=> fixed-threshold ET); FLS realized as 5 Gaussian bases on chi2_hat;
// hbar_dot from first-order-filtered numeric differentiation.
void PositionControl::_applyEtAdrc(const Vector3f &acc_sp_pid, float dt)
{
	// Bandwidth parameterization (Gao 2003): outer loop wc=kappa1*tau=1.2 rad/s
	// (2.4x margin over the w=0.5 reference), inner loop wc=kappa2*tau=4.8 rad/s
	// (cascade 3-5x rule). ESO gains per loop: L=[2wo, wo^2] with wo~3wc (pos)
	// and half-gain (Herbst 2020) on the velocity ESO to avoid exciting the
	// event-held-input quantization.
	static constexpr float TAU = 2.0f;                     // fixed adaptive parameter (eq.25 frozen)
	static constexpr float KAPPA1[3] = {0.6f, 0.6f, 0.6f};   // wc_pos = 1.2
	static constexpr float KAPPA2[3] = {0.8f, 0.8f, 2.4f};   // R3: soft wc_vel_xy=1.6 (was 2.4)
	static constexpr float D1P = 3.6f, D2P = 3.24f;        // pos ESO: wo=3.6
	// vel ESO: restore v7 wo~6.8 (R2 scale hurt rms without cutting sat)
	static constexpr float D1V_AX[3] = {4.8f, 4.8f, 4.8f};
	static constexpr float D2V_AX[3] = {11.5f, 11.5f, 11.5f};
	static constexpr float B2_CLAMP[3] = {4.f, 4.f, 4.f};
	static constexpr float RHO1 = 0.55f, RHO2 = 1.96f;     // thresh 0.02 m / 0.03 m/s
	static constexpr float GAMMA = 0.5f, LAMBDA_TH = 0.1f; // FLS adaptation (eq.23)
	static constexpr float FLS_C[5] = {-2.f, -1.f, 0.f, 1.f, 2.f};

	const uint64_t now_us = hrt_absolute_time();
	Vector3f acc_out{};
	bppsmc_debug_s dbg{};
	dbg.timestamp = now_us;
	dt = math::max(dt, 1e-4f);

	for (int i = 0; i < 3; i++) {
		if (!_bppsmcAxisActive(i) || !PX4_ISFINITE(_pos(i)) || !PX4_ISFINITE(_vel(i))
		    || !PX4_ISFINITE(_pd_pure(i)) || !PX4_ISFINITE(_vd_pure(i))) {
			acc_out(i) = acc_sp_pid(i);
			dbg.sliding_surface[i] = NAN;
			dbg.rho_center[i] = NAN;
			dbg.acceleration_sp[i] = acc_out(i);
			dbg.trigger_count[i] = _ea_trig_pos[i];
			continue;
		}

		const float r = _pd_pure(i);
		const float r_dot = _vd_pure(i);
		const float alpha1 = _pos(i) - r;      // chi_{i,1} - r_i

		if (!_bl_init[i]) {
			_bl_init[i] = true;
			_ea_alpha_hat[i][0] = alpha1;
			_ea_zeta[i] = alpha1;
			const float hbar0 = math::constrain(-KAPPA1[i] * TAU * alpha1 + r_dot, -3.f, 3.f);
			_ea_beta_hat[i][0] = _vel(i) - hbar0;
			_ea_eta[i] = _ea_beta_hat[i][0];
			_ea_hbar_prev[i] = hbar0;
			_bl_t0_us[i] = now_us;
		}

		// --- position-loop ET (eq.16 reduces to a held-error threshold under fixed tau) ---
		if (fabsf(_ea_zeta[i] - alpha1) >= RHO1 / ((D1P + D2P) * TAU * TAU)) {
			_ea_zeta[i] = alpha1;
			_ea_trig_pos[i]++;
		}

		// --- position-loop ESO (eq.13) ---
		const float sigma = TAU * (_ea_zeta[i] - _ea_alpha_hat[i][0]);
		// pseudo-control = commanded velocity; clamp to a reachable magnitude (anti-windup,
		// implementation deviation disclosed in the ledger)
		const float hbar_raw = -KAPPA1[i] * TAU * _ea_alpha_hat[i][0] - _ea_alpha_hat[i][1] + r_dot;
		const float hbar = math::constrain(hbar_raw, -3.f, 3.f);
		const float da1 = hbar + _ea_alpha_hat[i][1] - r_dot + D1P * sigma;   // alpha1_hat_dot
		const float da2 = TAU * D2P * sigma;                                   // alpha2_hat_dot
		_ea_alpha_hat[i][0] += dt * da1;
		_ea_alpha_hat[i][1] = math::constrain(_ea_alpha_hat[i][1] + dt * da2, -3.f, 3.f);

		// hbar_dot: ANALYTIC (chain rule through the ESO, r_ddot = _ad_pure feedforward);
		// zero while the pseudo-control clamp is active. Replaces the noisy numeric
		// derivative that amplified event-update steps into spikes.
		const float r_ddot = PX4_ISFINITE(_ad_pure(i)) ? _ad_pure(i) : 0.f;
		_ea_hbar_dot[i] = (fabsf(hbar_raw) < 3.f)
				  ? (-KAPPA1[i] * TAU * da1 - da2 + r_ddot) : 0.f;
		_ea_hbar_prev[i] = hbar;

		// --- velocity-loop ET (eq.22) ---
		const float beta1 = _vel(i) - hbar;

		if (fabsf(_ea_eta[i] - beta1) >= RHO2 / ((D1V_AX[i] + D2V_AX[i]) * TAU * TAU)) {
			_ea_eta[i] = beta1;
			_ea_trig_vel[i]++;
		}

		// --- FLS basis on chi2_hat (5 Gaussians, normalized) ---
		const float chi2_hat = hbar + _ea_beta_hat[i][0];
		float phi[5];
		float phi_sum = 1e-6f;

		for (int k = 0; k < 5; k++) {
			const float e = chi2_hat - FLS_C[k];
			phi[k] = expf(-0.5f * e * e);
			phi_sum += phi[k];
		}

		float theta_phi = 0.f;

		for (int k = 0; k < 5; k++) {
			phi[k] /= phi_sum;
			theta_phi += _ea_theta_hat[i][k] * phi[k];
		}

		// --- velocity-loop ESO (eq.20) + control (eq.24) ---
		const float rho_v = TAU * (_ea_eta[i] - _ea_beta_hat[i][0]);
		const float u = -KAPPA2[i] * TAU * _ea_beta_hat[i][0] - _ea_beta_hat[i][1]
				- theta_phi + _ea_hbar_dot[i];
		const float lim_i = _bppsmc_acc_max(i);
		const float u_sat = math::constrain(u, -lim_i, lim_i);   // actual applied input
		_ea_beta_hat[i][0] += dt * (u_sat + theta_phi + _ea_beta_hat[i][1] - _ea_hbar_dot[i] + D1V_AX[i] * rho_v);
		// Conditional anti-windup on beta2 (disturbance): freeze D2V integration
		// while the applied acceleration is saturated — stops integral windup that
		// rides B2_CLAMP under event-held quantized feedback (ET-ADRC tune R1).
		const bool u_at_limit = (fabsf(u - u_sat) > 1e-6f);
		const float db2 = u_at_limit ? 0.f : (TAU * D2V_AX[i] * rho_v);
		_ea_beta_hat[i][1] = math::constrain(_ea_beta_hat[i][1] + dt * db2,
						     -B2_CLAMP[i], B2_CLAMP[i]);

		// --- FLS weights (eq.23) with norm projection ---
		float th_norm2 = 0.f;

		for (int k = 0; k < 5; k++) {
			_ea_theta_hat[i][k] += dt * 2.f * GAMMA
					       * (_ea_beta_hat[i][0] * phi[k] / TAU - LAMBDA_TH * _ea_theta_hat[i][k]);
			th_norm2 += _ea_theta_hat[i][k] * _ea_theta_hat[i][k];
		}

		if (th_norm2 > 25.f) {
			const float sc = 5.f / sqrtf(th_norm2);

			for (int k = 0; k < 5; k++) { _ea_theta_hat[i][k] *= sc; }
		}

		acc_out(i) = PX4_ISFINITE(u_sat) ? u_sat : _ad_pure(i);
		// note: r_ddot feedforward enters through hbar_dot (hbar contains r_dot), matching eq.24

		dbg.sliding_surface[i] = beta1;
		dbg.rho_center[i] = _ea_alpha_hat[i][1];   // pos-loop disturbance estimate
		dbg.acceleration_sp[i] = acc_out(i);
		dbg.e0[i] = _ea_beta_hat[i][1];            // vel-loop disturbance estimate
		dbg.t0[i] = (float)_ea_trig_vel[i];
		dbg.trigger_count[i] = _ea_trig_pos[i];
	}

	_acc_sp = acc_out;
	_accelerationControl();
	_bppsmc_debug_pub.publish(dbg);
	_pd_pure_prev = _pd_pure;
}

// ET-IMR v2 — Zhao et al., IJRNC 35 (2025) internal-model framework (zhao2025event),
// PX4-adapted: 2nd-order error plant with measured (p_e,v_e), no double-filter / no ESO.
// u = -Kp pe - Kv ve - Kxi·xi;  xi <- Phi_d xi + N_d y_held  (gains: etimr_gains.h).
void PositionControl::_applyEtImr(const Vector3f &acc_sp_pid, float dt)
{
	const uint64_t now_us = hrt_absolute_time();
	Vector3f acc_out{};
	bppsmc_debug_s dbg{};
	dbg.timestamp = now_us;
	(void)dt;
	const float w_decay = expf(-etimr::ALPHA * etimr::DT);

	for (int i = 0; i < 3; i++) {
		if (!_bppsmcAxisActive(i) || !PX4_ISFINITE(_pos(i)) || !PX4_ISFINITE(_vel(i))
		    || !PX4_ISFINITE(_pd_pure(i)) || !PX4_ISFINITE(_vd_pure(i))) {
			acc_out(i) = acc_sp_pid(i);
			dbg.sliding_surface[i] = NAN;
			dbg.rho_center[i] = NAN;
			dbg.acceleration_sp[i] = acc_out(i);
			dbg.e0[i] = NAN;
			dbg.trigger_count[i] = _imr_trig[i];
			continue;
		}

		const float pe = _pos(i) - _pd_pure(i);
		const float ve = _vel(i) - _vd_pure(i);

		if (!_bl_init[i]) {
			_bl_init[i] = true;
			_imr_y_held[i] = pe;
			_imr_W[i] = etimr::W0;
			_imr_trig[i] = 0;
			_imr_sat[i] = false;

			for (int k = 0; k < etimr::XI_DIM; k++) { _imr_xi[i][k] = 0.f; }

			_bl_t0_us[i] = now_us;
		}

		const float thr2 = math::max(_imr_W[i] / etimr::THETA, etimr::DELTA * etimr::DELTA);

		if ((pe - _imr_y_held[i]) * (pe - _imr_y_held[i]) >= thr2) {
			_imr_y_held[i] = pe;
			_imr_trig[i]++;
		}

		const float (*Phi)[3] = (i == 0) ? etimr::Phi_d_x : ((i == 1) ? etimr::Phi_d_y : etimr::Phi_d_z);
		const float (*Nd)[1] = (i == 0) ? etimr::N_d_x : ((i == 1) ? etimr::N_d_y : etimr::N_d_z);
		const float (*Kxi)[3] = (i == 0) ? etimr::Kxi_x : ((i == 1) ? etimr::Kxi_y : etimr::Kxi_z);
		const float Kp = (i == 0) ? etimr::Kp_x : ((i == 1) ? etimr::Kp_y : etimr::Kp_z);
		const float Kv = (i == 0) ? etimr::Kv_x : ((i == 1) ? etimr::Kv_y : etimr::Kv_z);

		float xi_new[3] {};

		for (int r = 0; r < etimr::XI_DIM; r++) {
			float s = Nd[r][0] * _imr_y_held[i];

			for (int c = 0; c < etimr::XI_DIM; c++) {
				s += Phi[r][c] * _imr_xi[i][c];
			}

			xi_new[r] = s;
		}

		for (int k = 0; k < etimr::XI_DIM; k++) { _imr_xi[i][k] = xi_new[k]; }

		_imr_W[i] *= w_decay;

		float u = -Kp * pe - Kv * ve;

		for (int k = 0; k < etimr::XI_DIM; k++) { u -= Kxi[0][k] * _imr_xi[i][k]; }

		const float ad_ff = PX4_ISFINITE(_ad_pure(i)) ? _ad_pure(i) : 0.f;
		const float lim_i = _bppsmc_acc_max(i);
		const float acc_raw = ad_ff + u;
		acc_out(i) = math::constrain(acc_raw, -lim_i, lim_i);
		_imr_sat[i] = (fabsf(acc_raw - acc_out(i)) > 1e-6f);

		dbg.sliding_surface[i] = pe;
		dbg.rho_center[i] = u;
		dbg.e0[i] = _imr_xi[i][0];
		dbg.t0[i] = _imr_W[i];
		dbg.trigger_count[i] = _imr_trig[i];
		dbg.acceleration_sp[i] = acc_out(i);
	}

	_acc_sp = acc_out;
	_accelerationControl();
	_bppsmc_debug_pub.publish(dbg);
	_pd_pure_prev = _pd_pure;
}

// FxT-OF baseline - Shao, Tian & Yang, "Fixed-time trajectory following for quadrotors via
// output feedback", ISA Trans. 110 (2021) 213-224. Per-axis FTESO (eq.4, position
// measurement only - EKF velocity deliberately unused) + fixed-time law (eq.10) = acc_sp.
// sign() smoothed by tanh(./EPS) per the paper's Remark 5; attitude loop (eq.11-15) is
// PX4-native (consistent with the paper's inner-outer separation assumption).
void PositionControl::_applyFxtOf(const Vector3f &acc_sp_pid, float dt)
{
	static constexpr float KP[3] = {1.5f, 1.5f, 2.0f};
	static constexpr float KV[3] = {2.5f, 2.5f, 3.0f};
	static constexpr float A1 = 0.6f, A2 = 0.6f;          // fractional exponents alpha_1/2
	static constexpr float LV = 2.0f;                      // disturbance Lipschitz guess
	static constexpr float DELTA = 0.1f, TU = 1.0f, EPS = 0.05f;
	const float l1 = 2.f * powf(LV, 1.f / 3.f), l2 = 1.5f * powf(LV, 2.f / 3.f), l3 = 1.1f * LV;
	static constexpr float K1 = 6.f, K2 = 11.f, K3 = 6.f; // Hurwitz (poles -1,-2,-3)

	auto spow = [](float x, float p) { return powf(fabsf(x), p) * ((x > 0.f) - (x < 0.f)); };

	const uint64_t now_us = hrt_absolute_time();
	Vector3f acc_out{};
	bppsmc_debug_s dbg{};
	dbg.timestamp = now_us;
	dt = math::max(dt, 1e-4f);

	const float e1p = A1 / (2.f - A1), e2p = (4.f - 3.f * A1) / (2.f - A1);
	const float e1v = A2 / (2.f - A2), e2v = (4.f - 3.f * A2) / (3.f - 2.f * A2);

	for (int i = 0; i < 3; i++) {
		if (!_bppsmcAxisActive(i) || !PX4_ISFINITE(_pos(i)) || !PX4_ISFINITE(_pd_pure(i))) {
			acc_out(i) = acc_sp_pid(i);
			dbg.sliding_surface[i] = NAN;
			dbg.rho_center[i] = NAN;
			dbg.acceleration_sp[i] = acc_out(i);
			continue;
		}

		if (!_bl_init[i]) {
			_bl_init[i] = true;
			_fx_xp_hat[i] = _pos(i);
			_fx_xv_hat[i] = 0.f;
			_fx_phi_hat[i] = 0.f;
			_fx_f_prev[i] = 0.f;
			_bl_t0_us[i] = now_us;
		}

		const float t_seg = (now_us - _bl_t0_us[i]) * 1e-6f;
		const float th = (t_seg <= TU) ? 0.f : 1.f;        // theta(t), eq.(3) in paper Sec.3.1

		// --- FTESO (eq.4): only position measurement enters ---
		const float zp = _pos(i) - _fx_xp_hat[i];
		const float dxp = _fx_xv_hat[i] + l1 * th * spow(zp, 2.f / 3.f)
				  + K1 * (1.f - th) * spow(zp, (3.f + DELTA) / 3.f);
		const float dxv = _fx_phi_hat[i] + _fx_f_prev[i] + l2 * th * spow(zp, 1.f / 3.f)
				  + K2 * (1.f - th) * spow(zp, (3.f + 2.f * DELTA) / 3.f);
		const float dph = l3 * th * tanhf(zp / EPS) + K3 * (1.f - th) * spow(zp, 1.f + DELTA);
		_fx_xp_hat[i] += dt * dxp;
		_fx_xv_hat[i] += dt * dxv;
		_fx_phi_hat[i] += dt * dph;
		_fx_phi_hat[i] = math::constrain(_fx_phi_hat[i], -10.f, 10.f);

		// --- fixed-time control law (eq.10) ---
		const float sp = _pos(i) - _pd_pure(i);
		const float sv = _fx_xv_hat[i] - _vd_pure(i);
		const float F = -KP[i] * (spow(sp, e1p) + sp + spow(sp, e2p))
				- KV[i] * (spow(sv, e1v) + sv + spow(sv, e2v))
				- _fx_phi_hat[i] + _ad_pure(i);

		const float lim = _bppsmc_acc_max(i);
		acc_out(i) = PX4_ISFINITE(F) ? math::constrain(F, -lim, lim) : _ad_pure(i);
		_fx_f_prev[i] = acc_out(i);

		dbg.sliding_surface[i] = sp;
		dbg.rho_center[i] = _fx_xv_hat[i];
		dbg.acceleration_sp[i] = acc_out(i);
		dbg.e0[i] = _fx_phi_hat[i];
		dbg.t0[i] = t_seg;
		dbg.trigger_count[i] = 0;
	}

	_acc_sp = acc_out;
	_accelerationControl();
	_bppsmc_debug_pub.publish(dbg);
	_pd_pure_prev = _pd_pure;
}

bool PositionControl::update(const float dt)
{
	bool valid = _inputValid();

	if (valid) {
		_capturePureSetpoints();
		_positionControl();
		_velocityControl(dt);

		const Vector3f acc_sp_pid = _acc_sp;
		const Vector3f thr_sp_pid = _thr_sp;
		const bool bppsmc_ready = _bppsmcReady();

		if (!bppsmc_ready && _bppsmc_was_ready) {
			resetBppsmcState();
		}

		if (bppsmc_ready) {
#if BASELINE_CONTROLLER == 1
			_applyEtAdrc(acc_sp_pid, dt);
#elif BASELINE_CONTROLLER == 2
			_applyFxtOf(acc_sp_pid, dt);
#elif BASELINE_CONTROLLER == 3
			_applyEtImr(acc_sp_pid, dt);
#else
		_applyBppsmcOverride(acc_sp_pid);
#endif

			if (!_acc_sp.isAllFinite() || !_thr_sp.isAllFinite()) {
				_acc_sp = acc_sp_pid;
				_thr_sp = thr_sp_pid;
			}
		}

		_bppsmc_was_ready = bppsmc_ready;

		_yawspeed_sp = PX4_ISFINITE(_yawspeed_sp) ? _yawspeed_sp : 0.f;
		_yaw_sp = PX4_ISFINITE(_yaw_sp) ? _yaw_sp : _yaw; // TODO: better way to disable yaw control
	}

	// There has to be a valid output acceleration and thrust setpoint otherwise something went wrong
	return valid && _acc_sp.isAllFinite() && _thr_sp.isAllFinite();
}

void PositionControl::_positionControl()
{
	// P-position controller
	Vector3f vel_sp_position = (_pos_sp - _pos).emult(_gain_pos_p);
	// Position and feed-forward velocity setpoints or position states being NAN results in them not having an influence
	ControlMath::addIfNotNanVector3f(_vel_sp, vel_sp_position);
	// make sure there are no NAN elements for further reference while constraining
	ControlMath::setZeroIfNanVector3f(vel_sp_position);

	// Constrain horizontal velocity by prioritizing the velocity component along the
	// the desired position setpoint over the feed-forward term.
	_vel_sp.xy() = ControlMath::constrainXY(vel_sp_position.xy(), (_vel_sp - vel_sp_position).xy(), _lim_vel_horizontal);
	// Constrain velocity in z-direction.
	_vel_sp(2) = math::constrain(_vel_sp(2), -_lim_vel_up, _lim_vel_down);
}

void PositionControl::_velocityControl(const float dt)
{
	// Constrain vertical velocity integral
	_vel_int(2) = math::constrain(_vel_int(2), -CONSTANTS_ONE_G, CONSTANTS_ONE_G);

	// PID velocity control
	Vector3f vel_error = _vel_sp - _vel;
	Vector3f acc_sp_velocity = vel_error.emult(_gain_vel_p) + _vel_int - _vel_dot.emult(_gain_vel_d);

	// No control input from setpoints or corresponding states which are NAN
	ControlMath::addIfNotNanVector3f(_acc_sp, acc_sp_velocity);

	_accelerationControl();

	// Integrator anti-windup in vertical direction
	if ((_thr_sp(2) >= -_lim_thr_min && vel_error(2) >= 0.f) ||
	    (_thr_sp(2) <= -_lim_thr_max && vel_error(2) <= 0.f)) {
		vel_error(2) = 0.f;
	}

	// Prioritize vertical control while keeping a horizontal margin
	const Vector2f thrust_sp_xy(_thr_sp);
	const float thrust_sp_xy_norm = thrust_sp_xy.norm();
	const float thrust_max_squared = math::sq(_lim_thr_max);

	// Determine how much vertical thrust is left keeping horizontal margin
	const float allocated_horizontal_thrust = math::min(thrust_sp_xy_norm, _lim_thr_xy_margin);
	const float thrust_z_max_squared = thrust_max_squared - math::sq(allocated_horizontal_thrust);

	// Saturate maximal vertical thrust
	_thr_sp(2) = math::max(_thr_sp(2), -sqrtf(thrust_z_max_squared));

	// Determine how much horizontal thrust is left after prioritizing vertical control
	const float thrust_max_xy_squared = thrust_max_squared - math::sq(_thr_sp(2));
	float thrust_max_xy = 0.f;

	if (thrust_max_xy_squared > 0.f) {
		thrust_max_xy = sqrtf(thrust_max_xy_squared);
	}

	// Saturate thrust in horizontal direction
	if (thrust_sp_xy_norm > thrust_max_xy) {
		_thr_sp.xy() = thrust_sp_xy / thrust_sp_xy_norm * thrust_max_xy;
	}

	// Use tracking Anti-Windup for horizontal direction: during saturation, the integrator is used to unsaturate the output
	// see Anti-Reset Windup for PID controllers, L.Rundqwist, 1990
	const Vector2f acc_sp_xy_produced = Vector2f(_thr_sp) * (CONSTANTS_ONE_G / _hover_thrust);
	const float arw_gain = 2.f / _gain_vel_p(0);

	// The produced acceleration can be greater or smaller than the desired acceleration due to the saturations and the actual vertical thrust (computed independently).
	// The ARW loop needs to run if the signal is saturated only.
	const Vector2f acc_sp_xy = _acc_sp.xy();
	const Vector2f acc_limited_xy = (acc_sp_xy.norm_squared() > acc_sp_xy_produced.norm_squared())
					? acc_sp_xy_produced
					: acc_sp_xy;
	vel_error.xy() = Vector2f(vel_error) - arw_gain * (acc_sp_xy - acc_limited_xy);

	// Make sure integral doesn't get NAN
	ControlMath::setZeroIfNanVector3f(vel_error);
	// Update integral part of velocity control
	_vel_int += vel_error.emult(_gain_vel_i) * dt;
}

void PositionControl::_accelerationControl()
{
	// Assume standard acceleration due to gravity in vertical direction for attitude generation
	float z_specific_force = -CONSTANTS_ONE_G;

	if (!_decouple_horizontal_and_vertical_acceleration) {
		// Include vertical acceleration setpoint for better horizontal acceleration tracking
		z_specific_force += _acc_sp(2);
	}

	Vector3f body_z = Vector3f(-_acc_sp(0), -_acc_sp(1), -z_specific_force).normalized();
	ControlMath::limitTilt(body_z, Vector3f(0, 0, 1), _lim_tilt);
	// Convert to thrust assuming hover thrust produces standard gravity
	const float thrust_ned_z = _acc_sp(2) * (_hover_thrust / CONSTANTS_ONE_G) - _hover_thrust;
	// Project thrust to planned body attitude
	const float cos_ned_body = (Vector3f(0, 0, 1).dot(body_z));
	const float collective_thrust = math::min(thrust_ned_z / cos_ned_body, -_lim_thr_min);
	_thr_sp = body_z * collective_thrust;
}

bool PositionControl::_inputValid()
{
	bool valid = true;

	// Every axis x, y, z needs to have some setpoint
	for (int i = 0; i <= 2; i++) {
		valid = valid && (PX4_ISFINITE(_pos_sp(i)) || PX4_ISFINITE(_vel_sp(i)) || PX4_ISFINITE(_acc_sp(i)));
	}

	// x and y input setpoints always have to come in pairs
	valid = valid && (PX4_ISFINITE(_pos_sp(0)) == PX4_ISFINITE(_pos_sp(1)));
	valid = valid && (PX4_ISFINITE(_vel_sp(0)) == PX4_ISFINITE(_vel_sp(1)));
	valid = valid && (PX4_ISFINITE(_acc_sp(0)) == PX4_ISFINITE(_acc_sp(1)));

	// For each controlled state the estimate has to be valid
	for (int i = 0; i <= 2; i++) {
		if (PX4_ISFINITE(_pos_sp(i))) {
			valid = valid && PX4_ISFINITE(_pos(i));
		}

		if (PX4_ISFINITE(_vel_sp(i))) {
			valid = valid && PX4_ISFINITE(_vel(i)) && PX4_ISFINITE(_vel_dot(i));
		}
	}

	return valid;
}

void PositionControl::getLocalPositionSetpoint(vehicle_local_position_setpoint_s &local_position_setpoint) const
{
	local_position_setpoint.x = _pos_sp(0);
	local_position_setpoint.y = _pos_sp(1);
	local_position_setpoint.z = _pos_sp(2);
	local_position_setpoint.yaw = _yaw_sp;
	local_position_setpoint.yawspeed = _yawspeed_sp;
	local_position_setpoint.vx = _vel_sp(0);
	local_position_setpoint.vy = _vel_sp(1);
	local_position_setpoint.vz = _vel_sp(2);
	_acc_sp.copyTo(local_position_setpoint.acceleration);
	_thr_sp.copyTo(local_position_setpoint.thrust);
}

void PositionControl::getAttitudeSetpoint(vehicle_attitude_setpoint_s &attitude_setpoint) const
{
	ControlMath::thrustToAttitude(_thr_sp, _yaw_sp, attitude_setpoint);
	attitude_setpoint.yaw_sp_move_rate = _yawspeed_sp;
}
