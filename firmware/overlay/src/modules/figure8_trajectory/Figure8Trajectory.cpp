#include "Figure8Trajectory.hpp"

#include <cmath>

Figure8Trajectory::Figure8Trajectory() :
	ModuleParams(nullptr), ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
}

bool Figure8Trajectory::init()
{
	_enabled_at_start = _param_enable.get() == 1;
	ScheduleOnInterval(20_ms);
	return true;
}

bool Figure8Trajectory::inputs_valid(hrt_abstime now) const
{
	return _position.timestamp != 0 && now < _position.timestamp + 500_ms
	       && _position.xy_valid && _position.z_valid && _position.v_xy_valid && _position.v_z_valid
	       // heading_good_for_control requires in-air final mag alignment in v1.15.
	       // Commander checks yaw health for arming/mode entry; do not block ground preparation on that flag.
	       && PX4_ISFINITE(_position.x) && PX4_ISFINITE(_position.y) && PX4_ISFINITE(_position.z)
	       && PX4_ISFINITE(_position.vx) && PX4_ISFINITE(_position.vy) && PX4_ISFINITE(_position.vz)
	       && PX4_ISFINITE(_position.heading)
	       && _status.timestamp != 0 && now < _status.timestamp + 2_s && !_status.failsafe
	       && _status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
	       && _land.timestamp != 0 && now < _land.timestamp + 2_s
	       && _manual.valid && _manual.timestamp != 0 && now < _manual.timestamp + 500_ms
	       && !_armed.manual_lockdown && !_armed.force_failsafe;
}

bool Figure8Trajectory::read_config()
{
	_config.height = _param_height.get();
	_config.radius = _param_radius.get();
	_config.period = _param_period.get();
	_config.laps = _param_laps.get();
	_config.takeoff_time = _param_takeoff_time.get();
	_config.hover_time = _param_hover_time.get();
	_config.land_time = _param_land_time.get();
	_config.landing_depth = _param_landing_depth.get();
	_config.max_error = _param_max_error.get();

	if (!_config.valid()) { return false; }

	// Conservative bounds for the quintic phase and analytic feedforward.
	const float omega = 11.780973f / _config.period;
	const float alpha = 36.276f / (_config.period * _config.period);
	// Bernoulli phase derivatives satisfy |r'| <= a and |r''| <= 3a.
	const float vxy = _config.radius * omega;
	const float axy = _config.radius * (alpha + 3.f * omega * omega);
	const float vup = 1.875f * _config.height / _config.takeoff_time;
	const float vdown = fmaxf(0.35f, 2.f * (_config.height + _config.landing_depth) / _config.land_time);
	return vxy <= _param_max_vxy.get() && vup <= _param_max_up.get() && vdown <= _param_max_down.get()
	       && axy <= _param_max_acc.get() && (_param_method.get() == 0 || axy <= _param_bpp_acc.get());
}

void Figure8Trajectory::publish_mode(hrt_abstime now)
{
	offboard_control_mode_s mode{};
	mode.timestamp = now;
	mode.position = true;
	_mode_pub.publish(mode);
}

void Figure8Trajectory::publish_status(hrt_abstime now)
{
	figure8_status_s task{};
	task.timestamp = now;
	// Latch this policy for the module's lifetime, including abort/stop in flight.
	task.enabled = _enabled_at_start;
	task.active = active();
	task.stage = static_cast<uint8_t>(_stage);
	task.controller = _controller;
	task.lap = _mission.lap();
	task.failure = static_cast<uint8_t>(_mission.failure());
	task.tracking_error = _mission.error();
	task.stage_elapsed = _start_time ? _mission.stage_elapsed((now - _start_time) * 1e-6f) : 0.f;
	_task_status_pub.publish(task);
}

void Figure8Trajectory::send_command(uint32_t command, float param1, float param2)
{
	vehicle_command_s cmd{};
	cmd.timestamp = hrt_absolute_time();
	cmd.command = command;
	cmd.param1 = param1;
	cmd.param2 = param2;
	cmd.target_system = _status.system_id;
	cmd.target_component = _status.component_id;
	cmd.source_system = _status.system_id;
	cmd.source_component = _status.component_id;
	cmd.from_external = false;
	_command_pub.publish(cmd);
}

void Figure8Trajectory::abort_task(const char *reason, bool request_land)
{
	_stage = figure8::Stage::Aborted;
	_armed_on_ground = false;
	mavlink_log_critical(&_mavlink_log_pub, "Figure8 aborted: %s\t", reason);

	if (request_land && _status.arming_state == vehicle_status_s::ARMING_STATE_ARMED
	    && _status.nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD && !_armed.manual_lockdown) {
		// This is a normal PX4 landing request, never a forced disarm.
		send_command(vehicle_command_s::VEHICLE_CMD_NAV_LAND, 0.f);
	}
}

void Figure8Trajectory::adjust_origin()
{
	float delta[3] {};
	float heading_delta = 0.f;

	if (_xy_reset != _position.xy_reset_counter) {
		delta[0] = _position.delta_xy[0]; delta[1] = _position.delta_xy[1];
	}

	if (_z_reset != _position.z_reset_counter) { delta[2] = _position.delta_z; }

	if (_heading_reset != _position.heading_reset_counter) { heading_delta = _position.delta_heading; }

	for (int i = 0; i < 3; ++i) { _origin[i] += delta[i]; }

	_yaw = atan2f(sinf(_yaw + heading_delta), cosf(_yaw + heading_delta));

	if (active()) { _mission.translate(delta, heading_delta); }

	_xy_reset = _position.xy_reset_counter;
	_z_reset = _position.z_reset_counter;
	_heading_reset = _position.heading_reset_counter;
}

void Figure8Trajectory::Run()
{
	const hrt_abstime now = hrt_absolute_time();
	_position_sub.update(&_position);
	_land_sub.update(&_land);
	_status_sub.update(&_status);
	_manual_sub.update(&_manual);
	_armed_sub.update(&_armed);
	const bool armed = _status.arming_state == vehicle_status_s::ARMING_STATE_ARMED;
	const bool offboard = _status.nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD;

	if (_param_sub.updated()) {
		parameter_update_s update;
		_param_sub.copy(&update);
		updateParams();
	}

	if (should_exit()) {
		if (active()) { abort_task("module stopped", true); }

		_stage = figure8::Stage::Disabled;
		publish_status(now);
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	if (active()) {
		if (!offboard) { abort_task("pilot or failsafe left OFFBOARD", false); }

		else if (!armed) {
			if (_mission.airborne_seen() && _land.landed
			    && (_stage == figure8::Stage::Landing || _stage == figure8::Stage::Touchdown)) {
				_stage = figure8::Stage::Complete;
				mavlink_log_info(&_mavlink_log_pub, "Figure8 complete: landed and disarmed\t");

			} else { abort_task("disarmed", false); }

		} else if (!inputs_valid(now) || _param_enable.get() != 1 || now - _last_run > 500_ms) {
			abort_task("input lost, disabled, Kill or scheduling timeout", true);
		}
	}

	_last_run = now;

	if (!_enabled_at_start) {
		_stage = figure8::Stage::Disabled;

	} else if (!active() && _param_enable.get() != 1) {
		_stage = figure8::Stage::Disabled;

	} else if (!active() && !armed && !offboard && _land.landed && inputs_valid(now)) {
		if (read_config()) {
			if (_stage != figure8::Stage::Ready) {
				_stage = figure8::Stage::Ready;
				_ready_since = now;
				mavlink_log_info(&_mavlink_log_pub, "Figure8 ready: arm manually, then select OFFBOARD\t");
			}

			_origin[0] = _position.x; _origin[1] = _position.y; _origin[2] = _position.z;
			_yaw = _position.heading;
			_xy_reset = _position.xy_reset_counter;
			_z_reset = _position.z_reset_counter;
			_heading_reset = _position.heading_reset_counter;
			_controller = _param_method.get();
			_armed_on_ground = false;

		} else {
			if (_stage != figure8::Stage::Waiting) {
				mavlink_log_critical(&_mavlink_log_pub, "Figure8: invalid parameters or trajectory exceeds limits\t");
			}

			_stage = figure8::Stage::Waiting;
		}
	}

	if (_stage == figure8::Stage::Ready) {
		if (!inputs_valid(now) || _param_enable.get() != 1) {
			if (armed || offboard) { abort_task("not ready at start", offboard); }

			else { _stage = figure8::Stage::Waiting; }

		} else {
			adjust_origin();
			const float height = _origin[2] - _position.z;

			if (armed && !offboard && fabsf(height) < 0.5f
			    && hypotf(_position.vx, _position.vy) < 0.3f && fabsf(_position.vz) < 0.3f) {
				_armed_on_ground = true;
			}

			if (fabsf(height) >= 0.5f) { abort_task("start must be on the ground", offboard); }

			else if (armed && offboard) {
				if (!_armed_on_ground || now - _ready_since < 1_s) {
					abort_task("arm manually before entering OFFBOARD", true);

				} else {
					_mission.start(_config, _origin, _yaw);
					_start_time = now;
					_stage = figure8::Stage::Takeoff;
					mavlink_log_info(&_mavlink_log_pub, "Figure8 started: %s\t", _controller ? "BPPSMC" : "PID");
				}
			}

			if (_stage == figure8::Stage::Ready) { publish_mode(now); }
		}
	}

	if (active()) {
		adjust_origin();
		figure8::Sample sample{{_position.x, _position.y, _position.z}, {_position.vx, _position.vy, _position.vz}, _land.landed};
		const auto previous = _stage;
		const auto ref = _mission.step((now - _start_time) * 1e-6f, sample);
		_stage = _mission.stage();

		if (_stage == figure8::Stage::Aborted) {
			abort_task("envelope, tracking, contact or stage timeout", true);

		} else {
			if (_stage != previous) { mavlink_log_info(&_mavlink_log_pub, "Figure8: %s\t", stage_name(_stage)); }

			// Publish ownership before the setpoint; never publish trajectories in manual modes.
			publish_status(now);
			trajectory_setpoint_s setpoint{};
			setpoint.timestamp = now;

			for (int i = 0; i < 3; ++i) {
				setpoint.position[i] = ref.position[i];
				setpoint.velocity[i] = ref.velocity[i];
				setpoint.acceleration[i] = ref.acceleration[i];
				setpoint.jerk[i] = NAN;
			}

			setpoint.yaw = ref.yaw;
			setpoint.yawspeed = 0.f;
			_trajectory_pub.publish(setpoint);
			publish_mode(now);

			if (_stage == figure8::Stage::Touchdown && _land.landed && now - _last_disarm >= 1_s) {
				send_command(vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.f);
				_last_disarm = now;
			}
		}
	}

	publish_status(now);
}

const char *Figure8Trajectory::stage_name(figure8::Stage stage)
{
	switch (stage) {
	case figure8::Stage::Disabled: return "disabled";

	case figure8::Stage::Waiting: return "waiting";

	case figure8::Stage::Ready: return "ready";

	case figure8::Stage::Takeoff: return "takeoff";

	case figure8::Stage::Hover: return "hover";

	case figure8::Stage::Figure8: return "figure8";

	case figure8::Stage::ReturnHold: return "return_hold";

	case figure8::Stage::Landing: return "landing";

	case figure8::Stage::Touchdown: return "touchdown";

	case figure8::Stage::Complete: return "complete";

	case figure8::Stage::Aborted: return "aborted";
	}

	return "unknown";
}

int Figure8Trajectory::print_status()
{
	PX4_INFO("stage: %s; enabled at start: %s", stage_name(_stage), _enabled_at_start ? "yes" : "no");
	PX4_INFO("task controller: %s; lap: %d; error: %.3f m", _controller ? "BPPSMC" : "PID", _mission.lap(),
		 (double)_mission.error());
	return 0;
}

int Figure8Trajectory::task_spawn(int argc, char *argv[])
{
	auto *instance = new Figure8Trajectory();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) { return PX4_OK; }
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;
	return PX4_ERROR;
}

int Figure8Trajectory::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int Figure8Trajectory::print_usage(const char *reason)
{
	if (reason) { PX4_WARN("%s", reason); }

	PRINT_MODULE_DESCRIPTION("Onboard figure-eight references for PID/BPPSMC. Set F8_ENABLE=1 and reboot. Arm manually, then select OFFBOARD on the ground.");
	PRINT_MODULE_USAGE_NAME("figure8_trajectory", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int figure8_trajectory_main(int argc, char *argv[])
{
	return Figure8Trajectory::main(argc, argv);
}
