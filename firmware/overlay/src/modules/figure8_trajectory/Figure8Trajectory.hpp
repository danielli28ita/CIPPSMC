#pragma once

#include "Figure8Mission.hpp"

#include <drivers/drv_hrt.h>
#include <lib/systemlib/mavlink_log.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/figure8_status.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>

using namespace time_literals;

class Figure8Trajectory : public ModuleBase<Figure8Trajectory>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	Figure8Trajectory();
	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);
	int print_status() override;
	bool init();

private:
	void Run() override;
	bool inputs_valid(hrt_abstime now) const;
	bool read_config();
	void abort_task(const char *reason, bool request_land);
	void publish_status(hrt_abstime now);
	void publish_mode(hrt_abstime now);
	void send_command(uint32_t command, float param1, float param2 = 0.f);
	void adjust_origin();
	static const char *stage_name(figure8::Stage stage);
	bool active() const { return _stage >= figure8::Stage::Takeoff && _stage <= figure8::Stage::Touchdown; }

	uORB::Subscription _position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _land_sub{ORB_ID(vehicle_land_detected)};
	uORB::Subscription _status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _manual_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription _armed_sub{ORB_ID(actuator_armed)};
	uORB::Subscription _param_sub{ORB_ID(parameter_update)};
	uORB::Publication<trajectory_setpoint_s> _trajectory_pub{ORB_ID(trajectory_setpoint)};
	uORB::Publication<offboard_control_mode_s> _mode_pub{ORB_ID(offboard_control_mode)};
	uORB::Publication<figure8_status_s> _task_status_pub{ORB_ID(figure8_status)};
	uORB::Publication<vehicle_command_s> _command_pub{ORB_ID(vehicle_command)};
	vehicle_local_position_s _position{};
	vehicle_land_detected_s _land{};
	vehicle_status_s _status{};
	manual_control_setpoint_s _manual{};
	actuator_armed_s _armed{};
	orb_advert_t _mavlink_log_pub{nullptr};
	figure8::Config _config{};
	figure8::Mission _mission{};
	figure8::Stage _stage{figure8::Stage::Waiting};
	hrt_abstime _ready_since{0};
	hrt_abstime _start_time{0};
	hrt_abstime _last_run{0};
	hrt_abstime _last_disarm{0};
	float _origin[3] {};
	float _yaw{0.f};
	uint8_t _xy_reset{0}, _z_reset{0}, _heading_reset{0};
	uint8_t _controller{0};
	bool _enabled_at_start{false};
	bool _armed_on_ground{false};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::F8_ENABLE>) _param_enable,
		(ParamFloat<px4::params::F8_HEIGHT>) _param_height,
		(ParamFloat<px4::params::F8_RADIUS>) _param_radius,
		(ParamFloat<px4::params::F8_PERIOD>) _param_period,
		(ParamInt<px4::params::F8_LAPS>) _param_laps,
		(ParamFloat<px4::params::F8_TKO_TIME>) _param_takeoff_time,
		(ParamFloat<px4::params::F8_HOV_TIME>) _param_hover_time,
		(ParamFloat<px4::params::F8_LND_TIME>) _param_land_time,
		(ParamFloat<px4::params::F8_LND_DEPTH>) _param_landing_depth,
		(ParamFloat<px4::params::F8_ERR_MAX>) _param_max_error,
		(ParamInt<px4::params::MPC_CTRL_METHOD>) _param_method,
		(ParamFloat<px4::params::MPC_XY_VEL_MAX>) _param_max_vxy,
		(ParamFloat<px4::params::MPC_Z_VEL_MAX_UP>) _param_max_up,
		(ParamFloat<px4::params::MPC_Z_VEL_MAX_DN>) _param_max_down,
		(ParamFloat<px4::params::MPC_ACC_HOR_MAX>) _param_max_acc,
		(ParamFloat<px4::params::BPP_ACC_XY>) _param_bpp_acc
	)
};
