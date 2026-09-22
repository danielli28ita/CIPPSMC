#include <cmath>
#include <memory>
#include <string>

#include "px4_reference_generator/mission_state_machine.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace
{

constexpr uint8_t kSourceSystem = 191;

}  // namespace

class ReferenceGeneratorNode : public rclcpp::Node
{
public:
  ReferenceGeneratorNode()
  : Node("reference_generator"),
    state_machine_(load_config())
  {
    declare_parameters();
    load_parameters();

    auto qos = rclcpp::SensorDataQoS();

    status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
      topic_vehicle_status_, qos,
      [this](const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
        nav_state_ = msg->nav_state;
        const bool now_armed =
          msg->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
        if (now_armed && !was_armed_) {
          latched_armed_time_us_ = msg->timestamp;
          RCLCPP_INFO(
            get_logger(),
            "ARM latched @ px4_us=%lu | pos x=%.3f y=%.3f z=%.3f | setpoint z=%.2f | "
            "since_command=%.2fs",
            static_cast<unsigned long>(msg->timestamp),
            current_x_, current_y_, current_z_, -hover_altitude_m_,
            command_start_us_ > 0 && msg->timestamp > command_start_us_
              ? (msg->timestamp - command_start_us_) * 1e-6 : 0.0);
        }
        was_armed_ = now_armed;
        arming_state_ = msg->arming_state;
        status_time_us_ = msg->timestamp;
      });

    local_pos_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      topic_vehicle_local_position_, qos,
      [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
        pos_time_us_ = msg->timestamp;
        if (msg->xy_valid) {
          current_x_ = msg->x;
          current_y_ = msg->y;
        }
        if (msg->z_valid) {
          current_z_ = msg->z;
          has_altitude_ = true;
        }
        if (msg->xy_valid && msg->z_valid) {
          if (!has_local_pos_) {
            if (!use_fixed_anchor_ || anchor_from_spawn_) {
              anchor_x_ = msg->x;
              anchor_y_ = msg->y;
            }
            command_start_us_ = msg->timestamp;
            has_local_pos_ = true;
            RCLCPP_INFO(
              get_logger(),
              "EKF spawn snapshot: x=%.3f y=%.3f z=%.3f (NED) | anchor=(%.2f,%.2f) z_sp=%.2f anchor_from_spawn=%s",
              msg->x, msg->y, msg->z, anchor_x_, anchor_y_,
              anchor_from_spawn_ ? (msg->z - hover_altitude_m_) : -hover_altitude_m_,
              anchor_from_spawn_ ? "true" : "false");
          }
        }
      });

    offboard_mode_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
      topic_offboard_control_mode_, qos);
    trajectory_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
      topic_trajectory_setpoint_, qos);
    vehicle_cmd_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>(
      topic_vehicle_command_, qos);

    const auto period = std::chrono::duration<double>(1.0 / control_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&ReferenceGeneratorNode::control_loop, this));

    RCLCPP_INFO(
      get_logger(),
      "reference_generator: hold_pose_until_arm=%s | silent=%.0fs | arm@cmd+%.0fs | "
      "climb ramp=%.1fs | fixed_z_latch=%s | figure8@arm+%.0fs | anchor=(%.2f,%.2f) z=%.1fm anchor_from_spawn=%s",
      hold_pose_until_arm_ ? "true" : "false",
      silent_duration_s_, arm_after_command_s_, climb_ramp_duration_s_,
      use_fixed_arm_latch_z_ ? "true" : "false",
      figure8_start_after_arm_s_,
      anchor_x_, anchor_y_, hover_altitude_m_,
      anchor_from_spawn_ ? "true" : "false");
  }

private:
  px4_reference_generator::MissionConfig load_config()
  {
    px4_reference_generator::MissionConfig cfg;
    cfg.hover_altitude_m = hover_altitude_m_;
    cfg.silent_duration_s = silent_duration_s_;
    cfg.hover_duration_s = hover_duration_s_;
    cfg.figure8_start_command_s = figure8_start_command_s_;
    cfg.figure8_a = figure8_a_;
    cfg.circle_radius_m = circle_radius_m_;
    cfg.figure8_omega = figure8_omega_;
    cfg.figure8_z_amp_m = figure8_z_amp_m_;
    cfg.figure8_z_omega_scale = figure8_z_omega_scale_;
    cfg.mission_end_after_arm_s = mission_end_after_arm_s_;
    cfg.return_origin_timeout_s = return_origin_timeout_s_;
    cfg.return_origin_arrival_radius_m = return_origin_arrival_radius_m_;
    cfg.force_figure8_timeout_s = force_figure8_timeout_s_;
    cfg.auto_offboard_arm = auto_offboard_arm_;
    cfg.control_rate_hz = control_rate_hz_;
    cfg.use_fixed_anchor = use_fixed_anchor_;
    cfg.anchor_x_m = anchor_x_;
    cfg.anchor_y_m = anchor_y_;
    cfg.arm_after_command_s = arm_after_command_s_;
    cfg.hold_pose_until_arm = hold_pose_until_arm_;
    cfg.climb_after_arm_s = climb_after_arm_s_;
    cfg.climb_ramp_duration_s = climb_ramp_duration_s_;
    cfg.use_fixed_arm_latch_z = use_fixed_arm_latch_z_;
    cfg.fixed_arm_latch_z_ned = fixed_arm_latch_z_ned_;
    cfg.figure8_start_after_arm_s = figure8_start_after_arm_s_;
    cfg.anchor_from_spawn = anchor_from_spawn_;
    cfg.anchor_settle_s = anchor_settle_s_;
    cfg.anchor_settle_tol_m = anchor_settle_tol_m_;
    cfg.anchor_settle_max_s = anchor_settle_max_s_;
    cfg.altitude_step_after_arm_s = altitude_step_after_arm_s_;
    cfg.altitude_step_dz_m = altitude_step_dz_m_;
    return cfg;
  }

  void declare_parameters()
  {
    // Mission-clock source. Legacy default "vehicle_status" freezes the clock between
    // 2 Hz status heartbeats (504 ms steps -> published reference is a ~2 Hz staircase;
    // measured vehicle lag ~340 ms, P2 audit 2026-07-19). "local_position" advances the
    // clock at the vehicle_local_position stream rate. Default preserves the legacy
    // behavior so archived BPPSMC/P1 results stay reproducible; opt in via overlay.
    declare_parameter("clock_source", std::string("vehicle_status"));
    declare_parameter("hover_altitude_m", 5.0);
    declare_parameter("silent_duration_s", 5.0);
    declare_parameter("hover_duration_s", 10.0);
    declare_parameter("figure8_start_command_s", 15.0);
    declare_parameter("mission_end_after_arm_s", 80.0);
    declare_parameter("return_origin_timeout_s", 8.0);
    declare_parameter("return_origin_arrival_radius_m", 1.5);
    declare_parameter("figure8_a", 3.0);
    declare_parameter("circle_radius_m", 0.0);
    declare_parameter("figure8_omega", 0.2);
    declare_parameter("figure8_z_amp_m", 0.0);
    declare_parameter("figure8_z_omega_scale", 2.0);
    declare_parameter("force_figure8_timeout_s", 30.0);
    declare_parameter("auto_offboard_arm", true);
    declare_parameter("control_rate_hz", 100.0);
    declare_parameter("use_fixed_anchor", true);
    declare_parameter("anchor_x_m", 0.0);
    declare_parameter("anchor_y_m", 0.0);
    declare_parameter("arm_after_command_s", 6.0);
    declare_parameter("hold_pose_until_arm", true);
    declare_parameter("climb_after_arm_s", 1.0);
    declare_parameter("climb_ramp_duration_s", 0.0);
    declare_parameter("use_fixed_arm_latch_z", false);
    declare_parameter("fixed_arm_latch_z_ned", -1.5);
    declare_parameter("figure8_start_after_arm_s", 9.0);
    declare_parameter("anchor_from_spawn", false);
    declare_parameter("anchor_settle_s", 2.0);
    declare_parameter("anchor_settle_tol_m", 0.05);
    declare_parameter("anchor_settle_max_s", 10.0);
    declare_parameter("altitude_step_after_arm_s", 0.0);
    declare_parameter("altitude_step_dz_m", 2.0);
    declare_parameter("target_system_id", 1);
    declare_parameter("use_acceleration_feedforward", true);
    declare_parameter("topic.offboard_control_mode", "/fmu/in/offboard_control_mode");
    declare_parameter("topic.trajectory_setpoint", "/fmu/in/trajectory_setpoint");
    declare_parameter("topic.vehicle_command", "/fmu/in/vehicle_command");
    declare_parameter("topic.vehicle_status", "/fmu/out/vehicle_status");
    declare_parameter("topic.vehicle_local_position", "/fmu/out/vehicle_local_position");
  }

  void load_parameters()
  {
    hover_altitude_m_ = get_parameter("hover_altitude_m").as_double();
    silent_duration_s_ = get_parameter("silent_duration_s").as_double();
    hover_duration_s_ = get_parameter("hover_duration_s").as_double();
    figure8_start_command_s_ = get_parameter("figure8_start_command_s").as_double();
    mission_end_after_arm_s_ = get_parameter("mission_end_after_arm_s").as_double();
    return_origin_timeout_s_ = get_parameter("return_origin_timeout_s").as_double();
    return_origin_arrival_radius_m_ =
      get_parameter("return_origin_arrival_radius_m").as_double();
    figure8_a_ = get_parameter("figure8_a").as_double();
    circle_radius_m_ = get_parameter("circle_radius_m").as_double();
    figure8_omega_ = get_parameter("figure8_omega").as_double();
    figure8_z_amp_m_ = get_parameter("figure8_z_amp_m").as_double();
    figure8_z_omega_scale_ = get_parameter("figure8_z_omega_scale").as_double();
    force_figure8_timeout_s_ = get_parameter("force_figure8_timeout_s").as_double();
    auto_offboard_arm_ = get_parameter("auto_offboard_arm").as_bool();
    control_rate_hz_ = get_parameter("control_rate_hz").as_double();
    use_fixed_anchor_ = get_parameter("use_fixed_anchor").as_bool();
    anchor_x_ = get_parameter("anchor_x_m").as_double();
    anchor_y_ = get_parameter("anchor_y_m").as_double();
    arm_after_command_s_ = get_parameter("arm_after_command_s").as_double();
    hold_pose_until_arm_ = get_parameter("hold_pose_until_arm").as_bool();
    climb_after_arm_s_ = get_parameter("climb_after_arm_s").as_double();
    climb_ramp_duration_s_ = get_parameter("climb_ramp_duration_s").as_double();
    use_fixed_arm_latch_z_ = get_parameter("use_fixed_arm_latch_z").as_bool();
    fixed_arm_latch_z_ned_ = get_parameter("fixed_arm_latch_z_ned").as_double();
    figure8_start_after_arm_s_ = get_parameter("figure8_start_after_arm_s").as_double();
    anchor_from_spawn_ = get_parameter("anchor_from_spawn").as_bool();
    anchor_settle_s_ = get_parameter("anchor_settle_s").as_double();
    anchor_settle_tol_m_ = get_parameter("anchor_settle_tol_m").as_double();
    anchor_settle_max_s_ = get_parameter("anchor_settle_max_s").as_double();
    altitude_step_after_arm_s_ = get_parameter("altitude_step_after_arm_s").as_double();
    altitude_step_dz_m_ = get_parameter("altitude_step_dz_m").as_double();
    target_system_id_ = static_cast<uint8_t>(get_parameter("target_system_id").as_int());
    use_accel_ff_ = get_parameter("use_acceleration_feedforward").as_bool();
    topic_offboard_control_mode_ = get_parameter("topic.offboard_control_mode").as_string();
    topic_trajectory_setpoint_ = get_parameter("topic.trajectory_setpoint").as_string();
    topic_vehicle_command_ = get_parameter("topic.vehicle_command").as_string();
    topic_vehicle_status_ = get_parameter("topic.vehicle_status").as_string();
    topic_vehicle_local_position_ = get_parameter("topic.vehicle_local_position").as_string();
    clock_from_local_position_ =
      get_parameter("clock_source").as_string() == "local_position";
    state_machine_ = px4_reference_generator::MissionStateMachine(load_config());
  }

  uint64_t px4_time_us() const
  {
    if (clock_from_local_position_) {
      // High-rate clock: vehicle_local_position stream; status stamp only as fallback
      // before the first position message.
      if (pos_time_us_ > 0) {
        return pos_time_us_;
      }
      return status_time_us_;
    }

    if (status_time_us_ > 0) {
      return status_time_us_;
    }
    return pos_time_us_;
  }

  void control_loop()
  {
    px4_reference_generator::MissionInputs inputs;
    inputs.px4_time_us = px4_time_us();
    inputs.px4_armed_time_us =
      (arming_state_ == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED)
      ? latched_armed_time_us_ : 0;
    inputs.has_local_position = has_local_pos_ || use_fixed_anchor_;
    inputs.is_armed =
      arming_state_ == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
    inputs.is_offboard =
      nav_state_ == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;
    inputs.current_x = current_x_;
    inputs.current_y = current_y_;
    inputs.current_z = current_z_;
    inputs.hover.x = anchor_x_;
    inputs.hover.y = anchor_y_;
    inputs.hover.z_ned = -hover_altitude_m_;

    const auto out = state_machine_.update(inputs);

    if (out.phase != last_phase_) {
      const double since_arm = inputs.px4_armed_time_us > 0
        ? (inputs.px4_time_us - inputs.px4_armed_time_us) * 1e-6 : 0.0;
      RCLCPP_INFO(
        get_logger(), "Phase -> %s (command_el=%.2fs since_arm=%.2fs)",
        px4_reference_generator::phase_to_string(out.phase).c_str(),
        out.command_elapsed_s, since_arm);
      last_phase_ = out.phase;
    }

    const bool stream_offboard =
      out.phase == px4_reference_generator::MissionPhase::Hover ||
      out.phase == px4_reference_generator::MissionPhase::Figure8 ||
      out.phase == px4_reference_generator::MissionPhase::ReturnOrigin ||
      out.phase == px4_reference_generator::MissionPhase::Land;
    if (stream_offboard) {
      publish_offboard_mode();
    }

    if (out.allow_offboard_command && !inputs.is_offboard && (loop_tick_ % 50 == 0)) {
      publish_vehicle_command(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0);
    }
    if (out.allow_arm_command && !inputs.is_armed && (loop_tick_ % 50 == 0)) {
      publish_vehicle_command(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
    }

    if (out.trigger_land_command && !land_command_sent_) {
      publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
      land_command_sent_ = true;
      RCLCPP_WARN(get_logger(), "LAND command sent at origin");
    }

    if (out.publish_setpoint) {
      publish_setpoint(out);
    }

    loop_tick_++;
  }

  void publish_offboard_mode()
  {
    px4_msgs::msg::OffboardControlMode msg;
    msg.position = true;
    msg.velocity = true;
    msg.acceleration = use_accel_ff_;
    msg.timestamp = px4_time_us();
    offboard_mode_pub_->publish(msg);
  }

  void publish_setpoint(const px4_reference_generator::MissionOutputs & out)
  {
    px4_msgs::msg::TrajectorySetpoint msg;
    msg.position = {
      static_cast<float>(out.setpoint.x),
      static_cast<float>(out.setpoint.y),
      static_cast<float>(out.setpoint.z)};
    msg.velocity = {
      static_cast<float>(out.setpoint.vx),
      static_cast<float>(out.setpoint.vy),
      static_cast<float>(out.setpoint.vz)};

    if (use_accel_ff_ &&
      out.phase == px4_reference_generator::MissionPhase::Figure8)
    {
      msg.acceleration = {
        static_cast<float>(out.setpoint.ax),
        static_cast<float>(out.setpoint.ay),
        static_cast<float>(out.setpoint.az)};
    } else {
      msg.acceleration = {0.f, 0.f, 0.f};
    }

    msg.yaw = NAN;
    msg.yawspeed = NAN;
    msg.timestamp = px4_time_us();
    trajectory_pub_->publish(msg);
  }

  void publish_vehicle_command(uint16_t command, float param1 = 0.f, float param2 = 0.f)
  {
    px4_msgs::msg::VehicleCommand msg;
    msg.command = command;
    msg.param1 = param1;
    msg.param2 = param2;
    msg.target_system = target_system_id_;
    msg.target_component = 1;
    msg.source_system = kSourceSystem;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = px4_time_us();
    vehicle_cmd_pub_->publish(msg);
  }

  px4_reference_generator::MissionStateMachine state_machine_;
  px4_reference_generator::MissionPhase last_phase_{px4_reference_generator::MissionPhase::Warmup};

  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_pos_sub_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  double hover_altitude_m_{5.0};
  double silent_duration_s_{5.0};
  double hover_duration_s_{10.0};
  double figure8_start_command_s_{15.0};
  double mission_end_after_arm_s_{80.0};
  double return_origin_timeout_s_{8.0};
  double return_origin_arrival_radius_m_{1.5};
  double figure8_a_{3.0};
  double circle_radius_m_{0.0};
  double figure8_omega_{0.2};
  double figure8_z_amp_m_{0.0};
  double figure8_z_omega_scale_{2.0};
  double force_figure8_timeout_s_{20.0};
  bool auto_offboard_arm_{true};
  double control_rate_hz_{100.0};
  bool use_fixed_anchor_{true};
  double anchor_x_{0.0};
  double anchor_y_{0.0};
  double arm_after_command_s_{6.0};
  bool hold_pose_until_arm_{true};
  double climb_after_arm_s_{1.0};
  double climb_ramp_duration_s_{0.0};
  bool use_fixed_arm_latch_z_{false};
  double fixed_arm_latch_z_ned_{-1.5};
  double figure8_start_after_arm_s_{9.0};
  bool anchor_from_spawn_{false};
  double anchor_settle_s_{2.0};
  double anchor_settle_tol_m_{0.05};
  double anchor_settle_max_s_{10.0};
  double altitude_step_after_arm_s_{0.0};
  double altitude_step_dz_m_{2.0};
  uint8_t target_system_id_{1};
  bool use_accel_ff_{true};

  std::string topic_offboard_control_mode_;
  std::string topic_trajectory_setpoint_;
  std::string topic_vehicle_command_;
  std::string topic_vehicle_status_;
  std::string topic_vehicle_local_position_;

  double current_x_{0.0};
  double current_y_{0.0};
  double current_z_{0.0};
  bool land_command_sent_{false};
  int nav_state_{0};
  int arming_state_{0};
  bool was_armed_{false};
  uint64_t status_time_us_{0};
  uint64_t command_start_us_{0};
  uint64_t latched_armed_time_us_{0};
  uint64_t pos_time_us_{0};
  bool clock_from_local_position_{false};
  int loop_tick_{0};
  bool has_local_pos_{false};
  bool has_altitude_{false};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ReferenceGeneratorNode>());
  rclcpp::shutdown();
  return 0;
}
