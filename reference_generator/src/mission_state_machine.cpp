#include "px4_reference_generator/mission_state_machine.hpp"

#include <cmath>
#include <algorithm>

namespace px4_reference_generator
{

std::string phase_to_string(MissionPhase phase)
{
  switch (phase) {
    case MissionPhase::Warmup: return "warmup";
    case MissionPhase::Silent: return "silent";
    case MissionPhase::Hover: return "hover";
    case MissionPhase::Figure8: return "figure8";
    case MissionPhase::ReturnOrigin: return "return_origin";
    case MissionPhase::Land: return "land";
    case MissionPhase::Done: return "done";
    default: return "unknown";
  }
}

MissionStateMachine::MissionStateMachine(MissionConfig config)
: config_(config),
  trajectory_(
    config.figure8_a, config.figure8_omega,
    -config.hover_altitude_m, config.anchor_x_m, config.anchor_y_m)
{
  if (config_.use_fixed_anchor) {
    anchor_.x = config_.anchor_x_m;
    anchor_.y = config_.anchor_y_m;
    anchor_.z_ned = -config_.hover_altitude_m;
  }
}

void MissionStateMachine::reset()
{
  phase_ = MissionPhase::Warmup;
  command_start_us_ = 0;
  figure8_start_us_ = 0;
  return_origin_start_us_ = 0;
  hold_pose_latched_ = false;
  arm_hold_latched_ = false;
  hold_x_ = 0.0;
  hold_y_ = 0.0;
  hold_z_ = 0.0;
  arm_hold_x_ = 0.0;
  arm_hold_y_ = 0.0;
  arm_hold_z_ = 0.0;
  spawn_latched_ = false;
  spawn_z_ned_ = 0.0;
  settle_cand_z_ = 0.0;
  settle_cand_us_ = 0;
  settle_first_valid_us_ = 0;
  anchor_ = HoverTarget{};
  if (config_.use_fixed_anchor && !config_.anchor_from_spawn) {
    anchor_.x = config_.anchor_x_m;
    anchor_.y = config_.anchor_y_m;
    anchor_.z_ned = -config_.hover_altitude_m;
  }
}

double MissionStateMachine::since_arm_s(const MissionInputs & inputs) const
{
  if (inputs.px4_armed_time_us == 0 || inputs.px4_time_us <= inputs.px4_armed_time_us) {
    return 0.0;
  }
  return (inputs.px4_time_us - inputs.px4_armed_time_us) * 1e-6;
}

double MissionStateMachine::since_command_s(const MissionInputs & inputs) const
{
  if (command_start_us_ == 0 || inputs.px4_time_us <= command_start_us_) {
    return 0.0;
  }
  return (inputs.px4_time_us - command_start_us_) * 1e-6;
}

double MissionStateMachine::hover_end_command_s() const
{
  return config_.silent_duration_s + config_.hover_duration_s;
}

bool MissionStateMachine::at_origin(const MissionInputs & inputs) const
{
  const double dx = inputs.current_x - anchor_.x;
  const double dy = inputs.current_y - anchor_.y;
  return std::hypot(dx, dy) <= config_.return_origin_arrival_radius_m;
}

void MissionStateMachine::latch_hold_pose(const MissionInputs & inputs)
{
  if (hold_pose_latched_ || !inputs.has_local_position) {
    return;
  }
  hold_x_ = config_.use_fixed_anchor && !config_.anchor_from_spawn
    ? anchor_.x : inputs.current_x;
  hold_y_ = config_.use_fixed_anchor && !config_.anchor_from_spawn
    ? anchor_.y : inputs.current_y;
  hold_z_ = inputs.current_z;
  hold_pose_latched_ = true;
}

TrajectorySample MissionStateMachine::hold_pose_sample() const
{
  TrajectorySample sp;
  sp.x = hold_x_;
  sp.y = hold_y_;
  sp.z = hold_z_;
  return sp;
}

void MissionStateMachine::latch_arm_hold_pose(const MissionInputs & inputs)
{
  if (arm_hold_latched_ || !inputs.has_local_position || !inputs.is_armed) {
    return;
  }
  arm_hold_x_ = config_.use_fixed_anchor && !config_.anchor_from_spawn
    ? anchor_.x : inputs.current_x;
  arm_hold_y_ = config_.use_fixed_anchor && !config_.anchor_from_spawn
    ? anchor_.y : inputs.current_y;
  arm_hold_z_ = config_.use_fixed_arm_latch_z
    ? config_.fixed_arm_latch_z_ned
    : inputs.current_z;
  arm_hold_latched_ = true;
}

TrajectorySample MissionStateMachine::arm_hold_pose_sample() const
{
  TrajectorySample sp;
  sp.x = arm_hold_x_;
  sp.y = arm_hold_y_;
  sp.z = arm_hold_z_;
  return sp;
}

TrajectorySample MissionStateMachine::climb_hover_sample() const
{
  HoverTarget target = anchor_;
  target.z_ned = effective_hover_z();
  return hover_sample(target);
}

double MissionStateMachine::effective_hover_z() const
{
  return config_.anchor_from_spawn && spawn_latched_
    ? (spawn_z_ned_ - config_.hover_altitude_m)
    : -config_.hover_altitude_m;
}

void MissionStateMachine::latch_spawn_anchor(const MissionInputs & inputs)
{
  if (!config_.anchor_from_spawn || spawn_latched_ || !inputs.has_local_position) {
    return;
  }
  // Reject the EKF height-datum initialization transient: only latch once z has stayed
  // within ±anchor_settle_tol_m of a candidate for anchor_settle_s. Fallbacks: latch
  // unconditionally after anchor_settle_max_s, or immediately when already armed
  // (the mission must have an anchor before commanding).
  const uint64_t now = inputs.px4_time_us;
  if (settle_first_valid_us_ == 0) {
    settle_first_valid_us_ = now;
    settle_cand_z_ = inputs.current_z;
    settle_cand_us_ = now;
  }
  const bool stable = std::fabs(inputs.current_z - settle_cand_z_) <= config_.anchor_settle_tol_m;
  if (!stable) {
    settle_cand_z_ = inputs.current_z;
    settle_cand_us_ = now;
  }
  const bool settled = stable && (now - settle_cand_us_) * 1e-6 >= config_.anchor_settle_s;
  const bool timed_out = (now - settle_first_valid_us_) * 1e-6 >= config_.anchor_settle_max_s;
  if (!settled && !timed_out && !inputs.is_armed) {
    return;
  }
  anchor_.x = inputs.current_x;
  anchor_.y = inputs.current_y;
  spawn_z_ned_ = inputs.current_z;
  spawn_latched_ = true;
  anchor_.z_ned = effective_hover_z();
}

TrajectorySample MissionStateMachine::climb_ramp_sample(double since_arm_s) const
{
  const double hover_alt_z = effective_hover_z();
  const double ramp_start = config_.climb_after_arm_s;
  const double dur = config_.climb_ramp_duration_s;
  const double u = dur > 1e-6
    ? std::clamp((since_arm_s - ramp_start) / dur, 0.0, 1.0)
    : 1.0;
  const double s = u * u * (3.0 - 2.0 * u);

  TrajectorySample sp;
  sp.x = arm_hold_x_;
  sp.y = arm_hold_y_;
  sp.z = arm_hold_z_ + s * (hover_alt_z - arm_hold_z_);
  if (dur > 1e-6 && u < 1.0) {
    const double ds_du = 6.0 * u * (1.0 - u);
    sp.vz = (hover_alt_z - arm_hold_z_) / dur * ds_du;
  }
  return sp;
}

MissionOutputs MissionStateMachine::update(const MissionInputs & inputs)
{
  MissionOutputs out;
  out.phase = phase_;

  if (inputs.has_local_position && !config_.use_fixed_anchor && !config_.anchor_from_spawn) {
    anchor_.x = inputs.hover.x;
    anchor_.y = inputs.hover.y;
  }

  if (command_start_us_ == 0 && inputs.has_local_position && inputs.px4_time_us > 0) {
    command_start_us_ = inputs.px4_time_us;
  }

  const double command_elapsed = since_command_s(inputs);
  const double since_arm = since_arm_s(inputs);
  out.command_elapsed_s = command_elapsed;

  latch_spawn_anchor(inputs);

  const double hover_alt_z = effective_hover_z();
  if (!config_.hold_pose_until_arm || inputs.is_armed) {
    anchor_.z_ned = hover_alt_z;
  } else if (hold_pose_latched_) {
    anchor_.z_ned = hold_z_;
  } else if (inputs.has_local_position) {
    anchor_.z_ned = inputs.current_z;
  }

  switch (phase_) {
    case MissionPhase::Warmup:
      out.publish_setpoint = false;
      if (inputs.has_local_position && command_start_us_ > 0) {
        phase_ = MissionPhase::Silent;
      }
      break;

    case MissionPhase::Silent:
      out.publish_setpoint = false;
      if (command_elapsed >= config_.silent_duration_s) {
        phase_ = MissionPhase::Hover;
      }
      break;

    case MissionPhase::Hover:
      latch_hold_pose(inputs);
      latch_arm_hold_pose(inputs);
      out.publish_setpoint = true;
      if (config_.hold_pose_until_arm &&
        (!inputs.is_armed || since_arm < config_.climb_after_arm_s))
      {
        out.setpoint = inputs.is_armed ? arm_hold_pose_sample() : hold_pose_sample();
      } else if (config_.climb_ramp_duration_s > 1e-6 &&
        since_arm < config_.climb_after_arm_s + config_.climb_ramp_duration_s)
      {
        out.setpoint = climb_ramp_sample(since_arm);
      } else {
        out.setpoint = climb_hover_sample();
      }
      if (config_.altitude_step_after_arm_s > 1e-6 && inputs.is_armed &&
        since_arm >= config_.altitude_step_after_arm_s)
      {
        out.setpoint.z = effective_hover_z() - config_.altitude_step_dz_m;
      }
      out.allow_offboard_command = config_.auto_offboard_arm &&
        command_elapsed >= config_.silent_duration_s;
      out.allow_arm_command = out.allow_offboard_command &&
        inputs.is_offboard &&
        command_elapsed >= config_.arm_after_command_s;

      if (config_.hold_pose_until_arm) {
        if (inputs.is_armed && since_arm >= config_.figure8_start_after_arm_s) {
          figure8_start_us_ = inputs.px4_time_us;
          phase_ = MissionPhase::Figure8;
        } else if (command_elapsed >= config_.force_figure8_timeout_s) {
          figure8_start_us_ = inputs.px4_time_us;
          phase_ = MissionPhase::Figure8;
        }
      } else if (command_elapsed >= config_.figure8_start_command_s) {
        figure8_start_us_ = inputs.px4_time_us;
        phase_ = MissionPhase::Figure8;
      } else if (command_elapsed >= config_.force_figure8_timeout_s) {
        figure8_start_us_ = inputs.px4_time_us;
        phase_ = MissionPhase::Figure8;
      }
      break;

    case MissionPhase::Figure8:
      if (figure8_start_us_ == 0) {
        figure8_start_us_ = inputs.px4_time_us;
      }
      if (inputs.is_armed && since_arm >= config_.mission_end_after_arm_s) {
        return_origin_start_us_ = inputs.px4_time_us;
        phase_ = MissionPhase::ReturnOrigin;
        break;
      }
      trajectory_ = LemniscateTrajectory(
        config_.figure8_a, config_.figure8_omega,
        hover_alt_z, anchor_.x, anchor_.y,
        config_.figure8_z_amp_m,
        config_.figure8_z_omega_scale * config_.figure8_omega);
      out.figure8_time_s = (inputs.px4_time_us - figure8_start_us_) * 1e-6;
      out.publish_setpoint = true;
      out.setpoint = (config_.circle_radius_m > 0.0)
        ? circle_sample(
            config_.circle_radius_m, config_.figure8_omega, hover_alt_z,
            anchor_.x, anchor_.y, config_.figure8_z_amp_m,
            config_.figure8_z_omega_scale * config_.figure8_omega,
            out.figure8_time_s)
        : trajectory_.sample(out.figure8_time_s);
      break;

    case MissionPhase::ReturnOrigin:
      if (return_origin_start_us_ == 0) {
        return_origin_start_us_ = inputs.px4_time_us;
      }
      out.publish_setpoint = true;
      anchor_.z_ned = hover_alt_z;
      out.setpoint = hover_sample(anchor_);
      if (at_origin(inputs) ||
        (inputs.px4_time_us - return_origin_start_us_) * 1e-6 >= config_.return_origin_timeout_s)
      {
        phase_ = MissionPhase::Land;
      }
      break;

    case MissionPhase::Land: {
      // Smooth offboard descent (mirrors the smooth climb ramp) so BPPSMC tracks a
      // continuous z-ramp with velocity feed-forward, instead of handing off to PX4
      // AUTO_LAND -- which BPPSMC chatters on (acc_sp_z saturates +/-10 => the "trembling"
      // landing). Only at ground level do we trigger the land/disarm command.
      if (land_start_us_ == 0) {
        land_start_us_ = inputs.px4_time_us;
        land_start_z_ = inputs.current_z;
      }
      const double descent_rate = 0.7;                 // m/s, gentle (~MPC_LAND_SPEED)
      const double ground_z = spawn_z_ned_ + 0.15;     // just above the spawn ground
      const double t_land = (inputs.px4_time_us - land_start_us_) * 1e-6;
      double z_cmd = land_start_z_ + descent_rate * t_land;   // NED down grows
      const bool touched = z_cmd >= ground_z;
      if (touched) { z_cmd = ground_z; }
      anchor_.z_ned = z_cmd;
      out.publish_setpoint = true;
      out.setpoint = hover_sample(anchor_);
      out.setpoint.z = z_cmd;
      out.setpoint.vz = touched ? 0.0 : descent_rate;        // velocity feed-forward
      if (touched) {
        out.trigger_land_command = true;   // disarm at ground (already down => no chatter)
        phase_ = MissionPhase::Done;
      }
      break;
    }

    case MissionPhase::Done:
      out.publish_setpoint = false;
      break;
  }

  out.phase = phase_;
  return out;
}

}  // namespace px4_reference_generator
