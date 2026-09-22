#pragma once

#include <cstdint>
#include <string>

#include "px4_reference_generator/lemniscate_trajectory.hpp"

namespace px4_reference_generator
{

enum class MissionPhase
{
  Warmup,
  Silent,
  Hover,
  Figure8,
  ReturnOrigin,
  Land,
  Done
};

std::string phase_to_string(MissionPhase phase);

struct MissionConfig
{
  double hover_altitude_m{5.0};
  double silent_duration_s{5.0};
  double hover_duration_s{10.0};
  double figure8_start_command_s{15.0};
  double mission_end_after_arm_s{80.0};
  double return_origin_timeout_s{8.0};
  double return_origin_arrival_radius_m{1.5};
  double figure8_a{3.0};
  double figure8_omega{0.2};
  /** Vertical sinusoid superposed on the figure-8 (z = z_hover + amp*sin(scale*omega*t)).
   *  amp=0 → planar figure-8 (legacy). Adds a dynamic Z-tracking dimension; keep
   *  amp*(scale*omega)^2 well below the shared acc limit (feasibility logged at start). */
  double figure8_z_amp_m{0.0};
  double figure8_z_omega_scale{2.0};
  // P2 certified-corridor experiment: >0 replaces the lemniscate with a circle
  // of this radius. Default 0.0 keeps the figure-8 behaviour verbatim.
  double circle_radius_m{0.0};
  double force_figure8_timeout_s{30.0};
  bool auto_offboard_arm{true};
  double control_rate_hz{100.0};
  bool use_fixed_anchor{true};
  double anchor_x_m{0.0};
  double anchor_y_m{0.0};
  double arm_after_command_s{6.0};
  bool hold_pose_until_arm{true};
  /** Seconds after arm before switching setpoint to hover altitude. */
  double climb_after_arm_s{1.0};
  /** Ramp z from arm latch to hover over this duration (0 = step). */
  double climb_ramp_duration_s{0.0};
  /** Override arm-latch z (NED) for fair compare; off when false. */
  bool use_fixed_arm_latch_z{false};
  double fixed_arm_latch_z_ned{-1.5};
  /** Figure-8 entry delay after arm when hold_pose_until_arm (replaces command-based start). */
  double figure8_start_after_arm_s{9.0};
  /** Latch XY anchor at first EKF position; hover z = spawn_z - hover_altitude (relative to spawn).
   *  Makes initial tracking error ~0 in XY and exactly -hover_altitude in Z (EKF frame),
   *  independent of Gazebo spawn offset and EKF height datum. Implies use_fixed_anchor=false. */
  bool anchor_from_spawn{false};
  /** Anchor latch waits until EKF z stays within ±anchor_settle_tol_m for anchor_settle_s
   *  (skips the EKF height-datum initialization transient); latches unconditionally after
   *  anchor_settle_max_s or on arm. Keeps the spawn-relative initial error identical across
   *  vehicles/runs regardless of estimator warm-up. */
  double anchor_settle_s{2.0};
  double anchor_settle_tol_m{0.05};
  double anchor_settle_max_s{10.0};
  /** P4 timed-arrival excitation: at arm+altitude_step_after_arm_s the hover z setpoint
   *  steps up by altitude_step_dz_m (a true discontinuity, no ramp). 0 = disabled
   *  (legacy behaviour verbatim). Pair with a late figure8 start so Hover persists. */
  double altitude_step_after_arm_s{0.0};
  double altitude_step_dz_m{2.0};
};

struct MissionInputs
{
  uint64_t px4_time_us{0};
  uint64_t px4_armed_time_us{0};
  bool has_local_position{false};
  bool is_armed{false};
  bool is_offboard{false};
  double current_x{0.0};
  double current_y{0.0};
  double current_z{0.0};
  HoverTarget hover{};
};

struct MissionOutputs
{
  MissionPhase phase{MissionPhase::Warmup};
  TrajectorySample setpoint{};
  bool allow_offboard_command{false};
  bool allow_arm_command{false};
  bool publish_setpoint{false};
  bool trigger_land_command{false};
  double figure8_time_s{0.0};
  double command_elapsed_s{0.0};
};

class MissionStateMachine
{
public:
  explicit MissionStateMachine(MissionConfig config);

  void reset();
  MissionOutputs update(const MissionInputs & inputs);

  MissionPhase phase() const { return phase_; }

private:
  double since_arm_s(const MissionInputs & inputs) const;
  double since_command_s(const MissionInputs & inputs) const;
  bool at_origin(const MissionInputs & inputs) const;
  double hover_end_command_s() const;
  void latch_hold_pose(const MissionInputs & inputs);
  void latch_arm_hold_pose(const MissionInputs & inputs);
  TrajectorySample hold_pose_sample() const;
  TrajectorySample arm_hold_pose_sample() const;
  TrajectorySample climb_ramp_sample(double since_arm_s) const;
  TrajectorySample climb_hover_sample() const;
  /** Hover target z (NED): spawn-relative when anchor_from_spawn, else absolute -hover. */
  double effective_hover_z() const;
  void latch_spawn_anchor(const MissionInputs & inputs);

  MissionConfig config_;
  MissionPhase phase_{MissionPhase::Warmup};
  uint64_t command_start_us_{0};
  uint64_t figure8_start_us_{0};
  uint64_t return_origin_start_us_{0};
  uint64_t land_start_us_{0};
  double land_start_z_{0.0};
  bool hold_pose_latched_{false};
  bool arm_hold_latched_{false};
  double hold_x_{0.0};
  double hold_y_{0.0};
  double hold_z_{0.0};
  double arm_hold_x_{0.0};
  double arm_hold_y_{0.0};
  double arm_hold_z_{0.0};
  bool spawn_latched_{false};
  double spawn_z_ned_{0.0};
  // anchor settle tracking (EKF z transient rejection)
  double settle_cand_z_{0.0};
  uint64_t settle_cand_us_{0};
  uint64_t settle_first_valid_us_{0};
  HoverTarget anchor_{};
  LemniscateTrajectory trajectory_{5.0, 0.3, -5.0, 0.0, 0.0};
};

}  // namespace px4_reference_generator
