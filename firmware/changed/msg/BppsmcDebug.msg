## ET-BPPSMC algorithm holographic debug log (ULog only)
##
## NOTE:
## - Field names must be strictly lower_snake_case (no uppercase).
## - Absolute timestamps must be uint64 (microseconds) to avoid precision loss.

uint64 timestamp                # [us] absolute timestamp since boot

float32[3] rho_center           # [m] prescribed-performance tube center (rho)
float32[3] sliding_surface      # [-] sliding surface S
uint32[3] trigger_count         # [-] per-axis event trigger count

float32[3] acceleration_sp      # [m/s^2] injected total acceleration setpoint

float32[3] e0                   # [m] clipped initial error at last trigger
uint64[3] t0                    # [us] absolute timestamp of last trigger
