uint64 timestamp
bool enabled                 # Owns the onboard experiment; use native PID outside an active task
bool active                  # Actively generating OFFBOARD references
uint8 stage
uint8 STAGE_DISABLED = 0
uint8 STAGE_WAITING = 1
uint8 STAGE_READY = 2
uint8 STAGE_TAKEOFF = 3
uint8 STAGE_HOVER = 4
uint8 STAGE_FIGURE8 = 5
uint8 STAGE_RETURN_HOLD = 6
uint8 STAGE_LANDING = 7
uint8 STAGE_TOUCHDOWN = 8
uint8 STAGE_COMPLETE = 9
uint8 STAGE_ABORTED = 10
uint8 controller             # MPC_CTRL_METHOD latched at start
uint8 lap
uint8 failure                # 0 none, 1 envelope, 2 tracking, 3 hover timeout, 4 landing timeout, 5 contact lost
float32 stage_elapsed
float32 tracking_error
