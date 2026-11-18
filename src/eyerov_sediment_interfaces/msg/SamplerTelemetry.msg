# Real-time telemetry published by the subsea manipulator & sediment sampler
std_msgs/Header header

# Current manipulator & corer kinematics
float32 arm_extension_cm      # 0.0 = stowed, 50.0 = deployed over seabed
float32 penetration_depth_cm  # 0.0 = surface, >0.0 = depth inside seabed
bool scoop_closed             # true when clamshell/core shutter is sealed
bool canister_locked          # true when sample is docked in sealed bay

# Force, torque & electrical health
float32 axial_torque_nm       # Measured torque/resistance on corer/scoop
float32 motor_current_amp     # Actuator motor current draw
bool hardware_fault           # Internal hardware driver fault flag
