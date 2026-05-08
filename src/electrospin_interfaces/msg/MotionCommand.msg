# MotionCommand.msg — Mapped robot motion from human pose
# Published by motion_mapping node, consumed by robot_controller

std_msgs/Header header

# Target joint angles (radians) — maps to MyCobot 6 joints
float32[6] target_joint_angles

# Target end-effector pose (alternative representation)
float32[3] target_position            # x, y, z in robot base frame (m)
float32[4] target_orientation         # qx, qy, qz, w quaternion

# Motion metadata
float32 confidence                    # Mapping confidence 0–1
bool   is_safe                        # Motion passes safety checks
uint8  source                         # 0=TELEOP_LEFT, 1=TELEOP_RIGHT, 2=GESTURE, 3=AUTO

# Smoothing info
float32[6] velocity_hints             # Suggested joint velocities (rad/s)
float32 latency_ms                    # Estimated end-to-end latency
