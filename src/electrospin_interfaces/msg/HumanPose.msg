# HumanPose.msg — Full human upper-body pose from MediaPipe
# Published by human_tracking node

std_msgs/Header header

# Shoulder landmarks (left/right)
float32[3] left_shoulder_position     # x, y, z in camera frame (m)
float32[3] right_shoulder_position
float32 left_shoulder_visibility      # Detection confidence 0–1
float32 right_shoulder_visibility

# Elbow landmarks
float32[3] left_elbow_position
float32[3] right_elbow_position
float32 left_elbow_visibility
float32 right_elbow_visibility

# Wrist landmarks
float32[3] left_wrist_position
float32[3] right_wrist_position
float32 left_wrist_visibility
float32 right_wrist_visibility

# Hand landmarks (21 points per hand, flattened xyz)
float32[63] left_hand_landmarks       # 21 * 3 = 63 floats
float32[63] right_hand_landmarks
float32 left_hand_confidence          # Hand detection confidence 0–1
float32 right_hand_confidence

# Derived arm angles (radians)
float32 left_shoulder_angle           # Shoulder flexion/extension
float32 left_elbow_angle              # Elbow flexion
float32 right_shoulder_angle
float32 right_elbow_angle

# Arm orientation vectors (unit vectors)
float32[3] left_upper_arm_dir         # Shoulder -> elbow direction
float32[3] left_forearm_dir           # Elbow -> wrist direction
float32[3] right_upper_arm_dir
float32[3] right_forearm_dir

# Tracking quality
float32 overall_confidence            # Composite tracking confidence 0–1
bool    person_detected               # At least one person visible
uint8   num_persons                   # Number of detected persons
