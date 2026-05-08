# HandGesture.msg — Gesture recognition result from hand tracking
# Published by human_tracking node

std_msgs/Header header

uint8   gesture_id                    # Gesture enum: 0=NONE, 1=FIST, 2=OPEN, 3=POINT, 4=THUMBS_UP, 5=PEACE, 6=GRAB, 7=RELEASE
string  gesture_name                  # Human-readable gesture name
float32 confidence                    # Gesture classification confidence 0–1

# Which hand(s)
bool    left_hand                     # Gesture from left hand
bool    right_hand                    # Gesture from right hand

# Command mapping
uint8  command                        # Mapped robot command: 0=NONE, 1=START, 2=STOP, 3=ESTOP, 4=MODE_SWITCH, 5=RESET

# Finger states (0=extended, 1=flexed) for each hand
uint8[5] left_finger_states           # Thumb, Index, Middle, Ring, Pinky
uint8[5] right_finger_states

# Pinch distance (thumb tip to index tip, meters)
float32 left_pinch_distance
float32 right_pinch_distance
