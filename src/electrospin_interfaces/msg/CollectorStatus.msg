# CollectorStatus.msg — Collector drum motor status
# Published by collector_controller

std_msgs/Header header

float32 rpm                     # Current RPM
float32 target_rpm              # Target RPM setpoint
float32 torque                  # Motor torque (normalized 0–1)
bool    running                 # Motor is spinning
bool    at_setpoint             # RPM within tolerance of target
float32 vibration_score        # Vibration severity (0–1)
float32 temperature_c          # Motor temperature (Celsius)
float32 current_a              # Motor current draw (Amps)
float32 duty_cycle             # PWM duty cycle (0–1)
bool    emergency_stop          # E-stop is active
bool    ramp_active             # Velocity ramp in progress
float32 encoder_position       # Encoder angle (rad, modulo 2pi)
uint8   control_mode           # 0=IDLE, 1=OPEN_LOOP, 2=PID, 3=ESTOP
