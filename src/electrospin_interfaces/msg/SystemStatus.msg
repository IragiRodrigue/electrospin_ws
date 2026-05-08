# SystemStatus.msg — Platform-wide system health status
# Published by system_monitor

std_msgs/Header header

uint8   system_state            # 0=OFF, 1=INIT, 2=READY, 3=RUNNING, 4=ERROR, 5=ESTOP
bool    simulation_mode         # Running in simulation
float32 uptime_s                # Seconds since system start

# Node health (bitmask: each bit = one node alive)
uint8   nodes_alive_mask        # Bit 0=robot, 1=collector, 2=pump, 3=vision, 4=ai

# Safety
bool    emergency_stop          # Global e-stop state
bool    safety_interlock       # Safety interlock OK
float32 temperature_max_c      # Max component temperature

# Quality summary
float32 quality_current         # Latest overall quality
float32 quality_average         # Rolling average quality
uint8   quality_grade           # Current quality grade
