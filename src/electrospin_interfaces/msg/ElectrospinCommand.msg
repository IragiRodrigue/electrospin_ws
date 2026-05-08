# ElectrospinCommand.msg — Unified electrospinning process command
# Published by ai_controller, consumed by robot/collector/pump controllers

std_msgs/Header header

# Robot / needle positioning
float32 target_distance         # Needle-to-collector distance (mm)
float32 target_scan_speed       # Lateral scan speed (mm/s)
float32 scan_amplitude          # Scan peak-to-peak amplitude (mm)

# Collector
float32 target_rpm              # Target collector RPM
bool    collector_enable        # Enable collector motor

# Syringe pump
float32 target_flowrate         # Target flow rate (mL/hr)
bool    pump_enable             # Enable syringe pump

# High voltage
float32 target_voltage          # Target HV voltage (kV)
bool    hv_enable               # Enable high voltage supply

# Command metadata
uint8   source                  # 0=MANUAL, 1=AI_AUTO, 2=REPLAY
float32 confidence              # AI confidence in this command (0–1)
string  rationale               # Human-readable decision rationale
bool    immediate               # Execute immediately (skip smoothing)
