# FiberQuality.msg — Real-time nanofiber quality assessment
# Published by vision_system, consumed by ai_controller

std_msgs/Header header

# Morphology
float32 uniformity              # Fiber diameter uniformity (0–1)
float32 diameter                # Average fiber diameter (nm)
float32 bead_score              # Bead formation severity (0=none, 1=severe)
float32 alignment_score         # Fiber alignment quality (0–1)

# Jet / Taylor cone
bool    jet_stable              # Whether the electrospinning jet is stable
float32 taylor_cone_score       # Taylor cone shape quality (0–1)
float32 jet_straightness       # Jet straightness score (0–1)

# Deposition
float32 deposition_density      # Surface coverage density (0–1)
float32 coverage_uniformity    # Spatial uniformity of deposition (0–1)

# Composite
float32 overall_quality         # Weighted composite quality score (0–1)
uint8   quality_grade           # 0=unusable, 1=poor, 2=fair, 3=good, 4=excellent
string  diagnosis               # Human-readable diagnosis string
