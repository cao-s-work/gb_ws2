# Configurable weights for path selection criteria
# Usage: Allows dynamic adjustment of scoring priorities

# Safety criterion weight (recommended: 0.35)
float64 safety_weight

# Reference deviation criterion weight (recommended: 0.25)
float64 reference_deviation_weight

# Smoothness criterion_weight (recommended: 0.20)
float64 smoothness_weight

# Efficiency criterion weight (recommended: 0.15)
float64 efficiency_weight

# Kinematic feasibility criterion weight (recommended: 0.05)
float64 kinematic_feasibility_weight

# Flag to use default weights if all weights are zero
bool use_default_weights