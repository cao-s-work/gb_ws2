# Detailed breakdown of individual scoring criteria for path evaluation
# Usage: Debugging and analysis of path selection decisions

# Safety score (0.0 to 1.0) - obstacle clearance and collision risk
float64 safety_score

# Reference line deviation score (0.0 to 1.0) - adherence to reference path
float64 reference_deviation_score

# Smoothness score (0.0 to 1.0) - curvature and acceleration continuity
float64 smoothness_score

# Efficiency score (0.0 to 1.0) - path length and travel time 
float64 efficiency_score

# Kinematic feasibility score (0.0 to 1.0) - robot motion constraints
float64 kinematic_feasibility_score

# individual weights used in composite calculation
float64 safety_weight
float64 reference_deviation_weight
float64 smoothness_weight
float64 efficiency_weight
float64 kinematic_feasibility_weight

# Processing time for this path evaluation (milliseconds)
float64 evaluation_time_ms