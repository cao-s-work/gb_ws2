# Trajectory with associated scoring information for multi-path selection
# Usage: Used by MultiPathSelectorAction to evaluate and select optimal trajectories

# The trajectory definition with time and kinematic paramters
robots_dog_msgs/Trajectory trajectory

# Overall composite score (0.0 to 1.0, higher is better)
float64 composite_score

# Individual scoring breakdown for analysis/debugging
PathScoreBreakdown score_breakdown

# Unique identifier for this trajectory candidate
string trajectory_id

# Generation timestamp
builtin_interfaces/Time stamp