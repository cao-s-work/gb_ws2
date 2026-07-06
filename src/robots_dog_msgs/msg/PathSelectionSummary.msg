# Summary information about trajectory selection process
# Usage: Performance monitoring and Debugging

# Total number of input candidate trajectories
uint32 total_candidate_trajectories

# Number of trajectories that passed fast pruning
uint32 trajectories_passed_pruning

# Index of selected trajectory in original candidate array
uint32 selected_trajectory_index

# Total processing time breakdown (milliseconds)
float64 fast_pruning_time_ms
float64 detailed_scoring_time_ms
float64 total_processing_time_ms

# Best composite score achieved
float64 best_composite_score

# Array of failure reasons per trajectory
string[] trajectory_failure_reasons

#Selection timestamp
builtin_interfaces/Time selection_timestamp

# Performace flags
bool met_timeout_requirement # Processing completed within timeout
bool used_fast_pruning       # Fast pruning was applied
bool used_default_weights    # Default weights were used