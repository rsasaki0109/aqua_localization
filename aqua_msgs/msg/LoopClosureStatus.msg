std_msgs/Header header

# Endpoint keyframe stamps are copied into the status message so status-only
# bags still preserve replay-stable loop identity diagnostics.
builtin_interfaces/Time current_keyframe_stamp
builtin_interfaces/Time candidate_keyframe_stamp

uint32 current_id
# UINT32_MAX means no candidate submap was available for this keyframe.
uint32 candidate_id

bool accepted
bool converged

float64 fitness_score
float64 correction_translation_m
float64 correction_rotation_rad
# Correction from the odometry-based initial guess to the registration result.
# This preserves direction for offline consistency/audit tooling. False means
# registration did not produce a finite correction for this candidate.
bool correction_pose_valid
geometry_msgs/Pose correction_pose

# Diagnostics for the accepted-loop consistency guard. Counts are zero and
# deltas are NaN when the guard was not evaluated for this candidate.
uint32 consistency_support_count
uint32 consistency_required_support_count
float64 consistency_nearest_translation_delta_m
float64 consistency_nearest_rotation_delta_rad

# Pre-registration descriptor metrics. NaN means unavailable or not evaluated.
float64 descriptor_centroid_distance_m
float64 descriptor_extent_ratio
float64 descriptor_point_count_ratio

string status
