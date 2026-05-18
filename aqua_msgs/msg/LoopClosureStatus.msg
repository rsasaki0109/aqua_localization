std_msgs/Header header

uint32 current_id
# UINT32_MAX means no candidate submap was available for this keyframe.
uint32 candidate_id

bool accepted
bool converged

float64 fitness_score
float64 correction_translation_m
float64 correction_rotation_rad

string status
