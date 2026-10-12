# Health and bias diagnostics of the aqua_graph factor-graph estimator,
# published once per state-grid step.
std_msgs/Header header

uint64 state_index
uint64 imu_samples
uint64 dvl_received
uint64 dvl_applied
uint64 dvl_rejected
uint64 depth_received
uint64 depth_applied
uint64 position_received
uint64 position_applied
uint64 stereo_frames
uint64 stereo_observations
uint64 landmarks_created
# Measurements whose grid state had already left the smoother window.
uint64 measurements_too_old
uint64 smoother_updates
# Times the smoother was re-anchored after a numerically singular update.
uint64 smoother_resets
uint64 heading_received
uint64 heading_applied
# Map reuse: landmarks brought back from the map / re-identified tracks
# rejected by the reprojection check.
uint64 map_reobservations
uint64 map_rejections

geometry_msgs/Vector3 velocity_world
geometry_msgs/Vector3 accel_bias
geometry_msgs/Vector3 gyro_bias
