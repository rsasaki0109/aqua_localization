std_msgs/Header header

# Existing keyframe IDs in aqua_pose_graph.
uint32 from_id
uint32 to_id

# Relative SE(3) measurement from `from_id` to `to_id`.
geometry_msgs/Pose relative_pose

# Row-major 6x6 information matrix for [x, y, z, roll, pitch, yaw].
# Larger values make the loop closure tighter.
float64[36] information

# Run g2o immediately after accepting this constraint. Leave false when a
# front end batches several candidate closures and calls /aqua_pose_graph/optimize.
bool optimize_after_insert
