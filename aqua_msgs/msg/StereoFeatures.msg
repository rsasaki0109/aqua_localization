# Tracked features of one rectified stereo frame (left camera stamp).
# Feature ids persist across frames for as long as a feature is tracked, so a
# backend can build landmark tracks; u/v are pixel coordinates in the
# rectified left and right images (v is shared after rectification).
std_msgs/Header header

uint64[] ids
float32[] u_left
float32[] u_right
float32[] v
