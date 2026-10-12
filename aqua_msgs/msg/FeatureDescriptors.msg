# Appearance descriptors of feature tracks, published once per track when it
# starts (they do not change while it is tracked). descriptors holds
# descriptor_bytes bytes per id, in order (ORB: 32).
std_msgs/Header header

uint32 descriptor_bytes
uint64[] ids
uint8[] descriptors
