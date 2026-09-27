# Reference-plane tray frame in base_link; never a commanded TCP attitude.
string id
int32 class_id
string class_name
float64 confidence
geometry_msgs/Pose pose
# Metres. Length/width are sorted long/short; frame X need not be the long edge.
float64 length
float64 width
float64 extent_x
float64 extent_y
float64 center_distance_px
