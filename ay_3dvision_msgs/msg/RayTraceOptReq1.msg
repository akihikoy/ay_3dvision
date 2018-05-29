# Type of optimization:
#   "xyz_auto" Optimize x,y,z combining with a couple of lin2d and rot2d.
#   "lin2d"    Optimize two parameters on linear axis1 and axis2.
#   "rot2d"    Optimize two parameters on rotational axis1 and axis2.
string type

geometry_msgs/Point axis1
geometry_msgs/Point axis2
geometry_msgs/Point axis3

# Search ranges and number of division.
#   Actual search range is
#   [-range[0]/2,+range[0]/2], [-range[1]/2,+range[1]/2], ...
#   len(range)==d where d is the number of parameters.
float64[] range
float64 num_div

# Weights of evaluation function.
float64 weight_depth
float64 weight_normal

# Thresholds to check if the optimization is success:
int32   th_quality
float64 th_depth_diff
float64 th_normal_diff
