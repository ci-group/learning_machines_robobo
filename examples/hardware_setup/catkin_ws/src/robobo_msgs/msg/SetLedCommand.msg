# Commands the robot to change a led to a predifined color
#
# Led identifier; posible values: Front-C, Front-R, 
# Front-RR, Front-L, Front-LL, Front-C, Back-R, Back-L, all.
std_msgs/String id
# Color to set the values to; posible values: white,
# red, blue, cyan, magenta, yellow, green, orange, off.
std_msgs/String color
