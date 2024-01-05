# Commands the robot to move it's pan and/or
# tilt. It won't move unless the UnlockId is 
# different than 0.
#
# Desired pan position. Range 11 .. 343
std_msgs/Int16 panPos
# Desired pan speed
std_msgs/Int8 panSpeed
# Identifier for end-of-movement notification
std_msgs/Int16 panUnlockId
# Desired tilt position. Range 0 .. 100
std_msgs/Int16 tiltPos
# Desired tilt speed
std_msgs/Int8 tiltSpeed
# Identifier for end-of-movement notification
std_msgs/Int16 tiltUnlockId
