# ROS ROBOBO EXAMPLE

## Setup

- start docker with `./start-docker.sh`
- change folder to `/root/projects/` (inside docker)
- run `catkin_make install` (only run for the first setup, it's not needed even across different docker containers)
- run `source devel/setup.bash`
- run `ROS_MASTER_URI='http://192.168.1.11:11311' src/send_commands.py` command to test the connection

The environment variable `ROS_MASTER_URI` idendifies where the ROS MASTER NODE is running. Verify that your phone is in "Local ROS Master" and adjust the IP of the `ROS_MASTER_URI` variable with the address of the phone (visible at the first page of the application).