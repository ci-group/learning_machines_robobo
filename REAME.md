# ROS ROBOBO EXAMPLE

## Setup (with cigroup/learning-machines image)

- start docker with `./start-docker.sh`
- run `src/send_commands.py --ros_master_uri 'http://192.168.1.11:11311' ` command to test the connection

## Setup (with ros image)

- start docker with `docker run --rm -it -v "${PROJECT_FOLDER}:/root/projects/" cigroup/learning-machines bash`
- change folder to `/root/projects/` (inside docker)
- run `catkin_make install` (only run for the first setup, it's not needed even across different docker containers)
- run `source devel/setup.bash`
- run `src/send_commands.py --ros_master_uri 'http://192.168.1.11:11311' ` command to test the connection

### Important Notes

The environment variable `ROS_MASTER_URI` idendifies where the ROS MASTER NODE is running. Verify that your phone is in "Local ROS Master" and adjust the IP of the `ROS_MASTER_URI` variable with the address of the phone (visible at the first page of the application).