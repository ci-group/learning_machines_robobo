FROM ubuntu:20.04

# This installs ROS, and all its dependencies.
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get -y update && apt-get install -y curl gnupg cmake clang dos2unix && rm -rf /var/lib/apt/lists/*
# GCC, the ubuntu default, isn't supported for ROS.
ENV CXX=clang

RUN echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

RUN apt-get -y update && apt-get install -y ros-noetic-desktop && rm -rf /var/lib/apt/lists/*

# Install any tools you like to have in your terminal here.
RUN apt-get -y update && apt-get install -y git && rm -rf /var/lib/apt/lists/*

# This cd's into a new `catkin_ws` directory anyone starting the shell will end up in.
WORKDIR /root/catkin_ws

# This copies the local catkin_ws into the docker container.
COPY ./catkin_ws .

# Convert the line endings for the Windows users,
# calling `dos2unix` on all files ending in `.py` or `.bash`
RUN find . -type f \( -name '*.py' -o -name '*.bash' \) -exec 'dos2unix' -l -- '{}' \; && apt-get --purge remove -y dos2unix && rm -rf /var/lib/apt/lists/*

# After the line endings are fixed, we can compile the ros code.
RUN bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# Chmod (permit execution of) everything in the catkin_ws. This is not *recommended*,
# But it doesn't matter as long as you don't have any malware in there.
RUN chmod -R u+x /root/catkin_ws/src

# Source the things to bashrc to not have to do that manually.
RUN echo 'source /opt/ros/noetic/setup.bash' >> /root/.bashrc
RUN echo 'source /root/catkin_ws/devel/setup.bash' >> /root/.bashrc
