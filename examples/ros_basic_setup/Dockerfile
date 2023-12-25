FROM ros:noetic

RUN apt-get update -y && apt-get install -y python3 python3-pip git && rm -rf /var/lib/apt/lists/*

# Install dependencies.
# The python3 interpreter is already being shilled by ros:noetic, so no need for a venv.
COPY ./requirements.txt /requirements.txt
RUN python3 -m pip install -r /requirements.txt && rm /requirements.txt

# This cd's into a new `catkin_ws` directory anyone starting the shell will end up in.
WORKDIR /root/catkin_ws

# This copies the local catkin_ws into the docker container, and then runs catkin_make on it.
# The EOT creates a small inline bash script to execute. This is to allow setup.bash to run in the same shell,
# which exports some global variables catkin_make needs to find the python packages.
# (By default, Docker runs all RUN commands in seperately created shells)
COPY ./catkin_ws .
RUN <<EOT bash
    source /opt/ros/noetic/setup.bash
    catkin_make
EOT
# Chmod (permit execution of) everything in the catkin_ws. This is not *recommended*,
# But it doesn't matter as long as you don't have any malware in there.
RUN chmod -R +x /root/catkin_ws/src

# Source the things to bashrc to not have to do that manually, and have the initial bash shell set up properly
RUN echo 'source /opt/ros/noetic/setup.bash' >> /root/.bashrc
RUN echo 'source /root/catkin_ws/devel/setup.bash' >> /root/.bashrc
