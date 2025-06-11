FROM ros:noetic as base

RUN apt-get update -y && apt-get install -y python3 python3-pip git dos2unix && rm -rf /var/lib/apt/lists/*

# Install dependencies.

# These are package requirements for the dependencies.
# You should add to these if you add python packages that require c libraries to be installed
RUN apt-get update -y && apt-get install ffmpeg libsm6 libxext6 ros-noetic-opencv-apps -y && rm -rf /var/lib/apt/lists/*

# The python3 interpreter is already being shilled by ros:noetic, so no need for a venv.
COPY ./requirements.txt /requirements.txt
RUN python3 -m pip install -r /requirements.txt && rm /requirements.txt

# Build the CPP stuff in a seperate stage, such that we can take advantage of Docker's cashing mechanics.
# (As long as you don't touch the `robobo_msgs` package, this stage is going to be completely cached.)
FROM base as builder

WORKDIR /root/catkin_ws

COPY ./catkin_ws/.catkin_workspace ./.catkin_workspace
COPY ./catkin_ws/src/robobo_msgs ./src/robobo_msgs
RUN bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

FROM base

# Makking sure our ROS node has ports to connect trough.
# These are the ports specified in `rospy.init_node()` in hardware.py
EXPOSE 45100
EXPOSE 45101

# This cd's into a new `catkin_ws` directory anyone starting the shell will end up in.
WORKDIR /root/catkin_ws

# Copy from the cache to here. With the `build` and `devel` directories copied from cache,
# catkin_make can do an incremental compile instead, massivly speeding up build time.
COPY --from=builder /root/catkin_ws/ ./

# This copies the local catkin_ws into the docker container.
COPY ./catkin_ws .

# Set up the envoirement to actually run the code
COPY ./scripts/entrypoint.bash ./entrypoint.bash
COPY ./scripts/setup.bash ./setup.bash

# Convert the line endings for the Windows users,
# calling `dos2unix` on all files ending in `.py` or `.bash`
RUN find . -type f \( -name '*.py' -o -name '*.bash' \) -exec 'dos2unix' -l -- '{}' \; && apt-get --purge remove -y dos2unix && rm -rf /var/lib/apt/lists/*

# Compile the catkin_ws.
RUN bash -c 'source /opt/ros/noetic/setup.bash && catkin_make'

RUN chmod -R u+x /root/catkin_ws/

# Uncomment these lines and comment out the last line for debugging
# RUN echo 'source /opt/ros/noetic/setup.bash' >> /root/.bashrc
# RUN echo 'source /root/catkin_ws/devel/setup.bash' >> /root/.bashrc
# RUN echo 'source /root/catkin_ws/setup.bash' >> /root/.bashrc

ENTRYPOINT ["./entrypoint.bash"]
