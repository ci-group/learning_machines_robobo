FROM ros:noetic

# These are some extra dependencies we need to use the Robobo, don't worry about it.
RUN apt-get update -y && apt-get install ffmpeg libsm6 libxext6 ros-noetic-opencv-apps -y && rm -rf /var/lib/apt/lists/*

# Install CURL, as we need it for debugging.
# Add anything you want to also have available here.
RUN apt-get -y update && apt-get install -y curl && rm -rf /var/lib/apt/lists/*

# Set the working directory to something other than the root.
WORKDIR /root/catkin_ws

# Copy the stuff over and compile it.
# The main thing that's important is the setup.bash file. The catkin_ws stuff makes some commands available, but, as a student, you shouldn't worry about it.
COPY ./catkin_ws .
COPY ./scripts/setup.bash ./setup.bash

RUN bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# Load everything in the bashrc to make sure all commands are available and evoirement variables are set.
RUN echo 'source /opt/ros/noetic/setup.bash' >> /root/.bashrc
RUN echo 'source /root/catkin_ws/devel/setup.bash' >> /root/.bashrc
RUN echo 'source /root/catkin_ws/setup.bash' >> /root/.bashrc
