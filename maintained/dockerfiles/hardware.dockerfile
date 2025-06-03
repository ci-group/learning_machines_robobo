FROM ros:noetic

RUN rm /etc/apt/sources.list.d/ros1-latest.list \
    && rm /usr/share/keyrings/ros1-latest-archive-keyring.gpg

RUN apt-get update \
    && apt-get install -y ca-certificates curl

RUN export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') ;\
    curl -L -s -o /tmp/ros-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" \
    && apt-get update \
    && apt-get install /tmp/ros-apt-source.deb \
    && rm -f /tmp/ros-apt-source.deb

RUN apt-get update \
    && apt-get install -y ros-noetic-roscpp-tutorials

# These are some extra dependencies we need to use the Robobo, don't worry about it.
RUN apt-get update -y && apt-get install ffmpeg libsm6 libxext6 ros-noetic-opencv-apps dos2unix -y && rm -rf /var/lib/apt/lists/*

# Install CURL, as we need it for debugging.
# Add anything you want to also have available here.
RUN apt-get -y update && apt-get install -y curl && rm -rf /var/lib/apt/lists/*

# Set the working directory to something other than the root.
WORKDIR /root/catkin_ws

# Copy the stuff over and compile it.
# The main thing that's important is the setup.bash file. The catkin_ws stuff makes some commands available, but, as a student, you shouldn't worry about it.
COPY ./catkin_ws .
COPY ./scripts/setup.bash ./setup.bash

# Convert the line endings for the Windows users,
# calling `dos2unix` on all files ending in `.py` or `.bash`
RUN find . -type f \( -name '*.py' -o -name '*.bash' \) -exec 'dos2unix' -l -- '{}' \; && apt-get --purge remove -y dos2unix && rm -rf /var/lib/apt/lists/*

# Compile the ROS code.
RUN bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# Load everything in the bashrc to make sure all commands are available and evoirement variables are set.
RUN echo 'source /opt/ros/noetic/setup.bash' >> /root/.bashrc
RUN echo 'source /root/catkin_ws/devel/setup.bash' >> /root/.bashrc
RUN echo 'source /root/catkin_ws/setup.bash' >> /root/.bashrc
