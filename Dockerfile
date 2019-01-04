FROM ros:melodic

RUN apt-get update -y && apt-get install -y ros-melodic-compressed-image-transport
WORKDIR /root/projects/
RUN echo 'catkin_make install && source /root/projects/devel/setup.bash' >> /root/.bashrc
