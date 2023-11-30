FROM ros:noetic

RUN apt-get update -y && apt-get install -y ros-noetic-compressed-image-transport
RUN apt-get update -y && apt-get install -y python3-pip
RUN pip install tensorflow keras torch
WORKDIR /root/projects/

COPY . .

RUN echo 'catkin_make install && source /root/projects/devel/setup.bash' >> /root/.bashrc
