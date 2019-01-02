FROM ros:melodic

WORKDIR /root/projects/
RUN echo 'catkin_make install && source /root/projects/devel/setup.bash' >> /root/.bashrc
