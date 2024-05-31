FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get -y update && apt-get install -y python3 python3-pip tar xz-utils libx11-6 libxcb1 libxau6 libgl1-mesa-dev xvfb dbus-x11 x11-utils libxkbcommon-x11-0 libavcodec-dev libavformat-dev libswscale-dev && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install pyzmq cbor2

WORKDIR /root/workdir/

COPY ./CoppeliaSim.tar.xz /opt/

RUN tar -xf /opt/CoppeliaSim.tar.xz -C /opt/ && rm /opt/CoppeliaSim.tar.xz && mv /opt/CoppeliaSim* /opt/CoppeliaSim/

ENV COPPELIASIM_ROOT_DIR=/opt/CoppeliaSim
ENV LD_LIBRARY_PATH=$COPPELIASIM_ROOT_DIR:$LD_LIBRARY_PATH
ENV PATH=$COPPELIASIM_ROOT_DIR:$PATH

COPY ./to_open.ttt /root/workdir/to_open.ttt

RUN echo '#!/bin/bash\ncd $COPPELIASIM_ROOT_DIR\n/usr/bin/xvfb-run --server-args "-ac -screen 0, 1024x1024x24" coppeliaSim /root/workdir/to_open.ttt -h "$@"' > /root/workdir/entrypoint.bash && chmod a+x /root/workdir/entrypoint.bash

ENTRYPOINT [ "/root/workdir/entrypoint.bash" ]
