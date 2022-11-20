FROM osrf/ros:foxy-desktop

RUN apt-get update && apt-get upgrade -y

RUN mkdir -p /home/dev-ws/src

WORKDIR /home/dev-ws/src

RUN git clone https://github.com/packbionics/jetleg

WORKDIR /home/dev-ws

RUN /bin/bash -c ". /opt/ros/foxy/setup.bash; colcon build --packages-select jetleg_bringup jetleg_control jetleg_description jetleg_vision jetleg_vision_cpp"