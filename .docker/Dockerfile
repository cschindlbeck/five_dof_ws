FROM ros:noetic

COPY src/ /minimal_ws/src
WORKDIR /minimal_ws

RUN apt update

RUN apt install -y python3-catkin-tools

RUN rosdep install --from-paths src --ignore-src -r -y

RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; catkin_make'
