# Five DOF

Minimal files to test set_goal_orientation with moveit commander

## Install

On ROS noetic installed on Ubuntu 20.04 do

```bash
sudo apt update
sudo apt install -y python3-catkin-tools
rosdep install --from-paths src --ignore-src -r -y
catkin build
```

## Docker

From ws directory, build image via

```bash
docker build -f .docker/Dockerfile -t moveittest .
```

Run container via

```bash
docker run --mount type=bind,source=$(pwd)/src,target=/minimal_ws/src -it moveittest /bin/bash
source devel/setup.bash
```

## Usage

```bash
roslaunch five_dof run.launch set_goal_orientation:=false
```

set set_goal_orientation to true
