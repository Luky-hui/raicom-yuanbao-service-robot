# RAICOM Yuanbao Service Robot Source

This package contains the ROS source code extracted from the competition VM.

## Layout

- `bobac3_ws/src`: Bobac3 simulation, navigation, vision, face, supervisor, and service-group mission packages.
- `ros_workspace/src`: local voice bridge and REI voice message/service packages.

## Build

On Ubuntu with ROS Noetic:

```bash
source /opt/ros/noetic/setup.bash
cd ~/bobac3_ws
catkin_make
source devel/setup.bash

cd ~/ros_workspace
catkin_make
source devel/setup.bash --extend
```

## Notes

Large runtime files such as logs, videos, cached audio, build outputs, and model weights are intentionally not included.
