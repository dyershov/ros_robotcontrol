# ROS2 robotcontrol
Exposing librobotcontrol API as ROS2 nodes

## Installation

Create ROS2 workspace and clone the repository into `src`. Next, build `robotcontrol` packages. To build all packages run from the root of your ROS workspace

    colcon build

If using `ros-dev` docker images, build packages as follows

    docker run -v <host-path-to-ros-workspace>:<container-path-to-ros-workspace> --rm ros-dev:base zsh -c "cd <container-path-to-ros-workspace>; colcon build"

Additionaly, you may select a desired package using either `--packages-select` or `--packages-up-to` parameters in the above.

Available packages are:

* `robotcontrol_interfaces` in which messages are defined
* `robotcontrol_cape` which follows closely robotcontrol API
* `robotcontrol_extra` all extra nodes and services that are not directly linked to robotcontrol library

Finally install built packages by sourcing `setup` script from the root of your ROS workspace

    source ./install/setup.zsh

## Running nodes in docker container

An example of running mecanum drive node

    docker run --net host --pid host -v <host-path-to-ros-workspace>:<container-path-to-ros-workspace> --rm ros-dev:base zsh -c "source <container-path-to-ros-workspace>/install/setup.zsh; ros2 run robotcontrol_extra mecanum_drive"
