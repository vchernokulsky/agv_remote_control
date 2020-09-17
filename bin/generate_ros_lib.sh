#!/usr/bin/env bash

ROS_DISTRO_NAME=noetic

if [ "$INSIDE_OF_DOCKER" == "" ]; then
  ROOT_PATH=$(realpath "$(dirname ${0})/../")

  rm -rf $ROOT_PATH/components/rosserial_esp32/ros_lib

  docker run --rm \
    --mount type=bind,source=$ROOT_PATH,destination=/src \
    --env INSIDE_OF_DOCKER=1 \
    --entrypoint /src/bin/generate_ros_lib.sh \
    ros:$ROS_DISTRO_NAME-ros-base
else
  apt update
  apt install -y ros-$ROS_DISTRO_NAME-rosserial

  . /opt/ros/$ROS_DISTRO_NAME/setup.bash
  rosrun rosserial_client make_libraries /src/components/rosserial_esp32
fi

