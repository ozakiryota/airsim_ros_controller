#!/bin/bash

path_name=$(pwd)
image_name="airsim_ros_controller"

xhost +
nvidia-docker run -it --rm \
	--env="DISPLAY" \
	--env="QT_X11_NO_MITSHM=1" \
	--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
	--net=host \
	$image_name:latest
