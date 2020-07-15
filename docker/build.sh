#!/bin/bash

image_name="airsim_ros_controller"

docker build . \
	-t $image_name:latest \
	--build-arg CACHEBUST=$(date +%s)
