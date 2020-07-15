FROM osrf/ros:kinetic-desktop-full

########## nvidia-docker1 hooks ##########
LABEL com.nvidia.volumes.needed="nvidia_driver"
ENV PATH /usr/local/nvidia/bin:${PATH}
ENV LD_LIBRARY_PATH /usr/local/nvidia/lib:/usr/local/nvidia/lib64:${LD_LIBRARY_PATH}
########## basis ##########
RUN apt-get update && apt-get install -y \
	vim \
	wget \
	unzip \
	git
########## AirSim ##########
RUN apt-get update &&\
	apt-get install -y \
		rsync \
		g++-8 \
		python-catkin-tools \
		ros-kinetic-mavros* &&\
	mkdir /home/airsim_ws/ &&\
	cd /home/airsim_ws/ &&\
	git clone https://github.com/Microsoft/AirSim.git &&\
	cd AirSim &&\
	./setup.sh &&\
	./build.sh
########## AirSim ROS wrapper ##########
##### CMake >= 3.10 #####
RUN mkdir /home/cmake_ws &&\
	cd /home/cmake_ws &&\
	wget https://github.com/Kitware/CMake/releases/download/v3.17.3/cmake-3.17.3.tar.gz &&\
	tar xvf cmake-3.17.3.tar.gz &&\
	cd cmake-3.17.3 &&\
	./bootstrap && make && make install
##### Build #####
RUN sed -i "s/#include <filesystem>/#include <experimental\/filesystem>/g" /home/airsim_ws/AirSim/MavLinkCom/MavLinkTest/Commands.cpp &&\
	sed -i "s/using namespace std::filesystem;/using namespace std::experimental::filesystem;/g" /home/airsim_ws/AirSim/MavLinkCom/MavLinkTest/Commands.cpp &&\
	sed -i "s/#include <filesystem>/#include <experimental\/filesystem>/g" /home/airsim_ws/AirSim/MavLinkCom/MavLinkTest/main.cpp &&\
	sed -i "s/using namespace std::filesystem;/using namespace std::experimental::filesystem;/g" /home/airsim_ws/AirSim/MavLinkCom/MavLinkTest/main.cpp &&\
	cd /home/airsim_ws/AirSim/ros &&\
	/bin/bash -c "source /opt/ros/kinetic/setup.bash; catkin build" &&\
	echo "source /home/airsim_ws/AirSim/ros/devel/setup.bash" >> ~/.bashrc
##### Setting json #####
RUN mkdir -p /root/Documents/AirSim &&\
	cp /home/airsim_ws/AirSim/ros/src/airsim_tutorial_pkgs/settings/front_stereo_and_center_mono.json /root/Documents/AirSim/settings.json
##### Script #####
RUN echo "#!/bin/bash \n \
		source /opt/ros/kinetic/setup.bash \n \
		source /home/airsim_ws/AirSim/ros/devel/setup.bash \n \
		roslaunch airsim_ros_pkgs airsim_node.launch" \
		>> /home/airsim_ws/airsim_node.sh &&\
	chmod +x /home/airsim_ws/airsim_node.sh
########## NO cache ##########
ARG CACHEBUST=1
######### AirSim controler ##########
RUN cd /home/airsim_ws/AirSim/ros/src &&\
	git clone https://github.com/ozakiryota/airsim_ros_controller &&\
	/bin/bash -c "source /opt/ros/kinetic/setup.bash; catkin build"
##### Script #####
RUN echo "#!/bin/bash \n \
		source /opt/ros/kinetic/setup.bash \n \
		source /home/airsim_ws/AirSim/ros/devel/setup.bash \n \
		rosrun airsim_ros_controller random_flight" \
		>> /home/airsim_ws/random_flight.sh &&\
	chmod +x /home/airsim_ws/random_flight.sh
######### initial position ##########
WORKDIR /home/airsim_ws
