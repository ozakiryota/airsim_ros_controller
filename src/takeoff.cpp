#include <ros/ros.h>
#include <airsim_ros_pkgs/Takeoff.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "takeoff");
	ros::NodeHandle nh;

	ros::ServiceClient client = nh.serviceClient<airsim_ros_pkgs::Takeoff>("/airsim_node/drone_1/takeoff");

	airsim_ros_pkgs::Takeoff srv;

	if(client.call(srv)){
		std::cout << "takeoff: true" << std::endl;
		return 0;
	}
	else{
		std::cout << "takeoff: false" << std::endl;
		return 1;
	}
}
