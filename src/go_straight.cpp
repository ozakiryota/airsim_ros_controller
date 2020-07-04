#include <ros/ros.h>
#include <airsim_ros_pkgs/Takeoff.h>
#include <airsim_ros_pkgs/VelCmd.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_flight");
	ros::NodeHandle nh;

	/*pub srv*/
	ros::ServiceClient client_takeoff = nh.serviceClient<airsim_ros_pkgs::Takeoff>("/airsim_node/drone_1/takeoff");
	ros::Publisher pub_vel = nh.advertise<airsim_ros_pkgs::VelCmd>("/airsim_node/drone_1/vel_cmd_body_frame", 1);

	/*take off*/
	airsim_ros_pkgs::Takeoff srv_takeoff;
	if(client_takeoff.call(srv_takeoff)){
		std::cout << "takeoff: true" << std::endl;
	}
	else{
		std::cout << "takeoff: false" << std::endl;
		return 1;
	}
	/*draw square*/
	double mps = 1.0;
	airsim_ros_pkgs::VelCmd vel_msg;
	vel_msg.twist.linear.x = mps;
	vel_msg.twist.linear.y = 0.0;
	vel_msg.twist.linear.z = 0.0;
	vel_msg.twist.angular.x = 0.0;
	vel_msg.twist.angular.y = 0.0;
	vel_msg.twist.angular.z = 0.0;
	pub_vel.publish(vel_msg);

	return 0;
}
