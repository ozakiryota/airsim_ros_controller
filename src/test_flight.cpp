#include <ros/ros.h>
#include <airsim_ros_pkgs/Takeoff.h>
#include <airsim_ros_pkgs/VelCmd.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "draw_square");
	ros::NodeHandle nh;

	/*pub sub srv*/
	ros::ServiceClient client = nh.serviceClient<airsim_ros_pkgs::Takeoff>("/airsim_node/drone_1/takeoff");
	ros::Publisher pub_vel = nh.advertise<airsim_ros_pkgs::VelCmd>("/airsim_node/vel_cmd_body_frame", 1);

	/*take off*/
	airsim_ros_pkgs::Takeoff srv;
	if(client.call(srv)){
		std::cout << "takeoff: true" << std::endl;
	}
	else{
		std::cout << "takeoff: false" << std::endl;
		return 1;
	}
	/*draw square*/
	double side_length = 3.0;
	airsim_ros_pkgs::VelCmd vel_msg;
	vel_msg.twist.linear.x = 3.0;
	vel_msg.twist.linear.y = 0.0;
	vel_msg.twist.linear.z = 0.0;
	vel_msg.twist.angular.x = 0.0;
	vel_msg.twist.angular.y = 0.0;
	vel_msg.twist.angular.z = 0.0;
	pub_vel.publish(vel_msg);

	return 0;
}
