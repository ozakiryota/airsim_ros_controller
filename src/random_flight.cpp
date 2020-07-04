#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <airsim_ros_pkgs/Reset.h>
#include <airsim_ros_pkgs/Takeoff.h>
#include <airsim_ros_pkgs/VelCmd.h>

class RandomFlight{
	private:
		/*node handle*/
		ros::NodeHandle _nh;
		ros::NodeHandle _nhPrivate;
		/*subscriber*/
		ros::Subscriber _sub_odom;
		/*publisher*/
		ros::ServiceClient _client_takeoff;
		ros::Publisher _pub_vel;
		/*msg*/
		nav_msgs::Odometry _odom;
		airsim_ros_pkgs::VelCmd _vel_msg;
		/*time*/
		ros::Time _stamp;
		/*flag*/
		/*parameter*/
		std::string _vehicle_name;

	public:
		RandomFlight();
		void callbackOdom(const nav_msgs::OdometryConstPtr& msg);
		void takeoff(void);
		void inputZeroVel(void);
		void inputRandomVel(void);
		void publication(void);
};

RandomFlight::RandomFlight()
	: _nhPrivate("~")
{
	std::cout << "--- random_flight ---" << std::endl;
	/*parameter*/
	_nhPrivate.param("vehicle_name", _vehicle_name, std::string("drone_1"));
	std::cout << "_vehicle_name = " << _vehicle_name << std::endl;
	/*sub*/
	_sub_odom = _nh.subscribe("/airsim_node/drone_1/odom_local_ned", 1, &RandomFlight::callbackOdom, this);
	/*pub*/
	/* _client_takeoff = _nh.serviceClient<airsim_ros_pkgs::Takeoff>("/airsim_node/drone_1/takeoff"); */
	/* _pub_vel = _nh.advertise<airsim_ros_pkgs::VelCmd>("/airsim_node/drone_1/vel_cmd_body_frame", 1); */
	_client_takeoff = _nh.serviceClient<airsim_ros_pkgs::Takeoff>("/airsim_node/" + _vehicle_name + "/takeoff");
	_pub_vel = _nh.advertise<airsim_ros_pkgs::VelCmd>("/airsim_node/" + _vehicle_name + "/vel_cmd_body_frame", 1);
	/*initialize*/
	takeoff();
}

void RandomFlight::callbackOdom(const nav_msgs::OdometryConstPtr& msg)
{
	_odom = *msg;
	inputRandomVel();
	publication();
}

void RandomFlight::takeoff(void)
{
	airsim_ros_pkgs::Takeoff srv;
	if(_client_takeoff.call(srv)){
		std::cout << "takeoff: true" << std::endl;
	}
	else{
		std::cout << "takeoff: false" << std::endl;
		exit(1);
	}
}

void RandomFlight::inputZeroVel(void)
{
	_vel_msg.twist.linear.x = 0.0;
	_vel_msg.twist.linear.y = 0.0;
	_vel_msg.twist.linear.z = 0.0;
	_vel_msg.twist.angular.x = 0.0;
	_vel_msg.twist.angular.y = 0.0;
	_vel_msg.twist.angular.z = 0.0;
}

void RandomFlight::inputRandomVel(void)
{
	std::random_device rd;
	std::mt19937 mt(rd());
	std::uniform_real_distribution urd(-1.0, 1.0);

	_vel_msg.twist.linear.x = urd(mt);
	_vel_msg.twist.linear.y = urd(mt);
	_vel_msg.twist.linear.z = urd(mt);
	_vel_msg.twist.angular.x = 0.0;
	_vel_msg.twist.angular.y = 0.0;
	_vel_msg.twist.angular.z = 0.0;

	std::cout << "_vel_msg: " 
		<< _vel_msg.twist.linear.x << ", "
		<< _vel_msg.twist.linear.y << ", "
		<< _vel_msg.twist.linear.z << ", "
		<< _vel_msg.twist.angular.x << ", "
		<< _vel_msg.twist.angular.y << ", "
		<< _vel_msg.twist.angular.z << std::endl;
}

void RandomFlight::publication(void)
{
	_pub_vel.publish(_vel_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "random_flight");

	RandomFlight random_flight;

	ros::spin();
	while(ros::ok()){
	}
}
