#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <airsim_ros_pkgs/Takeoff.h>
// #include <airsim_ros_pkgs/SetLocalPosition.h>
#include <airsim_ros_pkgs/VelCmd.h>
#include <airsim_ros_pkgs/GimbalAngleEulerCmd.h>
#include <random>

class RandomFlight{
	private:
		/*node handle*/
		ros::NodeHandle _nh;
		ros::NodeHandle _nhPrivate;
		/*subscriber*/
		ros::Subscriber _sub_odom;
		/*publisher*/
		ros::ServiceClient _client_takeoff;
		// ros::ServiceClient _client_goal;
		ros::Publisher _pub_vel;
		ros::Publisher _pub_angle;
		/*msg*/
		nav_msgs::Odometry _odom;
		airsim_ros_pkgs::VelCmd _vel_msg;
		airsim_ros_pkgs::GimbalAngleEulerCmd _angle_msg;
		/*time*/
		ros::Time _stamp;
		/*flag*/
		bool _stop = true;
		/*parameter*/
		std::string _vehicle_name;

	public:
		RandomFlight();
		void callbackOdom(const nav_msgs::OdometryConstPtr& msg);
		void takeoff(void);
		// void setGoal(void);
		void inputZeroVel(void);
		void inputRandomVel(void);
		void inputRandomAngle(void);
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
	_client_takeoff = _nh.serviceClient<airsim_ros_pkgs::Takeoff>("/airsim_node/" + _vehicle_name + "/takeoff");
	// _client_goal = _nh.serviceClient<airsim_ros_pkgs::SetLocalPosition>("/airsim_node/local_position_goal");
	_pub_vel = _nh.advertise<airsim_ros_pkgs::VelCmd>("/airsim_node/" + _vehicle_name + "/vel_cmd_body_frame", 1);
	_pub_angle = _nh.advertise<airsim_ros_pkgs::GimbalAngleEulerCmd>("/airsim_node/gimbal_angle_euler_cmd", 1);
	/*initialize*/
	takeoff();
}

void RandomFlight::callbackOdom(const nav_msgs::OdometryConstPtr& msg)
{
	_odom = *msg;

	if(_stop){
		//inputZeroVel();
		_stop = false;
	}
	else{
		inputRandomVel();
		inputRandomAngle();
		_stop = true;
	}
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

/*
void RandomFlight::setGoal(void)
{
	airsim_ros_pkgs::SetLocalPosition srv;
	srv.request.x = 1.0;
	srv.request.y = 1.0;
	srv.request.z = 1.0;
	srv.request.yaw = 0.0;
	srv.vehicle_name = _vehicle_name;

	if(_client_goal.call(srv)){
		std::cout << "setGoal: true" << std::endl;
	}
	else{
		std::cout << "setGoal: false" << std::endl;
		exit(1);
	}
}
*/

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
	std::uniform_real_distribution<> urd(-1.0, 1.0);

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

void RandomFlight::inputRandomAngle(void)
{
	std::random_device rd;
	std::mt19937 mt(rd());
	std::uniform_real_distribution<> urd(-1.0, 1.0);

	_angle_msg.vehicle_name = _vehicle_name;
	_angle_msg.roll = urd(mt);
	_angle_msg.pitch = urd(mt);
	_angle_msg.yaw = urd(mt);
}

void RandomFlight::publication(void)
{
	_pub_vel.publish(_vel_msg);
	_pub_angle.publish(_angle_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "random_flight");

	RandomFlight random_flight;

	ros::spin();
	while(ros::ok()){
	}
}
