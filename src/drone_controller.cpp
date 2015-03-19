#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <ardrone_autonomy/Navdata.h>

enum DroneStatus
{
	EMERGENCY = 0,
	INITED    = 1,
	LANDED    = 2,
	FLYING    = 3,
	HOVERING  = 4,
	TEST      = 5,
	TAKINGOFF = 6,
	GOTOHOVER = 7,
	LANDING   = 8,
	LOOPING   = 9
};

class DroneController
{
private:
	ros::NodeHandle node_;

	ros::Publisher pub_takeoff_;
	ros::Publisher pub_land_;
	ros::Publisher pub_reset_;
	ros::Publisher pub_trim_;
	ros::Publisher pub_cmdvel_;

	ros::Subscriber sub_joy_;
	ros::Subscriber sub_navdata_;

	geometry_msgs::Twist cmdvel_;

	DroneStatus status_;

public:
	DroneController ()
	: status_(EMERGENCY)
	{
		pub_takeoff_ = node_.advertise<std_msgs::Empty>("/ardrone/takeoff", 1);
		pub_land_ = node_.advertise<std_msgs::Empty>("/ardrone/land", 1);
		pub_reset_ = node_.advertise<std_msgs::Empty>("/ardrone/reset", 1);
		pub_trim_ = node_.advertise<std_msgs::Empty>("/ardrone/flattrim", 1);
		pub_cmdvel_ = node_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

		sub_joy_ = node_.subscribe<sensor_msgs::Joy>(
			"joy", 1, &DroneController::joyCallback, this);
		sub_navdata_ = node_.subscribe<ardrone_autonomy::Navdata>(
			"/ardrone/navdata", 1, &DroneController::getNavdata, this);

		cmdvel_.linear.x = 0.0;
		cmdvel_.linear.y = 0.0;
		cmdvel_.linear.z = 0.0;
		cmdvel_.angular.x = 0.0;
		cmdvel_.angular.y = 0.0;
		cmdvel_.angular.z = 0.0;
	}

	~DroneController ()
	{
	}

	void takeoff ()
	{
		if (status_ == LANDED)
			pub_takeoff_.publish(std_msgs::Empty());
	}

	void land ()
	{
		pub_land_.publish(std_msgs::Empty());
	}

	void reset ()
	{
		pub_reset_.publish(std_msgs::Empty());
	}

	void trim ()
	{
		if (status_ == LANDED)
			pub_trim_.publish(std_msgs::Empty());
	}

	void sendCmdVel ()
	{
		if (status_ == FLYING || status_ == HOVERING || status_ == GOTOHOVER)
			pub_cmdvel_.publish(cmdvel_);
	}

	void step ()
	{
		switch(status_)
		{
		case TAKINGOFF:
			takeoff();
			break;
		case LANDING:
			land();
			break;
		case FLYING:
		case HOVERING:
		case GOTOHOVER:
			sendCmdVel();
			break;
		default:
			break;
		}
	}

	void joyCallback (sensor_msgs::Joy::ConstPtr const& joy)
	{
		if (joy->buttons[6])
		{
			ROS_INFO("Reset...");
			reset();
		}

		if (joy->buttons[7])
		{
			ROS_INFO("Trim...");
			trim();
		}

		if (joy->buttons[0])
		{
			ROS_INFO("Takeoff...");
			takeoff();
		}

		if (joy->buttons[1])
		{
			ROS_INFO("Landing...");
			land();
		}

		cmdvel_.linear.x = joy->axes[1];
		cmdvel_.linear.y = joy->axes[0];
		cmdvel_.linear.z = joy->axes[3];
		cmdvel_.angular.z = joy->axes[2];
	}

	void getNavdata (ardrone_autonomy::Navdata::ConstPtr const& navdata)
	{
		status_ = (DroneStatus) navdata->state;
	}
};

int main (int argc, char **argv)
{
	ros::init(argc, argv, "leapdrone_test_launch");
	ROS_INFO("LeapDrone Test Launch");

	DroneController drone;

	ros::Rate loopRate(30);

	while (ros::ok())
	{
		drone.step();

		ros::spinOnce();
		loopRate.sleep();
	}

	return 0;
}
