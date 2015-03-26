#pragma once

#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/Twist.h"
#include "ardrone_autonomy/Navdata.h"
#include "command-interface.h"

class DroneInterface
{
public:
	enum DroneStatus
	{
		EMERGENCY = 0,
		INITED,
		LANDED,
		FLYING,
		HOVERING,
		TEST,
		TAKINGOFF,
		GOTOHOVER,
		LANDING,
		LOOPING
	};

private:
	ros::NodeHandle &    nh_;
	ros::Publisher       pub_takeoff_;
	ros::Publisher       pub_land_;
	ros::Publisher       pub_reset_;
	ros::Publisher       pub_trim_;
	ros::Publisher       pub_cmdvel_;
	ros::Subscriber      sub_navdata_;
	geometry_msgs::Twist msg_cmdvel_;

	ardrone_autonomy::Navdata navdata_;

public:
	DroneInterface(ros::NodeHandle & n);

	void pushCmd(InputData const& input);

	void getNavdata (ardrone_autonomy::Navdata::ConstPtr const& navdata);
};
