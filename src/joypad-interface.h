#pragma once

#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "command-interface.h"

class JoypadInterface : public CommandInterface
{
	ros::NodeHandle & nh_;
	ros::Subscriber sub_joy_;

	sensor_msgs::Joy prevJoy_;

	InputData nextData_;

private:
	void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

public:
	JoypadInterface(ros::NodeHandle & n);
	virtual ~JoypadInterface();

	virtual InputData getInputData();
};
