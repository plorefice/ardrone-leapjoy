#include "joypad-interface.h"

JoypadInterface::JoypadInterface(ros::NodeHandle & n)
: nh_(n)
, prevJoy_()
, nextData_()
{
	/* Initialize joypad struct */
	prevJoy_.buttons.resize(10);
	prevJoy_.axes.resize(6);

	/* Receive joypad messages */
	sub_joy_ = nh_.subscribe<sensor_msgs::Joy>(
		"joy", 1, &JoypadInterface::joyCallback, this);

	ROS_INFO("Joypad created.");
}

JoypadInterface::~JoypadInterface()
{
	ROS_INFO("Joypad destroyed.");
}

void JoypadInterface::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	/* Toggle flight */
	if (joy->buttons[0] && prevJoy_.buttons[0] == 0)
		nextData_.toggleFlight = true;

	/* Reset */
	if (joy->buttons[6] && prevJoy_.buttons[6] == 0)
		nextData_.reset = true;

	/* Trim */
	if (joy->buttons[7] && prevJoy_.buttons[7] == 0)
		nextData_.trim = true;

	/* Set RPY */
	nextData_.roll  = joy->axes[0];
	nextData_.pitch = joy->axes[1];
	nextData_.yaw   = joy->axes[2];

	/* Set vertical velocity */
	nextData_.verticalVelocity = joy->axes[3];

	/* Save current joypad state */
	prevJoy_ = *joy;
}

InputData JoypadInterface::getInputData()
{
	InputData ret = nextData_;

	/* Reset gestures after query */
	nextData_.toggleFlight = false;
	nextData_.reset = false;
	nextData_.trim = false;

	return ret;
}
