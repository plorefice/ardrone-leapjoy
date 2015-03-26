#include "ros/ros.h"
#include "leap-interface.h"
#include "joypad-interface.h"
#include "drone-interface.h"

int main (int argc, char *argv[])
{
	ros::init(argc, argv, "ardrone_leapjoy");

	/* Create nodes */
	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");

	/* Create new drone interface */
	DroneInterface drone(nh);

	/* Parse params and create command interface */
	std::string cmdType;
	CommandInterface *cmdIf;

	/* Param:  controller
	 * Values: joypad, leap
	 */
	if (pnh.getParam("controller", cmdType))
	{
		if (cmdType == "joypad")
		{
			ROS_INFO("Using joypad");
			cmdIf = new JoypadInterface(nh);
		}
		else if (cmdType == "leap")
		{
			ROS_INFO("Using Leap Motion");
			cmdIf = new LeapInterface();
		}
	}
	else
	{
		ROS_INFO("No input method specified, falling back on joypad");
		cmdIf = new JoypadInterface(nh);
	}
	
	/* Define ROS loop rate */
	ros::Rate loopRate(30);

	while(ros::ok())
	{
		/* Receive raw input data from the command interface */
		InputData rawData = cmdIf->getInputData();

		/* Perform control */
		/* ... */
		InputData processedData = rawData;

		/* Send command to drone for execution */
		drone.pushCmd(processedData);

		/* Update and sleep */
		ros::spinOnce();
		loopRate.sleep();
	}

	/* Cleanup */
	delete cmdIf;

	return 0;
}
