#include "drone-interface.h"

DroneInterface::DroneInterface(ros::NodeHandle & n)
: nh_(n)
{
	/* Create topic publishers for drone commands */
	pub_takeoff_ = nh_.advertise<std_msgs::Empty>("/ardrone/takeoff", 1);
	pub_land_ = nh_.advertise<std_msgs::Empty>("/ardrone/land", 1);
	pub_reset_ = nh_.advertise<std_msgs::Empty>("/ardrone/reset", 1);
	pub_trim_ = nh_.advertise<std_msgs::Empty>("/ardrone/flattrim", 1);
	pub_cmdvel_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

	/* Create node subscribers for drone status data */
	sub_navdata_ = nh_.subscribe<ardrone_autonomy::Navdata>(
		"/ardrone/navdata", 1, &DroneInterface::getNavdata, this);

	/* Default empty cmd_vel message */
	msg_cmdvel_.linear.x = 0.0;
	msg_cmdvel_.linear.y = 0.0;
	msg_cmdvel_.linear.z = 0.0;
	msg_cmdvel_.angular.x = 0.0;
	msg_cmdvel_.angular.y = 0.0;
	msg_cmdvel_.angular.z = 0.0;
}

void DroneInterface::pushCmd(InputData const& input)
{
	/* Reset */
	if (input.reset)
	{
		ROS_INFO("Reset");
		pub_reset_.publish(std_msgs::Empty());
	}

	/* Trim */
	if (input.trim)
	{
		ROS_INFO("Trim");
		pub_trim_.publish(std_msgs::Empty());
	}

	/* Toggle flight */
	if (input.toggleFlight)
	{
		if ((DroneStatus) navdata_.state == LANDED)
		{
			ROS_INFO("Takeoff");
			pub_takeoff_.publish(std_msgs::Empty());
		}
		else
		{
			ROS_INFO("Land");
			pub_land_.publish(std_msgs::Empty());
		}
	}

	/* Send cmd_vel message */
	if ((DroneStatus) navdata_.state == FLYING   || 
		(DroneStatus) navdata_.state == HOVERING || 
		(DroneStatus) navdata_.state == GOTOHOVER)
	{
		/* Define velocities according to the input */
		msg_cmdvel_.linear.x = input.pitch;
		msg_cmdvel_.linear.y = input.roll;
		msg_cmdvel_.linear.z = input.verticalVelocity;
		msg_cmdvel_.angular.x = 0.0;
		msg_cmdvel_.angular.y = 0.0;
		msg_cmdvel_.angular.z = input.yaw;

		pub_cmdvel_.publish(msg_cmdvel_);
	}
}

void DroneInterface::getNavdata (ardrone_autonomy::Navdata::ConstPtr const& navdata)
{
	/* Store newest navdata info */
	navdata_ = *navdata;
}
