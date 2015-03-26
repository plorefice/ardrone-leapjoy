#pragma once

#include <stdint.h>

struct InputData
{
	int64_t id;
	int64_t ts;

	float roll;
	float pitch;
	float yaw;

	float verticalVelocity;

	bool toggleFlight;
	bool reset;
	bool trim;

public:
	InputData()
	{
		/* Initialize data */
		id = -1;
		ts = -1;
		roll = 0.0;
		pitch = 0.0;
		yaw = 0.0;
		verticalVelocity = 0.0;
		toggleFlight = false;
		reset = false;
		trim = false;
	}
};

class CommandInterface
{
public:
	virtual InputData getInputData() = 0;
};
