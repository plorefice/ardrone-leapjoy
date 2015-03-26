#pragma once

#include "command-interface.h"
#include "Leap.h"

class LeapListener : public Leap::Listener
{
	InputData nextData_;

	static const float DEADZONE_VZ_CENTER = 250.0; ///< [mm]
	static const float DEADZONE_VZ_RANGE = 100.0;  ///< [mm]
	static const float DEADZONE_ROLL = 0.35;       ///< [rad]
	static const float DEADZONE_PITCH = 0.35;      ///< [rad]
	static const float DEADZONE_YAW = 0.35;        ///< [rad]

	static const float VZ_FIXED = 0.5;
	static const float ROLL_FIXED = 0.5;
	static const float PITCH_FIXED = 0.5;
	static const float YAW_FIXED = 0.5;

public:
	LeapListener();

	virtual void onInit(const Leap::Controller&);
	virtual void onConnect(const Leap::Controller&);
	virtual void onDisconnect(const Leap::Controller&);
	virtual void onExit(const Leap::Controller&);
	virtual void onFrame(const Leap::Controller&);
	virtual void onServiceConnect(const Leap::Controller&);
	virtual void onServiceDisconnect(const Leap::Controller&);

	InputData data();
};

class LeapInterface : public CommandInterface
{
	LeapListener listener_;
	Leap::Controller controller_;

public:
	LeapInterface();
	virtual ~LeapInterface();

	virtual InputData getInputData();
};
