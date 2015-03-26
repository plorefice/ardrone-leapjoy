#include "leap-interface.h"

using namespace Leap;

LeapInterface::LeapInterface()
: controller_()
, listener_()
{
	/* Initialize controller and register listener */
	controller_.addListener(listener_);
}

LeapInterface::~LeapInterface()
{
	/* Remove listener */
	controller_.removeListener(listener_);
}

InputData LeapInterface::getInputData()
{
	return listener_.data();
}

LeapListener::LeapListener()
: nextData_()
{
}

void LeapListener::onInit(const Controller& controller)
{
	std::cout << "Leap Motion initialized." << std::endl;
}

void LeapListener::onConnect(const Controller& controller)
{
	std::cout << "Leap Motion connected." << std::endl;
	controller.enableGesture(Gesture::TYPE_KEY_TAP);
	controller.setPolicy(Leap::Controller::POLICY_BACKGROUND_FRAMES);
}

void LeapListener::onDisconnect(const Controller& controller)
{
	std::cout << "Leap Motion disconnected." << std::endl;
}

void LeapListener::onExit(const Controller& controller)
{
	std::cout << "Leap Motion exited." << std::endl;
}

int sgn(float val)
{
    return (float(0) < val) - (val < float(0));
}

void LeapListener::onFrame(const Controller& controller)
{
	/* Get the most recent frame */
	const Frame frame = controller.frame();
	InputData data = nextData_;

	/* Set frame informations */
	data.id = frame.id();
	data.ts = frame.timestamp();

	/* Get list of visible hands */
	HandList hands = frame.hands();
	if (hands.count() == 1)
	{
		const Hand hand = hands[0];
		
		/* Perform some processing on the new data */

		/* Set vertical velocity */
		float vertPos = hand.palmPosition().y;
		data.verticalVelocity = (
			vertPos > DEADZONE_VZ_CENTER - DEADZONE_VZ_RANGE / 2 &&
			vertPos < DEADZONE_VZ_CENTER + DEADZONE_VZ_RANGE / 2) ?
			0.0 : VZ_FIXED * sgn(vertPos - DEADZONE_VZ_CENTER);
		
		/* Set RPY */
		float angle = 0.0;

		angle = hand.palmNormal().roll();
		data.roll = (fabs(angle) < DEADZONE_ROLL) ? 
			0.0 : ROLL_FIXED * sgn(angle);

		angle = hand.direction().pitch();
		data.pitch = (fabs(angle) < DEADZONE_PITCH) ? 
			0.0 : PITCH_FIXED * sgn(-angle);

		data.yaw = 0.0;
		
		/* Get list of current gestures */
		const GestureList gestures = frame.gestures();
		for (int g = 0; g < gestures.count(); ++g)
		{
			Gesture gesture = gestures[g];
			
			switch (gesture.type())
			{
				/* Get left index key taps to toggle flight status */
				case Gesture::TYPE_KEY_TAP:
				{
					KeyTapGesture tap = gesture;
					
					if (tap.pointable() == hand.fingers().fingerType(Finger::TYPE_INDEX)[0])
					{
						data.toggleFlight = true;
					}
					break;
				}
				default:
					break;
			}
		}
	}
	else /* No hands or more than one hand detected */
	{
		nextData_ = InputData();
	}

	/* Make data available */
	nextData_ = data;
}

void LeapListener::onServiceConnect(const Controller& controller)
{
	std::cout << "Service Connected" << std::endl;
}

void LeapListener::onServiceDisconnect(const Controller& controller)
{
	std::cout << "Service Disconnected" << std::endl;
}

InputData LeapListener::data() 
{
	InputData ret = nextData_;

	/* Reset gestures after query */
	nextData_.toggleFlight = false;
	nextData_.reset = false;
	nextData_.trim = false;

	return ret; 
}
