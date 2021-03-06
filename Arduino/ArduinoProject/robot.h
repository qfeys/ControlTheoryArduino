#ifndef ROBOT_H
#define ROBOT_H

//!  ROBOT Class
/*!
  Class incorporating the robot. This class is used to define state machines, control algorithms, sensor readings,...
  It should be interfaced with the communicator to send data to the world.
*/

#include <inttypes.h>
#include "math.h"

#include <microOS.h>
#include <encoder_sensor.h>
#include <sharp41S.h>
#include <l293d.h>
#include "Controller.h"
#include "Tester.h"
#include "SensorEstimator.h"

class Robot
{
private:
	uint8_t _ID;			//give the robot an ID so that you can recognize it
	uint8_t _type;

	// Give the robot some sensors
	Sensor1D* _encoder1;
	Sensor1D* _encoder2;
	Sensor1D* _distance1;
	Sensor1D* _distance2;
	Sensor1D* _pendulum;

	// Give the robot some motors
	HBridgeInterface* _motor1;
	HBridgeInterface* _motor2;

	// Interface the buttons
	bool _button_states[8] = { false,false,false,false,false,false,false,false };
	bool toggleButton(uint8_t button);
	bool controlEnabled();
	bool testEnabled();

	// Controllers
	float Ts = 0.01;
	Controller velControl;
	Controller2 angleControl;
	Tester tester;
	SensorEstimator estimator;

	// Memory
	int posWheelA;
	int velWheelA;
	int posWheelB;
	int velWheelB;
	int theta0;
	int direction;

public:
	Robot(uint8_t ID = 0);

	////////
	/// FUNC
	void init();			//set up the robot
	void controllerHook();	//update function which can be executed continuously
	void resetEncoders();	//reset the encoders
	void resetPendulum();	//reset pendulum zero position

	///////
	/// GET
	uint8_t id();
	uint8_t type();

	// Event callbacks
	void button1callback();
	void button2callback();
	void button3callback();
	void button4callback();
	void button5callback();
	void button6callback();
	void button7callback();
	void button8callback();
};

#endif //ROBOT_H
