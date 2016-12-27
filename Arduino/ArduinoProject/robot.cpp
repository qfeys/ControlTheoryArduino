#include "robot.h"
#include "Controller.h"
#include <math.h>

#define ENC1_PINA		18
#define ENC1_PINB		32
#define ENC2_PINA		19
#define ENC2_PINB		42
#define DIST1_PIN		A0
#define DIST2_PIN		A1
#define MOT1_PIN_IN1	8u
#define MOT1_PIN_IN2	12u
#define MOT1_PIN_EN		6u
#define MOT2_PIN_IN1	7u
#define MOT2_PIN_IN2	4u
#define MOT2_PIN_EN		5u
#define PENDULUM_PIN	A3
#define LED1_PIN 		34
#define LED2_PIN 		40
#define BATTERY_VOLTAGE	6000


Robot::Robot(uint8_t ID):
	_ID(ID),
	_type(20),
	_encoder1(new EncoderSensor(ENC1_PINA,ENC1_PINB)),
	_encoder2(new EncoderSensor(ENC2_PINA,ENC2_PINB)),
	_distance1(new Sharp41S(DIST1_PIN)),
	_distance2(new Sharp41S(DIST2_PIN)),
	_pendulum(new AnalogSensor(PENDULUM_PIN,12)),
	_motor1(new L293D(MOT1_PIN_IN1,MOT1_PIN_IN2,MOT1_PIN_EN,BATTERY_VOLTAGE)),
	_motor2(new L293D(MOT2_PIN_IN1,MOT2_PIN_IN2,MOT2_PIN_EN,BATTERY_VOLTAGE))
{
	_pendulum->setScale(2.0f*M_PI/1023.0f);
}

void Robot::init() {
	//initialize the robot - sort of starting procedure
	resetEncoders();
	resetPendulum();
}

void Robot::controllerHook() {
	//do something that is periodic: reading from sensors, setting the motors, updating variables sent to the pc..

	// Read sensors
	int enc1_value = -_encoder1->readRawValue();
	int enc2_value = _encoder2->readRawValue();
	int pend_value = _pendulum->readRawValue();

	// Calculate velocity, incl filtering, in pulses per second (7350 p/meter)
	float va = (enc1_value - posa) / Ts;
	float vb = (enc2_value - posb) / Ts;
	direction = va == 0 ? 0 : va > 0 ? 1 : -1;

	posa = enc1_value;
	vela = (va + vela * 3) / 4;			// filtering: 1/4 * 1/(s-3)
	posb = enc2_value;
	velb = (vb + velb * 3) / 4;

	System.setGPoutInt(0, enc1_value);
	System.setGPoutInt(1, enc2_value);
	System.setGPoutInt(2, pend_value);
	System.setGPoutFloat(0, va);
	// Pendulum
	float thetaErr = ((0.0 - pend_value + theta0) * 2.0 * PI / (1024 * 1.0616)) - PI; // In radians
	System.setGPoutFloat(5, thetaErr * 1000);

	if (controlEnabled())
	{
		/// Set velocity:
		// float vSet = System.getGPinInt(1);

		/// Test response:
		// t++;
		// float vSet = sin(t  /40000.0* t) * 1000;

		/// Set Position
		int setPoint = System.getGPinInt(1);
		float errPos = enc1_value - setPoint;
		float vSet = -8 * errPos;

		/// Set v via theta
		// float vSet = angleControl.NextState(va / 7350.0, enc1_value / 7350, thetaErr) * 7350;

		/// Limit velocity to something archiveble
		if (vSet > 2400) vSet = 2400;
		if (vSet < -2400) vSet = -2400;

		float errVel = vSet - va;
		float u = velControl.NextState(errVel);
		float f = EstimateFriction2(vSet, direction);

		System.setGPoutFloat(4, u);
		u += f;
		System.setGPoutFloat(6, f);
		System.setGPoutFloat(2, errVel);
		System.setGPoutFloat(3, vSet);

		/// Limit u to something archieveble
		if (u > 6000) u = 6000;
		if (u < -6000) u = -6000;


		System.setGPoutFloat(7, u);
		_motor1->setBridgeVoltage(-u);
		_motor2->setBridgeVoltage(u);

	}
	else {
		//set motor voltage to zero or it will keep on running...
		_motor1->setBridgeVoltage(0);
		_motor2->setBridgeVoltage(0);
		System.setGPoutFloat(4, 0);
		//velControl.Reset();
	}
	if (testEnabled())
	{
		//set motor voltage to max value
		_motor1->setBridgeVoltage(-6000);
		_motor2->setBridgeVoltage(6000);
		System.setGPoutFloat(4, 6000);
	}
}

void Robot::resetEncoders()
{
	_encoder1->init();
	_encoder2->init();
}


void Robot::resetPendulum()
{
	_pendulum->setOffset(_pendulum->readRawValue());
	theta0 = _pendulum->readRawValue();
}

uint8_t Robot::id()
{
    return _ID;
}

uint8_t Robot::type()
{
	return _type;
}

bool Robot::toggleButton(uint8_t button)
{
	_button_states[button] = !_button_states[button];
	return _button_states[button];
}

bool Robot::controlEnabled()
{
	return _button_states[0];
}

bool Robot::testEnabled()
{
	return _button_states[2];
}

void Robot::button1callback()
{
	if (toggleButton(0)) {
		System.println("Controller enabled.");
	}
	else {
		System.println("Controller disabled.");
	}
}

void Robot::button2callback()
{
	toggleButton(1);

	resetEncoders();
	resetPendulum();
	velControl.Reset();
	angleControl.Reset();
	t = 0;

	System.println("Reset.");
}
void Robot::button3callback()
{
	if (toggleButton(2))
	{
		System.println("Test enabled.");
	} else
	{
		System.println("Test disabled.");
	}
}

void Robot::button4callback()
{
	toggleButton(3);
}

void Robot::button5callback()
{
	toggleButton(4);
}

void Robot::button6callback()
{
	toggleButton(5);
}

void Robot::button7callback()
{
	toggleButton(6);
}

void Robot::button8callback()
{
	toggleButton(7);
}
