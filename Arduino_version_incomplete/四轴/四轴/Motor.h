// Motor.h

#ifndef _MOTOR_h
#define _MOTOR_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

class MotorClass
{
 protected:


 public:
	void init(const int MotorPin[4]);
};

extern MotorClass Motor;

#endif

#ifdef MicroQuadcopter
#endif
