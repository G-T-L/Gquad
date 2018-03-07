// PID.h

#ifndef _PID_h
#define _PID_h
#define DIRECT  0
#define REVERSE  1


#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

//PIDClass(volatile double *Input, volatile double *Output, volatile double *Setpoint ...............
class PIDClass
{
 protected:
	 double kp;                  // * (P)roportional Tuning Parameter
	 double ki;                  // * (I)ntegral Tuning Parameter
	 double kd;                  // * (D)erivative Tuning Parameter

	 volatile double *myInput;
	 volatile double *myOutput;
	 volatile double *mySetpoint;

	 unsigned long lastTime;
	 double IntegralSum = 0, lastInput;
	 double outMin, outMax;
 public:
	 PIDClass(volatile double*, volatile double*, volatile double*, double, double, double, int);
	 void SetOutputLimits(double Min, double Max);
	 void Initialize();
	 bool Compute();
};

#endif


