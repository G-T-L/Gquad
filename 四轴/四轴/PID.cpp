// 
// 
// 

#include "PID.h"

PIDClass::PIDClass(volatile double *Input, volatile double *Output, volatile double *Setpoint, double Kp, double Ki, double Kd, int ControllerDirection)
{
	myOutput = Output;
	myInput = Input;
	mySetpoint = Setpoint;
	kp = Kp;
	ki = Ki;
	kd = Kd;
	if (ControllerDirection)
	{
		kp = (0 - kp);
		ki = (0 - ki);
		kd = (0 - kd);
	}
}

void PIDClass::Initialize()
{
	lastTime = millis();
	lastInput = *myInput;
	IntegralSum = *myOutput;
	if (IntegralSum > outMax) IntegralSum = outMax;
	else if (IntegralSum < outMin) IntegralSum = outMin;
}

void PIDClass::SetOutputLimits(double Min, double Max)
{
	if (Min >= Max) return;
	outMin = Min;
	outMax = Max;

	if (*myOutput > outMax) *myOutput = outMax;
	else if (*myOutput < outMin) *myOutput = outMin;

	if (IntegralSum > outMax) IntegralSum = outMax;
	else if (IntegralSum < outMin) IntegralSum = outMin;
}

bool PIDClass::Compute()
{
	unsigned long now = millis();
	//unsigned long timeChange = (now - lastTime);
		/*Compute all the working error variables*/
		double input = *myInput;
		double error = *mySetpoint - input;
		double dInput = (input - lastInput);

		double output;
		output = kp * error;
		output -= kd * dInput;

		IntegralSum += (ki * error);
		if (IntegralSum > outMax) IntegralSum = outMax;
		else if (IntegralSum < outMin) IntegralSum = outMin;

		output += IntegralSum;

		if (output > outMax) output = outMax;
		else if (output < outMin) output = outMin;
		*myOutput = output;

		lastInput = input;
		//lastTime = now;
		return true;
}
