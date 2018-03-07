
#include "LED.h"
LEDClass MotorLED;

void LEDClass::attach(const int motorLEDPin[4])
{
	for (int i = 0; i < 4; i++)
	{
		LedPin[i] = motorLEDPin[i];
		pinMode(LedPin[i], OUTPUT);
		digitalWrite(LedPin[i], LED_OFF);
	}
}

void LEDClass::rotateBlink()
{
	for (int i = 0; i < 4; i++)
	{
		digitalWrite(LedPin[i], LED_ON);
		delay(BlinkInterval);
		digitalWrite(LedPin[i], LED_OFF);
	}
}

void LEDClass::M1M2_M3M4Blink()
{
	for (int i = 0; i < 2; i++)
	{
		digitalWrite(LedPin[i], LED_ON);
		digitalWrite(LedPin[i + 2], LED_ON);
		delay(BlinkInterval);
		digitalWrite(LedPin[i], LED_OFF);
		digitalWrite(LedPin[i + 2], LED_OFF);
	}
}

void LEDClass::allBlink()
{
	for (int i = 0; i < 4; i++)
		digitalWrite(LedPin[i], LED_ON);
	delay(BlinkInterval);
	for (int i = 0; i < 4; i++)
		digitalWrite(LedPin[i], LED_OFF);
}

void LEDClass::M1_M2Blink()
{
	digitalWrite(LedPin[1], LED_ON);
	delay(BlinkInterval);
	digitalWrite(LedPin[1], LED_OFF);

	digitalWrite(LedPin[2], LED_ON);
	delay(BlinkInterval);
	digitalWrite(LedPin[2], LED_OFF);
}

void LEDClass::M1M2LongBright()
{
	digitalWrite(LedPin[1], LED_ON);
	digitalWrite(LedPin[2], LED_ON);
	delay(3000);
	digitalWrite(LedPin[1], LED_OFF);
	digitalWrite(LedPin[2], LED_OFF);
	delay(1000);
}

void LEDClass::M1Blink()
{
	digitalWrite(LedPin[1], LED_ON);
	delay(BlinkInterval);
	digitalWrite(LedPin[1], LED_OFF);
	delay(BlinkInterval);
}
