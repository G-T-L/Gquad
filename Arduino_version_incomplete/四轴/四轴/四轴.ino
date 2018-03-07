/*
 Name:		四轴.ino
 Created:	2017/8/26 0:08:06
 Author:		GTL

 //Digital Pins Usable For Interrupts on MEGA2560 : 2, 3, 18, 19, 20, 21
 20,21 used for IIC

To Do
传感器校准
PID参数初步确定
遥控器接收

To Test

遥控器接收

FIX ME : 

Do later:
无头模式
电机驱动  用定时器3,5;

//命名规则:常量开头大写,变量开头小写的驼峰命名法
//另外中断中改变的话则需要加 volatile
*/

#include <EEPROM.h>
#include "PID.h"
//#include "Motor.h"
//#include "LED.h"
#include <JY901.h>
//#define MicroQuadcopter       //小四轴的电机驱动不一样,对应下面的motorDriver函数不一样,其他地方都一样
#define DEBUG

#pragma region Paraments Declaration


#define BluetoothPort Serial2
#define BlueToothBuadRate 115200
const int BuzzerBeepInterval = 1000;

volatile boolean connectionLost = false;
volatile boolean newJY901ValueRequest = true;
volatile boolean loop_10ms = false;

#pragma region Channel Receiver Paraments
//////////////遥控器////////////////
//channel1：
//channel2：
//channel3：
//channel4：
//channel5：
//channel6：
//
volatile boolean channel1NewValue = false;
volatile boolean channel2NewValue = false;
volatile boolean channel3NewValue = false;
volatile boolean channel4NewValue = false;

volatile unsigned char Timer2OverflowCounts = 1;
const unsigned char Timer2PeriodTicks = 250;//OCR2A
const unsigned char Timer2Period = 16;    //us 256分频,OCR2A = 250时
const unsigned int  MinChannelValue = 500;
const unsigned int  MaxChannelValue = 2500;
//const unsigned char  MinChannelTicks = MinChannelValue / Timer2Period;//31
//const unsigned char  MaxChannelTicks = MaxChannelValue / Timer2Period;//156

volatile int channel_yaw = 1500, channel_pitch = 1500, channel_thrust = 1000, channel_roll = 1500, channel_switch = 1000, channel_rotaryKnob = 1000;

#pragma endregion

//注意yaw pitch 的方向和前进方向相反

#pragma region Pin Layout Paraments Declaration

#define MotorPin0 23
#define MotorPin1 22
#define MotorPin2 24
#define MotorPin3 25
const int MotorPin[4] = { MotorPin0 ,MotorPin1, MotorPin2, MotorPin3 };

#define MotorPin_PORT  PORTA
#define MotorPin0_Unmapped PA1
#define MotorPin1_Unmapped  PA0
#define MotorPin2_Unmapped  PA2
#define MotorPin3_Unmapped  PA3
/*
      灯语	                                   含义	                                     解决办法
M1/M2/M3/M4旋转闪烁	    未接收到遥控信号	              开启遥控器或连接手机APP
M1/M2和M3/M4交替闪烁	IMU未校准	                      将飞机水平放置地面上，按遥控器上“-”键
M1/M2/M3/M4同时闪烁	    低电压提示	                      电池充电
M1/M2闪烁	                        正常启动待机状态
M1/M2常亮3秒	                IMU校准中
M1闪烁	                            配置蓝牙模块中
*/
#define MotorLEDPin0 0
#define MotorLEDPin1 0
#define MotorLEDPin2 0
#define MotorLEDPin3 0
const int MotorLEDPin[4] = { MotorLEDPin0,MotorLEDPin1,MotorLEDPin2,MotorLEDPin3 };

#define ChannelReceiverPin0 0
#define ChannelReceiverPin1 0
#define ChannelReceiverPin2 0
#define ChannelReceiverPin3 0
#define ChannelReceiverPin4 0
#define ChannelReceiverPin5 0
const int ChannelReceiverPin[6] = { ChannelReceiverPin0,ChannelReceiverPin1,ChannelReceiverPin2,ChannelReceiverPin3,ChannelReceiverPin4,ChannelReceiverPin5 };

const int BuzzerPin = 11;
const int UltrasoundWaveSensorPin = 0;

#pragma endregion

#pragma region Motor Paraments Declaration
//////////////////// Y //////////////////////////////////////////////////////////////
//////////////////// ^ //////////////////////////////////////////////////////////////
///////// M3 ////// | ///// M2 /////////////////////////////////////////////////////
//////////////////// | ///////////////////////////////////////////////////////////////
/////---------------|-------------->X///////////////////////////////////////////////
//////////////////// | ///////////////////////////////////////////////////////////////
///////// M0 ////// | ////// M1 ////////////////////////////////////////////////////
//////////////////// | ///////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////
////Z轴符合右手定则,pitch roll yaw 均符合右手定则//////////////////////////////

#define motorPWMvaule0(vaule) (OCRB2_NEW_VALUE=(constrain(vaule, MinThrottle, MaxThrottle))/8-1)
#define motorPWMvaule1(vaule) (OCR1A=2*(constrain(vaule, MinThrottle, MaxThrottle))-1)
#define motorPWMvaule2(vaule) (OCR1B=2*(constrain(vaule, MinThrottle, MaxThrottle))-1)
#define motorPWMvaule3(vaule) (OCR1C=2*(constrain(vaule, MinThrottle, MaxThrottle))-1)
volatile unsigned char OCRB2_NEW_VALUE = 124;

const int MaxThrottle = 2000;
const int MinThrottle = 1000;

volatile int motorPWM[4];                                                      //输出值,在油门值的基础上加姿态的调控值而得
volatile int Throttle = 0;                                                          //油门值,接收于遥控器,再映射而得
volatile int pitch = 0;
volatile int roll = 0;
volatile int yaw = 0;

#pragma endregion

#pragma region Height PID Paraments Declaration

const double g = 1;

double HeightProportion = 1;
double HeightIntegral = 1;
double HeightDerivation = 1;

double ZheightProportion = 1;
double ZheightIntegral = 1;
double ZheightDerivation = 1;

double HeightNow = 0;
double ExpectedHeight = 0;
double ZAcceleration = g;
double ExpectedZAcceleration = g;
double HeightThrottleToChange = 0;

PIDClass HeightPID(&HeightNow, &ExpectedZAcceleration, &ExpectedHeight,
	HeightProportion, HeightIntegral, HeightDerivation, DIRECT);
PIDClass YawheightAccelerationPID(&ZAcceleration, &HeightThrottleToChange, &ExpectedZAcceleration,
	ZheightProportion, ZheightIntegral, ZheightDerivation, DIRECT);
#pragma endregion

#pragma region Position PID Paraments Declaration
//通过角度与角速度由PID计算出各电机油门的该变量

double AngleProportion = 1;
double AngleIntegral = 0.1;
double AngleDerivation = 0.1;

double GyroProportion = 10;
double GyroIntegral = 1;
double GyroDerivation = 1;

volatile double Pitchangle = 0;
volatile double Rollangle = 0;
volatile double Yawangle = 0;

volatile double ExpectedPitchangle = 0;
volatile double ExpectedRollangle = 0;
volatile double ExpectedYawangle = 0;

volatile double PitchGyro = 0;
volatile double RollGyro = 0;
volatile double YawGyro = g;        

volatile double PitchexpectedGyro = 0;
volatile double RollexpectedGyro = 0;
volatile double YawexpectedGyro = g;

volatile double pitchThrottleToChange = 0;
volatile double rollThrottleToChange = 0;
volatile double yawThrottleToChange = 0;

PIDClass PitchanglePID(&Pitchangle, &PitchexpectedGyro, &ExpectedPitchangle,
	AngleProportion, AngleIntegral, AngleDerivation, DIRECT);
PIDClass RollanglePID(&Rollangle, &RollexpectedGyro, &ExpectedRollangle,
	AngleProportion, AngleIntegral, AngleDerivation, DIRECT);
PIDClass YawanglePID(&Yawangle, &YawexpectedGyro, &ExpectedYawangle,
	AngleProportion, AngleIntegral, AngleDerivation, DIRECT);

PIDClass PitchGyroPID(&PitchGyro, &pitchThrottleToChange, &PitchexpectedGyro,
	GyroProportion, GyroIntegral, GyroDerivation, DIRECT);
PIDClass RollGyroPID(&RollGyro, &rollThrottleToChange, &RollexpectedGyro,
	GyroProportion, GyroIntegral, GyroDerivation, DIRECT);
PIDClass YawGyroPID(&YawGyro, &yawThrottleToChange, &YawexpectedGyro,
	GyroProportion, GyroIntegral, GyroDerivation, DIRECT);

#pragma endregion

#pragma endregion

void setup()
{
	motorInitialization();
	delay(3000);//
	pinMode(11, OUTPUT);
	pinMode(12, OUTPUT);
	pinMode(13, OUTPUT);
	
	Serial.begin(250000);
	BluetoothPort.begin(BlueToothBuadRate);

	//MotorLED.attach(MotorLEDPin);

	PitchanglePID.SetOutputLimits(-10, 10);
	RollanglePID.SetOutputLimits(-10, 10);
	YawanglePID.SetOutputLimits(-10, 10);
	PitchGyroPID.SetOutputLimits(-200, 200);
	RollGyroPID.SetOutputLimits(-200, 200);
	YawGyroPID.SetOutputLimits(-200, 200);

	PitchanglePID.Initialize(); 
	RollanglePID.Initialize();
	YawanglePID.Initialize();
	PitchGyroPID.Initialize();
	RollGyroPID.Initialize();
	YawGyroPID.Initialize();
	
	loadPIDparamentsFromEEPROM();
	JY901.StartIIC();
	decodeInterruptInitial();
	// Serial.print("millis : "); Serial.println(millis());Serial.println("Setup Finished");
	Timer4_10msPeriod_500nsPrecision();
	Timer3_32msPeriod_500nsPrecision();
}

void loop() 
{
	while (1)
	{
		bitSet(PINB, 6);
		if (loop_10ms)
		{
			loop_10ms = false;
			mainCirculation();
		}
		static unsigned int counts = 0;
		if (1)
			if (counts++ > 60000)
			{
				counts = 0;
				static long time = 0;
				if (BluetoothPort.available())
					if (updatePIDparamentFromSerial())
						;//Serial.println("update successed");
					else
						;// Serial.println("update failed");
				reportCurrentStatus();
				Serial.print("millis change : "); Serial.println(millis() - time);
				time = millis();
			}
		Serial.print("millis : "); Serial.println(millis());
	}
}

#pragma region decode PPM-PWM Signal by Interruption
//volatile int channelHighTime1, channelHighTime2, channelHighTime3, channelHighTime4, channelHighTime5, channelHighTime6;
void decodeInterruptInitial()
{
	//Digital Pins Usable For Interrupts on MEGA2560 : 2, 3, 18, 19, 20, 21

	//思路2:用两个中断引脚各连三个接收机引脚,因为高电平不同时,建立各全局变量count,当时间大于5ms则认定为新一轮数据,然后依次填入count++
	//思路3:每20us扫描一次,判断状态
	attachInterrupt(digitalPinToInterrupt(2), channel1, CHANGE);
	attachInterrupt(digitalPinToInterrupt(3), channel2, CHANGE);
	attachInterrupt(digitalPinToInterrupt(18), channel3, CHANGE);
	attachInterrupt(digitalPinToInterrupt(19), channel4, CHANGE);
	//attachInterrupt(digitalPinToInterrupt(20), channel5, CHANGE);
	//attachInterrupt(digitalPinToInterrupt(21), channel6, CHANGE);

}

void channel1()
{
	static unsigned int LastChannelValue = 1500;
	static unsigned long lastTime = 0;
	unsigned int interval = micros() - lastTime;
	if (interval > MinChannelValue&&interval < MaxChannelValue)
	{
		int channelValue = (LastChannelValue + channel_yaw + interval) / 3;
		LastChannelValue = channel_yaw;
		channel_yaw = channelValue;
		channel1NewValue = true;
	}
	lastTime = micros();
}

void channel2()
{
	static unsigned int LastChannelValue = 1500;
	static unsigned long lastTime = 0;
	unsigned int interval = micros() - lastTime;
	if (interval > MinChannelValue&&interval < MaxChannelValue)
	{
		int channelValue = (LastChannelValue + channel_pitch + interval) / 3;
		LastChannelValue = channel_pitch;
		channel_pitch = channelValue;
		channel2NewValue = true;
	}
	lastTime = micros();
}

void channel3()
{
	static unsigned int LastChannelValue = 1000;
	static unsigned long lastTime = 0;
	unsigned int interval = micros() - lastTime;
	if (interval > MinChannelValue&&interval < MaxChannelValue)
	{
		int channelValue = (LastChannelValue + channel_thrust + interval) / 3;
		LastChannelValue = channel_thrust;
		channel_thrust = channelValue;
		channel3NewValue = true;
	}
	lastTime = micros();
}

void channel4()
{
	static unsigned int LastChannelValue = 1500;
	static unsigned long lastTime = 0;
	unsigned int interval = micros() - lastTime;
	if (interval > MinChannelValue&&interval < MaxChannelValue)
	{
		int channelValue = (LastChannelValue + channel_roll + interval) / 3;
		LastChannelValue = channel_roll;
		channel_roll = channelValue;
		channel4NewValue = true;
	}
	lastTime = micros();
}

/*
void channel1()
{
	static unsigned int LastChannelValue = 1500;
	static unsigned int LastTCNT2 = 0;
	static unsigned int lastTimer2OverflowCounts = 0;
	int intervalTicks = ((int)TCNT2 - LastTCNT2 + Timer2PeriodTicks * (int)((int)Timer2OverflowCounts - lastTimer2OverflowCounts));
	//如果计算得为PWM低电平周期或Timer2OverflowCounts溢出导致负数,则舍去	
	if (intervalTicks > MinChannelTicks && intervalTicks < MaxChannelTicks)
	{
		int channelValue = (LastChannelValue + channel_yaw + intervalTicks * Timer2Period) / 3;
		LastChannelValue = channel_yaw;
		channel_yaw = channelValue;
		channel1NewValue = true;
	}

	lastTimer2OverflowCounts = Timer2OverflowCounts;
	LastTCNT2 = TCNT2;
}
void channel2()
{
	static unsigned int LastChannelValue = 1500;
	static unsigned int LastTCNT2 = 0;
	static unsigned int lastTimer2OverflowCounts = 0;
	int intervalTicks = ((int)TCNT2 - LastTCNT2 + Timer2PeriodTicks * (int)((int)Timer2OverflowCounts - lastTimer2OverflowCounts));

	//如果计算得为PWM低电平周期或Timer2OverflowCounts溢出导致负数,则舍去	
	if (intervalTicks > MinChannelTicks && intervalTicks < MaxChannelTicks)
	{
		int channelValue = (LastChannelValue + channel_pitch + intervalTicks * Timer2Period) / 3;
		LastChannelValue = channel_pitch;
		channel_pitch = channelValue;
	}
	lastTimer2OverflowCounts = Timer2OverflowCounts;
	LastTCNT2 = TCNT2;
}
void channel3()
{
	static unsigned int LastChannelValue = 1000;
	static unsigned int LastTCNT2 = 0;
	static unsigned int lastTimer2OverflowCounts = 0;
	int intervalTicks = ((int)TCNT2 - LastTCNT2 + Timer2PeriodTicks * (int)((int)Timer2OverflowCounts - lastTimer2OverflowCounts));

	//如果计算得为PWM低电平周期或Timer2OverflowCounts溢出导致负数,则舍去	
	if (intervalTicks > MinChannelTicks && intervalTicks < MaxChannelTicks)
	{
		int channelValue = (LastChannelValue + channel_thrust + intervalTicks * Timer2Period) / 3;
		LastChannelValue = channel_thrust;
		channel_thrust = channelValue;
	}

	lastTimer2OverflowCounts = Timer2OverflowCounts;
	LastTCNT2 = TCNT2;
}
void channel4()
{
	static unsigned int LastChannelValue = 1500;
	static unsigned int LastTCNT2 = 0;
	static unsigned int lastTimer2OverflowCounts = 0;
	int intervalTicks = ((int)TCNT2 - LastTCNT2 + Timer2PeriodTicks * (int)((int)Timer2OverflowCounts - lastTimer2OverflowCounts));

	//如果计算得为PWM低电平周期或Timer2OverflowCounts溢出导致负数,则舍去	
	if (intervalTicks > MinChannelTicks && intervalTicks < MaxChannelTicks)
	{
		int channelValue = (LastChannelValue + channel_roll + intervalTicks * Timer2Period) / 3;
		LastChannelValue = channel_roll;
		channel_roll = channelValue;
	}

	lastTimer2OverflowCounts = Timer2OverflowCounts;
	LastTCNT2 = TCNT2;
}
*/
#pragma endregion

#pragma region Motor Driver
/*
needed paraments declaration : 
#define MotorPin_PORT  PORTA
#define MotorPin0_Unmapped PA1
#define MotorPin1_Unmapped  PA0
#define MotorPin2_Unmapped  PA2
#define MotorPin3_Unmapped  PA3
#define motorPWMvaule0(vaule) (OCRB2_NEW_VALUE=(vaule)/8-1)
#define motorPWMvaule1(vaule) (OCR1A=2*(vaule)-1)
#define motorPWMvaule2(vaule) (OCR2A=2*(vaule)-1)
#define motorPWMvaule3(vaule) (OCR3A=2*(vaule)-1)
volatile unsigned char OCRB2_NEW_VALUE = 124;
*/
void Timer2_2msPeriod_8usPrecision()
{
	//遥控器精度20us? 应该够用
	//128分频
	//Mode 2 ----CTC
	cli();
	TCCR2A = 0;
	TCCR2B = 0;
	TCNT2 = 0;
	OCR2A = 249;//8us for 128 prescaling
	OCR2B = 149;
	TCCR2A |= (1 << WGM21);//CTC mode:
	TCCR2B |= (1 << CS20) | (0 << CS21) | (1 << CS22);// prescaling 128	
	TIMSK2 |= (1 << OCIE2A);
	TIMSK2 |= (1 << OCIE2B);
	sei();
}

ISR(TIMER2_OVF_vect)
{

}

ISR(TIMER2_COMPA_vect)
{
	static char counts = 0;
	if (counts++ == 9)
	{
		counts = 0;
		OCR2B = OCRB2_NEW_VALUE;
		bitSet(MotorPin_PORT, MotorPin0_Unmapped);
		TCNT2 = 0;
	}
}

ISR(TIMER2_COMPB_vect, ISR_NAKED)
{
	OCR2B = 255;
	bitClear(MotorPin_PORT, MotorPin0_Unmapped);
	reti();
}

void Timer1_32msPeriod_500nsPrecision()
{
	//8分频	
	//Mode 14 ----Fast PWM 
	//ICR1 Top  
	//update OCRnx at buttom
	//TOV1 Flag Set on Top

	cli();
	TCCR1A = 0;
	TCCR1B = 0;
	TCCR1A |= _BV(WGM31);
	TCCR1B |= _BV(WGM32) | _BV(WGM33);
	TCCR1B |= _BV(CS11); //8分频			
	ICR1 = 39999;//20ms		
	OCR1A = 1999;
	OCR1B = 1999;
	OCR1C = 1999;
	TIMSK1 |= (1 << TOIE1);
	TIMSK1 |= (1 << OCIE1A);
	TIMSK1 |= (1 << OCIE1B);
	TIMSK1 |= (1 << OCIE1C);
	sei();
}

ISR(TIMER1_OVF_vect, ISR_NAKED)
{
	bitSet(MotorPin_PORT, MotorPin1_Unmapped);
	bitSet(MotorPin_PORT, MotorPin2_Unmapped);
	bitSet(MotorPin_PORT, MotorPin3_Unmapped);
	reti();
}

ISR(TIMER1_COMPA_vect, ISR_NAKED)
{
	bitClear(MotorPin_PORT, MotorPin1_Unmapped);
	reti();
}

ISR(TIMER1_COMPB_vect, ISR_NAKED)
{
	bitClear(MotorPin_PORT, MotorPin2_Unmapped);
	reti();
}

ISR(TIMER1_COMPC_vect, ISR_NAKED)
{
	bitClear(MotorPin_PORT, MotorPin3_Unmapped);
	reti();
}

void motorInitialization()
{

	bitSet(DDRA, MotorPin0_Unmapped);
	bitSet(DDRA, MotorPin1_Unmapped);
	bitSet(DDRA, MotorPin2_Unmapped);
	bitSet(DDRA, MotorPin3_Unmapped);
	Timer1_32msPeriod_500nsPrecision();
	Timer2_2msPeriod_8usPrecision();
}

void motorThrustRangeReset()
{
	motorInitialization();

	motorPWMvaule0(2000);
	motorPWMvaule1(2000);
	motorPWMvaule2(2000);
	motorPWMvaule3(2000);
	delay(4000);
	
	motorPWMvaule0(1000);
	motorPWMvaule1(1000);
	motorPWMvaule2(1000);
	motorPWMvaule3(1000);
	delay(1000);
}

#pragma endregion

#pragma region Timer3 (unused for now)

void Timer3_32msPeriod_500nsPrecision()
{
	//8分频	
	//Mode 14 ----Fast PWM 
	//ICR3 Top  
	//update OCRnx at buttom
	//TOV3 Flag Set on Top

	cli();
	TCCR3A = 0;
	TCCR3B = 0;
	TCCR3A |= _BV(WGM31);
	TCCR3B |= _BV(WGM32) | _BV(WGM33);
	TCCR3B |= _BV(CS31); //8分频			
	ICR3 = 39999;//20ms		
	OCR3A = 3999;
	OCR3B = 3999;
	OCR3C = 3999;
	TIMSK3 |= (1 << TOIE3);
	//TIMSK3 |= (1 << OCIE3A);
	//TIMSK3 |= (1 << OCIE3B);
	//TIMSK3 |= (1 << OCIE3C);
	sei();
}

ISR(TIMER3_OVF_vect)
{
	bitSet(PINB, 7);

}

ISR(TIMER3_COMPA_vect)
{
}

ISR(TIMER3_COMPB_vect)
{
}

ISR(TIMER3_COMPC_vect)
{
}

#pragma endregion

#pragma region Timer4 (mainCirculation:10ms)

void Timer4_10msPeriod_500nsPrecision()
{
	//8分频	
	//Mode 14 ----Fast PWM 
	//ICR4 Top  
	//update OCR4x at buttom
	//TOV4 Flag Set on Top

	cli();
	TCCR4A = 0;
	TCCR4B = 0;
	TCCR4A |= _BV(WGM41);
	TCCR4B |= _BV(WGM43) | _BV(WGM42);
	TCCR4B |= (0 << CS42) | (1 << CS41) | (0 << CS40); //8分频			
	ICR4 = 20000;//10ms
	OCR4A = 0;//10ms
	OCR4B = 0;
	OCR4C = 0;
	TIMSK4 |= (1 << TOIE4);
	//TIMSK4 |= (1 << OCIE4A);
	//TIMSK4 |= (1 << OCIE4B);
	//TIMSK4 |= (1 << OCIE4C);
	sei();
}

ISR(TIMER4_COMPA_vect)
{

}

ISR(TIMER4_COMPB_vect)
{

}

ISR(TIMER4_COMPC_vect)
{

}

ISR(TIMER4_OVF_vect)
{
	loop_10ms = true;
	bitSet(PINB, 5);
	//static unsigned char i = 0;
	//if (i++ > 18)
	//{
	//	i = 0;
	//}
}

#pragma endregion

#pragma region Timer5 (unused for now)

void Timer5_4sPeriod_64usPrecision()
{
	//1024分频	
	//Mode 14 ----Fast PWM 
	//ICR5 Top  
	//update OCR5x at buttom
	//TOV5 Flag Set on Top

	cli();
	TCCR5A = 0;
	TCCR5B = 0;
	TCCR5A |= _BV(WGM51);
	TCCR5B |= _BV(WGM53) | _BV(WGM52);
	TCCR5B |= _BV(CS52) | _BV(CS50); //1024分频			
	ICR5 = 62500;//4s	
	OCR5A = 156;//10ms
	OCR5B = 0;
	OCR5C = 0;
	//TIMSK5 |= (1 << TOIE5);
	//TIMSK5 |= (1 << OCIE5A);
	//TIMSK5 |= (1 << OCIE5B);
	//TIMSK5 |= (1 << OCIE5C);
	sei();
}

ISR(TIMER5_COMPA_vect)
{

}

ISR(TIMER5_COMPB_vect)
{

}

ISR(TIMER5_COMPC_vect)
{

}

ISR(TIMER5_OVF_vect)
{

}

#pragma endregion

void mainCirculation()
{
	//to do 限定范围
	ExpectedPitchangle = -(constrain(channel_pitch, MinThrottle, MaxThrottle) - 1500) / 20;
	ExpectedRollangle = (constrain(channel_roll, MinThrottle, MaxThrottle) - 1500) / 20;
	ExpectedYawangle = -(constrain(channel_yaw, MinThrottle, MaxThrottle) - 1500) / 20;

	//IIC used, cannot be updated in ISR service
	JY901.GetGyro();
	JY901.GetAngle();

	PitchGyro = JY901.stcGyro.w[0] / 32768.0 * 2000;
	RollGyro = JY901.stcGyro.w[1] / 32768.0 * 2000;
	YawGyro = JY901.stcGyro.w[2] / 32768.0 * 2000;
	Pitchangle = JY901.stcAngle.Angle[0] / 32768.0 * 180 - 1.8;
	Rollangle = JY901.stcAngle.Angle[1] / 32768.0 * 180 + 1;
	Yawangle = JY901.stcAngle.Angle[2] / 32768.0 * 180;

	//newJY901ValueRequest = true;
	//Serial.print("Pitchangle : "); Serial.println(Pitchangle);

	PitchanglePID.Compute();
	RollanglePID.Compute();
	YawanglePID.Compute();
	PitchGyroPID.Compute();
	RollGyroPID.Compute();
	YawGyroPID.Compute();

	//HeightPID.Compute();
	//YawheightAccelerationPID.Compute();

	roll = (int)rollThrottleToChange;
	pitch = (int)pitchThrottleToChange;
	yaw = (int)yawThrottleToChange;
	Throttle = (channel_thrust - 1000)* (channel_thrust - 1000) / 1250 + 1000;      //平滑油门 1000-1800
	 //融合计算油门值			
	motorPWM[0] = Throttle - pitch + roll - yaw;
	motorPWM[1] = Throttle - pitch - roll + yaw;
	motorPWM[2] = Throttle + pitch - roll - yaw;
	motorPWM[3] = Throttle + pitch + roll + yaw;
	if (Throttle < 1030)
	{
		motorPWMvaule0(1000);
		motorPWMvaule1(1000);
		motorPWMvaule2(1000);
		motorPWMvaule3(1000);
	}

	motorPWMvaule0(motorPWM[0]);
	motorPWMvaule1(motorPWM[1]);
	motorPWMvaule2(motorPWM[2]);
	motorPWMvaule3(motorPWM[3]);

#ifndef DEBUG
	//when no connection found, don't emergencyShutDown
	static char channel1NoNewValueCount = 0;
	if (channel1NewValue)
	{
		channel1NewValue = false;
		channel1NoNewValueCount = 0;
	}
	else
	{
		if (channel1NoNewValueCount++ > 100)
		{
			connectionLost = true;
			emergencyShutDown();
		}
	}
#endif
}

void emergencyShutDown()
{
	cli();
	motorPWMvaule0(1000);
	motorPWMvaule1(1000);
	motorPWMvaule2(1000);
	motorPWMvaule3(1000);
	savePIDparamentsToEEPROM();
	while (1)
	{
		alarm();
		delay_ms(1000);
	}
}

#pragma region BlueTooth Operation

boolean updatePIDparamentFromSerial()
{
	int i = 0;
	char buffer[100] = { "" };
	while (BluetoothPort.available())
	{
		buffer[i++] = BluetoothPort.read();
	}
	if (i)
	{
		char *ap, *ai, *ad, *gp, *gi, *gd;
		if (NULL != (ap = strstr(buffer, "AP:")))
		{
			AngleProportion = atof(ap + 3);
		}
		if (NULL != (ai = strstr(buffer, "AI:")))
		{
			AngleIntegral = atof(ai + 3);
		}
		if (NULL != (ad = strstr(buffer, "AD:")))
		{
			AngleDerivation = atof(ad + 3);
		}
		if (NULL != (gp = strstr(buffer, "GP:")))
		{
			GyroProportion = atof(gp + 3);
		}
		if (NULL != (gi = strstr(buffer, "GI:")))
		{
			GyroIntegral = atof(gi + 3);
		}
		if (NULL != (gd = strstr(buffer, "GD:")))
		{
			GyroDerivation = atof(gd + 3);
		}
		return true;
	}
	else
	{
		return false;
	}
}

void reportCurrentStatus()
{
	BluetoothPort.println("*************Status Report**************");
	BluetoothPort.print("Angle Proportion : "); BluetoothPort.println(AngleProportion);
	BluetoothPort.print("Angle Integral : "); BluetoothPort.println(AngleIntegral);
	BluetoothPort.print("Angle Derivation : "); BluetoothPort.println(AngleDerivation);
	BluetoothPort.print("Gyro Proportion : "); BluetoothPort.println(GyroProportion);
	BluetoothPort.print("Gyro Integral : "); BluetoothPort.println(GyroIntegral);
	BluetoothPort.print("Gyro Derivation : "); BluetoothPort.println(GyroDerivation);
	BluetoothPort.println();
	BluetoothPort.print("channel_pitch : "); BluetoothPort.println(channel_pitch);
	BluetoothPort.print("ExpectedPitchangle : "); BluetoothPort.println(ExpectedPitchangle);
	BluetoothPort.print("Pitchangle : "); BluetoothPort.println(Pitchangle);
	BluetoothPort.print("PitchexpectedGyro : "); BluetoothPort.println(PitchexpectedGyro);
	BluetoothPort.print("PitchGyro : "); BluetoothPort.println(PitchGyro);
	BluetoothPort.print("pitchThrottleToChange : "); BluetoothPort.println(pitchThrottleToChange);
	BluetoothPort.println();
}

#pragma endregion

#pragma region EEPROM Operation

void savePIDparamentsToEEPROM()
{
	/*The purpose of this example is to show the EEPROM.put() method that writes data on EEPROM using also the EEPROM.update() that writes data only if it is different from the previous content of the locations to be written. The number of bytes written is related to the datatype or custom structure of the variable to be written. 
	*/

	int eeAddress = 0;   //Location we want the data to be put.
	EEPROM.update(eeAddress, 66);
	eeAddress++;
	double temp;
	//if(EEPROM.read(eeAddress))
	EEPROM.put(eeAddress, AngleProportion);
	eeAddress += sizeof(double);
	EEPROM.put(eeAddress, AngleIntegral);
	eeAddress += sizeof(double);
	EEPROM.put(eeAddress, AngleDerivation);
	eeAddress += sizeof(double);
	EEPROM.put(eeAddress, GyroProportion);
	eeAddress += sizeof(double);
	EEPROM.put(eeAddress, GyroIntegral);
	eeAddress += sizeof(double);
	EEPROM.put(eeAddress, GyroDerivation);
	eeAddress += sizeof(double);
	EEPROM.update(eeAddress, 67);
}

boolean loadPIDparamentsFromEEPROM()
{
	int eeAddress = 0;   //Location we want the data to be put.
	if (EEPROM.read(eeAddress++) == 66)
	{
		EEPROM.get(eeAddress, AngleProportion);
		eeAddress += sizeof(double);
		EEPROM.get(eeAddress, AngleIntegral);
		eeAddress += sizeof(double);
		EEPROM.get(eeAddress, AngleDerivation);
		eeAddress += sizeof(double);
		EEPROM.get(eeAddress, GyroProportion);
		eeAddress += sizeof(double);
		EEPROM.get(eeAddress, GyroIntegral);
		eeAddress += sizeof(double);
		EEPROM.get(eeAddress, GyroDerivation);
		eeAddress += sizeof(double);
		if (EEPROM.read(eeAddress) == 67)
			return true;
	}
}

void clearEEPROM()
{
	for (int x = 0; x < EEPROM.length(); x = x + 1)
	{    //Loop end of EEPROM address
		if (EEPROM.read(x) == 0)
		{              //If EEPROM address 0
					   // do nothing, already clear, go to the next address in order to save time and reduce writes to EEPROM
		}
		else
		{
			EEPROM.write(x, 0); 			// if not write 0 to clear, it takes 3.3mS
		}
	}
	//BluetoothPort.println(F("EEPROM Successfully Wiped"));
}

#pragma endregion

#pragma region Auxiliary Functions

void delay_ms(int t)
{
	for (int i = 0; i < t; i++)
		for (volatile unsigned long j = 0; j < 454; j++); //cost 1 ms
}

//di-----di-----
void alarm()
{
	digitalWrite(BuzzerPin, HIGH);
	delay_ms(BuzzerBeepInterval);
	digitalWrite(BuzzerPin, LOW);
	delay_ms(BuzzerBeepInterval);
	digitalWrite(BuzzerPin, HIGH);
	delay_ms(BuzzerBeepInterval);
	digitalWrite(BuzzerPin, LOW);
	delay_ms(BuzzerBeepInterval);
}

#pragma endregion

