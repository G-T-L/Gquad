// LED.h

#ifndef _LED_h
#define _LED_h
//#define CommonAnode
#define CommonCathode
#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

#ifdef CommonAnode
#define LED_ON LOW
#define LED_ HIGH
#endif // CommonAnode

#ifdef CommonCathode
//共低
#define LED_ON HIGH
#define LED_OFF LOW
#endif // CommonCathode

const int BlinkInterval = 200;


/*
灯语	                                    含义	                                  解决办法
M1/M2/M3/M4旋转闪烁	    未接收到遥控信号	              开启遥控器或连接手机APP
M1/M2和M3/M4交替闪烁	IMU未校准	                      将飞机水平放置地面上，按遥控器上“-”键
M1/M2/M3/M4同时闪烁	    低电压提示	                      电池充电
M1/M2闪烁	                        正常启动待机状态
M1/M2常亮3秒	                IMU校准中
M1闪烁	                            配置蓝牙模块中
*/

class LEDClass
{
 protected:
	 int LedPin[4];

 public:
	 void attach(const int motorLEDPin[4]);

	 //M1 / M2 / M3 / M4旋转闪烁	    未接收到遥控信号	              开启遥控器或连接手机APP
	 void rotateBlink();

	 //M1 / M2和M3 / M4交替闪烁	IMU未校准	                      将飞机水平放置地面上，按遥控器上“ - ”键
	 void M1M2_M3M4Blink();

	//M1 / M2 / M3 / M4同时闪烁	    低电压提示	                      电池充电
	 void allBlink();

	//	M1 / M2闪烁	                        正常启动待机状态
	 void M1_M2Blink();

	 //M1 / M2常亮3秒	                IMU校准中
	 void M1M2LongBright();

	 //M1闪烁	                            配置蓝牙模块中
	 void M1Blink();
};

extern LEDClass MotorLED;

#endif

