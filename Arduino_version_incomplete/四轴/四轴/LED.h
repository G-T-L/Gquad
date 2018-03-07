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
//����
#define LED_ON HIGH
#define LED_OFF LOW
#endif // CommonCathode

const int BlinkInterval = 200;


/*
����	                                    ����	                                  ����취
M1/M2/M3/M4��ת��˸	    δ���յ�ң���ź�	              ����ң�����������ֻ�APP
M1/M2��M3/M4������˸	IMUδУ׼	                      ���ɻ�ˮƽ���õ����ϣ���ң�����ϡ�-����
M1/M2/M3/M4ͬʱ��˸	    �͵�ѹ��ʾ	                      ��س��
M1/M2��˸	                        ������������״̬
M1/M2����3��	                IMUУ׼��
M1��˸	                            ��������ģ����
*/

class LEDClass
{
 protected:
	 int LedPin[4];

 public:
	 void attach(const int motorLEDPin[4]);

	 //M1 / M2 / M3 / M4��ת��˸	    δ���յ�ң���ź�	              ����ң�����������ֻ�APP
	 void rotateBlink();

	 //M1 / M2��M3 / M4������˸	IMUδУ׼	                      ���ɻ�ˮƽ���õ����ϣ���ң�����ϡ� - ����
	 void M1M2_M3M4Blink();

	//M1 / M2 / M3 / M4ͬʱ��˸	    �͵�ѹ��ʾ	                      ��س��
	 void allBlink();

	//	M1 / M2��˸	                        ������������״̬
	 void M1_M2Blink();

	 //M1 / M2����3��	                IMUУ׼��
	 void M1M2LongBright();

	 //M1��˸	                            ��������ģ����
	 void M1Blink();
};

extern LEDClass MotorLED;

#endif

