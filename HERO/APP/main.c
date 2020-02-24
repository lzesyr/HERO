#include "system.h"

uint32_t Remote_time = 0;
extern int SystemMonitor;
int main(void)
{
	System_Init();
	Remote_time = micros();
	while(1)
	{
		if((micros() >= Remote_time))//如果遥控器一段时间内没有收到信息，强行将遥控器复位为没有遥控的状态
		{
			RC_Ctl.rc.ch0 = 1024;
			RC_Ctl.rc.ch1 = 1024;
			RC_Ctl.rc.ch2 = 1024;
			RC_Ctl.rc.ch3 = 1024;
			RC_Ctl.rc.s1 = 2;
			RC_Ctl.rc.s2 = 2;
			Stop();
			SystemMonitor = Error_Mode;			
			delay_us(10);
		}
		else
			SystemMonitor = Normal_Mode;
		Loop();
	}
}
