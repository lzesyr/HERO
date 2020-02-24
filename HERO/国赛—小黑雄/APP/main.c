#include "system.h"

uint32_t Remote_time = 0;
extern int SystemMonitor;
int main(void)
{
	System_Init();
	Remote_time = micros();
	while(1)
	{
		if((micros() >= Remote_time))//���ң����һ��ʱ����û���յ���Ϣ��ǿ�н�ң������λΪû��ң�ص�״̬
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
