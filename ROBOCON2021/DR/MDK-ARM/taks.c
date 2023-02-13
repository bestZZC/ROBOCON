#include "taks.h"
#include "cmsis_os.h"
uint8_t flag = 0;
int state_cnt = 0;//用来统计时间的长度
void Ctrl_Task(uint32_t dT_ms)
{
//	static int  time = 0;
	if(flag==0)
	{
		state_cnt += dT_ms;
		if((state_cnt < 500))
           Set_Vel(2, 0, 0);
		 if(state_cnt >= 500)
        {
					state_cnt = 0;
					flag++;
					Set_Vel(0, 0, 0);
					osDelay(5000);
        }
	}
	else if(flag==1)
	{
		state_cnt += dT_ms;
		if((state_cnt < 500))
       Set_Vel(-2, 0, 0);
		 if(state_cnt >= 500)
        {
					state_cnt = 0;
					flag++;
					Set_Vel(0, 0, 0);
					osDelay(5000);
        }
	}
	else if(flag==2)
	{
		
		 Set_Vel(0, 0, 0);
	}
	
	
	
}

