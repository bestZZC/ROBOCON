#include "taks.h"
#include "cmsis_os.h"
#include "math.h"
#include "bsp_uart.h"
extern rc_info_t rc;
uint8_t flag = 0;
int state_cnt = 0;//用来统计时间的长度
float L1 = 0.848528f;
#define PI 3.14159265
float val1 = 180.0 / PI;
extern float orignal_yaw, yaw;//imu初始的角度和实时的角度
float vec_w1[4][3] = {0}; //
float _x1[4], _y1[4], zero_thea1[4] = {-45, 45, 135, -135};
float tagart2[4][2] = {0};
void Calculate2(float x, float y, float wz,float tartyaw)//舵轮的运动学模型
{
    double angle = yaw - orignal_yaw;
    float vec[2] = {x, y}; //
		if(__fabs(wz)<=0.01)
	      wz=0;
//		if(__fabs(angle-tartyaw)<=0.2)
//		{
//			wz=0;
//			orignal_yaw=yaw;
//		}
    for (int i = 0; i < 4; i++)
    {
        _x1[i] = L1 * cos((angle + zero_thea1[i]) / val1);
        _y1[i] = L1 * sin((angle + zero_thea1[i]) / val1);
    }
    for (int i = 0; i < 4; i++) //
    {
        vec_w1[i][0] = wz * _y1[i] + x;
        vec_w1[i][1] = -wz * _x1[i] + y;
    }
    for (int i = 0; i < 4; i++)
    {
        tagart2[i][0] = atan2(vec_w1[i][1], vec_w1[i][0]) * val1 - angle;
        tagart2[i][1] = sqrt(pow(vec_w1[i][0], 2) + pow(vec_w1[i][1], 2));
        if (rc.sw1 == 2)
        {
					 kinematics.exp_vel[i].jiado = tagart2[i][0];
            kinematics.exp_vel[i].speed = tagart2[i][1] * 60 * 19 / 0.32f;//m/s

        }
        else if (rc.sw1 == 1)
        {
					 kinematics.exp_vel[i].jiado = tagart2[i][0];
					kinematics.exp_vel[i].speed = tagart2[i][1]* 60 * 19 / 0.32f ;//m/s

        }
    }
		
	

}
void Ctrl_Task(uint32_t dT_ms)
{
//	static int  time = 0;
	state_cnt += dT_ms;
	if(flag==0)
	{
		if(state_cnt < 5500)
           Calculate2(0.5,-0.3,0,0);
		 if(state_cnt >= 5500)
        {
					state_cnt = 0;
					flag++;
					Calculate2(0,0,0,0);
					osDelay(1000);
        }
	}
	else if(flag==1)
	{
		if(state_cnt < 3000)
        Calculate2(0,0,-0.5,-180);
		if(state_cnt >= 3000)
        {
					state_cnt = 0;
					flag++;
					orignal_yaw+=180;
					Calculate2(0,0,0,0);
					osDelay(3000);
        }
			
	}
	else if(flag==2)
	{
		if(state_cnt < 2000)
           Calculate2(0,1,0,0);
		 if(state_cnt >= 2000)
        {
					state_cnt = 0;
					flag++;
					Calculate2(0,0,0,0);
					osDelay(1000);
        }
	}
	
	
	
}

