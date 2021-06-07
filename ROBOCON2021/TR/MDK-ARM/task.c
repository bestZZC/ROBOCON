#include "task.h"
#include "moter.h"
#include <math.h>
#include "bsp_imu.h"
#include "task.h"
#include "bsp_can.h"
int flag=0;
extern float yaw;//串口imu的yaw
float Position_X=0,Position_Y=0,Position_Z=0;//期望的位置
void Calculate(float x, float y, float wz)//麦轮的运动学模型
{
  pid_calc(&PID_Position_X, jiguag_data1[0],  Position_X);//
	pid_calc(&PID_Position_Y, jiguag_data1[1],  Position_Y);
	pid_sp_calc(&yaw_pos, yaw, Position_Z,imu.wz);
	kinematics.exp_vel[0].speed=-PID_Position_X.pos_out - PID_Position_Y.pos_out + yaw_pos.pos_out;
	kinematics.exp_vel[1].speed=-PID_Position_X.pos_out + PID_Position_Y.pos_out - yaw_pos.pos_out;
	kinematics.exp_vel[2].speed=-PID_Position_X.pos_out - PID_Position_Y.pos_out - yaw_pos.pos_out;
	kinematics.exp_vel[3].speed=-PID_Position_X.pos_out + PID_Position_Y.pos_out + yaw_pos.pos_out;
    
}
void cre_task(uint16_t dt)
{
	
	
	
	
}

