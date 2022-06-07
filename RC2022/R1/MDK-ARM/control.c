#include "control.h"
#include "moter.h"
#include "bsp_can.h"
#include "math.h"
#include "stdio.h"
#include "bsp_imu.h"
#include "pid.h"
#include "main.h"
#include "cmsis_os.h"
#include "stdbool.h"
#include "duoji.h"
/***********************************此c文件的函数是除舵轮电机外所有电机的操作**********************************************************/
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim9;
float my_wz, my_yaw;
pid_t high,duoji_jaidou; //高度的pid
extern float exp_jiaou;
c620moter c620_moter = {0};
#define hw_IN HAL_GPIO_ReadPin(hw_GPIO_Port, hw_Pin)
#define xw_1 HAL_GPIO_ReadPin(XW_0_GPIO_Port, XW_0_Pin)
#define xw_2 HAL_GPIO_ReadPin(XW_1_GPIO_Port, XW_1_Pin)
#define xw_3 HAL_GPIO_ReadPin(XW_2_GPIO_Port, XW_2_Pin)
#define xw_4 HAL_GPIO_ReadPin(XW_3_GPIO_Port, XW_3_Pin)
speed_wheel c610[8] = {0};
volatile float vec_limt = 0.5f;
messsage cmd_data[3]={0};
void send_com(UART_HandleTypeDef *huart, u8 data) //data 0x38
{
	uint8_t bytes[3] = {0};
	bytes[0] = 0XAA;
	bytes[1] = data; //功能字节
	bytes[2] = bytes[0] + bytes[1];
	HAL_UART_Transmit(huart, bytes, 3, 0xffff);
}
void init_guiwei()//机械臂归为函数
{
	c620_moter.distance[3]=0;
	c2006_chassis[3].round_cnt = 0;
	c2006_chassis[3].total_angle = 0;
	while (xw_1)
	{
		c620_moter.distance[3]-=0.5 * 8192 * 19 * 9 / 360.0;
		osDelay(5);
	}
	c620_moter.distance[3]=0;
	c2006_chassis[3].round_cnt = 0;
	c2006_chassis[3].total_angle = 0;

//	set_moto_current(&hcan2, c610[0].pid_shudu.pos_out, c610[1].pid_shudu.pos_out,
//					 c610[2].pid_shudu.pos_out, 0);
}
//  v 4.50f, 0.5f, 0.52f  1.2f, 0.001f, 45.2f    0.72f, 0.0f, 0.6f
void C610_init() //其他电机的初始化
{
	for (int i = 0; i < 5; i++) //can2的四个电机id为1,2,3,4
	{
		PID_struct_init(&c610[i].pid_weizhi,POSITION_PID, 20000, 2000,
						 0.32f, 0.001f, 3.2f);
		PID_struct_init(&c610[i].pid_shudu,DELTA_PID, 10000, 2000,
						 4.50f, 0.5f, 0.52f);
	}
	PID_struct_init(&high, POSITION_PID, 20000, 2500,
					2000.50f, 0.8f, 20.1f);//电机高度换
	PID_struct_init(&duoji_jaidou, POSITION_PID, 20000, 2000,
					152.50f, 0.001f, 5.8f);//舵机的角度环
}
void setSingleMotor2_juli(uint8_t _mNum, float angle) //设置单个速度环距离走多少
{

	if (_mNum == M1)
	{ // MOTOR 1
		pid_calc(&c610[0].pid_weizhi, c2006_chassis[0].total_angle, angle);
		pid_calc(&c610[0].pid_shudu, c2006_chassis[0].speed_rpm, c610[0].pid_weizhi.pos_out * vec_limt);
	}
	else if (_mNum == M2)
	{ // MOTOR 2
		pid_calc(&c610[1].pid_weizhi, c2006_chassis[1].total_angle, angle);
		pid_calc(&c610[1].pid_shudu, c2006_chassis[1].speed_rpm, c610[1].pid_weizhi.pos_out * vec_limt);
	}
	else if (_mNum == M3)
	{ // MOTOR 3
		pid_calc(&c610[2].pid_weizhi, c2006_chassis[2].total_angle, angle);
		pid_calc(&c610[2].pid_shudu, c2006_chassis[2].speed_rpm, c610[2].pid_weizhi.pos_out * vec_limt);
	}
	else if (_mNum == M4)
	{ // MOTOR 4
		pid_calc(&c610[3].pid_weizhi, c2006_chassis[3].total_angle, angle);
		pid_calc(&c610[3].pid_shudu, c2006_chassis[3].speed_rpm, c610[3].pid_weizhi.pos_out * vec_limt);
	}
	else if(_mNum==M5)
	{
		pid_calc(&c610[4].pid_weizhi, c2006_chassis[4].total_angle, angle);
		pid_calc(&c610[4].pid_shudu, c2006_chassis[4].speed_rpm, c610[4].pid_weizhi.pos_out );
		set_moto2_current(&hcan2, c610[4].pid_shudu.pos_out,0,0, 0);
	}
//	set_moto_current(&hcan2, c610[0].pid_shudu.pos_out, c610[1].pid_shudu.pos_out,
//					 c610[2].pid_shudu.pos_out, c610[3].pid_shudu.pos_out);
}
void setSingleMotor2(uint8_t _mNum, float _mspeed)
{
	limit_speedd(&_mspeed, 9500);
	if (_mNum == M1)
	{ // MOTOR 1
		pid_calc(&c610[0].pid_shudu, c2006_chassis[0].speed_rpm, _mspeed);
	}
	else if (_mNum == M2)
	{ // MOTOR 2
		pid_calc(&c610[1].pid_shudu, c2006_chassis[1].speed_rpm, _mspeed);
	}
	else if (_mNum == M3)
	{ // MOTOR 3
		pid_calc(&c610[2].pid_shudu, c2006_chassis[2].speed_rpm, _mspeed);
	}
	else if (_mNum == M4)
	{ // MOTOR 4
		pid_calc(&c610[3].pid_shudu, c2006_chassis[3].speed_rpm, _mspeed);
	}
	set_moto_current(&hcan2, c610[0].pid_shudu.pos_out, c610[1].pid_shudu.pos_out,
					 c610[2].pid_shudu.pos_out, c610[3].pid_shudu.pos_out);
}
void set_c620moter(float angle1, float angle2, float angle3, float angle4)
{
	setSingleMotor2_juli(M1, angle1);
	setSingleMotor2_juli(M2, angle2);
	setSingleMotor2_juli(M3, angle3);
	setSingleMotor2_juli(M4, angle4);
	set_moto_current(&hcan2, c610[0].pid_shudu.pos_out, c610[1].pid_shudu.pos_out,
					 c610[2].pid_shudu.pos_out, c610[3].pid_shudu.pos_out);
}
void read_data() //读取限位开关的数据
{
	printf("hongwai:%d  %d %d %d\r\n", xw_1, xw_2, xw_3, xw_4);
}

void set_exp(float x, float y) //设置小车期望的位置
{
	kinematics.exp_x_dis = x;
	kinematics.exp_y_dis = y;
}
