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
volatile float vec_limt = 0.2f;
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
		pid_calc(&c610[1].pid_shudu, c2006_chassis[1].speed_rpm, c610[1].pid_weizhi.pos_out * 0.45f);
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
	set_moto_current(&hcan2, c610[0].pid_shudu.pos_out, c610[1].pid_shudu.pos_out,
					 c610[2].pid_shudu.pos_out, c610[3].pid_shudu.pos_out);
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
	setSingleMotor2(M3, angle3);
	setSingleMotor2_juli(M4, angle4);
	set_moto_current(&hcan2, c610[0].pid_shudu.pos_out, c610[1].pid_shudu.pos_out,
					 c610[2].pid_shudu.pos_out, c610[3].pid_shudu.pos_out);
}
void read_data() //读取限位开关的数据
{
	printf("hongwai:%d  %d %d %d\r\n", xw_1, xw_2, xw_3, xw_4);
}
void qujian() //取件的动作
{
	
	osDelay(1500);
//	Grab_action(true);//底部的把箭抓起
	exp_jiaou=282;
	osDelay(500);
//	float tagart_angle = 125 * 8192 * 19 * 9 / 360.0;
	float tagart_angle = 130 * 8192 * 19 * 9 / 360.0;
	c620_moter.distance[3] = tagart_angle;
	
	
	osDelay(3000);
	
//	
//	Archery_action(10.2);//拉弹簧
//	set_high(229);//设置高度
//	clamp(true);//机械臂上端舵机抓住
//	Grab_action(false);//底部松开
//	c620_moter.distance[3]=65* 8192 * 19 * 9 / 360.0;
//	osDelay(2500);
//	
//	exp_jiaou=135;
//	osDelay(1500);
//	
//	
//	HAL_GPIO_WritePin(GPIOE, XW_3_Pin, GPIO_PIN_RESET);//发送给arduino
//	osDelay(300);
//	HAL_GPIO_WritePin(GPIOE, XW_3_Pin, GPIO_PIN_SET);
//	osDelay(2000);//等待箭羽转直
//	
//	
//	exp_jiaou=145;
//	osDelay(800);
//	
//	c620_moter.distance[3]=5* 8192 * 19 * 9 / 360.0;
//	osDelay(1200);
//	
//	clamp(false);//放箭
//	c620_moter.distance[3]=75* 8192 * 19 * 9 / 360.0;
//	osDelay(1500);
//	
//	
//	shot();
//	osDelay(1000);
//	set_high(215);
//	
//	
//	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, 500);
//	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 500);


}
void Archery_action(float tagart_angle) //射箭的动作
{
	tagart_angle=  -8192 *19 * tagart_angle;
	//set_high(175.0f);
	while (xw_2)
	{
		c620_moter.distance[1] -=8192*19;
		osDelay(5);
	}
	c2006_chassis[1].round_cnt = 0;
	c2006_chassis[1].total_angle = 0;
	c620_moter.distance[1] = tagart_angle;
	osDelay(4000);

}
//160 -190

void set_high(float jiadu) //设置射箭角度的函数
{
	if(jiadu<tuolun_angle[4])
	{
		jiadu+=3;
	}
	while (__fabs(tuolun_angle[4] - jiadu) >= 0.4)
	{
		pid_calc(&high, tuolun_angle[4], jiadu);
		if (high.pos_out > 0)
		{
			HAL_GPIO_WritePin(DIR_1_GPIO_Port, DIR_1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(DIR_2_GPIO_Port, DIR_2_Pin, GPIO_PIN_SET);
			__HAL_TIM_SetCompare(&htim9, TIM_CHANNEL_1, (int)high.pos_out); //1300-2200
		}
		else
		{
			HAL_GPIO_WritePin(DIR_1_GPIO_Port, DIR_1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(DIR_2_GPIO_Port, DIR_2_Pin, GPIO_PIN_RESET);
			__HAL_TIM_SetCompare(&htim9, TIM_CHANNEL_1, -(int)high.pos_out); //1300-2200
		}
	}
	__HAL_TIM_SetCompare(&htim9, TIM_CHANNEL_1, 0); //1300-2200
}
void set_duojjiaodu(float jiadu) //设置射箭角度的函数
{
	  float  mesure=0.0f;
//	  jiadu = jiadu + 90;
//    jiadu = fmod(jiadu, 360);
//    //	  angle=(int)angle%360;
//    if (jiadu < 0)
//        jiadu += 360;
    mesure = tuolun_angle[5];
//    if (mesure - jiadu > 180) //最多转180度
//    {
//        mesure = mesure - 360;
//    }
//    else if (jiadu - mesure > 180)
//        jiadu = jiadu - 360;

	pid_calc(&duoji_jaidou, mesure, jiadu);
	c620_moter.distance[2]=-duoji_jaidou.pos_out;
//	printf("%f\r\n",c620_moter.distance[2]);
}
void set_point(float x_get, float y_get)
{
	float x_err = kinematics.exp_x_dis - x_get; // kinematics.exp_x_dis-x_get;
	float y_err = kinematics.exp_y_dis - y_get;
	float theat = -atan2(y_err, x_err);
	if (y_err == 0)
	{
		theat = 0;
	}
	if (x_err == 0)
	{
		theat = -90 / (180 / 3.1415926f);
	}
	for (int i = 0; i < 4; i++)
	{
		kinematics.exp_vel[i].jiado = theat * 180 / 3.1415926f;
	}
	pid_calc(&PID_Position_X, x_get, kinematics.exp_x_dis);
	pid_calc(&PID_Position_Y, y_get, kinematics.exp_y_dis);
	if (__fabs(x_err) <= 10 && __fabs(y_err) <= 10)
	{
		PID_Position_X.pos_out=0;
		PID_Position_Y.pos_out=0;
	}
	Calculate(-PID_Position_X.pos_out, PID_Position_Y.pos_out, 0);
	//	pid_calc(&yaw_pos, imu.wz, 0);
	//	float sudu=-PID_Position_X.pos_out-PID_Position_Y.pos_out ;//= sqrt( pow(PID_Position_Y.pos_out, 2)+pow(PID_Position_X.pos_out, 2));
	//	if(x_err<0&&y_err<0)
	//	{
	//
	//
	//	}
	//	if((x_err>=0&&y_err>=0)||(x_err>=0&&y_err<=0))
	//	{
	//		kinematics.exp_vel[0].speed = -sudu;
	//		kinematics.exp_vel[3].speed = -sudu;
	//		kinematics.exp_vel[1].speed = -sudu;
	//		kinematics.exp_vel[2].speed = -sudu;
	//	}
	//	else
	//	{
	//		kinematics.exp_vel[0].speed = sudu;
	//		kinematics.exp_vel[3].speed = sudu;
	//		kinematics.exp_vel[1].speed = sudu;
	//		kinematics.exp_vel[2].speed = sudu;
	//	}
	//  pid_calc(&PID_Position_X, x_get, kinematics.exp_x_dis);
	////	pid_calc(&PID_Position_Y, y_get, kinematics.exp_y_dis);
	//	Calculate(-PID_Position_X.pos_out,PID_Position_Y.pos_out,0);
}
void set_exp(float x, float y) //设置小车期望的位置
{
	kinematics.exp_x_dis = x;
	kinematics.exp_y_dis = y;
}
