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
#define xw_1 HAL_GPIO_ReadPin(XW_0_GPIO_Port, XW_0_Pin)//机械臂
#define xw_2 HAL_GPIO_ReadPin(XW_1_GPIO_Port, XW_1_Pin)//拉箭红外
#define xw_3 HAL_GPIO_ReadPin(XW_2_GPIO_Port, XW_2_Pin)//转盘那个限位
#define xw_4 HAL_GPIO_ReadPin(XW_3_GPIO_Port, XW_3_Pin)//arduino 的信号线
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
		c620_moter.distance[3]-=0.1 * 8192 * 19 * 9 / 360.0;
		osDelay(2);
	}
	c620_moter.distance[3]=0;
	c2006_chassis[3].round_cnt = 0;
	c2006_chassis[3].total_angle = 0;
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
void set_c620moter(float angle1, float angle2, float angle3, float angle4,float angle5)
{
	setSingleMotor2_juli(M1, angle1);
	setSingleMotor2_juli(M2, angle2);
	setSingleMotor2(M3, angle3);//这是小臂
	setSingleMotor2_juli(M4, angle4);
	setSingleMotor2_juli(M5, angle5);
	set_moto_current(&hcan2, c610[0].pid_shudu.pos_out, c610[1].pid_shudu.pos_out,
					 c610[2].pid_shudu.pos_out, c610[3].pid_shudu.pos_out);
}
void read_data() //读取限位开关的数据
{
	printf("hongwai:%d  %d %d %d\r\n", xw_1, xw_2, xw_3, xw_4);
}


void qujian(int num) //取件的动作
{
	const float exp_init=260.0f;
	const float rate=8192 * 19 * 9 / 360.0;
	const float LOWEST=210;
	float tagart_angle =0;
	exp_jiaou=exp_init+45;
	/**横向的运动***/
//  hengxiang(num);//没有写	

	//测试开始初始化
	clamp(false);
	set_high(LOWEST);//设置高度
	shot();
	HAL_GPIO_WritePin(GPIOE, XW_3_Pin, GPIO_PIN_RESET);	
	Grab_action(num,false);//底部打开
	open_jian(false);//压箭舵机开
	
	
  osDelay(2000);
	
	
	//抓取箭杆
	exp_jiaou=275;
	Grab_action(num,true);//底部的把箭夹住
	osDelay(500);
	tagart_angle = 134.5f*rate;
	c620_moter.distance[3] = tagart_angle;
	
	//到达预计角度时，2006的total_angle在528400左右
	osDelay(2000);

	
	//机械臂上端舵机抓住
	clamp(true);//机械臂上端舵机抓住
  osDelay(1000);
	Grab_action(num,false);//底部打开
  osDelay(1000);
	

	//抬举翻转
	Archery_action(18);//拉弹簧
	exp_jiaou-=30;//- 考近射击
	osDelay(200);
	c620_moter.distance[3]=85*rate;
	exp_jiaou-=105;
	osDelay(200);
	
	
	
	set_high(220);//设置高度
	open_jian(false);
	osDelay(2000);
	
	//wait nano
	
	HAL_GPIO_WritePin(GPIOE, XW_3_Pin, GPIO_PIN_SET);//发送给arduino
	osDelay(260);
	clamp_half(true);
	osDelay(100);
	
	
	HAL_GPIO_WritePin(GPIOE, XW_3_Pin, GPIO_PIN_RESET);//还有一个引脚没有接
	osDelay(2000);//等待箭羽转直
	clamp(true);

	
	set_high(230);//设置高度
	osDelay(1000);
	
	//放机械臂
	osDelay(1000);
  c620_moter.distance[3]=15*rate;
	exp_jiaou+=20;
	
	osDelay(1000);
	
  c620_moter.distance[3]=0.5f*rate;
	osDelay(3000);
	


	clamp(false);//机械臂上端舵机开
//	open_jian(true);
	exp_jiaou+=5;
  osDelay(1000);
  c620_moter.distance[3]=75*rate;
  osDelay(1000);


	
	set_high(210);//设置高度
	osDelay(2000);
	shot();
	
	
	
	
	//测试复位
//	osDelay(2000);
//	clamp(false);//机械臂上端舵机开
//	set_high(LOWEST);//设置高度
//  //exp_jiaou=exp_init;
//	set_high(220);//
//	osDelay(2000);
//	shot();
//	set_high(LOWEST);//
//	
//	osDelay(3000);
	
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
	osDelay(3000);

}
//160 -190

void set_high(float jiadu) //设置射箭角度的函数
{
	if(jiadu<tuolun_angle[0])
	{
		jiadu+=3;
	}
	while (__fabs(tuolun_angle[0] - jiadu) >= 0.4)
	{
		pid_calc(&high, tuolun_angle[0], jiadu);
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
	mesure = tuolun_angle[1];
	pid_calc(&duoji_jaidou, mesure, jiadu);
	c620_moter.distance[2]=-duoji_jaidou.pos_out;
}

void set_exp(float x, float y) //设置小车期望的位置
{
	kinematics.exp_x_dis = x;
	kinematics.exp_y_dis = y;
}
