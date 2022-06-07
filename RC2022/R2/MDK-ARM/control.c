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
	for (int i = 0; i < 8; i++) //can2的四个电机id为1,2,3,4
	{
		PID_struct_init(&c610[i].pid_weizhi,POSITION_PID, 20000, 2000,
						 0.10f, 0.001f, 3.2f);
		PID_struct_init(&c610[i].pid_shudu,POSITION_PID, 10000, 2000,
						 0.40f, 0.5f, 0.52f);
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
	{// MOTOR 5
		pid_calc(&c610[4].pid_weizhi, c2006_chassis[4].total_angle, angle);
		pid_calc(&c610[4].pid_shudu, c2006_chassis[4].speed_rpm, c610[4].pid_weizhi.pos_out * vec_limt);
	}
	else if(_mNum==M6)
	{// MOTOR 6
		pid_calc(&c610[5].pid_weizhi, c2006_chassis[5].total_angle, angle);
		pid_calc(&c610[5].pid_shudu, c2006_chassis[5].speed_rpm, c610[5].pid_weizhi.pos_out * vec_limt );
	}
	else if(_mNum==M7)
	{// MOTOR 7
		pid_calc(&c610[6].pid_weizhi, c2006_chassis[6].total_angle, angle);
		pid_calc(&c610[6].pid_shudu, c2006_chassis[6].speed_rpm, c610[6].pid_weizhi.pos_out * vec_limt);
	}
	else if(_mNum==M8)
	{// MOTOR 8
		pid_calc(&c610[7].pid_weizhi, c2006_chassis[7].total_angle, angle);
		pid_calc(&c610[7].pid_shudu, c2006_chassis[7].speed_rpm, c610[7].pid_weizhi.pos_out * vec_limt);
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
	else if (_mNum == M5)
	{ // MOTOR 5
		pid_calc(&c610[4].pid_shudu, c2006_chassis[4].speed_rpm, _mspeed);
	}
	else if (_mNum == M6)
	{ // MOTOR 6
		pid_calc(&c610[5].pid_shudu, c2006_chassis[5].speed_rpm, _mspeed);
	}
	else if (_mNum == M7)
	{ // MOTOR 7
		pid_calc(&c610[6].pid_shudu, c2006_chassis[6].speed_rpm, _mspeed);
	}
	else if (_mNum == M8)
	{ // MOTOR 8
		pid_calc(&c610[7].pid_shudu, c2006_chassis[7].speed_rpm, _mspeed);
	}
}
void set_c620moter(float speed1,float speed2,float distance3,float distance4,float speed5)
{
			setSingleMotor2(M1, speed1);   //上层炮台摩擦轮转速
			setSingleMotor2(M2, speed2);  //上层炮台摩擦轮转速
			setSingleMotor2(M5, -speed5);  //上层后面用于送球的电机
			setSingleMotor2_juli(M3, distance3); //设置上下的旋转
			setSingleMotor2_juli(M4,distance4); //设置左右的旋转
			set_moto_current(&hcan2, c610[0].pid_shudu.pos_out, c610[1].pid_shudu.pos_out,
					 c610[2].pid_shudu.pos_out, c610[3].pid_shudu.pos_out);
			set_moto2_current(&hcan2, c610[4].pid_shudu.pos_out,0,0, 0);
}

void set_exp(float x, float y) //设置小车期望的位置
{
	kinematics.exp_x_dis = x;
	kinematics.exp_y_dis = y;
}
void move_qifa(void){
	 
	 HAL_GPIO_WritePin(GPIOI, QIFA1_Pin, GPIO_PIN_RESET);
	 HAL_GPIO_WritePin(GPIOI, QIFA2_Pin, GPIO_PIN_RESET);
	 osDelay(1000);
	 HAL_GPIO_WritePin(GPIOI, QIFA3_Pin, GPIO_PIN_SET);
	 HAL_GPIO_WritePin(GPIOI, QIFA1_Pin, GPIO_PIN_SET);
	 osDelay(5000);
	 HAL_GPIO_WritePin(GPIOI, QIFA1_Pin, GPIO_PIN_RESET);
	 HAL_GPIO_WritePin(GPIOI, QIFA3_Pin, GPIO_PIN_RESET);
	 osDelay(1000);
	
//	 HAL_GPIO_WritePin(GPIOI, QIFA2_Pin, GPIO_PIN_SET);
//	 HAL_GPIO_WritePin(GPIOI, QIFA1_Pin, GPIO_PIN_SET);
//	 osDelay(500);
//	 HAL_GPIO_WritePin(GPIOI, QIFA1_Pin, GPIO_PIN_RESET);
//	 osDelay(3000);
//	 
//	 HAL_GPIO_WritePin(GPIOI, QIFA1_Pin, GPIO_PIN_RESET);
//	 HAL_GPIO_WritePin(GPIOI, QIFA2_Pin, GPIO_PIN_RESET);
}
void move_qifa1(void){
	 
//	 HAL_GPIO_WritePin(GPIOI, QIFA1_Pin, GPIO_PIN_RESET);
//	 HAL_GPIO_WritePin(GPIOI, QIFA2_Pin, GPIO_PIN_RESET);
//	 osDelay(1000);
//	 HAL_GPIO_WritePin(GPIOI, QIFA3_Pin, GPIO_PIN_SET);
//	 HAL_GPIO_WritePin(GPIOI, QIFA1_Pin, GPIO_PIN_SET);
//	 osDelay(3000);
//	 HAL_GPIO_WritePin(GPIOI, QIFA1_Pin, GPIO_PIN_RESET);
//	 HAL_GPIO_WritePin(GPIOI, QIFA3_Pin, GPIO_PIN_RESET);
	 osDelay(1000);
	
	 HAL_GPIO_WritePin(GPIOI, QIFA2_Pin, GPIO_PIN_SET);
	 HAL_GPIO_WritePin(GPIOI, QIFA1_Pin, GPIO_PIN_SET);
	 osDelay(500);
	 HAL_GPIO_WritePin(GPIOI, QIFA1_Pin, GPIO_PIN_RESET);
	 osDelay(3000);
	 
	 HAL_GPIO_WritePin(GPIOI, QIFA1_Pin, GPIO_PIN_RESET);
	 HAL_GPIO_WritePin(GPIOI, QIFA2_Pin, GPIO_PIN_RESET);
}
void init_position(void)  //自动复位炮台位置
{
	int i;
	pid_t position[2];  //0是水平，1是俯仰
	PID_struct_init(&position[0],POSITION_PID, 20000, 2000,  //炮台水平角
						  50.0f, 0.5f,40);
	
	PID_struct_init(&position[1],POSITION_PID, 20000, 2000,  //炮台俯仰角
						  50.0f, 0.01f, 3.2f);
  for(i = 0 ;i < 600;i++)
	{
		pid_calc(&position[0],tuolun_angle[5],63);
//		pid_calc(&position[1],tuolun_angle[3],143);
//		set_moto_current(&hcan2, 0,0,position[1].pos_out, position[0].pos_out);
		set_moto_current(&hcan2, 0,0,0, position[0].pos_out);
		osDelay(20);
	}
}
void clear_motor(void)
{
  for (int i = 0; i < 8; i++)
	{
    moto_chassis[i].total_angle = 0; //清零
		c2006_chassis[i].total_angle=0;
		moto_chassis[i].round_cnt = 0;
		c2006_chassis[i].round_cnt = 0;
	}
} 
  

