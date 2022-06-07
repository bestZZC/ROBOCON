#include "duoji.h"
#include "stdio.h"
#include "cmsis_os.h"
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim9;
void set_duoji(uint8_t num,uint8_t state)
{
	if(num==1)
	{
	}
	else if(num==2)
	{
	}
	else if(num==3)
	{
	}
		else if(num==4)
	{
	}
		else if(num==5)
	{
	}
		else if(num==6)
	{
	}
		else if(num==7)
	{
	}
		else if(num==8)
	{
	}
  HAL_Delay(1000);
}

void Grab_action(bool state)//底部的把箭抓起来函数
{
	if(state==true)
	{
		 __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, 1500);//抓住
	}
	else{
		__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, 900);
	}
	osDelay(1000);
}
void clamp(bool state)//机械臂最上端舵机
{
	if(state==true)
	{
		__HAL_TIM_SetCompare(&htim9, TIM_CHANNEL_2, 1000);
	}
	else
	{
		__HAL_TIM_SetCompare(&htim9, TIM_CHANNEL_2, 2200);//舵机
	}
	osDelay(1000);
}

void shot()
{
	HAL_GPIO_WritePin(shejian_GPIO_Port, shejian_Pin, GPIO_PIN_SET); //电磁阀开关
	osDelay(500);
	HAL_GPIO_WritePin(shejian_GPIO_Port, shejian_Pin, GPIO_PIN_RESET);
}

