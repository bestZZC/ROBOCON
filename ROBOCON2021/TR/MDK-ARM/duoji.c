#include "duoji.h"
#include "stdio.h"
#include "cmsis_os.h"
#include "stm32_pca9685.h"
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim9;
void Grab_action(int num ,bool state)//�ײ��İѼ�ץ��������
{
	float zero_jiaodu=45.0f;
	switch (num)
  {
  	case 1:
			if(state==true)//true ��close  //1�Ż����ƶ�
			{
				 PCA_MG9XX(1,85);
			}
			else
				PCA_MG9XX(1,85-zero_jiaodu);
  		break;
  	case 2:
			if(state==true)//true ��close
			{
				 PCA_MG9XX(3,83);
			}
			else
				PCA_MG9XX(3,83-zero_jiaodu);
  		break;
		case 3: 
			if(state==true)//true ��close
			{
				 PCA_MG9XX(4,85);
			}
			else
				PCA_MG9XX(4,85-zero_jiaodu);
  		break;
		case 4: 
			if(state==true)//true ��close
			{
				 PCA_MG9XX(5,80);
			}
			else
				PCA_MG9XX(5,80-zero_jiaodu);
  		break;
		case 5:
			if(state==true)//true ��close �����ƶ�
			{
				 PCA_MG9XX(6,80);
			}
			else
				PCA_MG9XX(6,80-zero_jiaodu);
  		break;
  	default:
  		break;
}
		
}
void clamp(bool state)//��е�����϶˶��
{
	if(state==true)
	{
		__HAL_TIM_SetCompare(&htim9, TIM_CHANNEL_2, 1000);
	}
	else
	{
		__HAL_TIM_SetCompare(&htim9, TIM_CHANNEL_2, 2200);//���
	}

}

void shot()
{
	HAL_GPIO_WritePin(shejian_GPIO_Port, shejian_Pin, GPIO_PIN_SET); //��ŷ�����
	osDelay(500);
	HAL_GPIO_WritePin(shejian_GPIO_Port, shejian_Pin, GPIO_PIN_RESET);
}
void open_jian(bool state)
{
	if(state==true)
	{
		__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, 1500);
		__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 1500);
	}
	else
	{
		__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, 500);
		__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 2500);
	}
	
}
void clamp_half(bool state)
{
	if(state==true)
	{
		__HAL_TIM_SetCompare(&htim9, TIM_CHANNEL_2, 1850);
	}
	else
	{
		__HAL_TIM_SetCompare(&htim9, TIM_CHANNEL_2, 2000);
	}
	
}
//void hengxiang(int num)
//{
//	
//	
//}

