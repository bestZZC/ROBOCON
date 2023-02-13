/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "pid.h"
#include "bsp_can.h"
#include "mytype.h"
#include "tim.h"
#include "stdio.h"
#include "moter.h"
#include "main.h"
#include "taks.h"
#include "pca.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#include "bsp_imu.h"
#include "bsp_uart.h"
#include "control.h"
#include "iic.h"
#include "pca.h"
char buf[300];
extern UART_HandleTypeDef huart6;
extern ADC_HandleTypeDef hadc1;
extern rc_info_t rc;
uint32_t ADC_Value[120];
uint32_t ad1, ad2, ad3, ad4, ad5, ad6;
float tuolun_angle[6] = {0}, pre_angle[6] = {0};
#define fab(x) x > 0 ? x : -x
u8 dir = 1;
float orignal_yaw = 0.0f;
#define COUNTOF(__BUFFER__) (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))
extern float yaw;
extern volatile uint8_t rx_len, rx_len2, rx_len3;                      //接收一帧数据的长度
extern volatile uint8_t recv_end_flag, recv_end_flag2, recv_end_flag3,recv_cmd_flag2; //一帧数据接收完成标志
extern uint8_t rx_buffer[100], rx_buffer2[100], rx_buffer3[100];       //接收数据缓存数组
int big_flag=0;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId myTask02Handle;
osThreadId myTask03Handle;
osThreadId myTask04Handle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void set_total_angle(moto_measure_t *p, moto_measure_t *p1, int angle, u8 dir, u16 speed); //
void weizhihuan(moto_measure_t *p, float angle);
void weizhihuan_duolun(moto_measure_t *p, float mesure, float angle);
extern TIM_HandleTypeDef htim7;
float oringal_yaw = 0.0f;
float exp_jiaou=260.2f;
extern rc_info_t rc;
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartTask02(void const * argument);
void StartTask03(void const * argument);
void StartTask04(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize)
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of myTask02 */
  osThreadDef(myTask02, StartTask02, osPriorityNormal, 0, 128);
  myTask02Handle = osThreadCreate(osThread(myTask02), NULL);

  /* definition and creation of myTask03 */
  osThreadDef(myTask03, StartTask03, osPriorityNormal, 0, 128);
  myTask03Handle = osThreadCreate(osThread(myTask03), NULL);

  /* definition and creation of myTask04 */
  osThreadDef(myTask04, StartTask04, osPriorityNormal, 0, 128);
  myTask04Handle = osThreadCreate(osThread(myTask04), NULL);

  /* USER CODE BEGIN RTOS_THREADS */

  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  my_can_filter_init_recv_all(&hcan1);
  can_filter_recv_special(&hcan2);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING); // Enable interrupts
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING); // Enable interrupts
  duo_lun_speed_init();
  duo_lun_Dir_init();
  Kinematics_Init();
  C610_init();
  set_exp(60, 60);
  __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);
  HAL_UART_Receive_DMA(&huart6, rx_buffer, 100);
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
  HAL_UART_Receive_DMA(&huart2, rx_buffer2, 100);
  PID_struct_init(&yaw_pos, DELTA_PID, 3500, 500, 1272.6f, 12.5f, 0.3f);           //转向环
  PID_struct_init(&PID_Position_X, POSITION_PID, 5000, 1000, 52.6f, 0.01f, 28.0f); //x方向
  PID_struct_init(&PID_Position_Y, POSITION_PID, 5000, 1000, 52.6f, 0.01f, 28.0f); //y方向
  send_com(&huart2, 0x38);//imu_init
  osDelay(1000);
  orignal_yaw = yaw;
	__HAL_TIM_SetCompare(&htim9, TIM_CHANNEL_2, 2200);
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, 2200);
  for (int i = 0; i < 8; i++)
    moto_chassis[i].total_angle = 0; //清零

	 HAL_TIM_Base_Start_IT(&htim7);
//	 __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 20000);
//	 set_high(215);
//	 osDelay(1500);
//  init_guiwei(); //初始化机械臂 归为
//    for(int i=0;i<4;i++)
//		{
//			kinematics.exp_vel[i].jiado=45;
//		}
		osDelay(1000);
//  HAL_TIM_Base_Start_IT(&htim7);
  for (;;)
  {
//		pca_setpwm(0,0,130);
		printf("yaw:%f\r\n",yaw);
//			__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, 1200);
//		qujian();
//	if(big_flag)//串口屏的操作
//	{
////		printf("fhud");
//		set_high(cmd_data[0].shuju/10.0f);
//		c620_moter.distance[0] = cmd_data[1].shuju*1.2*819.2f;//pian yi jiaodu
//		Archery_action(cmd_data[2].shuju/10.0f);
//		big_flag=0;
//	}
    //        Calculate(rc.ch4 / 250.0f, rc.ch3 / 400.0f, rc.ch2 / 600.0f, -999);
    //				 printf("AD1_value=%1.3f AD2_value=%1.3f AD3_value=%1.3f AD5_value=%1.3f\r\n ", tuolun_angle[0], tuolun_angle[1],tuolun_angle[2],tuolun_angle[4]);
    osDelay(20);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the myTask02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void const * argument)
{
  /* USER CODE BEGIN StartTask02 */
  u8 i;

  float yingzi = 0.1;
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&ADC_Value, 120);
  /* Infinite loop */
  for (;;)
  {
    for (i = 0, ad1 = ad2 = ad3 = ad4 = ad5 = ad6 = 0; i < 120;)
    {
      ad1 += ADC_Value[i++];
      ad2 += ADC_Value[i++];
      ad3 += ADC_Value[i++];
      ad4 += ADC_Value[i++];
      ad5 += ADC_Value[i++];
      ad6 += ADC_Value[i++];
    }
    ad1 /= 20;
    ad2 /= 20;
    ad3 /= 20;
    ad4 /= 20;
    ad5 /= 20;
    ad6 /= 20;
    pre_angle[0] = 360 * (ad1 / 4096.0f) ; //(ad1 * 3.3f * 110.0917f) / (4096 * (3.26 / 3.3));//(ad1 * 3.3f * 110.0917f) / (4096 * (3.26 / 3.3))
    pre_angle[1] = 360 * (ad2 / 4096.0f) ;
    pre_angle[2] = 360 * (ad3 / 4096.0f) ;
    pre_angle[3] = 360 * (ad4 / 4096.0f) ;
    pre_angle[4] = 360 * (ad5 / 4096.0f);
    pre_angle[5] = 360 * (ad6 / 4096.0f);
    tuolun_angle[0] = yingzi * tuolun_angle[0] + (1 - yingzi) * pre_angle[0]; //低通滤波
    tuolun_angle[1] = yingzi * tuolun_angle[1] + (1 - yingzi) * pre_angle[1];
    tuolun_angle[2] = yingzi * tuolun_angle[2] + (1 - yingzi) * pre_angle[2];
    tuolun_angle[3] = yingzi * tuolun_angle[3] + (1 - yingzi) * pre_angle[3];
    tuolun_angle[4] = yingzi * tuolun_angle[4] + (1 - yingzi) * pre_angle[4];
    tuolun_angle[5] = yingzi * tuolun_angle[5] + (1 - yingzi) * pre_angle[5];
//      1_value=%1.3f AD2_value=%1.3f AD3_value=%1.3f AD5_value=%1.3f\r\n ", tuolun_angle[0], tuolun_angle[1],tuolun_angle[2],tuolun_angle[3]);
    osDelay(5);
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/*
* @brief Function implementing the myTask03 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void const * argument)
{
  /* USER CODE BEGIN StartTask03 */
  /* Infinite loop */
  for (;;)
  {
	 mpu_get_data();
	 imu_ahrs_update();
	 imu_attitude_update();
//	printf("%f  %f  \r\n",imu.wz,imu.yaw);
//   set_duojjiaodu(exp_jiaou);
    if (recv_end_flag == 1) //接收完成标志
    {
      HAL_UART_Transmit_DMA(&huart6, rx_buffer, rx_len);
      rx_len = 0;        //清除计数
      recv_end_flag = 0; //清除接收结束标志位
      memset(rx_buffer, 0, rx_len);
      HAL_UART_Receive_DMA(&huart6, rx_buffer, 100); //重新打开DMA接收z
    }
    if (recv_end_flag2 == 1) //接收完成标志
    {
//       HAL_UART_Transmit_DMA(&huart2, rx_buffer2, rx_len2);
       rx_len2 = 0;        //清除计数
      recv_end_flag2 = 0; //清除接收结束标志位
      HAL_UART_Receive_DMA(&huart2, rx_buffer2, 100); //重新打开DMA接收z
    }
//		if (recv_cmd_flag2 == 1) //接收完成标志
//    {
//      // HAL_UART_Transmit_DMA(&huart2, rx_buffer2, rx_len2);
//      // rx_len2 = 0;        //清除计数
//      recv_cmd_flag2 = 0; //清除接收结束标志位
//      // memset(rx_buffer2, 0, rx_len2);
//			for(int i=0;i<3;i++)
//			{
//				printf("%f  ",cmd_data[i].shuju/10.0f);
//			}
//			printf("\r\n");
//			big_flag=1;
////			Archery_action(cmd_data[2].shuju/10.0f);
//      HAL_UART_Receive_DMA(&huart2, rx_buffer2, 100); //重新打开DMA接收z
//    }
    //        printf("AD1_value=%1.3f AD2_value=%1.3f AD3_value=%1.3f AD5_value=%1.3f\r\n ", tuolun_angle[0], tuolun_angle[1],tuolun_angle[2],tuolun_angle[4]);
    osDelay(5);
  }
  /* USER CODE END StartTask03 */
}

/* USER CODE BEGIN Header_StartTask04 */
/**
* @brief Function implementing the myTask04 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask04 */
void StartTask04(void const * argument)
{
  /* USER CODE BEGIN StartTask04 */
  /* Infinite loop */
	 osDelay(5000);
  for (;;)
  {
		Ctrl_Task(10);
//	if(rc.sw1 == 1)
//	{
//		Ctrl_Task(50);
////    if (rc.sw1 == 1)
////		{
////			 osDelay(5);-
////		}
//	   
////      set_point(data1, data2);
////		  read_data();
////   	printf("  ceiju=%1.3f\r\n ",jiguag_data1[3]);
////   		printf("%f  %f   %f\r\n",data1,data2,kinematics.exp_vel[0].speed);
////    		sprintf(buf, "CH1: %4d  CH2: %4d  CH3: %4d  CH4: %4d  SW1: %1d  SW2: %1d \r\n", rc.ch1, rc.ch2, rc.ch3, rc.ch4, rc.sw1, rc.sw2);
////        HAL_UART_Transmit(&huart6, (uint8_t *)buf, (COUNTOF(buf) - 1), 0xffff);
//   
//	}
	 osDelay(10);
  }
  /* USER CODE END StartTask04 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
