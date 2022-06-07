/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f4xx_hal.h"
#include "bsp_imu.h"
#include "stdio.h"
#include "bsp_uart.h"
#include "moter.h"
#include "control.h"
#include "bsp_can.h"
extern speed_wheel baohu[4];
#define BUFFER_SIZE 100
volatile uint8_t rx_len = 0, rx_len2 = 0, rx_len3 = 0;                      //接收一帧数据的长度
volatile uint8_t recv_end_flag = 0, recv_end_flag2 = 0, recv_end_flag3 = 0,recv_cmd_flag2=0; //一帧数据接收完成标志
uint8_t rx_buffer[100] = {0}, rx_buffer2[100] = {0}, rx_buffer3[100] = {0}; //接收数据缓存数组
extern DMA_HandleTypeDef hdma_usart6_rx;
extern rc_info_t rc;
extern float orignal_yaw, yaw;

#define circle   0.329867f
#define Reduction  103765.3f   //减速比
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
  /* NOTE: This function should not be modified, when the callback is needed,
             the HAL_UART_RxCpltCallback could be implemented in the user file
     */
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_SPI5_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
  MX_ADC1_Init();
  MX_TIM7_Init();
  MX_USART3_UART_Init();
  MX_TIM9_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
  dbus_uart_init(); //dbus
  mpu_device_init();
	init_quaternion();

  ////    HAL_Delay(2000);
  /* USER CODE END 2 */

  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();
  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
  if (htim->Instance == TIM7)
  {
//		set_c620moter(150,150,150,150);
//    set_c620moter(c620_moter.distance[0], c620_moter.distance[1], c620_moter.distance[2], c620_moter.distance[3]); //其余电机的位置控制
    if (rc.ch5 == 3) //模式3 遥控模式 这个if是用于底盘控制的大类的
    {
	
//      Calculate(rc.ch4 / 250.0f, rc.ch3 / 250.0f, -rc.ch1/660.0f);
			//pid_calc(&c610[0].pid_shudu, c2006_chassis[0].speed_rpm, rc.ch2*35.0f);
      Calculate(rc.ch1 / 250.0f, rc.ch2 / 250.0f, -rc.ch3/660.0f);
			setMotor(kinematics.exp_vel[0].speed, kinematics.exp_vel[1].speed , kinematics.exp_vel[2].speed , kinematics.exp_vel[3].speed );
      setMotor_dir(kinematics.exp_vel[0].jiado, kinematics.exp_vel[1].jiado, kinematics.exp_vel[2].jiado, kinematics.exp_vel[3].jiado);
    }
    else if (rc.ch5 == 1) //模式一 保险模式
    {
      set_moto_current(&hcan1, 0,0,0,0);
			set_moto2_current(&hcan1, 0,0,0,0);
			set_moto_current(&hcan2, 0,0,0,0);
			set_moto2_current(&hcan2, 0,0,0,0);
    }
    if (rc.ch5 == 1) //模式一 保险模式 这个if else 用于上层炮台控制
    {
      set_moto_current(&hcan1, 0,0,0,0);
			set_moto2_current(&hcan1, 0,0,0,0);
			set_moto_current(&hcan2, 0,0,0,0);
			set_moto2_current(&hcan2, 0,0,0,0);
    }
		else if(rc.ch12 == 1)
		{
			setSingleMotor2(M1, -rc.ch15*10.0f);  //上层炮台摩擦轮转速
			setSingleMotor2(M2, rc.ch15*10.0f);  //上层炮台摩擦轮转速
			setSingleMotor2_juli(M3, rc.ch14*50.0f); //设置上下的旋转
			setSingleMotor2_juli(M4,rc.ch13*100.0f); //设置左右的旋转
			set_moto_current(&hcan2, c610[0].pid_shudu.pos_out, c610[1].pid_shudu.pos_out,
					 c610[2].pid_shudu.pos_out, baohu[3].pid_shudu.pos_out);
		}
		else if(rc.ch12 == 3)
		{
			setSingleMotor2(M1, 0); 
			setSingleMotor2(M2, 0);  //S_speed[0].pid_shudu.pos_out, S_speed[1].pid_shudu.pos_out,
			setSingleMotor2_juli(M3, rc.ch14*50.0f); //设置单个速度环距离走多少
			setSingleMotor2_juli(M4,rc.ch13*100.0f); //设置单个速度环距离走多少
			set_moto_current(&hcan2, c610[0].pid_shudu.pos_out, c610[1].pid_shudu.pos_out,
					 c610[2].pid_shudu.pos_out, baohu[3].pid_shudu.pos_out);
		}
  }

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
      ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
