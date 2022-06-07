/**
 ***************************************(C) COPYRIGHT 2018 DJI***************************************
 * @file       bsp_uart.h
 * @brief      this file contains the common defines and functions prototypes for 
 *             the bsp_uart.c driver
 * @note         
 * @Version    V1.0.0
 * @Date       Jan-30-2018      
 ***************************************(C) COPYRIGHT 2018 DJI***************************************
 */

#ifndef __BSP_UART_H__
#define __BSP_UART_H__

#include "usart.h"

#define UART_RX_DMA_SIZE (1024)
#define DBUS_MAX_LEN     (50)
#define DBUS_BUFLEN      (25)
#define DBUS_HUART       huart1 /* for dji remote controler reciever */
/** 
  * @brief  remote control information
  */
typedef __packed struct
{
  /* rocker channel information */
  int16_t ch1;
  int16_t ch2;
  int16_t ch3;
  int16_t ch4;
  int16_t ch5;
  int16_t ch6;
  int16_t ch7;
  int16_t ch8;
  int16_t ch9;
  int16_t ch10;
  int16_t ch11;
  int16_t ch12;
  int16_t ch13;
  int16_t ch14;
  int16_t ch15;
  int16_t ch16;
  /* left and right lever information */
  uint8_t sw1;
  uint8_t sw2;
} rc_info_t;

void uart_receive_handler(UART_HandleTypeDef *huart);
void dbus_uart_init(void);
void USART_SendData(USART_TypeDef* USARTx, uint16_t Data);
void sw_judge(rc_info_t *rc);
#endif

