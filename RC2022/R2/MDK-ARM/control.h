#ifndef _CONTROL_H
#define _CONTROL_H
#include "usart.h"
#include "moter.h"

typedef struct
{
   float distance[5];

} c620moter;
typedef union 
{
   short shuju;
   uint8_t buffer[2];
}messsage;
extern messsage cmd_data[3];
void set_exp(float vx,float vy); 
extern float my_wz,my_yaw;
void send_com(UART_HandleTypeDef *huart, uint8_t data);
extern speed_wheel c610[8];
void setSingleMotor2(uint8_t _mNum, float _mspeed);
void setSingleMotor2_juli(uint8_t _mNum, float angle) ;//设置单个速度环距离走多少
extern c620moter c620_moter;
void C610_init(void);
extern float tuolun_angle[6];
void Archery_action(float);
void set_high(float jiadu); 
void set_point(float x_get, float y_get);
void read_data(void);
void qujian(void);
void init_guiwei(void);
void set_duojjiaodu(float jiadu) ;
void shot(void);
void move_qifa(void);
void move_qifa1(void);
void init_position(void);
void clear_motor(void);
void set_c620moter(float speed1,float speed2,float distance3,float distance4,float speed3);
#endif

//	HAL_TIM_Base_Stop_IT(&htim7);
