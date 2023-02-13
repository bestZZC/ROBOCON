/******************************************************************************
/// @brief
/// @copyright Copyright (c) 2017 <dji-innovations, Corp. RM Dept.>
/// @license MIT License
/// Permission is hereby granted, free of charge, to any person obtaining a copy
/// of this software and associated documentation files (the "Software"), to deal
/// in the Software without restriction,including without limitation the rights
/// to use, copy, modify, merge, publish, distribute, sublicense,and/or sell
/// copies of the Software, and to permit persons to whom the Software is furnished
/// to do so,subject to the following conditions:
///
/// The above copyright notice and this permission notice shall be included in
/// all copies or substantial portions of the Software.
///
/// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
/// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
/// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
/// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
/// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
/// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
/// THE SOFTWARE.
*******************************************************************************/

#ifndef __BSP_CAN
#define __BSP_CAN


#include "stm32f4xx_hal.h"

#include "mytype.h"
#include "can.h"

/*CAN发送或是接收的ID*/
typedef enum
{

    CAN_TxPY12V_ID 	= 0x200,		//云台12V发送ID
    CAN_TxPY24V_ID	= 0x1FF,		//云台12V发送ID
//	CAN_Pitch_ID 	= 0x201,			//云台Pitch
//	CAN_Yaw_ID   	= 0x203,			//云台Yaw
    CAN_YAW_FEEDBACK_ID   = 0x205,		//云台Yaw24v
    CAN_PIT_FEEDBACK_ID  = 0x206,			//云台Yaw24v
    CAN_POKE_FEEDBACK_ID  = 0x207,
    CAN_ZGYRO_RST_ID 			= 0x404,
    CAN_ZGYRO_FEEDBACK_MSG_ID = 0x401,
    CAN_MotorLF_ID 	= 0x041,    //左前
    CAN_MotorRF_ID 	= 0x042,		//右前
    CAN_MotorLB_ID 	= 0x043,    //左后
    CAN_MotorRB_ID 	= 0x044,		//右后

    CAN_EC60_four_ID	= 0x200,	//EC60接收程序
    CAN_backLeft_EC60_ID = 0x203, //ec60
    CAN_frontLeft_EC60_ID = 0x201, //ec60
    CAN_backRight_EC60_ID = 0x202, //ec60
    CAN_frontRight_EC60_ID = 0x204, //ec60

    //add by langgo
    CAN_3510Moto_ALL_ID = 0x200,
    CAN_3510Moto1_ID = 0x201,
    CAN_3510Moto2_ID = 0x202,
    CAN_3510Moto3_ID = 0x203,
    CAN_3510Moto4_ID = 0x204,


    CAN_3510Moto_2_ID = 0x1FF,
    CAN_3510Moto5_ID = 0x205,
    CAN_3510Moto6_ID = 0x206,
    CAN_3510Moto7_ID = 0x207,
    CAN_3510Moto8_ID = 0x208,


    CAN_DriverPower_ID = 0x80,



    CAN_HeartBeat_ID = 0x156,

} CAN_Message_ID;

#define FILTER_BUF_LEN		5
/*接收到的云台电机的参数结构体*/
typedef struct {
    int16_t	 	speed_rpm;
    int16_t  	real_current;
    int16_t  	given_current;
    uint8_t  	hall;
    uint16_t 	angle;				//abs angle range:[0,8191]
    uint16_t 	last_angle;	//abs angle range:[0,8191]
    uint16_t	offset_angle;
    int32_t		round_cnt;
    int32_t		total_angle;
    u8			buf_idx;
    u16			angle_buf[FILTER_BUF_LEN];
    u16			fited_angle;
    u32			msg_cnt;
} moto_measure_t;


extern uint8_t save[8],start_meaure[8],stop_meaure[8],set_addr[8],set_mode[8],get_addr[8],get_infor[8],set_bote[8];
/* Extern  ------------------------------------------------------------------*/
extern moto_measure_t  moto_chassis[8],c2006_chassis[8];
extern moto_measure_t  moto_yaw, moto_pit, moto_poke, moto_info;
extern float real_current_from_judgesys; //unit :mA
extern float dynamic_limit_current;	//unit :mA,;	//from judge_sys
extern float ZGyroModuleAngle, yaw_zgyro_angle;
extern float  jiguag_data1[4];
int Can_TxMessage(CAN_HandleTypeDef *_hcan, uint8_t ide, uint32_t id, uint8_t len, uint8_t *data);
void my_can_filter_init(CAN_HandleTypeDef* hcan);
void my_can_filter_init_recv_all(CAN_HandleTypeDef* _hcan);
void can_filter_recv_special(CAN_HandleTypeDef* hcan);
void get_moto_measure(moto_measure_t *ptr, uint8_t Rx_Data[]);
void can_receive_onetime(CAN_HandleTypeDef* _hcan, int time);
void set_moto_current(CAN_HandleTypeDef* hcan, s16 iq1, s16 iq2, s16 iq3, s16 iq4);
void get_total_angle(moto_measure_t *p);
void set_moto2_current(CAN_HandleTypeDef* hcan, s16 iq1, s16 iq2, s16 iq3, s16 iq4); //舵轮4个电机转向；
void ceiju_intit(CAN_HandleTypeDef *hcan,uint8_t*data);
#endif
