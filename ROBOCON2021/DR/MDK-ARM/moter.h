#ifndef _MOTER_H
#define _MOTER_H

#include "pid.h"
#include "main.h"

#define M1 1
#define M2 2
#define M3 3
#define M4 4

#define M5 5
#define M6 6
#define M7 7
#define M8 8
#define MAll 9
typedef struct duolun
{

    uint8_t dir;      //表示AS5600的方向，0 表示顺时针增加，1逆时针
    float offset;     //每个舵轮转向的偏至角度
    int id;           //电调id
    pid_t pid_weizhi; //每个舵轮转向的位置环
    pid_t pid_shudu;  //每个舵轮的速度环
	
    float except;     //期望角度
    float meaure;     //测量角度
    float jiaodu;     //记录每个旋转的角度，防止线缠绕过多
} DUOLUN;
typedef struct speed_duolun
{
    int id;
    pid_t pid_weizhi; //每个舵轮转向的位置环
    pid_t pid_shudu;  //每个舵轮的速度环
    float speed;      //运行速度
    float rpm;        //转多少圈
} speed_wheel;
typedef struct
{
    volatile float speed; // x轴线速度 m/s
    volatile float jiado; // y轴线速度 m/s

} vel_st;
typedef struct
{
    int max_rpm_;               // 每分钟车轮的最大转速
    int total_wheels_;          // 车轮数量
    float wheels_x_distance_;   // x方向上车轮的距离				↑x
    float wheels_y_distance_;   // y方向上车轮的距离				|-->y
    float pwm_res_;             // PWM输出最大值
    float wheel_circumference_; // 车轮周长
    vel_st exp_vel[4];          //每个舵轮的速度和转向
    float  exp_x_dis;//车底盘期望的x，y距离
    float  exp_y_dis;
	  float  exp_vel_linear_x;//小车期望的x速度
	  float  exp_vel_linear_y;
	  float  exp_vel_angular_z;//小车期望的w速度

} kinematics_st;
extern kinematics_st kinematics;
extern DUOLUN D_Wheel[4];
extern speed_wheel S_speed[4];
void limit_speedd(float *a, float ABS_MAX);
void setSingleMotor(uint8_t _mNum, float _mspeed);
void setDirMotor(uint8_t _mNum, int16_t _mspeed);
void duo_lun_Dir_init(void);
void duo_lun_speed_init(void);
void Kinematics_Init(void);
void setMotor(float speedM1, float speedM2, float speedM3, float speedM4);
void setMotor_dir(float jiaoduM1, float jiaoduM2, float jiaoduM3, float jiaoduM4);
void setSingleMotor_jiaodu(DUOLUN *p, float angle);
void dir_safe(void);
void Car_to_point(float x, float y, float wz);
void setallMotor_juli(float speedM1, float speedM2, float speedM3, float speedM4);
void set_speed(float speedM1, float speedM2, float speedM3, float speedM4);
void set_dir(float jiaoduM1, float jiaoduM2, float jiaoduM3, float jiaoduM4);
void Calculate(float x, float y, float wz);
void setSingleMotor2(uint8_t _mNum, float _mspeed);
void setSingleMotor_juli(uint8_t _mNum, float angle);
void clear(void);
void Exp_Speed_Cal(uint32_t dT_us);
void Set_Vel(float linear_x, float linear_y, float angular_z);
#endif
