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

    uint8_t dir;      //��ʾAS5600�ķ���0 ��ʾ˳ʱ�����ӣ�1��ʱ��
    float offset;     //ÿ������ת���ƫ���Ƕ�
    int id;           //���id
    pid_t pid_weizhi; //ÿ������ת���λ�û�
    pid_t pid_shudu;  //ÿ�����ֵ��ٶȻ�
	
    float except;     //�����Ƕ�
    float meaure;     //�����Ƕ�
    float jiaodu;     //��¼ÿ����ת�ĽǶȣ���ֹ�߲��ƹ���
} DUOLUN;
typedef struct speed_duolun
{
    int id;
    pid_t pid_weizhi; //ÿ������ת���λ�û�
    pid_t pid_shudu;  //ÿ�����ֵ��ٶȻ�
    float speed;      //�����ٶ�
    float rpm;        //ת����Ȧ
} speed_wheel;
typedef struct
{
    volatile float speed; // x�����ٶ� m/s
    volatile float jiado; // y�����ٶ� m/s

} vel_st;
typedef struct
{
    int max_rpm_;               // ÿ���ӳ��ֵ����ת��
    int total_wheels_;          // ��������
    float wheels_x_distance_;   // x�����ϳ��ֵľ���				��x
    float wheels_y_distance_;   // y�����ϳ��ֵľ���				|-->y
    float pwm_res_;             // PWM������ֵ
    float wheel_circumference_; // �����ܳ�
    vel_st exp_vel[4];          //ÿ�����ֵ��ٶȺ�ת��
    float  exp_x_dis;//������������x��y����
    float  exp_y_dis;
	  float  exp_vel_linear_x;//С��������x�ٶ�
	  float  exp_vel_linear_y;
	  float  exp_vel_angular_z;//С��������w�ٶ�

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
