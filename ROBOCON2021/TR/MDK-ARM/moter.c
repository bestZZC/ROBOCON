#include "moter.h"
#include "bsp_can.h"
#include "math.h"
#include "stdio.h"
#include "bsp_imu.h"
#include "bsp_uart.h"
#include <math.h>
/***********************************此c文件的函数是对舵轮的操作**********************************************************/
#define fabs(a) a > 0 ? a : -a
const int offsetm1 = 1; //代表顺时针
const int offsetm2 = 1; //代表逆时针
const int offsetm3 = 1;
const int offsetm4 = -1;
float speed = 0.2;
#define R 0.1f
short ABS_MAX = 20000;               //数度最大值
#define circle 0.10688f * 3.1415926f //每个轮子的周长
#define Reduction 8192 * 19.2f * 3   //减速比
const int saft_angle = 859441;
u16 moter1[4] = {0};
s16 pos_out[4] = {0};
DUOLUN D_Wheel[4];
speed_wheel S_speed[4];
static double PI = 3.1415926f;
double LENGTH = 0.48, WIDTH = 0.48;
double Ri = 0.0;
kinematics_st kinematics;
extern rc_info_t rc;
extern float tuolun_angle[2];
void Kinematics_Init(void)
{
    kinematics.max_rpm_ = 330;             // 空载转速330rpm
    kinematics.wheels_x_distance_ = 100.0; //che de weizhi
    kinematics.wheels_y_distance_ = 0.0;
    kinematics.pwm_res_ = 500;
    kinematics.wheel_circumference_ = 0.10688f * 3.1415926f; //轮子周长
    kinematics.total_wheels_ = 4;
}
void set_dir(float jiaoduM1, float jiaoduM2, float jiaoduM3, float jiaoduM4) //改变角度的期望
{
    kinematics.exp_vel[0].jiado = jiaoduM1;
    kinematics.exp_vel[1].jiado = jiaoduM2;
    kinematics.exp_vel[2].jiado = jiaoduM3;
    kinematics.exp_vel[3].jiado = jiaoduM4;
}
float L = 0.848528f;
#define PI 3.14159265
float val = 180.0 / PI;
extern float orignal_yaw, yaw;//imu初始的角度和实时的角度
float vec_w[4][3] = {0}; //
float x1[4], y1[4], zero_thea[4] = {-45, 45, 135, -135};
float tagart[4][2] = {0};
void Speed_Cal(float x, float y, float wz)//用来计算小车的期望速度
{
	  float linear_vel_x_mins;
    float linear_vel_y_mins;
    float angular_vel_z_mins;
    float tangential_vel;
    float x_rpm;
    float y_rpm;
    float tan_rpm;
	  linear_vel_x_mins = x * 60.0f;//将期望的速度 m/s 转换为 m/min
    linear_vel_y_mins = y * 60.0f;
	  angular_vel_z_mins =wz * 60.0f;
    tangential_vel = angular_vel_z_mins * ((kinematics.wheels_x_distance_ / 2) + (kinematics.wheels_y_distance_ / 2));
	  x_rpm = linear_vel_x_mins*19/kinematics.wheel_circumference_;
    y_rpm = linear_vel_y_mins*19/kinematics.wheel_circumference_;
    tan_rpm = tangential_vel/kinematics.wheel_circumference_;
	  tan_rpm=pid_calc(&yaw_pos, imu.wz,tan_rpm)*15;
	  kinematics.exp_vel[0].speed=x_rpm - y_rpm + tan_rpm;
		kinematics.exp_vel[1].speed=x_rpm + y_rpm - tan_rpm;
		kinematics.exp_vel[2].speed=x_rpm - y_rpm - tan_rpm;
		kinematics.exp_vel[3].speed=x_rpm + y_rpm + tan_rpm;

}

void set_speed(float speedM1, float speedM2, float speedM3, float speedM4)
{
    kinematics.exp_vel[0].speed = speedM1;
    kinematics.exp_vel[1].speed = speedM2;
    kinematics.exp_vel[2].speed = speedM3;
    kinematics.exp_vel[3].speed = speedM4;
}
void limit_speedd(float *a, float ABS_MAX)
{
    if (*a > ABS_MAX)
        *a = ABS_MAX;
    if (*a < -ABS_MAX)
        *a = -ABS_MAX;
}

void dir_safe()
{
    for (int i = 4; i < 8; i++)
    {
        if (fabs(moto_chassis[i].total_angle - saft_angle) > 500)
        {
            D_Wheel[i - 4].pid_shudu.pos_out = 0; //超过360为零
        }
    }
}
void setSingleMotor(uint8_t _mNum, float _mspeed)
{
    limit_speedd(&_mspeed, ABS_MAX);
    if (_mNum == M1)
    { // MOTOR 1
        pid_calc(&S_speed[0].pid_shudu, moto_chassis[0].speed_rpm, offsetm1 * _mspeed);
    }
    else if (_mNum == M2)
    { // MOTOR 2
        pid_calc(&S_speed[1].pid_shudu, moto_chassis[1].speed_rpm, offsetm2 * _mspeed);
    }
    else if (_mNum == M3)
    { // MOTOR 3
        pid_calc(&S_speed[2].pid_shudu, moto_chassis[2].speed_rpm, offsetm3 * _mspeed);
    }
    else if (_mNum == M4)
    { // MOTOR 4
        pid_calc(&S_speed[3].pid_shudu, moto_chassis[3].speed_rpm, offsetm4 * _mspeed);
    }
}
void setMotor(float speedM1, float speedM2, float speedM3, float speedM4) //每个舵轮的速度 正speed代表前进 负代表后退 依次为左前 右前 右后 左后轮
{

    setSingleMotor(M1, speedM1);
    setSingleMotor(M2, -speedM2);
    setSingleMotor(M3, -speedM3);
    setSingleMotor(M4, -speedM4);
    set_moto_current(&hcan1, S_speed[0].pid_shudu.pos_out, S_speed[1].pid_shudu.pos_out,
                     S_speed[2].pid_shudu.pos_out, S_speed[3].pid_shudu.pos_out);
}

void setMotor_dir(float jiaoduM1, float jiaoduM2, float jiaoduM3, float jiaoduM4) //每个舵轮的方向 正speed代表前进 负代表后退
{
    setSingleMotor_jiaodu(&D_Wheel[0], jiaoduM1);
    setSingleMotor_jiaodu(&D_Wheel[1], jiaoduM2);
    setSingleMotor_jiaodu(&D_Wheel[2], jiaoduM3);
    setSingleMotor_jiaodu(&D_Wheel[3], jiaoduM4);
    //    dir_safe();
    set_moto2_current(&hcan1, D_Wheel[0].pid_shudu.pos_out,
                      D_Wheel[1].pid_shudu.pos_out,
                      D_Wheel[2].pid_shudu.pos_out,
                      D_Wheel[3].pid_shudu.pos_out);
}
void duo_lun_Dir_init()
{
    D_Wheel[0].dir = 1;
    D_Wheel[1].dir = 1;
    D_Wheel[2].dir = 1;
    D_Wheel[3].dir = 1;
    D_Wheel[0].id = 4;
    D_Wheel[1].id = 5;
    D_Wheel[2].id = 6;
    D_Wheel[3].id = 7; // P      I        D
    PID_struct_init(&D_Wheel[0].pid_weizhi, POSITION_PID, 20000, 20000, 221.0f, 0.01f, 0.233f);
    PID_struct_init(&D_Wheel[1].pid_weizhi, POSITION_PID, 20000, 20000, 221.0f, 0.010f, 0.233f);
    PID_struct_init(&D_Wheel[2].pid_weizhi, POSITION_PID, 20000, 20000, 221.0f, 0.010f, 0.233f);
    PID_struct_init(&D_Wheel[3].pid_weizhi, POSITION_PID, 20000, 20000, 221.0f, 0.010f, 0.233f);
    PID_struct_init(&D_Wheel[0].pid_shudu, POSITION_PID, 20000, 20000, 5.7f, 0.00f, 0.02f);
    PID_struct_init(&D_Wheel[1].pid_shudu, POSITION_PID, 20000, 20000, 5.7f, 0.00f, 0.02f);
    PID_struct_init(&D_Wheel[2].pid_shudu, POSITION_PID, 20000, 20000, 5.7f, 0.00f, 0.02f); // 5.7f,  0.01f, 0.02f
    PID_struct_init(&D_Wheel[3].pid_shudu, POSITION_PID, 20000, 20000, 5.7f, 0.00f, 0.02f);
    D_Wheel[0].offset = 163.5; //设置零点
    D_Wheel[1].offset = 230.5;
    D_Wheel[2].offset = 177.5;
    D_Wheel[3].offset = 140.2f;
    D_Wheel[0].except = 0;
    D_Wheel[1].except = 0;
    D_Wheel[2].except = 0;
    D_Wheel[3].except = 0;

    D_Wheel[0].jiaodu = 0;
    D_Wheel[1].jiaodu = 0;
    D_Wheel[2].jiaodu = 0;
    D_Wheel[3].jiaodu = 0;
}

void duo_lun_speed_init()
{
    S_speed[0].id = 0;
    S_speed[1].id = 1;
    S_speed[2].id = 2;
    S_speed[3].id = 3;
    // P      I        D
    PID_struct_init(&S_speed[0].pid_weizhi, POSITION_PID, 10000, 1000,
                    0.42f, 0.001f, 0.3f);
    PID_struct_init(&S_speed[1].pid_weizhi, POSITION_PID, 2000, 1000,
                    0.42f, 0.001f, 0.3f);
    PID_struct_init(&S_speed[2].pid_weizhi, POSITION_PID, 2000, 1000,
                    0.42f, 0.001f, 0.3f);
    PID_struct_init(&S_speed[3].pid_weizhi, POSITION_PID, 20000, 2000,
                    0.42f, 0.001f, 0.3f);

    PID_struct_init(&S_speed[0].pid_shudu, DELTA_PID, 10000, 2000,
                    8.50f, 1.5f, 0.52f); //20.0f, 1.2f, 0.8f
    PID_struct_init(&S_speed[1].pid_shudu, DELTA_PID, 20000, 2000,
                    8.50f, 1.5f, 0.52f); //12.0f, 0.00001f, 0.3f
    PID_struct_init(&S_speed[2].pid_shudu, DELTA_PID, 20000, 2000,
                    12.0f, 1.5f, 0.3f);
    PID_struct_init(&S_speed[3].pid_shudu, DELTA_PID, 10000, 2000,
                    8.50f, 1.5f, 0.52f);
    S_speed[0].rpm = 0;
    S_speed[1].rpm = 0;
    S_speed[2].rpm = 0;
    S_speed[3].rpm = 0;
    S_speed[0].speed = 0;
    S_speed[1].speed = 0;
    S_speed[2].speed = 0;
    S_speed[3].speed = 0;
}
/*** if (mesure - angle > 180) //最多转180度
    {
        mesure = mesure - 360;
    }
    else if (angle - mesure > 270)
        angle = angle - 360;**/
void setSingleMotor_jiaodu(DUOLUN *p, float angle) //控制转向 Id 为 5 6 7 8
{
    float mesure = 0;
    u8 i = 0;
    i = p - &D_Wheel[0];
	  angle = fmod(angle, 360);
    angle = angle + D_Wheel[i].offset;
    //	  angle=(int)angle%360;
    if (angle < 0)
        angle += 360;
		 angle = fmod(angle, 360);
    mesure = tuolun_angle[i];
   if (mesure - angle > 180) //最多转180度
    {
        mesure = mesure - 360;
    }
    else if (angle - mesure > 270)
        angle = angle - 360;
	 

    pid_calc(&p->pid_weizhi, mesure, angle);                                               //位置环
    pid_calc(&p->pid_shudu, moto_chassis[i + 4].speed_rpm, p->pid_weizhi.pos_out * speed); //速度环
    pos_out[i] = p->pid_shudu.pos_out;
}

void setSingleMotor_juli(uint8_t _mNum, float angle) //设置单个速度环距离走多少
{
    if (_mNum == M1)
    { // MOTOR 1
        pid_calc(&S_speed[0].pid_weizhi, moto_chassis[0].total_angle, angle);
        pid_calc(&S_speed[0].pid_shudu, moto_chassis[0].speed_rpm, S_speed[0].pid_weizhi.pos_out);
    }
    else if (_mNum == M2)
    { // MOTOR 2
        pid_calc(&S_speed[1].pid_weizhi, moto_chassis[1].total_angle, angle);
        pid_calc(&S_speed[1].pid_shudu, moto_chassis[1].speed_rpm, S_speed[1].pid_weizhi.pos_out);
    }
    else if (_mNum == M3)
    { // MOTOR 3
        pid_calc(&S_speed[2].pid_weizhi, moto_chassis[2].total_angle, angle);
        pid_calc(&S_speed[2].pid_shudu, moto_chassis[2].speed_rpm, S_speed[2].pid_weizhi.pos_out);
    }
    else if (_mNum == M4)
    { // MOTOR 4
        pid_calc(&S_speed[3].pid_weizhi, moto_chassis[3].total_angle, angle);
        pid_calc(&S_speed[3].pid_shudu, moto_chassis[3].speed_rpm, S_speed[3].pid_weizhi.pos_out);
    }
    set_moto_current(&hcan1, S_speed[0].pid_shudu.pos_out, S_speed[1].pid_shudu.pos_out,
                     S_speed[2].pid_shudu.pos_out, S_speed[3].pid_shudu.pos_out);
}

void setallMotor_juli(float speedM1, float speedM2, float speedM3, float speedM4) //设置所以速度轮走的距离 单位是m
{

    speedM1 = (speedM1 / circle) * Reduction / 10;
    speedM2 = (speedM2 / circle) * Reduction / 10;
    speedM3 = (speedM3 / circle) * Reduction / 10;
    speedM4 = (speedM4 / circle) * Reduction / 10;
    setSingleMotor_juli(M1, speedM1);
    setSingleMotor_juli(M2, -speedM2);
    setSingleMotor_juli(M3, -speedM3);
    setSingleMotor_juli(M4, -speedM4);
}
void set_jiaodu(float jiaodu)
{
    if (fabs(imu.yaw - jiaodu) < 0.5f)
        setMotor_dir(0, 0, 0, 0);
    else if (fabs(imu.yaw - jiaodu) > 0.5f)
    {
        setMotor_dir(45, 45, 45, 45);
        pid_sp_calc(&yaw_pos, imu.yaw, jiaodu, mpu_data.gz);
        setMotor(yaw_pos.pos_out, yaw_pos.pos_out, yaw_pos.pos_out, yaw_pos.pos_out);
    }
}
