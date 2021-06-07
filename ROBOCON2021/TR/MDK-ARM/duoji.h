#ifndef _DUOJI__H
#define _DUOJI__H
#include "main.h"
#include "stdbool.h"
void Grab_action(int num ,bool state);//底部舵机
void clamp(bool state);//机械臂最上端舵机
void shot(void);
void open_jian(bool state);
void clamp_half(bool state);
//void hengxiang(int num);
#endif


