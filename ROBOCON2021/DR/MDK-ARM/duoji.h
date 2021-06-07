#ifndef _DUOJI__H
#define _DUOJI__H
#include "main.h"
#include "stdbool.h"
void set_duoji(uint8_t num,uint8_t state);
void Grab_action(bool state);//底部的把箭抓起来函数
void clamp(bool state);//机械臂最上端舵机
void shot(void);
#endif


