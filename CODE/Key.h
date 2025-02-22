/*
 * keyy.h
 *
 *  Created on: 2022年1月23日
 *      Author: 摸鱼
 */
#ifndef CODE_KEY_H_
#define CODE_KEY_H_
#include "headfile.h"
#include "isr.h"
#define KeyDelayTime 80
typedef enum//按端口的枚举
{
    KEY0, //上
    KEY1,   //下
    KEY2,   //左
    KEY3,  //右
    KEYb1,    //增大
    KEYb2, //减小
    KEY_MAX
} KEY_e;
typedef enum
{
    IO_DOWN  =   0,         //按键按下时对应电平
    IO_UP    =   1,         //按键弹起时对应电平
    IO_HOLD,
} IO_STATUS_e;
extern void ALL_key_init(void);
IO_STATUS_e key_get(KEY_e key);//获取按键电平状态
extern IO_STATUS_e key_check(KEY_e key);//含消抖
extern uint8 keyval,keyold,keyup,keydown;
void scan_key(void);
void get_key(uint8 keynum);
#endif /* CODE_KEY_H_ */
