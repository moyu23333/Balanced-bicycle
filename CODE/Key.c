/*
 * key.c
 *
 *  Created on: 2022��1��23��
 *      Author: ����
 */
#include "headfile.h"
PIN_enum KEY_PTxn[KEY_MAX]={/*�ĸ�����*/ P22_0,P22_1,P22_2,P22_3,
                             /*���뿪��*/P33_12,P33_13 };
//key�˿ڳ�ʼ��
uint8 keyval=0,keyold=0,keyup=0,keydown=0;
void get_key(uint8 keynum)
 {
        keyval=0;
       if(key_get(keynum) == IO_DOWN)
  {
           systick_delay_ms(STM0,5);
           if( key_get(keynum) == IO_DOWN)
           {
               keyval=keynum+1;
           }
   }
       keydown=keyval&(keyold^keyval);
       keyup=~keyval&(keyold^keyval);
       keyold=keyval;
   }
void scan_key(void)
{
    keyval=0;
    if(key_get(KEY0) == IO_DOWN)
    {
        systick_delay_ms(STM0,5);
        if( key_get(KEY0) == IO_DOWN)
        {
            keyval=1;
        }
    }
    else if(key_get(KEY1) == IO_DOWN)
    {
        systick_delay_ms(STM0,5);
        if( key_get(KEY1) == IO_DOWN)
        {
            keyval=2;
        }
    }
    else  if(key_get(KEY2) == IO_DOWN)
    {
        systick_delay_ms(STM0,5);
        if( key_get(KEY2) == IO_DOWN)
        {
            keyval=3;
        }
    }
    else if(key_get(KEY3) == IO_DOWN)
    {
        systick_delay_ms(STM0,5);
        if( key_get(KEY3) == IO_DOWN)
        {
            keyval=4;
        }
    }
    keydown=keyval&(keyold^keyval);
    keyup=~keyval&(keyold^keyval);
    keyold=keyval;
}

void ALL_key_init(void)
{
         gpio_init(KEY0,GPI,0,PULLUP);
         gpio_init(KEY1,GPI,0,PULLUP);
         gpio_init(KEY2,GPI,0,PULLUP);
         gpio_init(KEY3,GPI,0,PULLUP);
}
//��ȡkey״̬��������ʱ������
IO_STATUS_e key_get(KEY_e key)
 {
      if(gpio_get(KEY_PTxn[key])==IO_DOWN)
    {
     return IO_DOWN;
    }
   return IO_UP;
 }
//���key״̬������ʱ������
 IO_STATUS_e key_check(KEY_e key)
{
    if(key_get(key) == IO_DOWN)
    {
        systick_delay_ms(STM0,KeyDelayTime);
        if( key_get(key) == IO_DOWN)
        {
            return IO_DOWN;
        }
    }
    return IO_UP;
}







