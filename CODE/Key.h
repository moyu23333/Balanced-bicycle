/*
 * keyy.h
 *
 *  Created on: 2022��1��23��
 *      Author: ����
 */
#ifndef CODE_KEY_H_
#define CODE_KEY_H_
#include "headfile.h"
#include "isr.h"
#define KeyDelayTime 80
typedef enum//���˿ڵ�ö��
{
    KEY0, //��
    KEY1,   //��
    KEY2,   //��
    KEY3,  //��
    KEYb1,    //����
    KEYb2, //��С
    KEY_MAX
} KEY_e;
typedef enum
{
    IO_DOWN  =   0,         //��������ʱ��Ӧ��ƽ
    IO_UP    =   1,         //��������ʱ��Ӧ��ƽ
    IO_HOLD,
} IO_STATUS_e;
extern void ALL_key_init(void);
IO_STATUS_e key_get(KEY_e key);//��ȡ������ƽ״̬
extern IO_STATUS_e key_check(KEY_e key);//������
extern uint8 keyval,keyold,keyup,keydown;
void scan_key(void);
void get_key(uint8 keynum);
#endif /* CODE_KEY_H_ */
