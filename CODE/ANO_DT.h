/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
��ƽ    ̨�������������ܿƼ�STC16���İ�
����    д��chiusir
��E-mail  ��chiusir@163.com
������汾��V1.1 ��Ȩ���У���λʹ��������ϵ��Ȩ
�������¡�2021��1��25��
�������Ϣ�ο����е�ַ��
����    վ��http://www.lqist.cn
���Ա����̡�http://longqiu.taobao.com
------------------------------------------------
��IDE��STC16
��Target �� C251Keil5.57������
��SYS PLL�� 30MHzʹ���ڲ�����
=================================================================
STC16���������Ƶ��
����Ƽ�Bվ��ַ��https://space.bilibili.com/95313236
STC16�������زο���Ƶ�� https://www.bilibili.com/video/BV1gy4y1p7T1/
STC16һ����ӽ�����Ƶ�� https://www.bilibili.com/video/BV1Jy4y1e7R4/
=================================================================
����ʱ, ѡ��ʱ�� 30MHZ (�û��������޸�Ƶ��).
STC16F��������:����IRCBND=0��ISP��������Ϊ24M��
оƬ���һ��2038...Ȼ��IRCBND=0������Ƶ��Ϊ30M��
оƬ���һ��2010...����IRCBND=1������Ƶ��Ϊ30M��
�ٷ��������β�ͬ�����в��죬����Ϊ׼������
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ
�����������վ��λ��
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
#ifndef CODE_ANO_DT_H_
#define CODE_ANO_DT_H_
#include "headfile.h"
#include "stdint.h"
/*!
  * @brief    ����λ�����ͷ���8��int16_t����
  *
  * @param    data1 - data8  �� ���͸���λ����ʾ����
  *
  * @return   ��
  *
  * @note     ��
  *
  * @see      ANO_DT_send_int16(1, 2, 3, 0, 0, 0, 0, 0);
  *
  * @date     2019/5/28 ���ڶ�
  */
void uart_send_senser(int16_t date1,int16_t date2,int16_t date3,int16_t date4);
void uart_send_senser_10(int16_t date1,int16_t date2,int16_t date3,int16_t date4,int16_t date5,int16_t date6,int16_t date7,int16_t date8,int16_t date9,int16_t date10);
void bluetooth_Init(void);//�������ڳ�ʼ��
#endif /*CODE_ANO_DT_H_*/
