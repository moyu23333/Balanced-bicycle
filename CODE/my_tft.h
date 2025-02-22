/*
 * mt_tft.h
 *
 *  Created on: 2022��4��18��
 *      Author: ����
 */

#ifndef CODE_MY_TFT_H_
#define CODE_MY_TFT_H_
#include "common.h"
#include "SEEKFREE_FONT.h"
#define NAVY        0x000F      //  ����ɫ
#define DGREEN      0x03E0      //  ����ɫ
#define DCYAN       0x03EF      //  ����ɫ
#define MAROON      0x7800      //  ���ɫ
#define PURPLEE     0x780F      //  ��ɫ
#define OLIVE       0x7BE0      //  �����
#define LGRAY       0xC618      //  �Ұ�ɫ
#define DGRAY       0x7BEF      //  ���ɫ
#define CYAN        0x07FF      //  ��ɫ
#define MAGENTA     0xF81F      //  Ʒ��

#define BackgroundColor                 RGB565_BLACK       //������
#define MainInterface1Pen               RGB565_YELLOW      //���˵���
#define MainInterface1Background        RGB565_BLACK       //���˵�����
#define NowOption_Background            NAVY        //ѡ�����



void lcd_showchar_color(uint16 x,uint16 y,const int8 dat,uint16 text,uint16 background);
void lcd_showstr_color(uint16 x,uint16 y,const int8 dat[],uint16 text,uint16 background);
void lcd_str(uint16* XY,uint8 dat[],uint16 Pen,uint16 Background);
void lcd_uint8(uint16* XY,uint8 dat,uint16 Pen,uint16 Background);
void lcd_uint16(uint16* XY,uint32 dat,uint16 Pen,uint16 Background);
//void lcd_float(uint16 *XY,double dat,uint8 num,uint8 pointnum,uint16 P,uint16 B);
void lcd_showuint8_color(uint16 x,uint16 y,uint8 dat,uint16 text,uint16 background);
void lcd_showint16_color(uint16 x,uint16 y,int16 dat,uint16 text,uint16 background);
void lcd_showuint16_color(uint16 x,uint16 y,uint16 dat,uint16 text,uint16 background);
void lcd_showint32_color(uint16 x,uint16 y,int32 dat,uint8 num,uint16 text,uint16 background);
void lcd_showfloat_color(uint16 x,uint16 y,double dat,uint8 num,uint8 pointnum,uint16 text,uint16 background);
void lcd_display_chinese_color(uint16 x, uint16 y, uint8 size, const uint8 *p, uint8 number, uint16 color,uint16 background);
void lcd_display_image(uint8 *p);
void lcd_displayimage(uint8 *p, uint16 width, uint16 height);
void Show_Chinese(uint16 *XY,uint8 size, const uint8 *p, uint8 number,uint16 pen,uint16 background);
void show_image_128_160(const unsigned char *p);
#endif /* CODE_MY_TFT_H_ */
