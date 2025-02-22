/*
 * my_tft.c
 *
 *  Created on: 2022��4��18��
 *      Author: ����
 */
#include "zf_spi.h"
#include "zf_gpio.h"
#include "zf_assert.h"
#include "zf_stm_systick.h"
#include "SEEKFREE_PRINTF.h"
#include "SEEKFREE_18TFT.h"
#include "SEEKFREE_MT9V03X.h"
#include "my_tft.h"
//-------------------------------------------------------------------------------------------------------------------
//  @brief      Һ����ʾ�ַ�(����������ɫ�ͱ�����ɫ��������)
//-------------------------------------------------------------------------------------------------------------------
void lcd_showchar_color(uint16 x,uint16 y,const int8 dat,uint16 text,uint16 background)
{
    uint8 i,j;
    uint8 temp;

    for(i=0; i<16; i++)
    {
        lcd_set_region(x,y+i,x+7,y+i);
        temp = tft_ascii[dat-32][i];//��32��Ϊ��ȡģ�Ǵӿո�ʼȡ�� �ո���ascii�������32
        for(j=0; j<8; j++)
        {
            if(temp&0x01)   lcd_writedata_16bit(text);
            else            lcd_writedata_16bit(background);
            temp>>=1;
        }
    }
}
//-------------------------------------------------------------------------------------------------------------------
//  @brief      Һ����ʾ�ַ������Զ�����ɫ��
//-------------------------------------------------------------------------------------------------------------------
void lcd_showstr_color(uint16 x,uint16 y,const int8 dat[],uint16 text,uint16 background)
{
    uint16 j;
    j = 0;
    while(dat[j] != '\0')
    {
        lcd_showchar_color(x+8*j,y*16,dat[j],text,background);
        j++;
    }
}
/*---------------------------------------------
 * �ַ�����ʾ   ������ڲ�����Ϊ ����
---------------------------------------------- */
void lcd_str(uint16* XY,uint8 dat[],uint16 Pen,uint16 Background)
{
    uint16 j;
    j = 0;
    while(dat[j] != '\0')
    {
        lcd_showchar_color(XY[0]+8*j,XY[1]*16,dat[j],Pen,Background);
        j++;
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      Һ����ʾ8λ�޷���(��ɫ��)
//-------------------------------------------------------------------------------------------------------------------
void lcd_showuint8_color(uint16 x,uint16 y,uint8 dat,uint16 text,uint16 background)
{
    uint8 a[3];
    uint8 i;

    a[0] = dat/100;
    a[1] = dat/10%10;
    a[2] = dat%10;
    i = 0;
    while(i<3)
    {
        lcd_showchar_color(x+(8*i),y*16,'0' + a[i],text,background);// lcd_showchar_color(x+8*j,y*16,dat[j],text,background);
        i++;
    }

}
//-------------------------------------------------------------------------------------------------------------------
//  @brief      Һ����ʾ16λ�з���(ɫ��)
//-------------------------------------------------------------------------------------------------------------------
void lcd_showint16_color(uint16 x,uint16 y,int16 dat,uint16 text,uint16 background)
{
    uint8 a[5];
    uint8 i;
    if(dat<0)
    {
        lcd_showchar_color(x,y*16,'-',text,background);
        dat = -dat;
    }
    else    lcd_showchar_color(x,y*16,' ',text,background);

    a[0] = dat/10000;
    a[1] = dat/1000%10;
    a[2] = dat/100%10;
    a[3] = dat/10%10;
    a[4] = dat%10;

    i = 0;
    while(i<5)
    {
        lcd_showchar_color(x+(8*(i+1)),y*16,'0' + a[i],text,background);
        i++;
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      Һ����ʾ16λ�޷��ţ�ɫ����
//-------------------------------------------------------------------------------------------------------------------
void lcd_showuint16_color(uint16 x,uint16 y,uint16 dat,uint16 text,uint16 background)
{
    uint8 a[5];
    uint8 i;

    a[0] = dat/10000;
    a[1] = dat/1000%10;
    a[2] = dat/100%10;
    a[3] = dat/10%10;
    a[4] = dat%10;

    i = 0;
    while(i<5)
    {
        lcd_showchar_color(x+(8*i),y*16,'0' + a[i],text,background);
        i++;
    }
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      Һ����ʾ32λ�з���(ȥ������������Ч��0)��ɫ����
//-------------------------------------------------------------------------------------------------------------------
void lcd_showint32_color(uint16 x,uint16 y,int32 dat,uint8 num,uint16 text,uint16 background)
{
    int8    buff[34];
    uint32   length;

    if(10<num)      num = 10;

    num++;
    if(0>dat)   length = zf_sprintf( &buff[0],"%d",dat);//����
    else
    {
        buff[0] = ' ';
        length = zf_sprintf( &buff[1],"%d",dat);
        length++;
    }
    while(length < num)
    {
        buff[length] = ' ';
        length++;
    }
    buff[num] = '\0';

    lcd_showstr_color(x, y, buff,text,background);    //��ʾ����
}
//-------------------------------------------------------------------------------------------------------------------
//  @brief      Һ����ʾ������(ȥ������������Ч��0)(ɫ��)
//-------------------------------------------------------------------------------------------------------------------
void lcd_showfloat_color(uint16 x,uint16 y,double dat,uint8 num,uint8 pointnum,uint16 text,uint16 background)
{
    uint32   length;
    int8    buff[34];
    int8    start,end,point;

    if(6<pointnum)  pointnum = 6;
    if(10<num)      num = 10;

    if(0>dat)   length = zf_sprintf( &buff[0],"%f",dat);//����
    else
    {
        length = zf_sprintf( &buff[1],"%f",dat);
        length++;
    }
    point = (int8)(length - 7); //����С����λ��
    start = point - num - 1;    //������ʼλ
    end = point + pointnum + 1; //�������λ
    while(0>start)//����λ����  ĩβӦ�����ո�
    {
        buff[end] = ' ';
        end++;
        start++;
    }

    if(0>dat)   buff[start] = '-';
    else        buff[start] = ' ';

    buff[end] = '\0';

    lcd_showstr_color(x, y, buff,text,background);    //��ʾ����
}
////��������ʾ   ������ڲ�����Ϊ ����
/*
void lcd_float(uint16 *XY,double dat,uint8 num,uint8 pointnum,uint16 P,uint16 B)
{
    uint8   length;
    int8    buff[34];
    int8    start,end,point;
    if(6<pointnum)  pointnum = 6;
    if(10<num)      num = 10;
    if(0>dat)   length = zf_sprintf( &buff[0],"%f",dat);//����
    else
    {
        length = zf_sprintf( &buff[1],"%f",dat);
        length++;
    }
    point = length - 7;         //����С����λ��
    start = point - num - 1;    //������ʼλ
    end = point + pointnum + 1; //�������λ
    while(0>start)//����λ����  ĩβӦ�����ո�
    {
        buff[end] = ' ';
        end++;
        start++;
    }
    if(0>dat)   buff[start] = '-';
    else        buff[start] = ' ';
    buff[end] = '\0';
    lcd_showstr_color(XY[0], XY[1], (uint8 *)buff,P,B); //��ʾ����
}
*/
void lcd_displayimage(uint8 *p, uint16 width, uint16 height)
{
    uint32 i,j;

    uint16 temp = 0;

    uint16 coord_x = 0;
    uint16 coord_y = 0;


    if(0==TFT_DISPLAY_DIR || 1==TFT_DISPLAY_DIR)//����
    {
        coord_x = height>TFT_X_MAX?TFT_X_MAX:height;
        coord_y = width>TFT_Y_MAX?TFT_Y_MAX:width;

        for(j=0;j<coord_y;j++)
        {
            lcd_set_region(0,j,coord_x-1,j);
            for(i=0;i<coord_x;i++)
            {
                temp = *(p+i*width+j*width/coord_y);//��ȡ���ص�
                if(temp==0) lcd_writedata_16bit(RGB565_BLACK); //temp=0;  //
                                else if(temp==1)              lcd_writedata_16bit(RGB565_WHITE);//temp=1;  //
                                else if(temp==2)                lcd_writedata_16bit(RGB565_BLUE);//temp=1;  //
                                else if(temp==3)                lcd_writedata_16bit(RGB565_RED);//temp=1;  //
                                else if(temp==4)                lcd_writedata_16bit(RGB565_GREEN);//temp=1;  //
            }
        }

    }
    else//����
    {
        coord_x = width>TFT_X_MAX?TFT_X_MAX:width;
        coord_y = height>TFT_Y_MAX?TFT_Y_MAX:height;
        lcd_set_region(0,0,coord_x-1,coord_y-1);

        for(j=0;j<coord_y;j++)
        {
            for(i=0;i<coord_x;i++)
            {
                temp = *(p+j*width+i*width/coord_x);//��ȡ���ص�
                if(temp==0) lcd_writedata_16bit(RGB565_BLACK); //temp=0;  //
                                else if(temp==1)              lcd_writedata_16bit(RGB565_WHITE);//temp=1;  //
                                else if(temp==2)                lcd_writedata_16bit(RGB565_BLUE);//temp=1;  //
                                else if(temp==3)                lcd_writedata_16bit(RGB565_RED);//temp=1;  //
                                else if(temp==4)                lcd_writedata_16bit(RGB565_GREEN);//temp=1;  //
            }
        }
    }
}
void lcd_display_image(uint8 *p)
{
    uint32 i,j;
    uint16 color = 0;
    uint16 temp = 0;
    uint16 coord_x = 0;
    uint16 coord_y = 0;
    if(0==TFT_DISPLAY_DIR || 1==TFT_DISPLAY_DIR)//����
    {
        coord_x = MT9V03X_H>TFT_X_MAX?TFT_X_MAX:MT9V03X_H;
        coord_y = MT9V03X_W>TFT_Y_MAX?TFT_Y_MAX:MT9V03X_W;
        for(j=0; j<coord_y; j++)
        {
            lcd_set_region(0,j,coord_x-1,j);
            for(i=0; i<coord_x; i++)
            {
                temp = *(p+i*MT9V03X_W+j*MT9V03X_W/coord_y);//��ȡ���ص�
                color=(0x001f&((temp)>>3))<<11;
                color=color|(((0x003f)&((temp)>>2))<<5);
                color=color|(0x001f&((temp)>>3));
                lcd_writedata_16bit(color);
            }
        }
    }
    else//����
    {
        coord_x = MT9V03X_W>TFT_X_MAX?TFT_X_MAX:MT9V03X_W;
        coord_y = MT9V03X_H>TFT_Y_MAX?TFT_Y_MAX:MT9V03X_H;
        lcd_set_region(0,0,coord_x-1,coord_y-1);

        for(j=0; j<coord_y; j++)
        {
            for(i=0; i<coord_x; i++)
            {
                temp = *(p+j*MT9V03X_W+i*MT9V03X_W/coord_x);//��ȡ���ص�
                color=(0x001f&((temp)>>3))<<11;
                color=color|(((0x003f)&((temp)>>2))<<5);
                color=color|(0x001f&((temp)>>3));
                lcd_writedata_16bit(color);
            }
        }
    }
}
//-------------------------------------------------------------------------------------------------------------------
//  @brief      ������ʾ����������ɫ��
//-------------------------------------------------------------------------------------------------------------------
void lcd_display_chinese_color(uint16 x, uint16 y, uint8 size, const uint8 *p, uint8 number, uint16 color,uint16 background)
{
    int i, j, k;
    uint8 temp, temp1, temp2;
    const uint8 *p_data;

    temp2 = size/8;

    lcd_set_region(x,y,number*size-1+x,y+size-1);

    for(i=0;i<size;i++)
    {
        temp1 = number;
        p_data = p+i*temp2;
        while(temp1--)
        {
            for(k=0;k<temp2;k++)
            {
                for(j=8;j>0;j--)
                {
                    temp = (*p_data>>(j-1)) & 0x01;
                    if(temp)    lcd_writedata_16bit(color);
                    else        lcd_writedata_16bit(background);
                }
                p_data++;
            }
            p_data = p_data - temp2 + temp2*size;
        }
    }
}

