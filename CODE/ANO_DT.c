
#include "headfile.h"
#include "stdint.h"
//使用匿名7.0上位机协议，除去帧格式，最多发送四个字节的数据
//数据拆分宏定义，在发送大于1字节的数据类型时，比如int16、float等，需要把数据拆分成单独字节进行发送
#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)    ) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )

void uart_send_senser(int16_t date1,int16_t date2,int16_t date3,int16_t date4)
{
	unsigned char data_to_send[20] = {0};
	unsigned char i = 0;
	unsigned char cnt = 0;
	unsigned char sum1 = 0;
	unsigned char sum2 = 0;
	int16_t dat1=date1;
	int16_t dat2=date2;
	int16_t dat3=date3;
	int16_t dat4=date4;
	data_to_send[cnt++]=0xAA;	 			//帧头：AAAA
	data_to_send[cnt++]=0xFF;
	data_to_send[cnt++]=0xF1;	 			//功能字
	data_to_send[cnt++]=0;	     			//需要发送数据的字节数，暂时给0，后面在赋值。
	data_to_send[cnt++] = BYTE0(dat1);//低字节
	data_to_send[cnt++] = BYTE1(dat1);//高字节
    data_to_send[cnt++] = BYTE0(dat2);
    data_to_send[cnt++] = BYTE1(dat2);
    data_to_send[cnt++] = BYTE0(dat3);
    data_to_send[cnt++] = BYTE1(dat3);
    data_to_send[cnt++] = BYTE0(dat4);
    data_to_send[cnt++] = BYTE1(dat4);
	data_to_send[3] = cnt-4;//计算总数据的字节数。
	for(i=0;i<cnt;i++) 						//对于for语句，当不写大括号的时候，只执行到下面第一个分号结束。
	{
		sum1 += data_to_send[i];
		sum2 += sum1;
	}
	data_to_send[cnt++] = sum1;	//计算校验位
	data_to_send[cnt++] = sum2;
//	uart_putbuff(UART_0,data_to_send,cnt);//有线
	bluetooth_ch9141_send_buff(data_to_send,cnt);//无线
}

/*---------------------------------------------------------------
 * 10个数据发送
 -----------------------------------------------------------------*/
void uart_send_senser_10(int16_t date1,int16_t date2,int16_t date3,int16_t date4,int16_t date5,int16_t date6,int16_t date7,int16_t date8,int16_t date9,int16_t date10)
{
    unsigned char data_to_send[40] = {0};
    unsigned char i = 0;
    unsigned char cnt = 0;
    unsigned char sum1 = 0;
    unsigned char sum2 = 0;
    int16_t dat1=date1;
    int16_t dat2=date2;
    int16_t dat3=date3;
    int16_t dat4=date4;
    int16_t dat5=date5;
    int16_t dat6=date6;
    int16_t dat7=date7;
    int16_t dat8=date8;
    int16_t dat9=date9;
    int16_t dat10=date10;
    data_to_send[cnt++]=0xAA;               //帧头：AAAA
    data_to_send[cnt++]=0xFF;
    data_to_send[cnt++]=0xF1;               //功能字
    data_to_send[cnt++]=0;                  //需要发送数据的字节数，暂时给0，后面在赋值。
    data_to_send[cnt++] = BYTE0(dat1);//低字节
    data_to_send[cnt++] = BYTE1(dat1);//高字节
    data_to_send[cnt++] = BYTE0(dat2);
    data_to_send[cnt++] = BYTE1(dat2);
    data_to_send[cnt++] = BYTE0(dat3);
    data_to_send[cnt++] = BYTE1(dat3);
    data_to_send[cnt++] = BYTE0(dat4);
    data_to_send[cnt++] = BYTE1(dat4);
    data_to_send[cnt++] = BYTE0(dat5);
    data_to_send[cnt++] = BYTE1(dat5);
    data_to_send[cnt++] = BYTE0(dat6);
    data_to_send[cnt++] = BYTE1(dat6);
    data_to_send[cnt++] = BYTE0(dat7);
    data_to_send[cnt++] = BYTE1(dat7);
    data_to_send[cnt++] = BYTE0(dat8);
    data_to_send[cnt++] = BYTE1(dat8);
    data_to_send[cnt++] = BYTE0(dat9);
    data_to_send[cnt++] = BYTE1(dat9);
    data_to_send[cnt++] = BYTE0(dat10);
    data_to_send[cnt++] = BYTE1(dat10);
    data_to_send[3] = cnt-4;//计算总数据的字节数。
    for(i=0;i<cnt;i++)                      //对于for语句，当不写大括号的时候，只执行到下面第一个分号结束。
    {
        sum1 += data_to_send[i];
        sum2 += sum1;
    }
    data_to_send[cnt++] = sum1; //计算校验位
    data_to_send[cnt++] = sum2;
//  uart_putbuff(UART_0,data_to_send,cnt);//有线
    bluetooth_ch9141_send_buff(data_to_send,cnt);//无线
}

/*---------------------------------------------------------
 * 蓝牙串口初始化
 ----------------------------------------------------------*/
void bluetooth_Init(void)
{
if(bluetooth_ch9141_init())                                                 // 判断是否通过初始化
 {
     while(1)                                                                // 初始化失败就在这进入死循环
     {
         gpio_toggle(P20_9);                                            // 翻转 LED 引脚输出电平 控制 LED 亮灭
         systick_delay_ms(STM0, 100);                                               // 短延时快速闪灯表示异常
     }
 }
}
