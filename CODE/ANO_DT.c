
#include "headfile.h"
#include "stdint.h"
//ʹ������7.0��λ��Э�飬��ȥ֡��ʽ����෢���ĸ��ֽڵ�����
//���ݲ�ֺ궨�壬�ڷ��ʹ���1�ֽڵ���������ʱ������int16��float�ȣ���Ҫ�����ݲ�ֳɵ����ֽڽ��з���
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
	data_to_send[cnt++]=0xAA;	 			//֡ͷ��AAAA
	data_to_send[cnt++]=0xFF;
	data_to_send[cnt++]=0xF1;	 			//������
	data_to_send[cnt++]=0;	     			//��Ҫ�������ݵ��ֽ�������ʱ��0�������ڸ�ֵ��
	data_to_send[cnt++] = BYTE0(dat1);//���ֽ�
	data_to_send[cnt++] = BYTE1(dat1);//���ֽ�
    data_to_send[cnt++] = BYTE0(dat2);
    data_to_send[cnt++] = BYTE1(dat2);
    data_to_send[cnt++] = BYTE0(dat3);
    data_to_send[cnt++] = BYTE1(dat3);
    data_to_send[cnt++] = BYTE0(dat4);
    data_to_send[cnt++] = BYTE1(dat4);
	data_to_send[3] = cnt-4;//���������ݵ��ֽ�����
	for(i=0;i<cnt;i++) 						//����for��䣬����д�����ŵ�ʱ��ִֻ�е������һ���ֺŽ�����
	{
		sum1 += data_to_send[i];
		sum2 += sum1;
	}
	data_to_send[cnt++] = sum1;	//����У��λ
	data_to_send[cnt++] = sum2;
//	uart_putbuff(UART_0,data_to_send,cnt);//����
	bluetooth_ch9141_send_buff(data_to_send,cnt);//����
}

/*---------------------------------------------------------------
 * 10�����ݷ���
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
    data_to_send[cnt++]=0xAA;               //֡ͷ��AAAA
    data_to_send[cnt++]=0xFF;
    data_to_send[cnt++]=0xF1;               //������
    data_to_send[cnt++]=0;                  //��Ҫ�������ݵ��ֽ�������ʱ��0�������ڸ�ֵ��
    data_to_send[cnt++] = BYTE0(dat1);//���ֽ�
    data_to_send[cnt++] = BYTE1(dat1);//���ֽ�
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
    data_to_send[3] = cnt-4;//���������ݵ��ֽ�����
    for(i=0;i<cnt;i++)                      //����for��䣬����д�����ŵ�ʱ��ִֻ�е������һ���ֺŽ�����
    {
        sum1 += data_to_send[i];
        sum2 += sum1;
    }
    data_to_send[cnt++] = sum1; //����У��λ
    data_to_send[cnt++] = sum2;
//  uart_putbuff(UART_0,data_to_send,cnt);//����
    bluetooth_ch9141_send_buff(data_to_send,cnt);//����
}

/*---------------------------------------------------------
 * �������ڳ�ʼ��
 ----------------------------------------------------------*/
void bluetooth_Init(void)
{
if(bluetooth_ch9141_init())                                                 // �ж��Ƿ�ͨ����ʼ��
 {
     while(1)                                                                // ��ʼ��ʧ�ܾ����������ѭ��
     {
         gpio_toggle(P20_9);                                            // ��ת LED ���������ƽ ���� LED ����
         systick_delay_ms(STM0, 100);                                               // ����ʱ�������Ʊ�ʾ�쳣
     }
 }
}
