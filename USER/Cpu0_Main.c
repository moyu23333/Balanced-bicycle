/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ����Ⱥ��824575535
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file       		main
 * @company	   		�ɶ���ɿƼ����޹�˾
 * @author     		��ɿƼ�(QQ3184284598)
 * @version    		�鿴doc��version�ļ� �汾˵��
 * @Software 		ADS v1.2.2
 * @Target core		TC264D
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2020-3-23
 ********************************************************************************************************************/
#include "headfile.h"
#pragma section all "cpu0_dsram"
//���������#pragma section all restore���֮���ȫ�ֱ���������CPU0��RAM��


//���̵��뵽���֮��Ӧ��ѡ�й���Ȼ����refreshˢ��һ��֮���ٱ���
//����Ĭ������Ϊ�ر��Ż��������Լ��һ�����ѡ��properties->C/C++ Build->Setting
//Ȼ�����Ҳ�Ĵ������ҵ�C/C++ Compiler->Optimization->Optimization level�������Ż��ȼ�
//һ��Ĭ���½����Ĺ��̶���Ĭ�Ͽ�2���Ż�����˴��Ҳ��������Ϊ2���Ż�

//����TCϵ��Ĭ���ǲ�֧���ж�Ƕ�׵ģ�ϣ��֧���ж�Ƕ����Ҫ���ж���ʹ��enableInterrupts();�������ж�Ƕ��
//�򵥵�˵ʵ���Ͻ����жϺ�TCϵ�е�Ӳ���Զ�������disableInterrupts();���ܾ���Ӧ�κε��жϣ������Ҫ�����Լ��ֶ�����enableInterrupts();�������жϵ���Ӧ��
uint16 i=0;//���Ա���
//extern int16 xishu;
int core0_main(void)
{
	get_clk();//��ȡʱ��Ƶ��  ��ر���
	lcd_init();
	ADC_init();
	gpio_init(BEEP, GPO, 0, PUSHPULL);//��������ʼ��
	lcd_showstr(0,0,"ICM20602 init");//�����ǳ�ʼ��
	systick_delay_ms(STM0,500);
	lcd_showstr(0,0,"ICM20602 init.");
	systick_delay_ms(STM0,500);
	lcd_showstr(0,0,"ICM20602 init..");
	systick_delay_ms(STM0,500);
	lcd_showstr(0,0,"ICM20602 init...");
	icm20602_init_spi();
	icm_gyro_setint();//��ƮУ׼
	lcd_showstr(0,0,"ICM20602 ok  ");//��ʼ���ɹ�
	lcd_clear(RGB565_BLACK);
	//������ʼ��
	ALL_key_init();//������ʼ��
	Momentum_int();//�����ֳ�ʼ��
	sd12_init();//�����ʼ��
	systick_delay_ms(STM0,500);
	//���ֵ����ʼ��
	motor_init();
	//��ų�ʼ��
	adc_filter1_int();//�����˲���ʼ��
	Encorder_Init();  //��������ʼ��
	bluetooth_Init();//�������ڳ�ʼ��
	move_filter_init(&gyro_fliter);//�������˲���ʼ��
	pit_interrupt_ms(CCU6_0,PIT_CH0,2);//ֱ�������ж�
	pit_interrupt_ms(CCU6_0,PIT_CH1,5);//���ѭ���ж�
	IfxCpu_emitEvent(&g_cpuSyncEvent);
	IfxCpu_waitEvent(&g_cpuSyncEvent, 0xFFFF);
	enableInterrupts();
	while (TRUE)
	{
//        adc_mean_filter(ADC_0, ADC_MIDDLE_PIN, ADC_12BIT, 5);
//        adc_mean_filter(ADC_0, ADC_NEIBA_LEFT_PIN, ADC_12BIT, 5);
//        adc_mean_filter(ADC_0, ADC_NEIBA_RIGHT_PIN, ADC_12BIT, 5);
//            adc_result();
//            lcd_showuint16(0,0,ramp_adc);
//            lcd_showuint16(0,1,neiba_left_adc);
//            lcd_showuint16(0,2,neiba_right_adc);
//        lcd_showuint16(0,0,adc_mean_filter(ADC_0, ADC_MIDDLE_PIN, ADC_12BIT, 5));
//        lcd_showuint16(0,1,adc_mean_filter(ADC_0, ADC_NEIBA_LEFT_PIN, ADC_12BIT, 5));
//        lcd_showuint16(0,2,adc_mean_filter(ADC_0, ADC_NEIBA_RIGHT_PIN, ADC_12BIT, 5));
//        printf("adc1:%d\n","adc2:%d\n","adc3:%d\n",adc_mean_filter(ADC_0, ADC_MIDDLE_PIN, ADC_12BIT,5),adc_mean_filter(ADC_0, ADC_NEIBA_LEFT_PIN, ADC_12BIT, 5),adc_mean_filter(ADC_0, ADC_NEIBA_RIGHT_PIN, ADC_12BIT, 5));
//        printf("adc1:%d\n adc2:%d\n adc3:%d\n",adc_mean_filter(ADC_0, ADC_MIDDLE_PIN, ADC_12BIT,5),adc_mean_filter(ADC_0, ADC_NEIBA_LEFT_PIN, ADC_12BIT, 5),adc_mean_filter(ADC_0, ADC_NEIBA_RIGHT_PIN, ADC_12BIT, 5));

	     //��������ʱ�����------------------------------
//	        systick_start(STM1);
//	        Inductorcontrol();
//	        printf("Time:%ld\n",systick_getval_us(STM1));//���ʱ��
	     //--------------------------------------------------
	    //�ɼ�����-----------------------------
//	        ADC_test();
//	        systick_delay_ms(STM0,5);
//	        lcd_showuint16(0,i,ADC_test_value[i]);
//	        uart_send_senser_10(0,0,ADC_test_value[0], ADC_test_value[1], ADC_test_value[2], ADC_test_value[3],ADC_test_value[4],ADC_test_value[5],ADC_test_value[6],0);
//          uart_send_senser(ADC_test_value[0], ADC_test_value[1], ADC_test_value[2], ADC_test_value[3]);
	    //------------------------------------------
//	    pwm_duty(ATOM0_CH2_P21_4,3000);
//	    for (count=0;count<100; count++)
//	        {
//	            get_icm20602_gyro_spi();
//	            systick_delay_ms(STM0,100);
//	            tempval_buf[count]=icm_gyro_x;
//	        }
//	        track_scan();
         Main_Interface();
//	    lcd_showint32(0, 0, cheku_distance, 5);
//	    lcd_showfloat(0,0,cheku_distance,4,2);
//	    test_Interface();
//	    sd12_correct_lcd();

//---------------------------------------------------------------------------------------------------------------------------
//	      MomentumCtrl(4000);
//        hubu_set_Interface();
//        ��������----------------------------------------
//        scan_key();
//        switch(keyup)
//        {
//            case 1:
//                momentum_duty+=50;
//                break;
//            case 2:momentum_duty-=50;
//                break;
//            case 3:momentum_duty=0;
//                break;
//            case 4:
//                break;
//            default:break;
//        }
//        MomentumCtrl(momentum_duty);
//        printf("momentum_duty:%d\n",momentum_duty);

	      //   track_scan();

	}
}

#pragma section all restore


