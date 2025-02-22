/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：三群：824575535
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file       		main
 * @company	   		成都逐飞科技有限公司
 * @author     		逐飞科技(QQ3184284598)
 * @version    		查看doc内version文件 版本说明
 * @Software 		ADS v1.2.2
 * @Target core		TC264D
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2020-3-23
 ********************************************************************************************************************/
#include "headfile.h"
#pragma section all "cpu0_dsram"
//将本语句与#pragma section all restore语句之间的全局变量都放在CPU0的RAM中


//工程导入到软件之后，应该选中工程然后点击refresh刷新一下之后再编译
//工程默认设置为关闭优化，可以自己右击工程选择properties->C/C++ Build->Setting
//然后在右侧的窗口中找到C/C++ Compiler->Optimization->Optimization level处设置优化等级
//一般默认新建立的工程都会默认开2级优化，因此大家也可以设置为2级优化

//对于TC系列默认是不支持中断嵌套的，希望支持中断嵌套需要在中断内使用enableInterrupts();来开启中断嵌套
//简单点说实际上进入中断后TC系列的硬件自动调用了disableInterrupts();来拒绝响应任何的中断，因此需要我们自己手动调用enableInterrupts();来开启中断的响应。
uint16 i=0;//调试变量
//extern int16 xishu;
int core0_main(void)
{
	get_clk();//获取时钟频率  务必保留
	lcd_init();
	ADC_init();
	gpio_init(BEEP, GPO, 0, PUSHPULL);//蜂鸣器初始化
	lcd_showstr(0,0,"ICM20602 init");//陀螺仪初始化
	systick_delay_ms(STM0,500);
	lcd_showstr(0,0,"ICM20602 init.");
	systick_delay_ms(STM0,500);
	lcd_showstr(0,0,"ICM20602 init..");
	systick_delay_ms(STM0,500);
	lcd_showstr(0,0,"ICM20602 init...");
	icm20602_init_spi();
	icm_gyro_setint();//零飘校准
	lcd_showstr(0,0,"ICM20602 ok  ");//初始化成功
	lcd_clear(RGB565_BLACK);
	//按键初始化
	ALL_key_init();//按键初始化
	Momentum_int();//动量轮初始化
	sd12_init();//舵机初始化
	systick_delay_ms(STM0,500);
	//后轮电机初始化
	motor_init();
	//电磁初始化
	adc_filter1_int();//滑动滤波初始化
	Encorder_Init();  //编码器初始化
	bluetooth_Init();//蓝牙串口初始化
	move_filter_init(&gyro_fliter);//陀螺仪滤波初始化
	pit_interrupt_ms(CCU6_0,PIT_CH0,2);//直立控制中断
	pit_interrupt_ms(CCU6_0,PIT_CH1,5);//电磁循迹中断
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

	     //代码运行时间测试------------------------------
//	        systick_start(STM1);
//	        Inductorcontrol();
//	        printf("Time:%ld\n",systick_getval_us(STM1));//输出时间
	     //--------------------------------------------------
	    //采集测试-----------------------------
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
//        测量死区----------------------------------------
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


