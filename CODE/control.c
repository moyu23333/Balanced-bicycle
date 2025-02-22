/*
 * control.c
 *
 *  Created on: 2022年1月23日
 *      Author: 摸鱼
 */
#include "headfile.h"
#include "stdlib.h"
#include "stdint.h"
//后轮驱动-------------------------------------------
int16 real_speed=30;//闭环期望值
short  Motor_Bias, Motor_Last_Bias, Motor_Integration; // 后轮电机所用参数
int16 Motor_Kp=223,Motor_Ki=2;//后轮增量式PI参数
int16 ECPULSE2=0;//后轮编码器计数值
int16 rear_temp_pwm=0;
//电磁采集-----------------------------------------
//滑动滤波变量
move_filter_struct ADC_flitler[7];//电磁滤波结构体
float track_left_adc=0,track_right_adc=0,ramp_adc=0,neiba_left_adc=0,neiba_right_adc=0,shu_left_adc=0,shu_right_adc=0;//电感归一化值
uint16 ADC_test_value[]={0,0,0,0,0,0,0};//所有电磁转换一次数值
uint16 lastadc[]={0,0,0,0,0,0,0};//上次adc采集的值
uint16 adc_min[]={4095,4095,4095,4095,4095,4095,4095},adc_max[]={3910,3905,3898,3896,3870,3904,3902};//电感最大最小值
uint16 MagneticField=0;// 磁场整体强度
float adc_sum=0;//差比和的和
float adc_diff=0;  //差比和的差
float adc_error=0;//差比和输出结果
//电磁元素识别-------------------------------------------------
uint8 track_PID_flag=0;//循迹方式选择
uint8 Uphill_flag=0;//坡道
uint8 cheku_flag=0;//车库标志位
float cheku_distance=0;
uint32 element_time=0;
uint32 cheku_time=0;
//环岛
uint8 huandao_flag=0,flag_right_round=0,flag_left_round=0,huandao_gyro_flag=0;//环岛标志位
float huandao_gyro_distance=0;//环岛积分
uint8 CircleNumber = 1;   // 入环次数，0结束；默认1次
uint8 TangentPoint = 1;   // 切点判断   0切点结束；默认1可以入环，读取脉冲为入环准备
uint8 EnterCircle = 0;    // 允许进环  默认 0不可进环；1可以进环
uint8 OutCircle = 0;      // 允许出环   默认0不可出环；1可以出环
uint8 flag_Left_round=0,flag_Right_round=0;//左侧环还是右侧环 默认0原始；1左环；2右环
uint8 chakou_ON=0,sanchakou_flag=0;//岔口
uint8 huan_flag=0;//
uint8 LoopFlag=0;
int32 Uphill_distance=0;

//舵机---------------------------------------------
float steer_duty;//舵机信号
/*------------------------------------------------
 * 电磁初始化
 -------------------------------------------------*/
void ADC_init(void)
{
    adc_init(ADC_0, ADC_LEFT_PIN);  //循迹左
    adc_init(ADC_0, ADC_RIGHT_PIN); //循迹右
    adc_init(ADC_0,ADC_MIDDLE_PIN);//坡道中
    adc_init(ADC_0, ADC_NEIBA_LEFT_PIN);         //内八左
    adc_init(ADC_0, ADC_NEIBA_RIGHT_PIN);         //内八右
    adc_init(ADC_0, ADC_SHU_LEFT_PIN);         //竖左
    adc_init(ADC_0, ADC_SHU_RIGHT_PIN);         //竖右
}
/*----------------------------------------------------
 * 所有电磁信号转换一次
 -----------------------------------------------------*/
void ADC_test(void)
{
    ADC_test_value[0]  = adc_convert(ADC_0, ADC_LEFT_PIN, ADC_12BIT);//循迹左
    ADC_test_value[1]  = adc_convert(ADC_0, ADC_RIGHT_PIN, ADC_12BIT);//循迹右
    ADC_test_value[2]  = adc_convert(ADC_0, ADC_MIDDLE_PIN, ADC_12BIT);//坡道中间电感
    ADC_test_value[3]  = adc_convert(ADC_0, ADC_NEIBA_LEFT_PIN, ADC_12BIT);//内八左
    ADC_test_value[4]  = adc_convert(ADC_0, ADC_NEIBA_RIGHT_PIN, ADC_12BIT);//内八右
    ADC_test_value[5]  = adc_convert(ADC_0, ADC_SHU_LEFT_PIN, ADC_12BIT);//外八左
    ADC_test_value[6]  = adc_convert(ADC_0, ADC_SHU_RIGHT_PIN, ADC_12BIT);//外八右
}

/*--------------------------------------------------------------------------------
 * 电磁赛道扫描程序
 ---------------------------------------------------------------------------------*/
void track_scan(void)
{
    uint8 i;
    lcd_clear(RGB565_WHITE);
    for(i=0;i<7;i++)adc_max[i]=0;
    while(TRUE)
    {
//        adc_filter1();
        for(i=0;i<7;i++)
        {
        if(ADC_flitler[i].data_average>adc_max[i])
            adc_max[i]=ADC_flitler[i].data_average;
    //    if(ADC_flitler[i].data_average<adc_min[i]&&ADC_flitler[i].data_average>80)
    //        adc_min[i]=ADC_flitler[i].data_average;
        lcd_showuint16(0,i,adc_max[i]);
        }
        scan_key();
        if(keyup==4)return;
    }

}
/*-----------------------------------------------
 * 原电感限幅
 * 4000后期用adc_max[i]+100代替
 ----------------------------------------------*/
void adc_limit(void)
{
    uint8 i=0;
    ADC_test();
    for(i=0;i<7;i++)
    {
    if(ADC_test_value[i]>adc_max[i]+100)ADC_test_value[i] = lastadc[i];//数据更新
    }
}
/*----------------------------------------------------------
 * 电磁滤波初始化
 ----------------------------------------------------------*/
void adc_filter1_int(void)
{
    uint8 i=0;
    for(i=0;i<7;i++)
    {
        move_filter_init(&ADC_flitler[i]);
    }

}
/*---------------------------------------------------
 * 滑动平均限幅滤波
 *
 -----------------------------------------------------*/
void adc_filter1(void)
{
    uint8 i=0;
//    adc_limit();//限幅
    ADC_test();
    for(i=0;i<7;i++)
    {
        move_filter_calc(&ADC_flitler[i],ADC_test_value[i]);
        lastadc[i]=ADC_flitler[i].data_average;
    }
}
/*-------------------------------------------------------------------
 * 差比和算法
 -------------------------------------------------------------------*/
float Diff_ratio_sum(float dat1,float dat2)
{
    static float diff=0,sum=0;
    float err_out=0;
    sum=dat1+dat2;
    diff=dat1-dat2;
    err_out=diff*100/(sum+1);
    return err_out;
}
/*------------------------------------------------------------------------------
 * 采集信号后进行归一化并计算差值
 * 先用差比和处理数据，如果效果不好尝试 “差比和加权和差比和差加权” 算法
 * 边界保护
 *有待改进
------------------------------------------------------------------------------------*/
void adc_result(void)
{
    adc_filter1();//滤波
    track_left_adc    = (float)ADC_flitler[0].data_average *100 / (adc_max[0]+30);// 各偏移量归一化到0--100以内 adc_max[0]+30
    track_right_adc   = (float)ADC_flitler[1].data_average  * 100 / (adc_max[1]+30);
    ramp_adc          = (float)ADC_flitler[2].data_average * 100 / (adc_max[2]+30);
    neiba_left_adc    = (float)ADC_flitler[3].data_average  * 100 / (adc_max[3]+30) ;
    neiba_right_adc   = (float)ADC_flitler[4].data_average  * 100 / (adc_max[4]+30);
    shu_left_adc      = (float)ADC_flitler[5].data_average * 100 / (adc_max[5]+30) ;
    shu_right_adc     = (float)ADC_flitler[6].data_average  * 100 / (adc_max[6]+30);
//    MagneticField = track_left_adc + track_right_adc + ramp_adc + neiba_left_adc;
//    差比和处理
//    adc_sum=track_left_adc*0.7+neiba_left_adc*0.3+track_right_adc*0.7+neiba_right_adc*0.3;
//    adc_diff=track_left_adc*0.7+neiba_left_adc*0.3-track_right_adc*0.7+neiba_right_adc*0.3;
    adc_sum=track_left_adc+track_right_adc;
    adc_diff=track_left_adc-track_right_adc;
    adc_error=adc_diff*100/(adc_sum+1);//偏差
//    if(adc_sum>0)//边界保护 //0需要自己设定
//
//            {
//                adc_error = (adc_diff<<7)/(adc_sum+1); //计算差比和数值 并放大128倍
//            }
}


/*-------------------------------------------------------------------------------
 * 电磁PID算法
 *尽量将kp ki kd改为uint类型
 ----------------------------------------------------------------------------------*/
void Inductorcontrol(void)
{
    adc_result();//数据处理
    switch((uint8)test_temp_num[2])
    {
        case 0:
            electr_track_PID.error=adc_error;//普通循迹
            break;
        case 1:
            electr_track_PID.error=Diff_ratio_sum(ramp_adc,track_right_adc)-test_temp_num[0];//岔口右
            break;
        case 2:
            electr_track_PID.error=Diff_ratio_sum(ramp_adc,track_left_adc)-test_temp_num[1];//岔口左
            break;
        case 3:
            electr_track_PID.error=Diff_ratio_sum(neiba_left_adc,neiba_right_adc);//右环岛
            break;
        case 4:
            electr_track_PID.error=Diff_ratio_sum(neiba_right_adc,neiba_left_adc);//左环岛
            break;
        case 5:
            electr_track_PID.error=Diff_ratio_sum(shu_left_adc,shu_right_adc);//竖电感循迹
            break;
        case 6:
            electr_track_PID.error=Diff_ratio_sum(track_left_adc+ramp_adc,track_left_adc+ramp_adc);//添加中间电感的循迹差比和
            break;
        default:break;
    }
    electr_track_PID.integral+=electr_track_PID.error;
    electr_track_PID.d_error=electr_track_PID.error-electr_track_PID.last_error;
    electr_track_PID.out=electr_track_PID.Kp*electr_track_PID.error+electr_track_PID.Ki*electr_track_PID.integral+electr_track_PID.Kd*electr_track_PID.d_error;
//    if(electr_track_PID.out>300)electr_track_PID.out=300;
//    else if(electr_track_PID.out<-300)electr_track_PID.out=300;
    electr_track_PID.last_error=electr_track_PID.error;
    //死区
//    if(electr_track_PID.out<-5)electr_track_PID.out-=5;
//    else if(electr_track_PID.out>5)electr_track_PID.out+=5;
//    steer_duty=STEER_MOTOR_MID+electr_track_PID.out;


//    real_speed=40-abs(electr_track_PID.out)*0.25;
//    if(real_speed<15)real_speed=15;//限速
//    dyn_zero(electr_track_PID.out,0);
}
/*-----------------------------------------------------------------------
 * \               /    \               /
 *  \             /      \             /
 *   \           /        \           /
 *    \ _______ /          \ _______ /
 * 电感元素处理
 * 坡道：
 * 方案一：坡道识别还需添加电感特征（添加中间电感识别似乎比较显著）
 * 方案二：用陀螺仪解算俯仰角识别坡道过程（用互补滤波效果应该不错）
 *         跟电磁识别坡道上有延迟，可能无法造成坡道环岛误判
 * 三岔口：内八电感识别
 * 环岛：根据整体磁场变化识别环岛位置一
 *       编码器记步+陀螺仪判断是否为坡道
 *       编码器计步到入环切点判断条件是否能够入环
 *       循迹入环：1、电磁循迹（似乎有点难）2、摄像头循迹
 *       电磁识别+编码器计步判断出环时机
 *       入环标记+编码器计步+陀螺仪积分通过环岛
 ---------------------------------------------------------------------*/
void Inductiveprocess(void)
{
    //十字

    //上坡标记

//      if(Uphill_flag==0&&icm_gyro_y>100)
//      {
//          flag_roundabout=0;
//          Uphill_flag++;        //上坡标记
//          Up_distance=0;//编码器
//      }
//      //上坡中
//      else if(Uphill_flag==1&&Up_distance>Uphill_distance)
//      {
//          Uphill_flag=0;
//          Up_distance=0; //计步清零
//      }
//
//   //环岛
//    if(Uphill_flag==0&&huandao_flag)
//    {
//          if(flag_roundabout==0 && MagneticField>4900)
//          {
//              flag_roundabout=1;               //电感第一次翻倍  入环标志位   1
//              chakou_ON=0;
//          }
//          else if(flag_roundabout==1&&MagneticField<4900)
//          {
//              flag_roundabout=2;                //电感正常以后   2    代表现在在环岛里面走
//          }
//          else if(flag_roundabout==2 &&MagneticField>4900)
//          {
//              flag_roundabout=3;                   //电感再次翻倍   3    出环标志位
//              if(flag_Left_round==3)
//                  flag_Left_round=4;
//              if(flag_Right_round==3)
//                 flag_Right_round=4;
//          }
//          else if(flag_roundabout==1)
//          {
//              flag_roundabout=4;           //  电感再次正常   4    环岛结束
//          }
//    }

    //三岔口
//           MagneticField=track_left_adc+track_right_adc+ramp_adc;
           if(sanchakou_flag==0&&ramp_adc<66&&ramp_adc>56&&shu_left_adc>10&&shu_right_adc>17&&neiba_left_adc<40&&neiba_left_adc>34&&neiba_right_adc<41&&neiba_right_adc>36.5)  //入岔口
           {
                   BEEP_ON//蜂鸣器响
        //           steer_duty=STEER_MOTOR_MID+20;
                   track_PID_flag=1;//岔口右
                   electr_track_PID.Kp=track_fenduan[0][1];
                   electr_track_PID.Kd=track_fenduan[1][1];
                   sanchakou_flag=1;

           }
           else if(sanchakou_flag==1&&ramp_adc<34&&neiba_left_adc>28.4&&neiba_right_adc<13&&adc_error>4)//岔口中
           {
               BEEP_OFF
               track_PID_flag=0;
               electr_track_PID.Kp=track_fenduan[0][0];
               electr_track_PID.Kd=track_fenduan[1][0];
               sanchakou_flag=2;
           }
           else if(sanchakou_flag==2&&(track_right_adc-track_left_adc)>24&&neiba_right_adc>69&&shu_left_adc>52&&shu_right_adc>54)
           {
               BEEP_ON
               sanchakou_flag=3;
               cheku_flag=1;
           }
    //       else if(sanchakou_flag==1&&track_left_adc<19&&track_left_adc>14&&track_right_adc<38&&track_right_adc>21.6&&ramp_adc<20&&ramp_adc>15)//出岔口 切换为正常循迹
    //       {
    //           BEEP_OFF
    //           track_PID_flag=0;//正常循迹
    //           electr_track_PID.Kp=track_fenduan[0];
    //           sanchakou_flag=2;
    //       }
//           else if(sanchakou_flag==2&&track_left_adc<36&&track_left_adc>31.7&&track_right_adc<39.8&&track_right_adc>34&&ramp_adc<30.6&&ramp_adc>21.6&&shu_left_adc<16&&shu_left_adc>13.3&&shu_right_adc<5.4&&shu_right_adc>3.3)//第二次入
//           {
//               BEEP_ON//蜂鸣器响
//               track_PID_flag=2;//左循迹P
//               electr_track_PID.Kp=track_fenduan[0][2];
//               electr_track_PID.Kd=track_fenduan[1][2];
//               sanchakou_flag=3;
//           }
//           else if(sanchakou_flag==3&&ramp_adc<18.5&&ramp_adc>16&&neiba_left_adc<18.6&&neiba_right_adc>17.3&&shu_left_adc>2&&shu_right_adc>2.8)//岔口中
//           {
//               BEEP_OFF//蜂鸣器响
//               track_PID_flag=0;//正常循迹
//               electr_track_PID.Kp=track_fenduan[0][0];
//               electr_track_PID.Kd=track_fenduan[1][0];
//               sanchakou_flag=4;
//           }

// //车库
           else if(sanchakou_flag==3&&cheku_flag==1&&cheku_distance<-108)//入车库
           {
               //延时完成入库
//               cheku_time++;

//               if(cheku_time<1000)
//               {
                  steering_control(STEER_MOTOR_MID+56);
//               }
                  systick_delay_ms(STM0,5000);
//               else if(cheku_time>1000)
//               {
                   BEEP_OFF
                   cheku_distance=0;
                   real_speed=0;//后轮停转
                   cheku_flag=2;
//               }

           }
 //环岛
           //右环岛
//           if(huandao_flag==0&&MagneticField>180&&shu_left_adc>10&&shu_right_adc>27&&CircleNumber<4)//右环岛？
//           {
//               huandao_flag=1;
//               flag_right_round=1;
//           }
//           else if(huandao_flag==1&&flag_right_round==1&&MagneticField>280&&shu_right_adc<10&&shu_left_adc<4.6&&neiba_left_adc>70)//真的右环岛？
//           {
//               BEEP_ON
//               huandao_flag=2;
//               track_PID_flag=3;
//               huandao_gyro_flag=1;//开始积分
//           }
//           else if(huandao_flag==2&&flag_right_round==2&&huandao_gyro_distance>20&&MagneticField>28)//出环岛，切换为正常循迹
//           {
//               BEEP_OFF
//               track_PID_flag=0;//切换为正常循迹
//               flag_right_round=0;
//               huandao_flag=3;
//           }
//           else if(huandao_flag==3&&huandao_gyro_distance>20&&MagneticField<20)//确实出环
//           {
//               huandao_gyro_flag=0;//积分结束
//               huandao_gyro_distance=0;//积分清零
//               CircleNumber++;//过环次数加一
//               huandao_flag=0;
//           }

           //左环岛
//           if(huandao_flag==0&&MagneticField>291&&shu_left_adc>72&&neiba_right_adc>82&&CircleNumber<4)//左环岛？
//           {
//               huandao_flag=1;
//               flag_left_round=1;
//           }
//           else if(huandao_flag==1&&flag_left_round==1&&MagneticField>295&&shu_left_adc>38.5&&neiba_left_adc>88)//真的左环岛？
//           {
//               BEEP_ON
//               huandao_flag=2;
//               track_PID_flag=5;
//               huandao_gyro_flag=1;//开始积分
//           }
//           else if(huandao_flag==2&&flag_left_round==2&&huandao_gyro_distance>20&&ramp_adc>92&&shu_right_adc>92)//出环岛，切换为正常循迹
//           {
//                   track_PID_flag=0;//切换为正常循迹
//                   flag_left_round=0;
//                   huandao_flag=3;
//           }
//           else if(huandao_flag==3&&huandao_gyro_distance>20&&MagneticField<20)//确实出环
//           {
//               BEEP_OFF
//               huandao_gyro_flag=0;//积分结束
//               huandao_gyro_distance=0;//积分清零
//               CircleNumber++;//过环次数加一
//               huandao_flag=0;
//           }
}

/*-------------------------------------
 * 舵机初始化
 --------------------------------------*/
void sd12_init(void)
{
    lcd_showstr(0,0,"sd12 int");
    steer_duty = STEER_MOTOR_MID;//舵机回正中间位置
    gtm_pwm_init(S_MOTOR_PIN, 50, steer_duty);
    systick_delay_ms(STM0,500);
    lcd_showstr(0,0,"sd12 int ok");
    systick_delay_ms(STM0,500);
    lcd_clear(RGB565_BLACK);
    }

/*---------------------------------
 * 舵机打角校准
 ---------------------------------*/
void sd12_correct(void)
{
    if(key_check(KEY0)==0)                        //按键0控制舵机向左转
            {
                steer_duty+=2;
                if((1250-20) < steer_duty) steer_duty = 270;//限位
                if((250+20) >= steer_duty)
                printf("steer_duty: %f\n",steer_duty);
                pwm_duty(S_MOTOR_PIN,steer_duty);
                systick_delay_ms(STM0, 1);
            }
            if(key_check(KEY1)==0)              //按键1控制舵机向右转
            {
                steer_duty-=2;
                if((250+20) > steer_duty) steer_duty = 270;//限位
                if((250+20) >= steer_duty)
                systick_delay_ms(STM0, 1000);
                printf("steer_duty: %f\n",steer_duty);
                pwm_duty(S_MOTOR_PIN,steer_duty);
                systick_delay_ms(STM0, 1);
            }
            if(key_check(KEY2)==0)             //使舵机回正
            {
                steer_duty = STEER_MOTOR_MID;//舵机回正
                pwm_duty(S_MOTOR_PIN,steer_duty);
                systick_delay_ms(STM0, 1000);
            }
    }
/*---------------------------------
 * 舵机打角校准 显示屏显示
 ---------------------------------*/
void sd12_correct_lcd(void)
{
    lcd_showstr(0,1,"sd12 angle: ");
    while(1)
    {
    if(key_check(KEY0)==0)                        //按键0控制舵机向左转
            {
                steer_duty+=2;
                if((1250-20) < steer_duty) steer_duty = 1230;//限位
                pwm_duty(S_MOTOR_PIN,steer_duty);
                lcd_showfloat(90,1,steer_duty,5,1);
                systick_delay_ms(STM0, 1);
            }
      if(key_check(KEY1)==0)              //按键1控制舵机向右转
            {
                steer_duty-=2;
                if((250+20) > steer_duty) steer_duty=270; //限位
               pwm_duty(S_MOTOR_PIN,steer_duty);
               lcd_showfloat(90,1,steer_duty,5,1);
                systick_delay_ms(STM0, 1);
            }
        if(key_check(KEY2)==0)             //使舵机回正
            {
                steer_duty = STEER_MOTOR_MID;//舵机回正
                lcd_showstr(0,1,"sd12 rest ");
                pwm_duty(S_MOTOR_PIN,steer_duty);
                systick_delay_ms(STM0, 500);
                lcd_showstr(0,1,"sd12 angle: ");
                lcd_showfloat(90,1,steer_duty,5,1);
            }
        if(key_check(KEY3)==0)
            return;
    }
    }
/*-------------------------------------------------
 * 舵机打角控制
 -------------------------------------------------*/
void steering_control(uint16 steerpwm)
{
//dyn_zero(steerpwm,0);//动态零点补偿
//限位
if(abs(steerpwm)<=270)steerpwm=270;
if(abs(steerpwm)>=538)steerpwm=538;
pwm_duty(S_MOTOR_PIN,steerpwm);
}

/*----------------------------------------------------
 * 舵机控制平衡PID
 -----------------------------------------------------*/
void steering_balance(float Angle,float Gyro,float expect_angle)
{
   steering_PID.error=Angle+expect_angle;
   steering_PID.integral+=steering_PID.error;
   steering_PID.integral=limit_ab(steering_PID.integral, -steering_PID.I_L, steering_PID.I_L);
   steering_PID.out=steering_PID.Kp*steering_PID.error+steering_PID.Ki*steering_PID.integral+steering_PID.Kd*Gyro;
//   steering_PID.last_error=steering_PID.error;
//   steering_PID.d_error=steering_PID.error-steering_PID.last_error;
   //死区
//   if(steering_PID.out>-5)steering_PID.out=0;
//   else if(steering_PID.out<5)steering_PID.out=0;
//   steering_control(STEER_MOTOR_MID-steering_PID.out);
}

//车轮电机----------------------------------------------

/*----------------------------------------------------
 * 后驱电机初始化
 ------------------------------------------------------*/
void motor_init(void)
{
    gpio_init(REAR_MOTOR_DIR, GPO, 0, PUSHPULL);
    gtm_pwm_init(REAR_MOTOR_PWM, 17000, 0);
}
/*-----------------------------------------------------
 * 控制电机转动
 ------------------------------------------------------*/
void motor_control(int16 pwm)
{
    if (pwm > 0)
    {
        gpio_set(REAR_MOTOR_DIR, 1);
        pwm_duty(REAR_MOTOR_PWM, pwm);
    }
    else
    {   gpio_set(REAR_MOTOR_DIR, 0);
        pwm_duty(REAR_MOTOR_PWM, (-pwm));
    }

}
/***************************************************************************
【函数名】int Motor_PI (int Encoder,int Target)
【功  能】电机控制增量式PI
【参数值】int Encoder 编码器脉采集的冲数
【参数值】int Target  期望脉冲数
【返回值】电机PWM
【作  者】chiusir
【最后更新】2021年1月22日
【软件版本】V1.0
**********************************************************************************/
int16 Rear_Motor_PI (int16 Encoder,int16 Target)
{
    static int16 Rear_Motor_Pwm;
    Motor_Bias = Encoder - Target;            // 计算偏差
    Rear_Motor_Pwm += Motor_Kp * (Motor_Bias - Motor_Last_Bias) + Motor_Ki * Motor_Bias;
    // ==增量式PI控制器
    if(Rear_Motor_Pwm > 5000) Rear_Motor_Pwm = 5000;               // 限幅
    else if(Rear_Motor_Pwm < -5000)Rear_Motor_Pwm = -5000;         // 限幅
    Motor_Last_Bias = Motor_Bias;            // 保存上一次偏差
    return Rear_Motor_Pwm;                              // 增量输出
}

void Rear_Motor_Control(int16 Target)
{
    ECPULSE2=Encorder_get(GPT12_T2);
    ECPULSE2=Low_pass_filter(ECPULSE2);//低通滤波
    motor_control(Rear_Motor_PI(ECPULSE2,Target));//闭环控制
}
