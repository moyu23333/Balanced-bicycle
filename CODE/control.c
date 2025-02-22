/*
 * control.c
 *
 *  Created on: 2022��1��23��
 *      Author: ����
 */
#include "headfile.h"
#include "stdlib.h"
#include "stdint.h"
//��������-------------------------------------------
int16 real_speed=30;//�ջ�����ֵ
short  Motor_Bias, Motor_Last_Bias, Motor_Integration; // ���ֵ�����ò���
int16 Motor_Kp=223,Motor_Ki=2;//��������ʽPI����
int16 ECPULSE2=0;//���ֱ���������ֵ
int16 rear_temp_pwm=0;
//��Ųɼ�-----------------------------------------
//�����˲�����
move_filter_struct ADC_flitler[7];//����˲��ṹ��
float track_left_adc=0,track_right_adc=0,ramp_adc=0,neiba_left_adc=0,neiba_right_adc=0,shu_left_adc=0,shu_right_adc=0;//��й�һ��ֵ
uint16 ADC_test_value[]={0,0,0,0,0,0,0};//���е��ת��һ����ֵ
uint16 lastadc[]={0,0,0,0,0,0,0};//�ϴ�adc�ɼ���ֵ
uint16 adc_min[]={4095,4095,4095,4095,4095,4095,4095},adc_max[]={3910,3905,3898,3896,3870,3904,3902};//��������Сֵ
uint16 MagneticField=0;// �ų�����ǿ��
float adc_sum=0;//��Ⱥ͵ĺ�
float adc_diff=0;  //��Ⱥ͵Ĳ�
float adc_error=0;//��Ⱥ�������
//���Ԫ��ʶ��-------------------------------------------------
uint8 track_PID_flag=0;//ѭ����ʽѡ��
uint8 Uphill_flag=0;//�µ�
uint8 cheku_flag=0;//�����־λ
float cheku_distance=0;
uint32 element_time=0;
uint32 cheku_time=0;
//����
uint8 huandao_flag=0,flag_right_round=0,flag_left_round=0,huandao_gyro_flag=0;//������־λ
float huandao_gyro_distance=0;//��������
uint8 CircleNumber = 1;   // �뻷������0������Ĭ��1��
uint8 TangentPoint = 1;   // �е��ж�   0�е������Ĭ��1�����뻷����ȡ����Ϊ�뻷׼��
uint8 EnterCircle = 0;    // �������  Ĭ�� 0���ɽ�����1���Խ���
uint8 OutCircle = 0;      // �������   Ĭ��0���ɳ�����1���Գ���
uint8 flag_Left_round=0,flag_Right_round=0;//��໷�����Ҳ໷ Ĭ��0ԭʼ��1�󻷣�2�һ�
uint8 chakou_ON=0,sanchakou_flag=0;//���
uint8 huan_flag=0;//
uint8 LoopFlag=0;
int32 Uphill_distance=0;

//���---------------------------------------------
float steer_duty;//����ź�
/*------------------------------------------------
 * ��ų�ʼ��
 -------------------------------------------------*/
void ADC_init(void)
{
    adc_init(ADC_0, ADC_LEFT_PIN);  //ѭ����
    adc_init(ADC_0, ADC_RIGHT_PIN); //ѭ����
    adc_init(ADC_0,ADC_MIDDLE_PIN);//�µ���
    adc_init(ADC_0, ADC_NEIBA_LEFT_PIN);         //�ڰ���
    adc_init(ADC_0, ADC_NEIBA_RIGHT_PIN);         //�ڰ���
    adc_init(ADC_0, ADC_SHU_LEFT_PIN);         //����
    adc_init(ADC_0, ADC_SHU_RIGHT_PIN);         //����
}
/*----------------------------------------------------
 * ���е���ź�ת��һ��
 -----------------------------------------------------*/
void ADC_test(void)
{
    ADC_test_value[0]  = adc_convert(ADC_0, ADC_LEFT_PIN, ADC_12BIT);//ѭ����
    ADC_test_value[1]  = adc_convert(ADC_0, ADC_RIGHT_PIN, ADC_12BIT);//ѭ����
    ADC_test_value[2]  = adc_convert(ADC_0, ADC_MIDDLE_PIN, ADC_12BIT);//�µ��м���
    ADC_test_value[3]  = adc_convert(ADC_0, ADC_NEIBA_LEFT_PIN, ADC_12BIT);//�ڰ���
    ADC_test_value[4]  = adc_convert(ADC_0, ADC_NEIBA_RIGHT_PIN, ADC_12BIT);//�ڰ���
    ADC_test_value[5]  = adc_convert(ADC_0, ADC_SHU_LEFT_PIN, ADC_12BIT);//�����
    ADC_test_value[6]  = adc_convert(ADC_0, ADC_SHU_RIGHT_PIN, ADC_12BIT);//�����
}

/*--------------------------------------------------------------------------------
 * �������ɨ�����
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
 * ԭ����޷�
 * 4000������adc_max[i]+100����
 ----------------------------------------------*/
void adc_limit(void)
{
    uint8 i=0;
    ADC_test();
    for(i=0;i<7;i++)
    {
    if(ADC_test_value[i]>adc_max[i]+100)ADC_test_value[i] = lastadc[i];//���ݸ���
    }
}
/*----------------------------------------------------------
 * ����˲���ʼ��
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
 * ����ƽ���޷��˲�
 *
 -----------------------------------------------------*/
void adc_filter1(void)
{
    uint8 i=0;
//    adc_limit();//�޷�
    ADC_test();
    for(i=0;i<7;i++)
    {
        move_filter_calc(&ADC_flitler[i],ADC_test_value[i]);
        lastadc[i]=ADC_flitler[i].data_average;
    }
}
/*-------------------------------------------------------------------
 * ��Ⱥ��㷨
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
 * �ɼ��źź���й�һ���������ֵ
 * ���ò�Ⱥʹ������ݣ����Ч�����ó��� ����Ⱥͼ�Ȩ�Ͳ�ȺͲ��Ȩ�� �㷨
 * �߽籣��
 *�д��Ľ�
------------------------------------------------------------------------------------*/
void adc_result(void)
{
    adc_filter1();//�˲�
    track_left_adc    = (float)ADC_flitler[0].data_average *100 / (adc_max[0]+30);// ��ƫ������һ����0--100���� adc_max[0]+30
    track_right_adc   = (float)ADC_flitler[1].data_average  * 100 / (adc_max[1]+30);
    ramp_adc          = (float)ADC_flitler[2].data_average * 100 / (adc_max[2]+30);
    neiba_left_adc    = (float)ADC_flitler[3].data_average  * 100 / (adc_max[3]+30) ;
    neiba_right_adc   = (float)ADC_flitler[4].data_average  * 100 / (adc_max[4]+30);
    shu_left_adc      = (float)ADC_flitler[5].data_average * 100 / (adc_max[5]+30) ;
    shu_right_adc     = (float)ADC_flitler[6].data_average  * 100 / (adc_max[6]+30);
//    MagneticField = track_left_adc + track_right_adc + ramp_adc + neiba_left_adc;
//    ��Ⱥʹ���
//    adc_sum=track_left_adc*0.7+neiba_left_adc*0.3+track_right_adc*0.7+neiba_right_adc*0.3;
//    adc_diff=track_left_adc*0.7+neiba_left_adc*0.3-track_right_adc*0.7+neiba_right_adc*0.3;
    adc_sum=track_left_adc+track_right_adc;
    adc_diff=track_left_adc-track_right_adc;
    adc_error=adc_diff*100/(adc_sum+1);//ƫ��
//    if(adc_sum>0)//�߽籣�� //0��Ҫ�Լ��趨
//
//            {
//                adc_error = (adc_diff<<7)/(adc_sum+1); //�����Ⱥ���ֵ ���Ŵ�128��
//            }
}


/*-------------------------------------------------------------------------------
 * ���PID�㷨
 *������kp ki kd��Ϊuint����
 ----------------------------------------------------------------------------------*/
void Inductorcontrol(void)
{
    adc_result();//���ݴ���
    switch((uint8)test_temp_num[2])
    {
        case 0:
            electr_track_PID.error=adc_error;//��ͨѭ��
            break;
        case 1:
            electr_track_PID.error=Diff_ratio_sum(ramp_adc,track_right_adc)-test_temp_num[0];//�����
            break;
        case 2:
            electr_track_PID.error=Diff_ratio_sum(ramp_adc,track_left_adc)-test_temp_num[1];//�����
            break;
        case 3:
            electr_track_PID.error=Diff_ratio_sum(neiba_left_adc,neiba_right_adc);//�һ���
            break;
        case 4:
            electr_track_PID.error=Diff_ratio_sum(neiba_right_adc,neiba_left_adc);//�󻷵�
            break;
        case 5:
            electr_track_PID.error=Diff_ratio_sum(shu_left_adc,shu_right_adc);//�����ѭ��
            break;
        case 6:
            electr_track_PID.error=Diff_ratio_sum(track_left_adc+ramp_adc,track_left_adc+ramp_adc);//����м��е�ѭ����Ⱥ�
            break;
        default:break;
    }
    electr_track_PID.integral+=electr_track_PID.error;
    electr_track_PID.d_error=electr_track_PID.error-electr_track_PID.last_error;
    electr_track_PID.out=electr_track_PID.Kp*electr_track_PID.error+electr_track_PID.Ki*electr_track_PID.integral+electr_track_PID.Kd*electr_track_PID.d_error;
//    if(electr_track_PID.out>300)electr_track_PID.out=300;
//    else if(electr_track_PID.out<-300)electr_track_PID.out=300;
    electr_track_PID.last_error=electr_track_PID.error;
    //����
//    if(electr_track_PID.out<-5)electr_track_PID.out-=5;
//    else if(electr_track_PID.out>5)electr_track_PID.out+=5;
//    steer_duty=STEER_MOTOR_MID+electr_track_PID.out;


//    real_speed=40-abs(electr_track_PID.out)*0.25;
//    if(real_speed<15)real_speed=15;//����
//    dyn_zero(electr_track_PID.out,0);
}
/*-----------------------------------------------------------------------
 * \               /    \               /
 *  \             /      \             /
 *   \           /        \           /
 *    \ _______ /          \ _______ /
 * ���Ԫ�ش���
 * �µ���
 * ����һ���µ�ʶ������ӵ������������м���ʶ���ƺ��Ƚ�������
 * ���������������ǽ��㸩����ʶ���µ����̣��û����˲�Ч��Ӧ�ò���
 *         �����ʶ���µ������ӳ٣������޷�����µ���������
 * ����ڣ��ڰ˵��ʶ��
 * ��������������ų��仯ʶ�𻷵�λ��һ
 *       �������ǲ�+�������ж��Ƿ�Ϊ�µ�
 *       �������Ʋ����뻷�е��ж������Ƿ��ܹ��뻷
 *       ѭ���뻷��1�����ѭ�����ƺ��е��ѣ�2������ͷѭ��
 *       ���ʶ��+�������Ʋ��жϳ���ʱ��
 *       �뻷���+�������Ʋ�+�����ǻ���ͨ������
 ---------------------------------------------------------------------*/
void Inductiveprocess(void)
{
    //ʮ��

    //���±��

//      if(Uphill_flag==0&&icm_gyro_y>100)
//      {
//          flag_roundabout=0;
//          Uphill_flag++;        //���±��
//          Up_distance=0;//������
//      }
//      //������
//      else if(Uphill_flag==1&&Up_distance>Uphill_distance)
//      {
//          Uphill_flag=0;
//          Up_distance=0; //�Ʋ�����
//      }
//
//   //����
//    if(Uphill_flag==0&&huandao_flag)
//    {
//          if(flag_roundabout==0 && MagneticField>4900)
//          {
//              flag_roundabout=1;               //��е�һ�η���  �뻷��־λ   1
//              chakou_ON=0;
//          }
//          else if(flag_roundabout==1&&MagneticField<4900)
//          {
//              flag_roundabout=2;                //��������Ժ�   2    ���������ڻ���������
//          }
//          else if(flag_roundabout==2 &&MagneticField>4900)
//          {
//              flag_roundabout=3;                   //����ٴη���   3    ������־λ
//              if(flag_Left_round==3)
//                  flag_Left_round=4;
//              if(flag_Right_round==3)
//                 flag_Right_round=4;
//          }
//          else if(flag_roundabout==1)
//          {
//              flag_roundabout=4;           //  ����ٴ�����   4    ��������
//          }
//    }

    //�����
//           MagneticField=track_left_adc+track_right_adc+ramp_adc;
           if(sanchakou_flag==0&&ramp_adc<66&&ramp_adc>56&&shu_left_adc>10&&shu_right_adc>17&&neiba_left_adc<40&&neiba_left_adc>34&&neiba_right_adc<41&&neiba_right_adc>36.5)  //����
           {
                   BEEP_ON//��������
        //           steer_duty=STEER_MOTOR_MID+20;
                   track_PID_flag=1;//�����
                   electr_track_PID.Kp=track_fenduan[0][1];
                   electr_track_PID.Kd=track_fenduan[1][1];
                   sanchakou_flag=1;

           }
           else if(sanchakou_flag==1&&ramp_adc<34&&neiba_left_adc>28.4&&neiba_right_adc<13&&adc_error>4)//�����
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
    //       else if(sanchakou_flag==1&&track_left_adc<19&&track_left_adc>14&&track_right_adc<38&&track_right_adc>21.6&&ramp_adc<20&&ramp_adc>15)//����� �л�Ϊ����ѭ��
    //       {
    //           BEEP_OFF
    //           track_PID_flag=0;//����ѭ��
    //           electr_track_PID.Kp=track_fenduan[0];
    //           sanchakou_flag=2;
    //       }
//           else if(sanchakou_flag==2&&track_left_adc<36&&track_left_adc>31.7&&track_right_adc<39.8&&track_right_adc>34&&ramp_adc<30.6&&ramp_adc>21.6&&shu_left_adc<16&&shu_left_adc>13.3&&shu_right_adc<5.4&&shu_right_adc>3.3)//�ڶ�����
//           {
//               BEEP_ON//��������
//               track_PID_flag=2;//��ѭ��P
//               electr_track_PID.Kp=track_fenduan[0][2];
//               electr_track_PID.Kd=track_fenduan[1][2];
//               sanchakou_flag=3;
//           }
//           else if(sanchakou_flag==3&&ramp_adc<18.5&&ramp_adc>16&&neiba_left_adc<18.6&&neiba_right_adc>17.3&&shu_left_adc>2&&shu_right_adc>2.8)//�����
//           {
//               BEEP_OFF//��������
//               track_PID_flag=0;//����ѭ��
//               electr_track_PID.Kp=track_fenduan[0][0];
//               electr_track_PID.Kd=track_fenduan[1][0];
//               sanchakou_flag=4;
//           }

// //����
           else if(sanchakou_flag==3&&cheku_flag==1&&cheku_distance<-108)//�복��
           {
               //��ʱ������
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
                   real_speed=0;//����ͣת
                   cheku_flag=2;
//               }

           }
 //����
           //�һ���
//           if(huandao_flag==0&&MagneticField>180&&shu_left_adc>10&&shu_right_adc>27&&CircleNumber<4)//�һ�����
//           {
//               huandao_flag=1;
//               flag_right_round=1;
//           }
//           else if(huandao_flag==1&&flag_right_round==1&&MagneticField>280&&shu_right_adc<10&&shu_left_adc<4.6&&neiba_left_adc>70)//����һ�����
//           {
//               BEEP_ON
//               huandao_flag=2;
//               track_PID_flag=3;
//               huandao_gyro_flag=1;//��ʼ����
//           }
//           else if(huandao_flag==2&&flag_right_round==2&&huandao_gyro_distance>20&&MagneticField>28)//���������л�Ϊ����ѭ��
//           {
//               BEEP_OFF
//               track_PID_flag=0;//�л�Ϊ����ѭ��
//               flag_right_round=0;
//               huandao_flag=3;
//           }
//           else if(huandao_flag==3&&huandao_gyro_distance>20&&MagneticField<20)//ȷʵ����
//           {
//               huandao_gyro_flag=0;//���ֽ���
//               huandao_gyro_distance=0;//��������
//               CircleNumber++;//����������һ
//               huandao_flag=0;
//           }

           //�󻷵�
//           if(huandao_flag==0&&MagneticField>291&&shu_left_adc>72&&neiba_right_adc>82&&CircleNumber<4)//�󻷵���
//           {
//               huandao_flag=1;
//               flag_left_round=1;
//           }
//           else if(huandao_flag==1&&flag_left_round==1&&MagneticField>295&&shu_left_adc>38.5&&neiba_left_adc>88)//����󻷵���
//           {
//               BEEP_ON
//               huandao_flag=2;
//               track_PID_flag=5;
//               huandao_gyro_flag=1;//��ʼ����
//           }
//           else if(huandao_flag==2&&flag_left_round==2&&huandao_gyro_distance>20&&ramp_adc>92&&shu_right_adc>92)//���������л�Ϊ����ѭ��
//           {
//                   track_PID_flag=0;//�л�Ϊ����ѭ��
//                   flag_left_round=0;
//                   huandao_flag=3;
//           }
//           else if(huandao_flag==3&&huandao_gyro_distance>20&&MagneticField<20)//ȷʵ����
//           {
//               BEEP_OFF
//               huandao_gyro_flag=0;//���ֽ���
//               huandao_gyro_distance=0;//��������
//               CircleNumber++;//����������һ
//               huandao_flag=0;
//           }
}

/*-------------------------------------
 * �����ʼ��
 --------------------------------------*/
void sd12_init(void)
{
    lcd_showstr(0,0,"sd12 int");
    steer_duty = STEER_MOTOR_MID;//��������м�λ��
    gtm_pwm_init(S_MOTOR_PIN, 50, steer_duty);
    systick_delay_ms(STM0,500);
    lcd_showstr(0,0,"sd12 int ok");
    systick_delay_ms(STM0,500);
    lcd_clear(RGB565_BLACK);
    }

/*---------------------------------
 * ������У׼
 ---------------------------------*/
void sd12_correct(void)
{
    if(key_check(KEY0)==0)                        //����0���ƶ������ת
            {
                steer_duty+=2;
                if((1250-20) < steer_duty) steer_duty = 270;//��λ
                if((250+20) >= steer_duty)
                printf("steer_duty: %f\n",steer_duty);
                pwm_duty(S_MOTOR_PIN,steer_duty);
                systick_delay_ms(STM0, 1);
            }
            if(key_check(KEY1)==0)              //����1���ƶ������ת
            {
                steer_duty-=2;
                if((250+20) > steer_duty) steer_duty = 270;//��λ
                if((250+20) >= steer_duty)
                systick_delay_ms(STM0, 1000);
                printf("steer_duty: %f\n",steer_duty);
                pwm_duty(S_MOTOR_PIN,steer_duty);
                systick_delay_ms(STM0, 1);
            }
            if(key_check(KEY2)==0)             //ʹ�������
            {
                steer_duty = STEER_MOTOR_MID;//�������
                pwm_duty(S_MOTOR_PIN,steer_duty);
                systick_delay_ms(STM0, 1000);
            }
    }
/*---------------------------------
 * ������У׼ ��ʾ����ʾ
 ---------------------------------*/
void sd12_correct_lcd(void)
{
    lcd_showstr(0,1,"sd12 angle: ");
    while(1)
    {
    if(key_check(KEY0)==0)                        //����0���ƶ������ת
            {
                steer_duty+=2;
                if((1250-20) < steer_duty) steer_duty = 1230;//��λ
                pwm_duty(S_MOTOR_PIN,steer_duty);
                lcd_showfloat(90,1,steer_duty,5,1);
                systick_delay_ms(STM0, 1);
            }
      if(key_check(KEY1)==0)              //����1���ƶ������ת
            {
                steer_duty-=2;
                if((250+20) > steer_duty) steer_duty=270; //��λ
               pwm_duty(S_MOTOR_PIN,steer_duty);
               lcd_showfloat(90,1,steer_duty,5,1);
                systick_delay_ms(STM0, 1);
            }
        if(key_check(KEY2)==0)             //ʹ�������
            {
                steer_duty = STEER_MOTOR_MID;//�������
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
 * �����ǿ���
 -------------------------------------------------*/
void steering_control(uint16 steerpwm)
{
//dyn_zero(steerpwm,0);//��̬��㲹��
//��λ
if(abs(steerpwm)<=270)steerpwm=270;
if(abs(steerpwm)>=538)steerpwm=538;
pwm_duty(S_MOTOR_PIN,steerpwm);
}

/*----------------------------------------------------
 * �������ƽ��PID
 -----------------------------------------------------*/
void steering_balance(float Angle,float Gyro,float expect_angle)
{
   steering_PID.error=Angle+expect_angle;
   steering_PID.integral+=steering_PID.error;
   steering_PID.integral=limit_ab(steering_PID.integral, -steering_PID.I_L, steering_PID.I_L);
   steering_PID.out=steering_PID.Kp*steering_PID.error+steering_PID.Ki*steering_PID.integral+steering_PID.Kd*Gyro;
//   steering_PID.last_error=steering_PID.error;
//   steering_PID.d_error=steering_PID.error-steering_PID.last_error;
   //����
//   if(steering_PID.out>-5)steering_PID.out=0;
//   else if(steering_PID.out<5)steering_PID.out=0;
//   steering_control(STEER_MOTOR_MID-steering_PID.out);
}

//���ֵ��----------------------------------------------

/*----------------------------------------------------
 * ���������ʼ��
 ------------------------------------------------------*/
void motor_init(void)
{
    gpio_init(REAR_MOTOR_DIR, GPO, 0, PUSHPULL);
    gtm_pwm_init(REAR_MOTOR_PWM, 17000, 0);
}
/*-----------------------------------------------------
 * ���Ƶ��ת��
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
����������int Motor_PI (int Encoder,int Target)
����  �ܡ������������ʽPI
������ֵ��int Encoder ���������ɼ��ĳ���
������ֵ��int Target  ����������
������ֵ�����PWM
����  �ߡ�chiusir
�������¡�2021��1��22��
������汾��V1.0
**********************************************************************************/
int16 Rear_Motor_PI (int16 Encoder,int16 Target)
{
    static int16 Rear_Motor_Pwm;
    Motor_Bias = Encoder - Target;            // ����ƫ��
    Rear_Motor_Pwm += Motor_Kp * (Motor_Bias - Motor_Last_Bias) + Motor_Ki * Motor_Bias;
    // ==����ʽPI������
    if(Rear_Motor_Pwm > 5000) Rear_Motor_Pwm = 5000;               // �޷�
    else if(Rear_Motor_Pwm < -5000)Rear_Motor_Pwm = -5000;         // �޷�
    Motor_Last_Bias = Motor_Bias;            // ������һ��ƫ��
    return Rear_Motor_Pwm;                              // �������
}

void Rear_Motor_Control(int16 Target)
{
    ECPULSE2=Encorder_get(GPT12_T2);
    ECPULSE2=Low_pass_filter(ECPULSE2);//��ͨ�˲�
    motor_control(Rear_Motor_PI(ECPULSE2,Target));//�ջ�����
}
