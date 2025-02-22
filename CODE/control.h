/*
 * control.h
 *
 *  Created on: 2022��1��23��
 *      Author: ����
 */

#ifndef CODE_CONTROL_H_
#define CODE_CONTROL_H_
#define S_MOTOR_PIN   ATOM1_CH1_P33_9 //�����������
#define STEER_MOTOR_MID   404 //�����ֵ

#define REAR_MOTOR_DIR   P21_3 //���ֵ�������������
#define REAR_MOTOR_PWM   ATOM1_CH0_P21_2 //���ֵ��PWM�������
#define BEEP            P33_10
#define BEEP_ON         gpio_set(BEEP,1);//������
#define BEEP_OFF        gpio_set(BEEP,0);
#include "headfile.h"
#include "stdlib.h"
#include "stdint.h"
#define ADC_LEFT_PIN           ADC0_CH4_A4                  //ѭ����
#define ADC_NEIBA_LEFT_PIN     ADC0_CH1_A1                  //�ڰ���
#define ADC_MIDDLE_PIN         ADC0_CH5_A5                  //�µ���
#define ADC_NEIBA_RIGHT_PIN    ADC0_CH2_A2                 //�ڰ���
#define ADC_RIGHT_PIN          ADC0_CH6_A6                  //ѭ����
#define ADC_SHU_LEFT_PIN       ADC0_CH0_A0                  //����
#define ADC_SHU_RIGHT_PIN      ADC0_CH3_A3                  //����
//���
void sd12_correct(void);
extern float steer_duty;//���pwm
void sd12_init(void);
void sd12_correct_lcd(void);
void steering_control(uint16 steerpwm);
void steering_balance(float Angle,float Gyro,float expect_angle);//���ƽ��PID
//���ֿ���
extern int16 real_speed;
extern int16 ECPULSE2;//���ֱ���������ֵ
void motor_init(void);//���ֵ����ʼ��
void motor_control(int16 pwm);//���ֵ������
//��������PI
extern int16 rear_temp_pwm;
extern short  Motor_Bias, Motor_Last_Bias, Motor_Integration; // ���ֵ�����ò���
extern int16 Motor_Kp,Motor_Ki;//��������ʽPI����
int16 Rear_Motor_PI (int16 Encoder,int16 Target);//��������ʽPI
void Rear_Motor_Control(int16 pwm);//���ֱջ�����
//���
//��ű�־λ
extern uint8 track_PID_flag;//ѭ��������־λ
extern uint8 cheku_flag;//�����־λ
extern uint32 cheku_time;
extern uint8 huandao_gyro_flag;
extern float huandao_gyro_distance;//�����Ǽ�������
extern float cheku_distance;//����Ʋ�
extern float track_left_adc,track_right_adc,ramp_adc,neiba_left_adc,neiba_right_adc,shu_left_adc,shu_right_adc;//��й�һ��ֵ
extern uint16 MagneticField;//������ǿ��
extern uint16 ADC_test_value[7];//���е��ת��һ����ֵ
void  ADC_init(void);//��вɼ���ʼ��
void  ADC_test(void);//��вɼ�һ��
//�����˲����
extern move_filter_struct ADC_flitler[7];//����˲��ṹ��
void adc_filter1_int(void);//ADC�����˲���ʼ��
//������ݴ���
extern uint16 adc_min[],adc_max[];
void adc_limit(void);
void adc_filter1( void );
void track_scan(void);
void adc_result(void);
float Diff_ratio_sum(float dat1,float dat2);//��Ⱥ��㷨
extern float adc_sum;//��Ⱥ͵ĺ�
extern float adc_diff;  //��Ⱥ͵Ĳ�
extern float adc_error;//��Ⱥ�������
void Inductorcontrol(void);//ѭ��
void Inductiveprocess(void);//Ԫ�ش���
#endif /* CODE_CONTROL_H_ */
