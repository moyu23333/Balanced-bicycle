/*
 * control.h
 *
 *  Created on: 2022年1月23日
 *      Author: 摸鱼
 */

#ifndef CODE_CONTROL_H_
#define CODE_CONTROL_H_
#define S_MOTOR_PIN   ATOM1_CH1_P33_9 //舵机控制引脚
#define STEER_MOTOR_MID   404 //舵机中值

#define REAR_MOTOR_DIR   P21_3 //后轮电机方向控制引脚
#define REAR_MOTOR_PWM   ATOM1_CH0_P21_2 //后轮电机PWM输出引脚
#define BEEP            P33_10
#define BEEP_ON         gpio_set(BEEP,1);//蜂鸣器
#define BEEP_OFF        gpio_set(BEEP,0);
#include "headfile.h"
#include "stdlib.h"
#include "stdint.h"
#define ADC_LEFT_PIN           ADC0_CH4_A4                  //循迹左
#define ADC_NEIBA_LEFT_PIN     ADC0_CH1_A1                  //内八左
#define ADC_MIDDLE_PIN         ADC0_CH5_A5                  //坡道中
#define ADC_NEIBA_RIGHT_PIN    ADC0_CH2_A2                 //内八右
#define ADC_RIGHT_PIN          ADC0_CH6_A6                  //循迹右
#define ADC_SHU_LEFT_PIN       ADC0_CH0_A0                  //竖左
#define ADC_SHU_RIGHT_PIN      ADC0_CH3_A3                  //竖右
//舵机
void sd12_correct(void);
extern float steer_duty;//舵机pwm
void sd12_init(void);
void sd12_correct_lcd(void);
void steering_control(uint16 steerpwm);
void steering_balance(float Angle,float Gyro,float expect_angle);//舵机平衡PID
//后轮控制
extern int16 real_speed;
extern int16 ECPULSE2;//后轮编码器计数值
void motor_init(void);//后轮电机初始化
void motor_control(int16 pwm);//后轮电机控制
//后轮增量PI
extern int16 rear_temp_pwm;
extern short  Motor_Bias, Motor_Last_Bias, Motor_Integration; // 后轮电机所用参数
extern int16 Motor_Kp,Motor_Ki;//后轮增量式PI参数
int16 Rear_Motor_PI (int16 Encoder,int16 Target);//后驱增量式PI
void Rear_Motor_Control(int16 pwm);//后轮闭环控制
//电磁
//电磁标志位
extern uint8 track_PID_flag;//循迹方案标志位
extern uint8 cheku_flag;//车库标志位
extern uint32 cheku_time;
extern uint8 huandao_gyro_flag;
extern float huandao_gyro_distance;//陀螺仪计数积分
extern float cheku_distance;//车库计步
extern float track_left_adc,track_right_adc,ramp_adc,neiba_left_adc,neiba_right_adc,shu_left_adc,shu_right_adc;//电感归一化值
extern uint16 MagneticField;//整体电感强度
extern uint16 ADC_test_value[7];//所有电磁转换一次数值
void  ADC_init(void);//电感采集初始化
void  ADC_test(void);//电感采集一次
//滑动滤波相关
extern move_filter_struct ADC_flitler[7];//电磁滤波结构体
void adc_filter1_int(void);//ADC滑动滤波初始化
//电磁数据处理
extern uint16 adc_min[],adc_max[];
void adc_limit(void);
void adc_filter1( void );
void track_scan(void);
void adc_result(void);
float Diff_ratio_sum(float dat1,float dat2);//差比和算法
extern float adc_sum;//差比和的和
extern float adc_diff;  //差比和的差
extern float adc_error;//差比和输出结果
void Inductorcontrol(void);//循迹
void Inductiveprocess(void);//元素处理
#endif /* CODE_CONTROL_H_ */
