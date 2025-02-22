/*
 * menu.h
 *
 *  Created on: 2022��1��24��
 *      Author: ����
 */

#ifndef CODE_KEYSETTINGMENU_H_
#define CODE_KEYSETTINGMENU_H_
#include "headfile.h"
//�������ڷֶ�ֵ
//���ٶȻ�
//KP
#define ANGULAR_KP_GRAD       0.2  //���ڷֶ�ֵ
#define ANGULAR_KP_DISP_INT   2   //����
#define ANGULAR_KP_DISP_POINT 2   //С��
//KI
#define ANGULAR_KI_GRAD       0.02//���ڷֶ�ֵ
#define ANGULAR_KI_DISP_INT   2   //����
#define ANGULAR_KI_DISP_POINT 2     //С��
//KD
#define ANGULAR_KD_GRAD       0.02//���ڷֶ�ֵ
#define ANGULAR_KD_DISP_INT   2   //����
#define ANGULAR_KD_DISP_POINT 2   //С��
//IL
#define ANGULAR_IL_GRAD       50//���ڷֶ�ֵ
#define ANGULAR_IL_DISP_INT   5   //����
#define ANGULAR_IL_DISP_POINT 1   //С��
//�ǶȻ�
//KP
#define ANGLE_KP_GRAD         0.01//���ڷֶ�ֵ
#define ANGLE_KP_DISP_INT     2 //����
#define ANGLE_KP_DISP_POINT   3 //С��
//KI
#define ANGLE_KI_GRAD         0.001//���ڷֶ�ֵ
#define ANGLE_KI_DISP_INT     2 //����
#define ANGLE_KI_DISP_POINT   3 //С��
//KD
#define ANGLE_KD_GRAD         0.001//���ڷֶ�ֵ
#define ANGLE_KD_DISP_INT     1 //����
#define ANGLE_KD_DISP_POINT   3 //С��
//IL
#define ANGLE_IL_GRAD         20//���ڷֶ�ֵ
#define ANGLE_IL_DISP_INT     5 //����
#define ANGLE_IL_DISP_POINT   1 //С��

//�ٶȻ�
//KP
#define VELOCITY_KP_GRAD        0.002 //���ڷֶ�ֵ
#define VELOCITY_KP_DISP_INT    1//����
#define VELOCITY_KP_DISP_POINT  4//С��
//KI
#define VELOCITY_KI_GRAD        0.0002//���ڷֶ�ֵ
#define VELOCITY_KI_DISP_INT    1//����
#define VELOCITY_KI_DISP_POINT  6//С��
//KD
#define VELOCITY_KD_GRAD        0.00001//���ڷֶ�ֵ
#define VELOCITY_KD_DISP_INT    1//����
#define VELOCITY_KD_DISP_POINT  5//С��
//IL
#define VELOCITY_IL_GRAD        20//���ڷֶ�ֵ
#define VELOCITY_IL_DISP_INT    5//����
#define VELOCITY_IL_DISP_POINT  1//С��

//��������PI��
//KP
#define REAR_KP_GRAD        1//���ڷֶ�ֵ
#define REAR_KP_DISP_INT    3//����
#define REAR_KP_DISP_POINT  1//С��
//KI
#define REAR_KI_GRAD        1//���ڷֶ�ֵ
#define REAR_KI_DISP_INT    3//����
#define REAR_KI_DISP_POINT  1//С��
//IL
#define REAR_IL_GRAD        0.01//���ڷֶ�ֵ
#define REAR_IL_DISP_INT    2//����
#define REAR_IL_DISP_POINT  2//С��
//SPEED
#define REAR_SPEED_GRAD        5//���ڷֶ�ֵ
#define REAR_SPEED_DISP_INT    4//����
#define REAR_SPEED_DISP_POINT  1//С��
//����ں�
#define STEER_KP_GRAD        0.1//���ڷֶ�ֵ
#define STEER_KP_DISP_INT    3//����
#define STEER_KP_DISP_POINT  1//С��
//KI
#define STEER_KI_GRAD        0.01//���ڷֶ�ֵ
#define STEER_KI_DISP_INT    3//����
#define STEER_KI_DISP_POINT  2//С��
//KD
#define STEER_KD_GRAD        0.01//���ڷֶ�ֵ
#define STEER_KD_DISP_INT    1//����
#define STEER_KD_DISP_POINT  2//С��
//IL
#define STEER_IL_GRAD        2//���ڷֶ�ֵ
#define STEER_IL_DISP_INT    3//����
#define STEER_IL_DISP_POINT  1//С��

//���ѭ��PID
//KP
#define TRACK_KP_GRAD        0.2//���ڷֶ�ֵ
#define TRACK_KP_DISP_INT    2//����
#define TRACK_KP_DISP_POINT  1//С��
//KI
#define TRACK_KI_GRAD        1//���ڷֶ�ֵ
#define TRACK_KI_DISP_INT    4//����
#define TRACK_KI_DISP_POINT  1//С��
//KD
#define TRACK_KD_GRAD        0.1//���ڷֶ�ֵ
#define TRACK_KD_DISP_INT    2//����
#define TRACK_KD_DISP_POINT  1//С��
//IL
#define TRACK_IL_GRAD        20//���ڷֶ�ֵ
#define TRACK_IL_DISP_INT    5//����
#define TRACK_IL_DISP_POINT  1//С��

extern int16 xishu;
extern float test_temp_num[];//���Բ���
extern float expect_angle;
void test_Interface(void);//����
extern float motor_duty;
extern _Bool Rear_Motor_FLAG;//����PI������־λ
void motor_control_int(void);
void Main_Interface(void);//���˵�
void KeySettingMenustart(void);
void Kalman_set_Interface(void);
void ADC_date_interface(void);//ADC���ݲɼ�����
#endif /* CODE_KEYSETTINGMENU_H_ */
