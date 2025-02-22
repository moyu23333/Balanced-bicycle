/*
 * menu.h
 *
 *  Created on: 2022年1月24日
 *      Author: 摸鱼
 */

#ifndef CODE_KEYSETTINGMENU_H_
#define CODE_KEYSETTINGMENU_H_
#include "headfile.h"
//各环调节分度值
//角速度环
//KP
#define ANGULAR_KP_GRAD       0.2  //调节分度值
#define ANGULAR_KP_DISP_INT   2   //整数
#define ANGULAR_KP_DISP_POINT 2   //小数
//KI
#define ANGULAR_KI_GRAD       0.02//调节分度值
#define ANGULAR_KI_DISP_INT   2   //整数
#define ANGULAR_KI_DISP_POINT 2     //小数
//KD
#define ANGULAR_KD_GRAD       0.02//调节分度值
#define ANGULAR_KD_DISP_INT   2   //整数
#define ANGULAR_KD_DISP_POINT 2   //小数
//IL
#define ANGULAR_IL_GRAD       50//调节分度值
#define ANGULAR_IL_DISP_INT   5   //整数
#define ANGULAR_IL_DISP_POINT 1   //小数
//角度环
//KP
#define ANGLE_KP_GRAD         0.01//调节分度值
#define ANGLE_KP_DISP_INT     2 //整数
#define ANGLE_KP_DISP_POINT   3 //小数
//KI
#define ANGLE_KI_GRAD         0.001//调节分度值
#define ANGLE_KI_DISP_INT     2 //整数
#define ANGLE_KI_DISP_POINT   3 //小数
//KD
#define ANGLE_KD_GRAD         0.001//调节分度值
#define ANGLE_KD_DISP_INT     1 //整数
#define ANGLE_KD_DISP_POINT   3 //小数
//IL
#define ANGLE_IL_GRAD         20//调节分度值
#define ANGLE_IL_DISP_INT     5 //整数
#define ANGLE_IL_DISP_POINT   1 //小数

//速度环
//KP
#define VELOCITY_KP_GRAD        0.002 //调节分度值
#define VELOCITY_KP_DISP_INT    1//整数
#define VELOCITY_KP_DISP_POINT  4//小数
//KI
#define VELOCITY_KI_GRAD        0.0002//调节分度值
#define VELOCITY_KI_DISP_INT    1//整数
#define VELOCITY_KI_DISP_POINT  6//小数
//KD
#define VELOCITY_KD_GRAD        0.00001//调节分度值
#define VELOCITY_KD_DISP_INT    1//整数
#define VELOCITY_KD_DISP_POINT  5//小数
//IL
#define VELOCITY_IL_GRAD        20//调节分度值
#define VELOCITY_IL_DISP_INT    5//整数
#define VELOCITY_IL_DISP_POINT  1//小数

//后轮增量PI环
//KP
#define REAR_KP_GRAD        1//调节分度值
#define REAR_KP_DISP_INT    3//整数
#define REAR_KP_DISP_POINT  1//小数
//KI
#define REAR_KI_GRAD        1//调节分度值
#define REAR_KI_DISP_INT    3//整数
#define REAR_KI_DISP_POINT  1//小数
//IL
#define REAR_IL_GRAD        0.01//调节分度值
#define REAR_IL_DISP_INT    2//整数
#define REAR_IL_DISP_POINT  2//小数
//SPEED
#define REAR_SPEED_GRAD        5//调节分度值
#define REAR_SPEED_DISP_INT    4//整数
#define REAR_SPEED_DISP_POINT  1//小数
//舵机融合
#define STEER_KP_GRAD        0.1//调节分度值
#define STEER_KP_DISP_INT    3//整数
#define STEER_KP_DISP_POINT  1//小数
//KI
#define STEER_KI_GRAD        0.01//调节分度值
#define STEER_KI_DISP_INT    3//整数
#define STEER_KI_DISP_POINT  2//小数
//KD
#define STEER_KD_GRAD        0.01//调节分度值
#define STEER_KD_DISP_INT    1//整数
#define STEER_KD_DISP_POINT  2//小数
//IL
#define STEER_IL_GRAD        2//调节分度值
#define STEER_IL_DISP_INT    3//整数
#define STEER_IL_DISP_POINT  1//小数

//电磁循迹PID
//KP
#define TRACK_KP_GRAD        0.2//调节分度值
#define TRACK_KP_DISP_INT    2//整数
#define TRACK_KP_DISP_POINT  1//小数
//KI
#define TRACK_KI_GRAD        1//调节分度值
#define TRACK_KI_DISP_INT    4//整数
#define TRACK_KI_DISP_POINT  1//小数
//KD
#define TRACK_KD_GRAD        0.1//调节分度值
#define TRACK_KD_DISP_INT    2//整数
#define TRACK_KD_DISP_POINT  1//小数
//IL
#define TRACK_IL_GRAD        20//调节分度值
#define TRACK_IL_DISP_INT    5//整数
#define TRACK_IL_DISP_POINT  1//小数

extern int16 xishu;
extern float test_temp_num[];//调试参数
extern float expect_angle;
void test_Interface(void);//调试
extern float motor_duty;
extern _Bool Rear_Motor_FLAG;//后轮PI启动标志位
void motor_control_int(void);
void Main_Interface(void);//主菜单
void KeySettingMenustart(void);
void Kalman_set_Interface(void);
void ADC_date_interface(void);//ADC数据采集界面
#endif /* CODE_KEYSETTINGMENU_H_ */
