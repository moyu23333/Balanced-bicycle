/*
 * PID.h
 *
 *  Created on: 2022年4月24日
 *      Author: 摸鱼
 */

#ifndef CODE_PID_H_
#define CODE_PID_H_
#include "headfile.h"
//PID (float型)
typedef struct
{
    float error;//误差
    float last_error;//上一误差
    float Kp,Ki,Kd;
    float integral;//积分值
    float I_L;//积分限幅
    float d_error; //误差之差
    float out;//PWM输出
}PID_float;

////PID (int型)
//typedef struct
//{
//        int16 error;//误差
//        int16 last_error;//上一误差
//        int16 Kp,Ki,Kd;
//        int16 integral;//积分值
//        int16 I_L;//积分限幅
//        int16 d_error; //误差之差
//        int16 out;//PWM输出
//}PID_int;


extern PID_float steering_PID;//舵机平衡环
extern PID_float angular_PID;//直立角速度环
extern PID_float angle_PID;//直立角度环
extern PID_float momentum_PID;//直立速度环
//extern PID_float dyn_ero_PID;//动态零点PID
extern PID_float electr_track_PID;//电磁循迹PID
extern float track_fenduan[2][3];
#endif /* CODE_PID_H_ */
