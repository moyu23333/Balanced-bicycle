/*
 * PID.h
 *
 *  Created on: 2022��4��24��
 *      Author: ����
 */

#ifndef CODE_PID_H_
#define CODE_PID_H_
#include "headfile.h"
//PID (float��)
typedef struct
{
    float error;//���
    float last_error;//��һ���
    float Kp,Ki,Kd;
    float integral;//����ֵ
    float I_L;//�����޷�
    float d_error; //���֮��
    float out;//PWM���
}PID_float;

////PID (int��)
//typedef struct
//{
//        int16 error;//���
//        int16 last_error;//��һ���
//        int16 Kp,Ki,Kd;
//        int16 integral;//����ֵ
//        int16 I_L;//�����޷�
//        int16 d_error; //���֮��
//        int16 out;//PWM���
//}PID_int;


extern PID_float steering_PID;//���ƽ�⻷
extern PID_float angular_PID;//ֱ�����ٶȻ�
extern PID_float angle_PID;//ֱ���ǶȻ�
extern PID_float momentum_PID;//ֱ���ٶȻ�
//extern PID_float dyn_ero_PID;//��̬���PID
extern PID_float electr_track_PID;//���ѭ��PID
extern float track_fenduan[2][3];
#endif /* CODE_PID_H_ */
