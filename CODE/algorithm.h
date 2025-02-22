/*
 * math.h
 *
 *  Created on: 2022��1��22��
 *      Author: ����
 */

#ifndef CODE_ALGORITHM_H_
#define CODE_ALGORITHM_H_
#include "headfile.h"
//����������
#define MOMENTU_DIR  P21_4 //DIR
#define MOMENTU_PWM  ATOM1_CH3_P21_5//PWM
//��������Ưֵ
extern int16 icm_gyro_x_err,icm_gyro_y_err,icm_gyro_z_err;
void icm_gyro_setint (void);//��������Ư�ɼ�

float MinMax_scaling(int date,int max,int min);//���Թ�һ��

//һ�׻����˲�
extern float angle,angle_dot;//�Ƕȣ����ٶ�
extern float acc_ratio;      //���ٶȼƱ���
extern float gyro_ratio;    //�����Ǳ���
extern float yijie_dt;
float angle_calc(float angle_m, float gyro_m);//һ�׻����˲�
void hubu_icm20602_result(void);//�����˲�����
//һ�׵�ͨ�˲�
int16 Low_pass_filter(int16 Lowpass);
//----------------------------------------------------------------
//�����˲�
#define MOVE_AVERAGE_SIZE   15  //���建������С

//�����˲��ṹ��
typedef struct
{
    uint8 index;            //�±�
    uint8 buffer_size;      //buffer��С
    int16 data_buffer[MOVE_AVERAGE_SIZE];  //������
    int32 data_sum;         //���ݺ�
    int16 data_average;     //����ƽ��ֵ
}move_filter_struct;

extern move_filter_struct gyro_fliter;//���ٶ��˲��ṹ��
void move_filter_init(move_filter_struct *move_filter);//�����˲���ʼ��
void move_filter_calc(move_filter_struct *move_average, int16 new_data);//�����˲�

//--------------------------------------------------------------------------
extern float Pitch_Zero;//����Pitch��Ƕ����
extern float dyn_zero_val;//��̬���
extern int16 momentum_duty;//������ռ�ձ�
extern int16 ECPULSE1;//�����ֱ�����ֵ
extern uint8 Velocity_Ctrl_cnt,angle_control_cnt;//���ƻ��������ڼ���
void Encorder_Init(void);//��������ʼ��
int16 Encorder_get(GPTN_enum gptn);//��ȡ��������ֵ

void Momentum_int(void);//�����ֳ�ʼ��
void MomentumCtrl(int16 motor1);//�����ֵ������

float Velocity_Control(int16 actual);// �ٶȻ�PID
float angle_control(float target,float actual);//�ǶȻ�PID
float angular_control(float target,float actual);//���ٶȻ�PID
void balance_control(void);//ֱ�����ƹ���


void dyn_zero(uint16 Actuator,int16 Encorder);//��̬���
#endif /* CODE_ALGORITHM_H_ */
