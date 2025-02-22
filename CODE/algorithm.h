/*
 * math.h
 *
 *  Created on: 2022年1月22日
 *      Author: 摸鱼
 */

#ifndef CODE_ALGORITHM_H_
#define CODE_ALGORITHM_H_
#include "headfile.h"
//动量轮引脚
#define MOMENTU_DIR  P21_4 //DIR
#define MOMENTU_PWM  ATOM1_CH3_P21_5//PWM
//陀螺仪温漂值
extern int16 icm_gyro_x_err,icm_gyro_y_err,icm_gyro_z_err;
void icm_gyro_setint (void);//陀螺仪温漂采集

float MinMax_scaling(int date,int max,int min);//线性归一化

//一阶互补滤波
extern float angle,angle_dot;//角度，角速度
extern float acc_ratio;      //加速度计比例
extern float gyro_ratio;    //陀螺仪比例
extern float yijie_dt;
float angle_calc(float angle_m, float gyro_m);//一阶互补滤波
void hubu_icm20602_result(void);//互补滤波计算
//一阶低通滤波
int16 Low_pass_filter(int16 Lowpass);
//----------------------------------------------------------------
//滑动滤波
#define MOVE_AVERAGE_SIZE   15  //定义缓冲区大小

//滑动滤波结构体
typedef struct
{
    uint8 index;            //下标
    uint8 buffer_size;      //buffer大小
    int16 data_buffer[MOVE_AVERAGE_SIZE];  //缓冲区
    int32 data_sum;         //数据和
    int16 data_average;     //数据平均值
}move_filter_struct;

extern move_filter_struct gyro_fliter;//角速度滤波结构体
void move_filter_init(move_filter_struct *move_filter);//滑动滤波初始化
void move_filter_calc(move_filter_struct *move_average, int16 new_data);//滑动滤波

//--------------------------------------------------------------------------
extern float Pitch_Zero;//设置Pitch轴角度零点
extern float dyn_zero_val;//动态零点
extern int16 momentum_duty;//动量轮占空比
extern int16 ECPULSE1;//动量轮编码器值
extern uint8 Velocity_Ctrl_cnt,angle_control_cnt;//控制环控制周期计数
void Encorder_Init(void);//编码器初始化
int16 Encorder_get(GPTN_enum gptn);//获取编码器的值

void Momentum_int(void);//动量轮初始化
void MomentumCtrl(int16 motor1);//动量轮电机控制

float Velocity_Control(int16 actual);// 速度环PID
float angle_control(float target,float actual);//角度环PID
float angular_control(float target,float actual);//角速度环PID
void balance_control(void);//直立控制过程


void dyn_zero(uint16 Actuator,int16 Encorder);//动态零点
#endif /* CODE_ALGORITHM_H_ */
