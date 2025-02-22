/*
 * math.c
 *
 *  Created on: 2022年1月22日
 *      Author: 摸鱼
 */
#include "headfile.h"
float angle=0,angle_dot=0;//角度，角速度
move_filter_struct gyro_fliter;//滑动滤波结构体
move_filter_struct acc_fliter;//加速计滤波结构体
/*-----------------------------------------------------------
 * 一阶低通滤波
 *
 -------------------------------------------------------------*/
int16 Low_pass_filter(int16 Lowpass)
{
    static float Least_Lowpass;
    Least_Lowpass = Lowpass;
    Lowpass *= 0.7;
    Lowpass+=Least_Lowpass*0.3;
    return Lowpass;
}
//----------------------------------------------------------------
//  @brief      一阶互补滤波
//  @param      angle_m     加速度计数据
//  @param      gyro_m      陀螺仪数据
//  @return     float       数据融合后的角度
//----------------------------------------------------------------
float acc_ratio = 0.83;      //加速度计比例 //3
float gyro_ratio =3.2;    //陀螺仪比例  //4.8
float yijie_dt = 0.002;           //采样周期
float angle_calc(float angle_m, float gyro_m)
{
    float temp_angle=0;
    float gyro_now=0;
    float error_angle=0;
    static float last_angle;
    static uint8 first_angle;
    if(!first_angle)//判断是否为第一次运行本函数
    {
        //如果是第一次运行，则将上次角度值设置为与加速度值一致
        first_angle = 1;
        last_angle = angle_m;
    }
   gyro_now = gyro_m * gyro_ratio;
    //根据测量到的加速度值转换为角度之后与上次的角度值求偏差
    error_angle = (angle_m - last_angle)*acc_ratio;
    //根据偏差与陀螺仪测量得到的角度值计算当前角度值
    temp_angle = last_angle + (error_angle + gyro_now)*yijie_dt;
    //保存当前角度值
    last_angle = temp_angle;
    return temp_angle;
}

//-------------------------------------------------------------------------------------------------------------------
// icm20602陀螺仪校准
// 采用中位值滤波的方法
//-------------------------------------------------------------------------------------------------------------------
int16 icm_gyro_x_err,icm_gyro_y_err,icm_gyro_z_err;
void icm_gyro_setint (void)
{
    uint8 i=0;
    uint8 count=0;
    uint8 j=0;
    uint8 k=0;
    int16 tempval=0;
    int16 tempval_buf[3][100];//
    for (count=0;count<100; count++)
        {
            get_icm20602_gyro_spi();
            systick_delay_ms(STM0,30);
            tempval_buf[0][count]=icm_gyro_x;
            tempval_buf[1][count]=icm_gyro_y;
            tempval_buf[2][count]=icm_gyro_z;
        }
    //冒泡排序
    for(j=0;j<99;j++)
             {
                 for(i=0;i<99-j;i++)
                     {
                     //数组并行排序
                      for(k=0;k<3;k++)
                      {
                         if ( tempval_buf[k][i] > tempval_buf[k][i+1] )
                         {
                          tempval =  tempval_buf[k][i];
                          tempval_buf[k][i] = tempval_buf[k][i+1];
                          tempval_buf[k][i+1] = tempval;
                          }
                      }
                     }
             }
    for(i=0;i<6;i++)
    {
        icm_gyro_x_err+=tempval_buf[0][48+i];
        icm_gyro_y_err+=tempval_buf[1][48+i];
        icm_gyro_z_err+=tempval_buf[2][48+i];
    }
    icm_gyro_x_err/=6;
    icm_gyro_y_err/=6;
    icm_gyro_z_err/=6;
}

/*----------------------------------
 * 动态零点
 * 建立舵机、后轮转速与零点的关系
 * Actuator：舵机打角
 * Encorder：后轮转速
 -----------------------------------*/
float dyn_zero_val=0;//动态零点
float dyn_zero_KP=-1.4;//舵机打角补偿参数
float Real_KP=0;//后轮速度补偿参数
void dyn_zero(uint16 Actuator,int16 Encorder)
{
    if((STEER_MOTOR_MID-20)>Actuator||(STEER_MOTOR_MID+20)<Actuator)
        {
        //舵机打角补偿
            dyn_zero_val=dyn_zero_KP*(Actuator-STEER_MOTOR_MID)/134+Pitch_Zero;
        //向心力补偿
//            if(Actuator>STEER_MOTOR_MID)
//                dyn_zero_val+=Real_KP*Encorder;
//            else if(Actuator<STEER_MOTOR_MID)
//                dyn_zero_val-=Real_KP*Encorder;
        }
        else
            dyn_zero_val=Pitch_Zero;
}

/*-------------------------------------------------------
 * 线性归一化
 *
 ------------------------------------------------------*/
float MinMax_scaling(int date,int max,int min)
{
    int16 datesum=0,datediff=0;
    datediff=max-min;
    datesum=date-min;
    return (float)datesum/datediff;
}
//-------------------------------------------------------------------

/*---------------------------------------------------
 * 滑动滤波初始化
 ---------------------------------------------------*/
void move_filter_init(move_filter_struct *move_filter)
    {
        move_filter->data_average = 0;
        move_filter->data_sum = 0;
        move_filter->index = 0;
        //设置缓冲区大小
        move_filter->buffer_size = MOVE_AVERAGE_SIZE;

        uint8 i;
        for(i=0; i < move_filter->buffer_size; i++)
        {
            move_filter->data_buffer[i] = 0;
        }
    }

//-------------------------------------------------------------------------------------------------------------------
//  @brief      滑动平均滤波计算
//  @param      void
//  @return     void
//  @since      主要用于对数据滤波，存储目标数据最近的n个数据，并求出平均值
//-------------------------------------------------------------------------------------------------------------------
void move_filter_calc(move_filter_struct *move_filter, int16 new_data)
{
    //加上新的数值 减去最末尾的数值 求得最新的和
    move_filter->data_sum = move_filter->data_sum + new_data - move_filter->data_buffer[move_filter->index];
    //重新求平均值
    move_filter->data_average =(int16)(move_filter->data_sum / move_filter->buffer_size);

    //将数据写入缓冲区
    move_filter->data_buffer[move_filter->index] = new_data;
    move_filter->index++;
    if(move_filter->buffer_size <= move_filter->index)
    {
        move_filter->index = 0;
    }
}

/*-----------------------------------------------------------
 * 互补滤波计算最终值
 -------------------------------------------------------------*/
void hubu_icm20602_result(void)
{
    get_icm20602_accdata_spi();
    get_icm20602_gyro_spi();
    move_filter_calc(&gyro_fliter,icm_gyro_x);//陀螺仪滑动滤波
    angle=angle_calc(icm_acc_y,gyro_fliter.data_average-icm_gyro_x_err)-Pitch_Zero;
    angle_dot=(gyro_fliter.data_average-icm_gyro_x_err);
}
//-----------------------------------------------------------------------------------------------------------

//---------------------------------------
//直立平衡控制
float Pitch_Zero=-276;//设置Pitch轴角度零点 135 1300amh
uint8 Velocity_Ctrl_cnt=0,angle_control_cnt=0;//时序计数变量
int16 momentum_duty=0;//动量轮占空比
int16 ECPULSE1=0;//动量轮编码器计数值

//------编码器 初始化---------------
void Encorder_Init(void)
{
       gpt12_init(GPT12_T5,  GPT12_T5INB_P10_3, GPT12_T5EUDB_P10_1);//
       gpt12_init(GPT12_T2,GPT12_T2INB_P33_7,GPT12_T2EUDB_P33_6);//
}

/**************************************************************************
 * 获取动量轮编码器的值
**************************************************************************/
//GPT12_T5 后轮
//GPT12_T2 动量轮
int16 Encorder_get(GPTN_enum gptn)
{
    int16 ECPULSE_val=0;
    ECPULSE_val = gpt12_get(gptn);// 动量轮反馈 母板编码器1
    gpt12_clear(gptn);
    return ECPULSE_val;
}
/**************************************************************
 * 动量轮初始化
***************************************************************/
void Momentum_int(void)
{
    gpio_init(MOMENTU_DIR, GPO, 0, PUSHPULL);// dir
    gtm_pwm_init(MOMENTU_PWM, 17000, 0);//pwm
}

/********************************************************
 * 动量轮电机控制
 ******************************************************/
void MomentumCtrl (int16 motor1)
{
    if (motor1 > 0)
    {
        pwm_duty(MOMENTU_PWM, motor1);
        gpio_set(MOMENTU_DIR, 0);
    }
    else
    {
        pwm_duty(MOMENTU_PWM, (-motor1));
        gpio_set(MOMENTU_DIR, 1);
    }
}

/*---------------------------------------------------------
 * 最终直立控制
 ---------------------------------------------------------*/
void balance_control(void)//2ms中断
{
      Velocity_Ctrl_cnt++;
      angle_control_cnt++;
      if(Velocity_Ctrl_cnt>=50)//100ms 速度环
      {
          Velocity_Ctrl_cnt=0;
          ECPULSE1=Encorder_get(GPT12_T5);
          Velocity_Control(-ECPULSE1);
      }
      if(angle_control_cnt>=5)//10ms 角度环
      {
        angle_control_cnt=0;
        angle_control(momentum_PID.out,angle+dyn_zero_val);
      }
      momentum_duty=angular_control(angle_PID.out,angle_dot);//2ms 角速度环
      if(momentum_duty<-700)momentum_duty-=700;  //死区
      else if(momentum_duty>700)momentum_duty+=700;
       momentum_duty=limit_ab(momentum_duty,-8000,8000);   //电机PWM限幅
      if(angle>1200||angle<-1200)//摔倒清零
      {
          momentum_PID.integral=0;//速度环积分
          momentum_duty=0;
          angular_PID.integral=0;//角速度环积分
      }

      MomentumCtrl(momentum_duty);//控制动量轮转动
}
/********************************************************
 * 速度环
 * PID
 ********************************************************/
float Velocity_Control(int16 actual)
{
    static float Encoder;
    float Encoder_Least =0;
    Encoder_Least = actual;                                  //速度滤波
    Encoder *= 0.7;                                           //一阶低通滤波器
    Encoder += Encoder_Least*0.3;                             //一阶低通滤波
    momentum_PID.error = Encoder; //计算偏差
    momentum_PID.integral += momentum_PID.error; //累计积分
    momentum_PID.integral=limit_ab(momentum_PID.integral ,- momentum_PID.I_L , momentum_PID.I_L ); //积分限幅
    momentum_PID.d_error = momentum_PID.error - momentum_PID.last_error; //计算偏差之差
    momentum_PID.out = momentum_PID.Kp * momentum_PID.error + momentum_PID.Ki * momentum_PID.integral + momentum_PID.Kd*momentum_PID.d_error;//计算输出值
    momentum_PID.last_error = momentum_PID.error;//更新偏差
    return momentum_PID.out;
}
/********************************************************************************
 * 角速度环
 * PI
 * 积分分离值需要确定
 *****************************************************************************/
float angular_control(float target,float actual)
{
    angular_PID.error=target-actual;//计算偏差
    angular_PID.integral+=angular_PID.error;   //累积误差
    angular_PID.integral=limit_ab(angular_PID.integral,-angular_PID.I_L,angular_PID.I_L); //积分限幅
//    if(angular_PID.error>80||angular_PID.error<-80)angular_PID.integral=0;//积分分离
    angular_PID.d_error=angular_PID.error-angular_PID.last_error;  //计算误差之差
    angular_PID.out=angular_PID.Kp*angular_PID.error+angular_PID.Ki*angular_PID.integral+angular_PID.Kd*angular_PID.d_error;//计算最终值
    angular_PID.last_error=angular_PID.error;
    return angular_PID.out;
}
/********************************************************************************
 * 增量角速度环
 * PI
 *****************************************************************************/
//float angular_increment(float target,float actual)
//{
//
//
//
//
//}
/**************************************************************************
* 角度环
* PD
**************************************************************************/
float angle_control(float target,float actual)
{
    angle_PID.error=target-actual;//计算偏差
//    angle_PID.integral+=angle_PID.error;//累计误差
//  angle_PID.integral=limit_ab(angle_PID.integral,-angle_PID.I_L,angle_PID.I_L);//积分限幅
    angle_PID.d_error=angle_PID.error-angle_PID.last_error;//计算误差之差
    angle_PID.out=angle_PID.Kp*angle_PID.error+angle_PID.Ki*angle_PID.integral+angle_PID.Kd*angle_PID.d_error;//计算输出值
    angle_PID.last_error = angle_PID.error;
    return angle_PID.out;
}




