/*
 * math.c
 *
 *  Created on: 2022��1��22��
 *      Author: ����
 */
#include "headfile.h"
float angle=0,angle_dot=0;//�Ƕȣ����ٶ�
move_filter_struct gyro_fliter;//�����˲��ṹ��
move_filter_struct acc_fliter;//���ټ��˲��ṹ��
/*-----------------------------------------------------------
 * һ�׵�ͨ�˲�
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
//  @brief      һ�׻����˲�
//  @param      angle_m     ���ٶȼ�����
//  @param      gyro_m      ����������
//  @return     float       �����ںϺ�ĽǶ�
//----------------------------------------------------------------
float acc_ratio = 0.83;      //���ٶȼƱ��� //3
float gyro_ratio =3.2;    //�����Ǳ���  //4.8
float yijie_dt = 0.002;           //��������
float angle_calc(float angle_m, float gyro_m)
{
    float temp_angle=0;
    float gyro_now=0;
    float error_angle=0;
    static float last_angle;
    static uint8 first_angle;
    if(!first_angle)//�ж��Ƿ�Ϊ��һ�����б�����
    {
        //����ǵ�һ�����У����ϴνǶ�ֵ����Ϊ����ٶ�ֵһ��
        first_angle = 1;
        last_angle = angle_m;
    }
   gyro_now = gyro_m * gyro_ratio;
    //���ݲ������ļ��ٶ�ֵת��Ϊ�Ƕ�֮�����ϴεĽǶ�ֵ��ƫ��
    error_angle = (angle_m - last_angle)*acc_ratio;
    //����ƫ���������ǲ����õ��ĽǶ�ֵ���㵱ǰ�Ƕ�ֵ
    temp_angle = last_angle + (error_angle + gyro_now)*yijie_dt;
    //���浱ǰ�Ƕ�ֵ
    last_angle = temp_angle;
    return temp_angle;
}

//-------------------------------------------------------------------------------------------------------------------
// icm20602������У׼
// ������λֵ�˲��ķ���
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
    //ð������
    for(j=0;j<99;j++)
             {
                 for(i=0;i<99-j;i++)
                     {
                     //���鲢������
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
 * ��̬���
 * �������������ת�������Ĺ�ϵ
 * Actuator��������
 * Encorder������ת��
 -----------------------------------*/
float dyn_zero_val=0;//��̬���
float dyn_zero_KP=-1.4;//�����ǲ�������
float Real_KP=0;//�����ٶȲ�������
void dyn_zero(uint16 Actuator,int16 Encorder)
{
    if((STEER_MOTOR_MID-20)>Actuator||(STEER_MOTOR_MID+20)<Actuator)
        {
        //�����ǲ���
            dyn_zero_val=dyn_zero_KP*(Actuator-STEER_MOTOR_MID)/134+Pitch_Zero;
        //����������
//            if(Actuator>STEER_MOTOR_MID)
//                dyn_zero_val+=Real_KP*Encorder;
//            else if(Actuator<STEER_MOTOR_MID)
//                dyn_zero_val-=Real_KP*Encorder;
        }
        else
            dyn_zero_val=Pitch_Zero;
}

/*-------------------------------------------------------
 * ���Թ�һ��
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
 * �����˲���ʼ��
 ---------------------------------------------------*/
void move_filter_init(move_filter_struct *move_filter)
    {
        move_filter->data_average = 0;
        move_filter->data_sum = 0;
        move_filter->index = 0;
        //���û�������С
        move_filter->buffer_size = MOVE_AVERAGE_SIZE;

        uint8 i;
        for(i=0; i < move_filter->buffer_size; i++)
        {
            move_filter->data_buffer[i] = 0;
        }
    }

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ����ƽ���˲�����
//  @param      void
//  @return     void
//  @since      ��Ҫ���ڶ������˲����洢Ŀ�����������n�����ݣ������ƽ��ֵ
//-------------------------------------------------------------------------------------------------------------------
void move_filter_calc(move_filter_struct *move_filter, int16 new_data)
{
    //�����µ���ֵ ��ȥ��ĩβ����ֵ ������µĺ�
    move_filter->data_sum = move_filter->data_sum + new_data - move_filter->data_buffer[move_filter->index];
    //������ƽ��ֵ
    move_filter->data_average =(int16)(move_filter->data_sum / move_filter->buffer_size);

    //������д�뻺����
    move_filter->data_buffer[move_filter->index] = new_data;
    move_filter->index++;
    if(move_filter->buffer_size <= move_filter->index)
    {
        move_filter->index = 0;
    }
}

/*-----------------------------------------------------------
 * �����˲���������ֵ
 -------------------------------------------------------------*/
void hubu_icm20602_result(void)
{
    get_icm20602_accdata_spi();
    get_icm20602_gyro_spi();
    move_filter_calc(&gyro_fliter,icm_gyro_x);//�����ǻ����˲�
    angle=angle_calc(icm_acc_y,gyro_fliter.data_average-icm_gyro_x_err)-Pitch_Zero;
    angle_dot=(gyro_fliter.data_average-icm_gyro_x_err);
}
//-----------------------------------------------------------------------------------------------------------

//---------------------------------------
//ֱ��ƽ�����
float Pitch_Zero=-276;//����Pitch��Ƕ���� 135 1300amh
uint8 Velocity_Ctrl_cnt=0,angle_control_cnt=0;//ʱ���������
int16 momentum_duty=0;//������ռ�ձ�
int16 ECPULSE1=0;//�����ֱ���������ֵ

//------������ ��ʼ��---------------
void Encorder_Init(void)
{
       gpt12_init(GPT12_T5,  GPT12_T5INB_P10_3, GPT12_T5EUDB_P10_1);//
       gpt12_init(GPT12_T2,GPT12_T2INB_P33_7,GPT12_T2EUDB_P33_6);//
}

/**************************************************************************
 * ��ȡ�����ֱ�������ֵ
**************************************************************************/
//GPT12_T5 ����
//GPT12_T2 ������
int16 Encorder_get(GPTN_enum gptn)
{
    int16 ECPULSE_val=0;
    ECPULSE_val = gpt12_get(gptn);// �����ַ��� ĸ�������1
    gpt12_clear(gptn);
    return ECPULSE_val;
}
/**************************************************************
 * �����ֳ�ʼ��
***************************************************************/
void Momentum_int(void)
{
    gpio_init(MOMENTU_DIR, GPO, 0, PUSHPULL);// dir
    gtm_pwm_init(MOMENTU_PWM, 17000, 0);//pwm
}

/********************************************************
 * �����ֵ������
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
 * ����ֱ������
 ---------------------------------------------------------*/
void balance_control(void)//2ms�ж�
{
      Velocity_Ctrl_cnt++;
      angle_control_cnt++;
      if(Velocity_Ctrl_cnt>=50)//100ms �ٶȻ�
      {
          Velocity_Ctrl_cnt=0;
          ECPULSE1=Encorder_get(GPT12_T5);
          Velocity_Control(-ECPULSE1);
      }
      if(angle_control_cnt>=5)//10ms �ǶȻ�
      {
        angle_control_cnt=0;
        angle_control(momentum_PID.out,angle+dyn_zero_val);
      }
      momentum_duty=angular_control(angle_PID.out,angle_dot);//2ms ���ٶȻ�
      if(momentum_duty<-700)momentum_duty-=700;  //����
      else if(momentum_duty>700)momentum_duty+=700;
       momentum_duty=limit_ab(momentum_duty,-8000,8000);   //���PWM�޷�
      if(angle>1200||angle<-1200)//ˤ������
      {
          momentum_PID.integral=0;//�ٶȻ�����
          momentum_duty=0;
          angular_PID.integral=0;//���ٶȻ�����
      }

      MomentumCtrl(momentum_duty);//���ƶ�����ת��
}
/********************************************************
 * �ٶȻ�
 * PID
 ********************************************************/
float Velocity_Control(int16 actual)
{
    static float Encoder;
    float Encoder_Least =0;
    Encoder_Least = actual;                                  //�ٶ��˲�
    Encoder *= 0.7;                                           //һ�׵�ͨ�˲���
    Encoder += Encoder_Least*0.3;                             //һ�׵�ͨ�˲�
    momentum_PID.error = Encoder; //����ƫ��
    momentum_PID.integral += momentum_PID.error; //�ۼƻ���
    momentum_PID.integral=limit_ab(momentum_PID.integral ,- momentum_PID.I_L , momentum_PID.I_L ); //�����޷�
    momentum_PID.d_error = momentum_PID.error - momentum_PID.last_error; //����ƫ��֮��
    momentum_PID.out = momentum_PID.Kp * momentum_PID.error + momentum_PID.Ki * momentum_PID.integral + momentum_PID.Kd*momentum_PID.d_error;//�������ֵ
    momentum_PID.last_error = momentum_PID.error;//����ƫ��
    return momentum_PID.out;
}
/********************************************************************************
 * ���ٶȻ�
 * PI
 * ���ַ���ֵ��Ҫȷ��
 *****************************************************************************/
float angular_control(float target,float actual)
{
    angular_PID.error=target-actual;//����ƫ��
    angular_PID.integral+=angular_PID.error;   //�ۻ����
    angular_PID.integral=limit_ab(angular_PID.integral,-angular_PID.I_L,angular_PID.I_L); //�����޷�
//    if(angular_PID.error>80||angular_PID.error<-80)angular_PID.integral=0;//���ַ���
    angular_PID.d_error=angular_PID.error-angular_PID.last_error;  //�������֮��
    angular_PID.out=angular_PID.Kp*angular_PID.error+angular_PID.Ki*angular_PID.integral+angular_PID.Kd*angular_PID.d_error;//��������ֵ
    angular_PID.last_error=angular_PID.error;
    return angular_PID.out;
}
/********************************************************************************
 * �������ٶȻ�
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
* �ǶȻ�
* PD
**************************************************************************/
float angle_control(float target,float actual)
{
    angle_PID.error=target-actual;//����ƫ��
//    angle_PID.integral+=angle_PID.error;//�ۼ����
//  angle_PID.integral=limit_ab(angle_PID.integral,-angle_PID.I_L,angle_PID.I_L);//�����޷�
    angle_PID.d_error=angle_PID.error-angle_PID.last_error;//�������֮��
    angle_PID.out=angle_PID.Kp*angle_PID.error+angle_PID.Ki*angle_PID.integral+angle_PID.Kd*angle_PID.d_error;//�������ֵ
    angle_PID.last_error = angle_PID.error;
    return angle_PID.out;
}




