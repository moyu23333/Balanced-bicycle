/*
 * menu.c
 *
 *  Created on: 2022年1月24日
 *      Author: 摸鱼
 */
#include "headfile.h"
float motor_duty;
int16 xishu=0;
uint8 OptionNum = 7;           //此菜单选项个数
int8 *MainOption[]={"Balance set  ","steer_set ","Mechzero set ","Motor_PI_set ","ADC_date ","track_PID_set","track_scan"};//主菜单选项
int8 *Balance_name[]={"Angular","Angle","Velocity","steer_PID"};//直立三环选项
int8 *comple_name[]={"acc_ratio ","gyro_ratio","yijie_dt"};//互补
int8 *PID_name[]={"Kp ","Ki ","Kd","I_L","angle"};
float expect_angle=0;
float angular_num[]={0,0,0,0};//角速度环
float angle_num[]={0,0,0,0};//角度环
float Velocity_num[]={0,0,0,0};//速度环
float steer_num[]={0,0,0,0,0};//舵机平衡环
float track_PID_num[]={0,0,0,0};//电磁循迹PID
float comple_num[]={0,0,0};//互补滤波参数
float Rear_Motor_PI_num[]={0,0,0,0};//后轮增量PI环参数 P I I_L SPEED
int8 *Rear_PID_name[]={"Kp ","Ki ","Kd","I_L","SPEED","OFF  "};//后轮PI参数
float test_temp_num[]={0,0,0,0,0};//调试参数
_Bool Rear_Motor_FLAG=1;//增量后轮控制开关

/*****************************************************************************
 * 参数处理函数
 * units:单位
 * param：要处理的参数
 * way:加减    0：加   1：减
 ******************************************************************************/
void ParameterChanges_float(float units,float *param,uint8 way)
{
    if(way == 0)
        (*param) +=  units;
    else if(way == 1)
        (*param) -=  units;
}

//菜单参数初始化------------------------------------------------------------------------
void Parameters_int(void)
{

    angular_num[0]=angular_PID.Kp;  //读取角速度环参数
    angular_num[1]=angular_PID.Ki;
    angular_num[2]=angular_PID.Kd;
    angular_num[3]=angular_PID.I_L;

    angle_num[0]=angle_PID.Kp; //读取角度环参数
    angle_num[1]=angle_PID.Ki;
    angle_num[2]=angle_PID.Kd;
    angle_num[3]=angle_PID.I_L;

    Velocity_num[0]= momentum_PID.Kp;//读取速度环参数
    Velocity_num[1]=momentum_PID.Ki;
    Velocity_num[2]=momentum_PID.Kd;
    Velocity_num[3]=momentum_PID.I_L;

    Rear_Motor_PI_num[0]=Motor_Kp;//读取后轮PI环参数
    Rear_Motor_PI_num[1]=Motor_Ki;
    Rear_Motor_PI_num[2]=8000;//输出限幅
    Rear_Motor_PI_num[3]=real_speed;//期望速度
    //电磁循迹
    track_PID_num[0]=electr_track_PID.Kp;
    track_PID_num[1]=electr_track_PID.Ki;
    track_PID_num[2]=electr_track_PID.Kd;
    track_PID_num[3]=electr_track_PID.I_L;
    //舵机融合
    steer_num[0]=steering_PID.Kp;
    steer_num[1]=steering_PID.Ki;
    steer_num[2]=steering_PID.Kd;
    steer_num[3]=steering_PID.I_L;
    steer_num[4]=expect_angle;
}
void Parameters_reverse(void)
{
    angular_PID.Kp=angular_num[0];
    angular_PID.Ki=angular_num[1];
    angle_PID.Kp=angle_num[0];
    angle_PID.Kd=angle_num[2];
    momentum_PID.Kp=Velocity_num[0];
}
//菜单参数保存------------------------------------------------------------------------
void Parameters_save(void)
{

}

//子菜单界面-----------------------------------------------------------------


//舵机零点设置------------------------------------------------------------------------
void motor_set_Interface(void)
{
    lcd_clear(RGB565_BLACK);
    sd12_correct_lcd();
    systick_delay_ms(STM0,100);
}
// 界面显示名字---------------------------------------------------------------------------
// 入口参数 name：要显示的字符，number：字符个数
void Interface_dispayname(int8**name,uint8 number,uint8 pos)
{
    uint8 i=0;
    lcd_clear(MainInterface1Background);
    for(i=0;i<number;i++)
    lcd_showstr_color(0,i,name[i],MainInterface1Pen,MainInterface1Background);
    lcd_showstr_color(0,pos,name[pos],MainInterface1Pen,NowOption_Background);
}
//界面显示参数数据
//入口参数 num：要显示的数据来源，number：数据个数，integer：整数几位 decimal：小数几位 position：显示的横坐标位置
void Interface_dispaynum(float*num,float number,uint8 integer,uint8 decimal,uint16 position)
{
    uint8 i=0;
    for(i=0;i<number;i++)
   lcd_showfloat_color(position,i,*(num+i),integer,decimal,MainInterface1Pen,MainInterface1Background);
}

//角速度环设置界面-------------------------------------------------------------------
void angular_set_Interface(void)
{
    uint8 pos=0,lastpos=1;
    Interface_dispayname(PID_name,4,pos);
    lcd_showfloat_color(50,0,angular_num[0],ANGULAR_KP_DISP_INT,ANGULAR_KP_DISP_POINT,MainInterface1Pen,MainInterface1Background);
    lcd_showfloat_color(50,1,angular_num[1],ANGULAR_KI_DISP_INT,ANGULAR_KI_DISP_POINT,MainInterface1Pen,MainInterface1Background);
    lcd_showfloat_color(50,2,angular_num[2],ANGULAR_KD_DISP_INT,ANGULAR_KD_DISP_POINT,MainInterface1Pen,MainInterface1Background);
    lcd_showfloat_color(50,3,angular_num[3],ANGULAR_IL_DISP_INT,ANGULAR_IL_DISP_POINT,MainInterface1Pen,MainInterface1Background);
    while(TRUE)
    {
        uart_send_senser(momentum_duty,angular_PID.error,angular_PID.integral,angle_dot);
//        uart_send_senser(Momentum_fliter.data_average,momentum_duty,angular_PID.error,angle_dot*100);//角速度环输出 电机占空比 角度 角速度
        scan_key();
        switch(keyup)
        {
            case 1:
                lastpos=pos;
                if(pos>2) pos =0;
                else      pos++;
                lcd_showstr_color(0,pos,PID_name[pos],MainInterface1Pen,NowOption_Background);     //此刻选项改变
                lcd_showstr_color(0,lastpos,PID_name[lastpos],MainInterface1Pen,MainInterface1Background);  //上一项背景
                break;
            case 2:
                switch(pos)//加
                {
                    case 0:
                        ParameterChanges_float(ANGULAR_KP_GRAD,angular_num+pos,0);//P
                        lcd_showfloat_color(50,pos,*(angular_num+pos),ANGULAR_KP_DISP_INT,ANGULAR_KP_DISP_POINT,MainInterface1Pen,MainInterface1Background);
                        break;
                    case 1:
                        ParameterChanges_float(ANGULAR_KI_GRAD,angular_num+pos,0);//I
                        lcd_showfloat_color(50,pos,*(angular_num+pos),ANGULAR_KI_DISP_INT,ANGULAR_KI_DISP_POINT,MainInterface1Pen,MainInterface1Background);
                        break;
                    case 2:
                        ParameterChanges_float(ANGULAR_KD_GRAD,angular_num+pos,0);//D
                        lcd_showfloat_color(50,pos,*(angular_num+pos),ANGULAR_KD_DISP_INT,ANGULAR_KD_DISP_POINT,MainInterface1Pen,MainInterface1Background);
                        break;
                    case 3:
                        ParameterChanges_float(ANGULAR_IL_GRAD,angular_num+pos,0);//I_L
                        lcd_showfloat_color(50,pos,*(angular_num+pos),ANGULAR_IL_DISP_INT,ANGULAR_IL_DISP_POINT,MainInterface1Pen,MainInterface1Background);
                        break;
                    default:break;
                }
                //参数保存
                angular_PID.Kp=angular_num[0];
                angular_PID.Ki=angular_num[1];
                angular_PID.Kd=angular_num[2];
                angular_PID.I_L=angular_num[3];
                break;
            case 3:
                switch(pos)//减
                {
                    case 0:
                        ParameterChanges_float(ANGULAR_KP_GRAD,angular_num+pos,1);//P
                        lcd_showfloat_color(50,pos,*(angular_num+pos),ANGULAR_KP_DISP_INT,ANGULAR_KP_DISP_POINT,MainInterface1Pen,MainInterface1Background);
                        break;
                    case 1:
                        ParameterChanges_float(ANGULAR_KI_GRAD,angular_num+pos,1);//I
                        lcd_showfloat_color(50,pos,*(angular_num+pos),ANGULAR_KI_DISP_INT,ANGULAR_KI_DISP_POINT,MainInterface1Pen,MainInterface1Background);
                        break;
                    case 2:
                        ParameterChanges_float(ANGULAR_KD_GRAD,angular_num+pos,1);//D
                        lcd_showfloat_color(50,pos,*(angular_num+pos),ANGULAR_KD_DISP_INT,ANGULAR_KD_DISP_POINT,MainInterface1Pen,MainInterface1Background);
                        break;
                    case 3:
                        ParameterChanges_float(ANGULAR_IL_GRAD,angular_num+pos,1);//I_L
                        lcd_showfloat_color(50,pos,*(angular_num+pos),ANGULAR_IL_DISP_INT,ANGULAR_IL_DISP_POINT,MainInterface1Pen,MainInterface1Background);
                        break;
                    default:break;
                }
                //参数保存
                angular_PID.Kp=angular_num[0];
                angular_PID.Ki=angular_num[1];
                angular_PID.Kd=angular_num[2];
                angular_PID.I_L=angular_num[3];
                break;
            case 4:
                return;break;
            default:break;
        }
//        printf("运行时间:%ld\n",systick_getval_us(STM1));
    }
}
//角度环设置界面------------------------------------------------------------------------
void angle_set_Interface(void)
{
    uint8 pos=0,lastpos=1;
    Interface_dispayname(PID_name,4,pos);
    lcd_showfloat_color(50,0,angle_num[0],ANGLE_KP_DISP_INT,ANGLE_KP_DISP_POINT,MainInterface1Pen,MainInterface1Background);
    lcd_showfloat_color(50,1,angle_num[1],ANGLE_KI_DISP_INT,ANGLE_KI_DISP_POINT,MainInterface1Pen,MainInterface1Background);
    lcd_showfloat_color(50,2,angle_num[2],ANGLE_KD_DISP_INT,ANGLE_KD_DISP_POINT,MainInterface1Pen,MainInterface1Background);
    lcd_showfloat_color(50,3,angle_num[3],ANGLE_IL_DISP_INT,ANGLE_IL_DISP_POINT,MainInterface1Pen,MainInterface1Background);
    while(TRUE)
    {
//          uart_send_senser(Momentum_fliter.data_average,angle_PID.out,angular_PID.out,momentum_PID.out);
//        uart_send_senser(momentum_duty, angle_PID.out, angle, angle_dot);
        uart_send_senser(momentum_duty,angle_PID.error,angle_PID.out,angle);//角速度环输出 电机占空比 角度 角速度
        scan_key();
        switch(keyup)
        {
            case 1:
                lastpos=pos;
                if(pos>2) pos =0;
                else      pos++;
                lcd_showstr_color(0,pos,PID_name[pos],MainInterface1Pen,NowOption_Background);     //此刻选项改变
                lcd_showstr_color(0,lastpos,PID_name[lastpos],MainInterface1Pen,MainInterface1Background);  //上一项背景
                break;
            case 2://加
                switch(pos)
                {
                    case 0:
                        ParameterChanges_float(ANGLE_KP_GRAD,angle_num+pos,0);//P
                        lcd_showfloat_color(50,pos,*(angle_num+pos),ANGLE_KP_DISP_INT,ANGLE_KP_DISP_POINT,MainInterface1Pen,MainInterface1Background);
                        break;
                    case 1:
                        ParameterChanges_float(ANGLE_KI_GRAD,angle_num+pos,0);//I
                        lcd_showfloat_color(50,pos,*(angle_num+pos),ANGLE_KI_DISP_INT,ANGLE_KI_DISP_POINT,MainInterface1Pen,MainInterface1Background);
                        break;
                    case 2:
                        ParameterChanges_float(ANGLE_KD_GRAD,angle_num+pos,0);//D
                        lcd_showfloat_color(50,pos,*(angle_num+pos),ANGLE_KD_DISP_INT,ANGLE_KD_DISP_POINT,MainInterface1Pen,MainInterface1Background);
                        break;
                    case 3:
                        ParameterChanges_float(ANGLE_IL_GRAD,angle_num+pos,0);//I_L
                        lcd_showfloat_color(50,pos,*(angle_num+pos),ANGLE_IL_DISP_INT,ANGLE_IL_DISP_POINT,MainInterface1Pen,MainInterface1Background);
                        break;
                    default:break;
                }
                //参数保存
                angle_PID.Kp=angle_num[0];
                angle_PID.Ki=angle_num[1];
                angle_PID.Kd=angle_num[2];
                angle_PID.I_L=angle_num[3];
                break;
            case 3://减
                switch(pos)
                  {
                      case 0:
                          ParameterChanges_float(ANGLE_KP_GRAD,angle_num+pos,1);//P
                          lcd_showfloat_color(50,pos,*(angle_num+pos),ANGLE_KP_DISP_INT,ANGLE_KP_DISP_POINT,MainInterface1Pen,MainInterface1Background);
                          break;
                      case 1:
                          ParameterChanges_float(ANGLE_KI_GRAD,angle_num+pos,1);//I
                          lcd_showfloat_color(50,pos,*(angle_num+pos),ANGLE_KI_DISP_INT,ANGLE_KI_DISP_POINT,MainInterface1Pen,MainInterface1Background);
                          break;
                      case 2:
                          ParameterChanges_float(ANGLE_KD_GRAD,angle_num+pos,1);//D
                          lcd_showfloat_color(50,pos,*(angle_num+pos),ANGLE_KD_DISP_INT,ANGLE_KD_DISP_POINT,MainInterface1Pen,MainInterface1Background);
                          break;
                      case 3:
                          ParameterChanges_float(ANGLE_IL_GRAD,angle_num+pos,1);//I_L
                          lcd_showfloat_color(50,pos,*(angle_num+pos),ANGLE_IL_DISP_INT,ANGLE_IL_DISP_POINT,MainInterface1Pen,MainInterface1Background);
                          break;
                      default:break;
                  }
                //参数保存
                angle_PID.Kp=angle_num[0];
                angle_PID.Ki=angle_num[1];
                angle_PID.Kd=angle_num[2];
                angle_PID.I_L=angle_num[3];
                break;
            case 4:
                return;break;
            default:break;
        }
    }
}
//速度环设置界面-------------------------------------------------------------------
void  Velocity_set_Interface(void)
{
    uint8 pos=0,lastpos=1;
    Interface_dispayname(PID_name,4,pos);
    lcd_showfloat_color(50,0,Velocity_num[0],VELOCITY_KP_DISP_INT,VELOCITY_KP_DISP_POINT,MainInterface1Pen,MainInterface1Background);
    lcd_showfloat_color(50,1,Velocity_num[1],VELOCITY_KI_DISP_INT,VELOCITY_KI_DISP_POINT,MainInterface1Pen,MainInterface1Background);
    lcd_showfloat_color(50,2,Velocity_num[2],VELOCITY_KD_DISP_INT,VELOCITY_KD_DISP_POINT,MainInterface1Pen,MainInterface1Background);
    lcd_showfloat_color(50,3,Velocity_num[3],VELOCITY_IL_DISP_INT,VELOCITY_IL_DISP_POINT,MainInterface1Pen,MainInterface1Background);
    while(TRUE)
    {
        uart_send_senser(momentum_duty,momentum_PID.error,momentum_PID.integral,momentum_PID.out);//角速度环输出 电机占空比 角度 角速度
        scan_key();
        switch(keyup)
        {
            case 1:
                lastpos=pos;
                if(pos>2) pos =0;
                else      pos++;
                lcd_showstr_color(0,pos,PID_name[pos],MainInterface1Pen,NowOption_Background);     //此刻选项改变
                lcd_showstr_color(0,lastpos,PID_name[lastpos],MainInterface1Pen,MainInterface1Background);  //上一项背景
                break;
            case 2:
                switch(pos)//加
           {
                case 0:
                ParameterChanges_float(VELOCITY_KP_GRAD,Velocity_num+pos,0);//P
                lcd_showfloat_color(50,pos,*(Velocity_num+pos),VELOCITY_KP_DISP_INT,VELOCITY_KP_DISP_POINT,MainInterface1Pen,MainInterface1Background);
                break;
                case 1:
                ParameterChanges_float(VELOCITY_KI_GRAD,Velocity_num+pos,0);//I
                lcd_showfloat_color(50,pos,*(Velocity_num+pos),VELOCITY_KI_DISP_INT,VELOCITY_KI_DISP_POINT,MainInterface1Pen,MainInterface1Background);
                break;
                case 2:
                ParameterChanges_float(VELOCITY_KD_GRAD,Velocity_num+pos,0);//D
                lcd_showfloat_color(50,pos,*(Velocity_num+pos),VELOCITY_KD_DISP_INT,VELOCITY_KD_DISP_POINT,MainInterface1Pen,MainInterface1Background);
                break;
                case 3:
                ParameterChanges_float(VELOCITY_IL_GRAD,Velocity_num+pos,0);//I_L
                lcd_showfloat_color(50,pos,*(Velocity_num+pos),VELOCITY_IL_DISP_INT,VELOCITY_IL_DISP_POINT,MainInterface1Pen,MainInterface1Background);
                break;
                default:break;
            }
                //参数保存
                momentum_PID.Kp=Velocity_num[0];
                momentum_PID.Ki=Velocity_num[1];
                momentum_PID.Kd=Velocity_num[2];
                momentum_PID.I_L=Velocity_num[3];
                break;
            case 3:
                switch(pos)//减
           {
                case 0:
                ParameterChanges_float(VELOCITY_KP_GRAD,Velocity_num+pos,1);//P
                lcd_showfloat_color(50,pos,*(Velocity_num+pos),VELOCITY_KP_DISP_INT,VELOCITY_KP_DISP_POINT,MainInterface1Pen,MainInterface1Background);
                break;
                case 1:
                ParameterChanges_float(VELOCITY_KI_GRAD,Velocity_num+pos,1);//I
                lcd_showfloat_color(50,pos,*(Velocity_num+pos),VELOCITY_KI_DISP_INT,VELOCITY_KI_DISP_POINT,MainInterface1Pen,MainInterface1Background);
                break;
                case 2:
                ParameterChanges_float(VELOCITY_KD_GRAD,Velocity_num+pos,1);//D
                lcd_showfloat_color(50,pos,*(Velocity_num+pos),VELOCITY_KD_DISP_INT,VELOCITY_KD_DISP_POINT,MainInterface1Pen,MainInterface1Background);
                break;
                case 3:
                ParameterChanges_float(VELOCITY_IL_GRAD,Velocity_num+pos,1);//I_L
                lcd_showfloat_color(50,pos,*(Velocity_num+pos),VELOCITY_IL_DISP_INT,VELOCITY_IL_DISP_POINT,MainInterface1Pen,MainInterface1Background);
                break;
                default:break;
            }
                //参数保存
                momentum_PID.Kp=Velocity_num[0];
                momentum_PID.Ki=Velocity_num[1];
                momentum_PID.Kd=Velocity_num[2];
                momentum_PID.I_L=Velocity_num[3];
                break;
            case 4:
                return;break;
            default:break;
        }
    }
}
//舵机融合----------
void balance_steer_interface(void)
{
    uint8 pos=0,lastpos=1;
    Interface_dispayname(PID_name,5,pos);
    lcd_showfloat_color(50,0,steer_num[0],STEER_KP_DISP_INT,STEER_KP_DISP_POINT,MainInterface1Pen,MainInterface1Background);
    lcd_showfloat_color(50,1,steer_num[1],STEER_KI_DISP_INT,STEER_KI_DISP_POINT,MainInterface1Pen,MainInterface1Background);
    lcd_showfloat_color(50,2,steer_num[2],STEER_KD_DISP_INT,STEER_KD_DISP_POINT,MainInterface1Pen,MainInterface1Background);
    lcd_showfloat_color(50,3,steer_num[3],STEER_IL_DISP_INT,STEER_IL_DISP_POINT,MainInterface1Pen,MainInterface1Background);
    lcd_showfloat_color(50,4,steer_num[4],STEER_IL_DISP_INT,STEER_IL_DISP_POINT,MainInterface1Pen,MainInterface1Background);
    while(TRUE)
    {
        uart_send_senser(steering_PID.error,steering_PID.integral,steering_PID.out,electr_track_PID.out);//角速度环输出 电机占空比 角度 角速度
        scan_key();
        switch(keyup)
        {
            case 1:
                lastpos=pos;
                if(pos>3) pos =0;
                else      pos++;
                lcd_showstr_color(0,pos,PID_name[pos],MainInterface1Pen,NowOption_Background);     //此刻选项改变
                lcd_showstr_color(0,lastpos,PID_name[lastpos],MainInterface1Pen,MainInterface1Background);  //上一项背景
                break;
            case 2:
                switch(pos)//加
           {
                case 0:
                ParameterChanges_float(STEER_KP_GRAD,steer_num+pos,0);//P
                lcd_showfloat_color(50,pos,*(steer_num+pos),STEER_KP_DISP_INT,STEER_KP_DISP_POINT,MainInterface1Pen,MainInterface1Background);
                break;
                case 1:
                ParameterChanges_float(STEER_KI_GRAD,steer_num+pos,0);//I
                lcd_showfloat_color(50,pos,*(steer_num+pos),STEER_KI_DISP_INT,STEER_KI_DISP_POINT,MainInterface1Pen,MainInterface1Background);
                break;
                case 2:
                ParameterChanges_float(STEER_KD_GRAD,steer_num+pos,0);//D
                lcd_showfloat_color(50,pos,*(steer_num+pos),STEER_KD_DISP_INT,STEER_KD_DISP_POINT,MainInterface1Pen,MainInterface1Background);
                break;
                case 3:
                ParameterChanges_float(STEER_IL_GRAD,steer_num+pos,0);//I_L
                lcd_showfloat_color(50,pos,*(steer_num+pos),STEER_IL_DISP_INT,STEER_IL_DISP_POINT,MainInterface1Pen,MainInterface1Background);
                break;
                case 4:
                ParameterChanges_float(10,steer_num+pos,0);//打角角度
                lcd_showfloat_color(50,pos,*(steer_num+pos),3,1,MainInterface1Pen,MainInterface1Background);
                    break;
                default:break;
            }
                //参数保存
                steering_PID.Kp=steer_num[0];
                steering_PID.Ki=steer_num[1];
                steering_PID.Kd=steer_num[2];
                steering_PID.I_L=steer_num[3];
                expect_angle=steer_num[4];
                break;
            case 3:
                switch(pos)//减
           {
                case 0:
                ParameterChanges_float(STEER_KP_GRAD,steer_num+pos,1);//P
                lcd_showfloat_color(50,pos,*(steer_num+pos),STEER_KP_DISP_INT,STEER_KP_DISP_POINT,MainInterface1Pen,MainInterface1Background);
                break;
                case 1:
                ParameterChanges_float(STEER_KI_GRAD,steer_num+pos,1);//I
                lcd_showfloat_color(50,pos,*(steer_num+pos),STEER_KI_DISP_INT,STEER_KI_DISP_POINT,MainInterface1Pen,MainInterface1Background);
                break;
                case 2:
                ParameterChanges_float(STEER_KD_GRAD,steer_num+pos,1);//D
                lcd_showfloat_color(50,pos,*(steer_num+pos),STEER_KD_DISP_INT,STEER_KD_DISP_POINT,MainInterface1Pen,MainInterface1Background);
                break;
                case 3:
                ParameterChanges_float(STEER_IL_GRAD,steer_num+pos,1);//I_L
                lcd_showfloat_color(50,pos,*(steer_num+pos),STEER_IL_DISP_INT,STEER_IL_DISP_POINT,MainInterface1Pen,MainInterface1Background);
                break;
                case 4:
                ParameterChanges_float(10,steer_num+pos,1);//打角角度
                lcd_showfloat_color(50,pos,*(steer_num+pos),3,1,MainInterface1Pen,MainInterface1Background);
                break;
                default:break;
            }
                //参数保存
                steering_PID.Kp=steer_num[0];
                steering_PID.Ki=steer_num[1];
                steering_PID.Kd=steer_num[2];
                steering_PID.I_L=steer_num[3];
                expect_angle=steer_num[4];
                break;
            case 4:
                return;break;
            default:break;
        }
    }

}
//直立三环调参界面-----------------------------------------------------------------------
void Balance_set_Interface(void)
{
    uint8 pos=0,lastpos=1;
    lcd_clear(MainInterface1Background);
    //Balance_name
    lcd_showstr_color(0,0,Balance_name[0],MainInterface1Pen,NowOption_Background);
    lcd_showstr_color(0,1,Balance_name[1],MainInterface1Pen,MainInterface1Background);
    lcd_showstr_color(0,2,Balance_name[2],MainInterface1Pen,MainInterface1Background);
    lcd_showstr_color(0,3,Balance_name[3],MainInterface1Pen,MainInterface1Background);
while(1)
        {
    scan_key();
    switch(keyup)
    {
        case 1:
        lastpos=pos;
        if(pos>2) pos =0;
        else      pos++;
        lcd_showstr_color(0,pos,Balance_name[pos],MainInterface1Pen,NowOption_Background);     //此刻选项改变
        lcd_showstr_color(0,lastpos,Balance_name[lastpos],MainInterface1Pen,MainInterface1Background);  //上一项背景
        break;
        case 2:
        lastpos=pos;
        if(pos<1) pos =2;
        else      pos--;
        lcd_showstr_color(0,pos,Balance_name[pos],MainInterface1Pen,NowOption_Background);     //此刻选项改变
        lcd_showstr_color(0,lastpos,Balance_name[lastpos],MainInterface1Pen,MainInterface1Background);  //上一项背景
        break;
        case 3:
            switch(pos)
            {
                case 0:
                    angular_set_Interface();
                    break;
                case 1:
                    angle_set_Interface();
                    break;
                case 2:
                    Velocity_set_Interface();
                    break;
                case 3:
                    balance_steer_interface();
                    break;
                default:break;
            }
            lcd_clear(MainInterface1Background);
            Interface_dispayname(Balance_name,4,pos);
            break;
        case 4:
            return;
        default:break;
    }
        }
}

//设定机械零点------------------------
void Mechzero_set(void)
{
//    angular_PID.Kp=0;
//    angular_PID.Ki=0;
//    angle_PID.Kp=0;
//    angle_PID.Kd=0;
//    momentum_PID.Kp=0;

    lcd_clear(MainInterface1Background);
    lcd_showstr_color(0,0,"Mechzero: ",MainInterface1Pen,NowOption_Background);
    while(1)
    {
        scan_key();
        switch(keyup)
        {
            case 1:
                Pitch_Zero=angle;
                break;
            case 2:
                Pitch_Zero=0;//清零
                break;
            case 3:
                break;
            case 4:
                Parameters_reverse();//恢复参数
                return;
            default:break;
        }
        lcd_showfloat_color(85,0,angle,4,2,MainInterface1Pen,MainInterface1Background);
    }
}
//动态零点调节---------------------------------------------------------------------------------
void dyn_zero_set(void)
{






}
//后轮增量PI调节-------------------------------------------------------------------------------------------
void Rear_Motor_PI_set(void)
{
    uint8 pos=0,lastpos=1;
    lcd_clear(MainInterface1Background);
    Interface_dispayname(Rear_PID_name,6,pos);
    lcd_showfloat_color(50,0,Rear_Motor_PI_num[0],REAR_KP_DISP_INT,REAR_KP_DISP_POINT,MainInterface1Pen,MainInterface1Background);
    lcd_showfloat_color(50,1,Rear_Motor_PI_num[1],REAR_KI_DISP_INT,REAR_KI_DISP_POINT,MainInterface1Pen,MainInterface1Background);
    lcd_showfloat_color(50,3,Rear_Motor_PI_num[2],REAR_IL_DISP_INT,REAR_IL_DISP_POINT,MainInterface1Pen,MainInterface1Background);
    lcd_showfloat_color(50,4,Rear_Motor_PI_num[3],REAR_SPEED_DISP_INT,REAR_SPEED_DISP_POINT,MainInterface1Pen,MainInterface1Background);
    while(1)
    {
        uart_send_senser(ECPULSE2, rear_temp_pwm, real_speed, Motor_Bias);
//        uart_send_senser_10(track_left_adc*100,track_right_adc*100,ramp_adc*100,neiba_left_adc*100,neiba_right_adc*100,shu_left_adc*100,shu_right_adc*100,adc_error*100,adc_sum*100,adc_diff*100);
        scan_key();
        switch(keyup)
        {
            case 1://移动
                lastpos=pos;
                if(pos>4) pos =0;
                else      pos++;
                lcd_showstr_color(0,pos,Rear_PID_name[pos],MainInterface1Pen,NowOption_Background);     //此刻选项改变
                lcd_showstr_color(0,lastpos,Rear_PID_name[lastpos],MainInterface1Pen,MainInterface1Background);  //上一项背景
                break;
            case 2://加
                switch(pos)
                {
                    case 0:
                        ParameterChanges_float(REAR_KP_GRAD,Rear_Motor_PI_num,0);//P
                        lcd_showfloat_color(50,pos,*(Rear_Motor_PI_num),REAR_KP_DISP_INT,REAR_KP_DISP_POINT,MainInterface1Pen,MainInterface1Background);
                        break;
                    case 1:
                        ParameterChanges_float(REAR_KI_GRAD,Rear_Motor_PI_num+1,0);//I
                        lcd_showfloat_color(50,pos,*(Rear_Motor_PI_num+1),REAR_KI_DISP_INT,REAR_KI_DISP_POINT,MainInterface1Pen,MainInterface1Background);
                        break;
                    case 2:break;
                    case 3://IL
                        ParameterChanges_float(REAR_IL_GRAD,Rear_Motor_PI_num+2,0);//I_l
                        lcd_showfloat_color(50,pos,*(Rear_Motor_PI_num+2),REAR_IL_DISP_INT,REAR_IL_DISP_POINT,MainInterface1Pen,MainInterface1Background);
                        break;
                    case 4:
                        ParameterChanges_float(REAR_SPEED_GRAD,Rear_Motor_PI_num+3,0);//SPEED
                        lcd_showfloat_color(50,pos,*(Rear_Motor_PI_num+3),REAR_SPEED_DISP_INT,REAR_SPEED_DISP_POINT,MainInterface1Pen,MainInterface1Background);
                        break;
                    case 5:
                        Rear_Motor_FLAG=1;
                        Rear_PID_name[5]="ON  ";
                        lcd_showstr_color(0,pos,"ON  ",MainInterface1Pen,NowOption_Background);
                        break;
                    default:break;
                }
                //参数保存
                Motor_Kp  =Rear_Motor_PI_num[0];
                Motor_Ki  =Rear_Motor_PI_num[1];
                real_speed=Rear_Motor_PI_num[3];
                break;
             case 3://减
                 switch(pos)
                 {
                     case 0:
                         ParameterChanges_float(REAR_KP_GRAD,Rear_Motor_PI_num,1);//P
                         lcd_showfloat_color(50,pos,*(Rear_Motor_PI_num),REAR_KP_DISP_INT,REAR_KP_DISP_POINT,MainInterface1Pen,MainInterface1Background);
                         break;
                         case 1:
                         ParameterChanges_float(REAR_KI_GRAD,Rear_Motor_PI_num+1,1);//I
                         lcd_showfloat_color(50,pos,*(Rear_Motor_PI_num+1),REAR_KI_DISP_INT,REAR_KI_DISP_POINT,MainInterface1Pen,MainInterface1Background);
                         break;
                         case 2:break;
                         case 3://IL
                         ParameterChanges_float(REAR_IL_GRAD,Rear_Motor_PI_num+2,1);//I_l
                         lcd_showfloat_color(50,pos,*(Rear_Motor_PI_num+2),REAR_IL_DISP_INT,REAR_IL_DISP_POINT,MainInterface1Pen,MainInterface1Background);
                         break;
                         case 4://SPEED
                         ParameterChanges_float(REAR_SPEED_GRAD,Rear_Motor_PI_num+3,1);//SPEED
                         lcd_showfloat_color(50,pos,*(Rear_Motor_PI_num+3),REAR_SPEED_DISP_INT,REAR_SPEED_DISP_POINT,MainInterface1Pen,MainInterface1Background);
                         break;
                         case 5:
                         Rear_Motor_FLAG=0;
                         Rear_PID_name[5]="OFF ";
                         lcd_showstr_color(0,pos,"OFF ",MainInterface1Pen,NowOption_Background);
                         break;
                         default:break;
                 }
                 //参数保存
                 Motor_Kp=Rear_Motor_PI_num[0];
                 Motor_Ki=Rear_Motor_PI_num[1];
                 real_speed=Rear_Motor_PI_num[3];
                 break;
             case 4:return;break;
             default:break;
        }
    }
}
//ADC数据显示界面
void ADC_date_interface(void)
{
    lcd_clear(RGB565_WHITE);
    lcd_showstr(0,0,"ADC_date ");
    lcd_showstr(0,0,"xl:");
    lcd_showstr(0,1,"xr:");
    lcd_showstr(0,2,"mid:");
    lcd_showstr(0,3,"/l:");
    lcd_showstr(0,4,"/r:");
    lcd_showstr(0,5,"|l:");
    lcd_showstr(0,6,"|r:");
//    lcd_showstr(0,7,"err:");
//    lcd_showstr(74,5,"sum:");
//    lcd_showstr(74,6,"dif:");

//    float temp_kp=electr_track_PID.Kp,temp_kd=electr_track_PID.Kd;//储存临时变量
//    electr_track_PID.Kp=0;
//    electr_track_PID.Kd=0;
//    pit_disable_interrupt(CCU6_0,PIT_CH1);
    pit_disable_interrupt(CCU6_0,PIT_CH0);
    while(1)
    {
//     adc_result();
//     uart_send_senser_10(track_left_adc*100,track_right_adc*100,ramp_adc*100,neiba_left_adc*100,neiba_right_adc*100,shu_left_adc*100,shu_right_adc*100,adc_error*100,adc_sum*100,adc_diff*100);
     lcd_showfloat(26,0,track_left_adc,3,2);
     lcd_showfloat(26,1,track_right_adc,3,2);
     lcd_showfloat(34,2,ramp_adc,3,2);
     lcd_showfloat(26,3,neiba_left_adc,3,2);
     lcd_showfloat(26,4,neiba_right_adc,3,2);
     lcd_showfloat(26,5,shu_left_adc,3,2);
     lcd_showfloat(26,6,shu_right_adc,3,2);
//   lcd_showfloat(32,7,adc_error,3,2);
//   lcd_showfloat(108,5,adc_sum,3,2);
//   lcd_showfloat(108,6,adc_diff,3,2);
     scan_key();
     if(keyup==4)
         {
//         electr_track_PID.Kp=temp_kp;
//         electr_track_PID.Kd=temp_kd;
//         pit_enable_interrupt(CCU6_0,PIT_CH1);
         pit_enable_interrupt(CCU6_0,PIT_CH0);
         return;
         }
    }
}
//电磁循迹PID调节----------------------------------------------------------------------------------------------
void ADC_track_PID_set(void)
{
   uint8 pos=0,lastpos=1;
   Interface_dispayname(PID_name,4,pos);
   lcd_showfloat_color(50,0,track_PID_num[0],TRACK_KP_DISP_INT,TRACK_KP_DISP_POINT,MainInterface1Pen,MainInterface1Background);
   lcd_showfloat_color(50,1,track_PID_num[1],TRACK_KI_DISP_INT,TRACK_KI_DISP_POINT,MainInterface1Pen,MainInterface1Background);
   lcd_showfloat_color(50,2,track_PID_num[2],TRACK_KD_DISP_INT,TRACK_KD_DISP_POINT,MainInterface1Pen,MainInterface1Background);
   lcd_showfloat_color(50,3,track_PID_num[3],TRACK_IL_DISP_INT,TRACK_IL_DISP_POINT,MainInterface1Pen,MainInterface1Background);
   while(TRUE)
   {
       uart_send_senser(adc_error*100,electr_track_PID.out*100,adc_sum*100,adc_diff*100);//
       scan_key();
       switch(keyup)
       {
           case 1:
           lastpos=pos;
           if(pos>2) pos =0;
           else      pos++;
           lcd_showstr_color(0,pos,PID_name[pos],MainInterface1Pen,NowOption_Background);     //此刻选项改变
           lcd_showstr_color(0,lastpos,PID_name[lastpos],MainInterface1Pen,MainInterface1Background);  //上一项背景
           break;
           case 2:
           switch(pos)//加
              {
                case 0:
                ParameterChanges_float(TRACK_KP_GRAD,track_PID_num+pos,0);//P
                lcd_showfloat_color(50,pos,*(track_PID_num+pos),TRACK_KP_DISP_INT,TRACK_KP_DISP_POINT,MainInterface1Pen,MainInterface1Background);
                break;
                case 1:
                ParameterChanges_float(TRACK_KI_GRAD,track_PID_num+pos,0);//I
                lcd_showfloat_color(50,pos,*(track_PID_num+pos),TRACK_KI_DISP_INT,TRACK_KI_DISP_POINT,MainInterface1Pen,MainInterface1Background);
                break;
                case 2:
                ParameterChanges_float(TRACK_KD_GRAD,track_PID_num+pos,0);//D
                lcd_showfloat_color(50,pos,*(track_PID_num+pos),TRACK_KD_DISP_INT,TRACK_KD_DISP_POINT,MainInterface1Pen,MainInterface1Background);
                break;
                case 3:
                ParameterChanges_float(TRACK_IL_GRAD,track_PID_num+pos,0);//I_L
                lcd_showfloat_color(50,pos,*(track_PID_num+pos),TRACK_IL_DISP_INT,TRACK_IL_DISP_POINT,MainInterface1Pen,MainInterface1Background);
                break;
                default:break;
               }
                   //参数保存
                   electr_track_PID.Kp=track_PID_num[0];
                   electr_track_PID.Ki=track_PID_num[1];
                   electr_track_PID.Kd=track_PID_num[2];
                   electr_track_PID.I_L=track_PID_num[3];
                   break;
               case 3:
                   switch(pos)//减
              {
                   case 0:
                   ParameterChanges_float(TRACK_KP_GRAD,track_PID_num+pos,1);//P
                   lcd_showfloat_color(50,pos,*(track_PID_num+pos),TRACK_KP_DISP_INT,TRACK_KP_DISP_POINT,MainInterface1Pen,MainInterface1Background);
                   break;
                   case 1:
                   ParameterChanges_float(TRACK_KI_GRAD,track_PID_num+pos,1);//I
                   lcd_showfloat_color(50,pos,*(track_PID_num+pos),TRACK_KI_DISP_INT,TRACK_KI_DISP_POINT,MainInterface1Pen,MainInterface1Background);
                   break;
                   case 2:
                   ParameterChanges_float(TRACK_KD_GRAD,track_PID_num+pos,1);//D
                   lcd_showfloat_color(50,pos,*(track_PID_num+pos),TRACK_KD_DISP_INT,TRACK_KD_DISP_POINT,MainInterface1Pen,MainInterface1Background);
                   break;
                   case 3:
                   ParameterChanges_float(TRACK_IL_GRAD,track_PID_num+pos,1);//I_L
                   lcd_showfloat_color(50,pos,*(track_PID_num+pos),TRACK_IL_DISP_INT,TRACK_IL_DISP_POINT,MainInterface1Pen,MainInterface1Background);
                   break;
                   default:break;
               }
                   //参数保存
                   electr_track_PID.Kp=track_PID_num[0];
                   electr_track_PID.Ki=track_PID_num[1];
                   electr_track_PID.Kd=track_PID_num[2];
                   electr_track_PID.I_L=track_PID_num[3];
                   break;
               case 4:
                   return;break;
               default:break;
       }
   }
}
//调试参数
void test_Interface(void)
{
    uint8 pos=0,lastpos=0;
    int8*test_name[]={"num1","num2","num3","num4","num5"};
    Interface_dispayname(test_name,5,pos);
   while(1)
   {
       scan_key();
       switch(keyup)
       {
       case 1:
       lastpos=pos;
       if(pos>3) pos =0;
       else      pos++;
       lcd_showstr_color(0,pos,test_name[pos],MainInterface1Pen,NowOption_Background);     //此刻选项改变
       lcd_showstr_color(0,lastpos,test_name[lastpos],MainInterface1Pen,MainInterface1Background);  //上一项背景
       break;
       case 2:
           switch(pos)
           {
               case 0:
                   ParameterChanges_float(0.2,test_temp_num+pos,0);
                   lcd_showfloat_color(50,pos,*(test_temp_num+pos),3,1,MainInterface1Pen,MainInterface1Background);
                   break;
               case 1:
                   ParameterChanges_float(0.2,test_temp_num+pos,0);
                   lcd_showfloat_color(50,pos,*(test_temp_num+pos),3,1,MainInterface1Pen,MainInterface1Background);
                   break;
               case 2:
                   ParameterChanges_float(1,test_temp_num+pos,0);
                   lcd_showfloat_color(50,pos,*(test_temp_num+pos),3,1,MainInterface1Pen,MainInterface1Background);
                   break;
               case 3:
                   ParameterChanges_float(0.1,test_temp_num+pos,0);
                   lcd_showfloat_color(50,pos,*(test_temp_num+pos),3,1,MainInterface1Pen,MainInterface1Background);
                   break;
               case 4:
                   ParameterChanges_float(0.1,test_temp_num+pos,0);
                   lcd_showfloat_color(50,pos,*(test_temp_num+pos),3,1,MainInterface1Pen,MainInterface1Background);
                   break;
               default:break;
           }
           break;
       case 3:
           switch(pos)
           {
               case 0:
                   ParameterChanges_float(0.2,test_temp_num+pos,1);
                   lcd_showfloat_color(50,pos,*(test_temp_num+pos),3,1,MainInterface1Pen,MainInterface1Background);
                   break;
               case 1:
                   ParameterChanges_float(0.2,test_temp_num+pos,1);
                   lcd_showfloat_color(50,pos,*(test_temp_num+pos),3,1,MainInterface1Pen,MainInterface1Background);
                   break;
               case 2:
                   ParameterChanges_float(1,test_temp_num+pos,1);
                   lcd_showfloat_color(50,pos,*(test_temp_num+pos),3,1,MainInterface1Pen,MainInterface1Background);
                   break;
               case 3:
                   ParameterChanges_float(0.1,test_temp_num+pos,1);
                   lcd_showfloat_color(50,pos,*(test_temp_num+pos),3,1,MainInterface1Pen,MainInterface1Background);
                   break;
               case 4:
                   ParameterChanges_float(0.1,test_temp_num+pos,1);
                   lcd_showfloat_color(50,pos,*(test_temp_num+pos),3,1,MainInterface1Pen,MainInterface1Background);
                   break;
               default:break;
           }
           break;
       default:break;
       }
   }
}
//主菜单选项--------------------------------------------------------------------------------------------------
void Main_Interface(void)
{
    uint8 pos=0,lastpos=1,i=0;
    lcd_clear(MainInterface1Background); //清屏
    for(i=0; i<OptionNum; i++)
    {
        lcd_showstr_color(0,i,MainOption[i],MainInterface1Pen,MainInterface1Background);
    }
    lcd_showstr_color(0,pos,MainOption[pos],MainInterface1Pen,NowOption_Background);
    Parameters_int();
    while(1)
    {
       systick_delay_ms(STM0,50);
       scan_key();
     switch(keyup)
     {
         case 1://下
         lastpos=pos;
         if(pos>(OptionNum-2)) pos = 0;
         else      pos++;
         lcd_showstr_color(0,pos,MainOption[pos],MainInterface1Pen,NowOption_Background);     //此刻选项改变
         lcd_showstr_color(0,lastpos,MainOption[lastpos],MainInterface1Pen,MainInterface1Background);  //上一项背景
         break;
         case 2://上
         lastpos=pos;
         if(pos<1) pos = (OptionNum-1);
         else      pos--;
         lcd_showstr_color(0,pos,MainOption[pos],MainInterface1Pen,NowOption_Background);     //此刻选项改变
         lcd_showstr_color(0,lastpos,MainOption[lastpos],MainInterface1Pen,MainInterface1Background);  //上一项背景
         break;
             lastpos=pos;
             if(pos>(OptionNum-2)) pos = 0;
             else      pos++;
             lcd_showstr_color(0,pos,MainOption[pos],MainInterface1Pen,NowOption_Background);     //此刻选项改变
             lcd_showstr_color(0,lastpos,MainOption[lastpos],MainInterface1Pen,MainInterface1Background);  //上一项背景
             break;
         case 3://确定
             switch(pos)
            {
                 case 0:
                     Balance_set_Interface();
                     lcd_clear(MainInterface1Background);
                     for(i=0; i<OptionNum; i++)
                       {
                           lcd_showstr_color(0,i,MainOption[i],MainInterface1Pen,MainInterface1Background);
                       }
                       lcd_showstr_color(0,pos,MainOption[pos],MainInterface1Pen,NowOption_Background);
                     break;
                 case 1:
                 motor_set_Interface();
                 lcd_clear(MainInterface1Background);
                 for(i=0; i<OptionNum; i++)
                  {
                    lcd_showstr_color(0,i,MainOption[i],MainInterface1Pen,MainInterface1Background);
                  }
                    lcd_showstr_color(0,pos,MainOption[pos],MainInterface1Pen,NowOption_Background);
                    break;
                 case 2:
                     Mechzero_set();
                     lcd_clear(MainInterface1Background);
                     for(i=0; i<OptionNum; i++)
                        {
                         lcd_showstr_color(0,i,MainOption[i],MainInterface1Pen,MainInterface1Background);
                        }
                     lcd_showstr_color(0,pos,MainOption[pos],MainInterface1Pen,NowOption_Background);
                     break;
                 case 3:
                     Rear_Motor_PI_set();
                     lcd_clear(MainInterface1Background);
                     for(i=0; i<OptionNum; i++)
                        {
                         lcd_showstr_color(0,i,MainOption[i],MainInterface1Pen,MainInterface1Background);
                        }
                     lcd_showstr_color(0,pos,MainOption[pos],MainInterface1Pen,NowOption_Background);
                     break;
                 case 4:
                     ADC_date_interface();
                     lcd_clear(MainInterface1Background);
                     for(i=0; i<OptionNum; i++)
                        {
                         lcd_showstr_color(0,i,MainOption[i],MainInterface1Pen,MainInterface1Background);
                        }
                     lcd_showstr_color(0,pos,MainOption[pos],MainInterface1Pen,NowOption_Background);
                     break;
                 case 5:
                     ADC_track_PID_set();
                     lcd_clear(MainInterface1Background);
                     for(i=0; i<OptionNum; i++)
                     {
                        lcd_showstr_color(0,i,MainOption[i],MainInterface1Pen,MainInterface1Background);
                     }
                     lcd_showstr_color(0,pos,MainOption[pos],MainInterface1Pen,NowOption_Background);
                     break;
                 case 6:
                     track_scan();
                     lcd_clear(MainInterface1Background);
                     for(i=0; i<OptionNum; i++)
                     {
                        lcd_showstr_color(0,i,MainOption[i],MainInterface1Pen,MainInterface1Background);
                     }
                     lcd_showstr_color(0,pos,MainOption[pos],MainInterface1Pen,NowOption_Background);
                     break;
                 default :break;
            }break;
             case 4://返回
             return;break;
             default:break;
     }
    }
}
void KeySettingMenustart(void)//主函数菜单运行函数每五百毫秒判断一次
{
    if(systick_getval(STM0)<500)return;
    systick_start(STM0);
    get_key(KEY0);
    if(keyup==1)
    {
        Main_Interface();
        lcd_clear(MainInterface1Background);
    }
    lcd_showstr_color(40, 3,"lcd off", MainInterface1Pen, MainInterface1Background);
}
