#ifndef __ENCODER_H
#define __ENCODER_H
#include <sys.h>	 
/**************************************************************************
作    者：大鱼电子
淘宝地址：https://shop119207236.taobao.com
**************************************************************************/
#define ENCODER_TIM_PERIOD (u16)(65535)   //103的定时器是16位 2的16次方最大是65536
typedef struct
{
  float Kp;                       //比例系数Proportional
  float Ki;                       //积分系数Integral
  float Kd;                       //微分系数Derivative
 
  float Ek;                       //当前误差
  float Ek1;                      //前一次误差 e(k-1)
  float Ek2;                      //再前一次误差 e(k-2)
  float LocSum;                   //累计积分位置
  float SetValue_last;
  int Max_speed;		//百分比PWM使用，用来将百分比PWM目标转换为实际转速的满速值
  int Max_wheel_diff;
}PID_LocTypeDef;

extern u8 MY_TIM4_CC3_STA;
extern u8 MY_TIM4_CC4_STA;
extern u32 MY_TIM4_CNT;
extern int Encoder_Left;
extern int Encoder_Right;
extern int PID_MOTOR_R;
extern int PID_MOTOR_L;
extern PID_LocTypeDef PID_Left;
extern PID_LocTypeDef* sPID_Left;
extern PID_LocTypeDef PID_Right;
extern PID_LocTypeDef* sPID_Right;
void Encoder_Init_TIM2(void);
void Encoder_Init_TIM4(void);
int Read_Encoder(u8 TIMX);
float PID_Loc(float SetValue, float ActualValue, PID_LocTypeDef* PID);
void PID_Init(PID_LocTypeDef* PID,int max_speed,int max_wheel_diff);
void TIM4_IRQHandler(void);
void TIM2_IRQHandler(void);
#endif
