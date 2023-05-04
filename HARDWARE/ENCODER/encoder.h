#ifndef __ENCODER_H
#define __ENCODER_H
#include <sys.h>	 
/**************************************************************************
��    �ߣ��������
�Ա���ַ��https://shop119207236.taobao.com
**************************************************************************/
#define ENCODER_TIM_PERIOD (u16)(65535)   //103�Ķ�ʱ����16λ 2��16�η������65536
typedef struct
{
  float Kp;                       //����ϵ��Proportional
  float Ki;                       //����ϵ��Integral
  float Kd;                       //΢��ϵ��Derivative
 
  float Ek;                       //��ǰ���
  float Ek1;                      //ǰһ����� e(k-1)
  float Ek2;                      //��ǰһ����� e(k-2)
  float LocSum;                   //�ۼƻ���λ��
  float SetValue_last;
  int Max_speed;		//�ٷֱ�PWMʹ�ã��������ٷֱ�PWMĿ��ת��Ϊʵ��ת�ٵ�����ֵ
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
