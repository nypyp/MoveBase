#ifndef __MOTOR_H
#define __MOTOR_H
#include "sys.h"

#define PWMAIN1   TIM3->CCR1
#define PWMAIN2   TIM3->CCR2
//#define AIN1   PAout(6)
//#define AIN2   PAout(7)
#define PWMBIN1   TIM3->CCR3
#define PWMBIN2   TIM3->CCR4
//#define BIN1   PBout(0)
//#define BIN2   PBout(1)

void Motor_Init(void);
void MOTOR_PWM_Init(u16 arr, u16 psc);
void Set_Pwm(int motor1, int motor2);
int myabs(int num);
void Turn_Left(void);
void Turn_Right(void);
extern u16 MATLAB_FLAG;
extern u16 MATLAB_SET;
void Matlab_Draw(void);
#endif
