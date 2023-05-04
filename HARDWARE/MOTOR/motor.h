#ifndef __MOTOR_H
#define __MOTOR_H
#include "sys.h"

#define PWMA   TIM3->CCR1  
#define AIN1   PAout(5)
#define AIN2   PAout(4)
#define PWMB   TIM3->CCR2 
#define BIN1   PCout(5)
#define BIN2   PCout(4)

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
