#ifndef __TIMER_H
#define __TIMER_H
#include "sys.h"



void TIM1_Int_Init(u16 arr,u16 psc);
void TIM1_UP_IRQHandler(void);
void TIM1_CC_IRQHandler(void);
#endif
