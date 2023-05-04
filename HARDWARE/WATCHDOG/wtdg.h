#ifndef __WTDG_H
#define __WTDG_H
#include "sys.h"

void IWDG_Init(u8 prer,u16 rlr);
void IWDG_Feed(void);

#endif

