#ifndef __BSP_STEERING_H
#define __BSP_STEERING_H
#include "stm32f4xx.h"


void open1(void);
void close1(void);

void open2(void);
void close2(void);

void TIM14_PWM_Init(u32 arr,u32 psc);
void TIM13_PWM_Init(u32 arr,u32 psc);
void Bsp_STERRING_Init(void);		//¶æ»ú³õÊ¼»¯

#endif
