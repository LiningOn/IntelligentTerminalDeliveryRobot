#ifndef __BSP_H
#define __BSP_H

#include "stm32f4xx.h" 
//TB6612
void TB6612_Init(void);
//PWM ‰≥ˆ
void TIM9_PWM_Init(u32 arr,u32 psc);
void TIM12_PWM_Init(u32 arr,u32 psc);
//±‡¬Î∆˜≈‰÷√
void TIM3_ENCODER_Init(void); 
void TIM1_ENCODER_Init(void);
void TIM4_ENCODER_Init(void);
void TIM2_ENCODER_Init(void);

void Bsp_Motor_Init(void);	


#endif
