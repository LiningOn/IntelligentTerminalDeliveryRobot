#ifndef __BSP_HC_SR04_H
#define __BSP_HC_SR04_H	
#include "sys.h"

////////////////////////////						超声波模块1					///////////////////////////
//定时器2设置
void hcsr04_NVIC(void);

//IO口初始化 及其他初始化
void Hcsr04Init(void);

//打开定时器2
static void OpenTimerForHc(void);

//关闭定时器2
static void CloseTimerForHc(void);

//定时器7终中断
void TIM7_IRQHandler(void);

//获取定时器2计数器值
u32 GetEchoTimer(void);

//通过定时器2计数器值推算距离
float Hcsr04GetLength(void);

////////////////////////////						超声波模块2					///////////////////////////
//定时器5设置
void hcsr04_NVIC2(void);

//IO口初始化 及其他初始化
void Hcsr04Init2(void);

//打开定时器2
static void OpenTimerForHc2(void);

//关闭定时器2
static void CloseTimerForHc2(void);

//定时器5终中断
void TIM5_IRQHandler(void);

//获取定时器2计数器值
u32 GetEchoTimer2(void);

//通过定时器2计数器值推算距离
float Hcsr04GetLength2(void);
void Bsp_HC_SR04_Init(void);

#endif

