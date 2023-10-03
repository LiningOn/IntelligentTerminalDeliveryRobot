#ifndef __BSP_HC_SR04_H
#define __BSP_HC_SR04_H	
#include "sys.h"

////////////////////////////						������ģ��1					///////////////////////////
//��ʱ��2����
void hcsr04_NVIC(void);

//IO�ڳ�ʼ�� ��������ʼ��
void Hcsr04Init(void);

//�򿪶�ʱ��2
static void OpenTimerForHc(void);

//�رն�ʱ��2
static void CloseTimerForHc(void);

//��ʱ��7���ж�
void TIM7_IRQHandler(void);

//��ȡ��ʱ��2������ֵ
u32 GetEchoTimer(void);

//ͨ����ʱ��2������ֵ�������
float Hcsr04GetLength(void);

////////////////////////////						������ģ��2					///////////////////////////
//��ʱ��5����
void hcsr04_NVIC2(void);

//IO�ڳ�ʼ�� ��������ʼ��
void Hcsr04Init2(void);

//�򿪶�ʱ��2
static void OpenTimerForHc2(void);

//�رն�ʱ��2
static void CloseTimerForHc2(void);

//��ʱ��5���ж�
void TIM5_IRQHandler(void);

//��ȡ��ʱ��2������ֵ
u32 GetEchoTimer2(void);

//ͨ����ʱ��2������ֵ�������
float Hcsr04GetLength2(void);
void Bsp_HC_SR04_Init(void);

#endif

