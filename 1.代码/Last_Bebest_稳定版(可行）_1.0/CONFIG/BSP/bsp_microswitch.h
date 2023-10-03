#ifndef __BSP_MICROSWITCH_H
#define __BSP_MICROSWITCH_H	 
#include "sys.h" 
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F407������
//����������������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2014/5/3
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	 

/*����ķ�ʽ��ͨ��ֱ�Ӳ����⺯����ʽ��ȡIO*/
#define PE3 		GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_3)	//PE3 
#define PE2 		GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_2) //PE2



typedef uint8_t  u8;

void KEY_Init(void);	//IO��ʼ��
u8 KEY_Scan(u8);  		//����ɨ�躯��	
int check(void);

#endif
