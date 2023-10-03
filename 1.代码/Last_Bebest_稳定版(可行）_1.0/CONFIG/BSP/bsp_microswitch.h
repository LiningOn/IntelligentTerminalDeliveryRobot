#ifndef __BSP_MICROSWITCH_H
#define __BSP_MICROSWITCH_H	 
#include "sys.h" 
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F407开发板
//按键输入驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2014/5/3
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	 

/*下面的方式是通过直接操作库函数方式读取IO*/
#define PE3 		GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_3)	//PE3 
#define PE2 		GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_2) //PE2



typedef uint8_t  u8;

void KEY_Init(void);	//IO初始化
u8 KEY_Scan(u8);  		//按键扫描函数	
int check(void);

#endif
