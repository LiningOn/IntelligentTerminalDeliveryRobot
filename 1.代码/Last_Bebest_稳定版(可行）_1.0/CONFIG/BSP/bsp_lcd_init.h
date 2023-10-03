#ifndef __BSP_LCD_INIT_H
#define __BSP_LCD_INIT_H

#include "sys.h"

#define USE_HORIZONTAL 0  //���ú�������������ʾ 
                          //0:����
                          //1:��ת180��
													//2:��ת90��
													//3:��ת270��
#if USE_HORIZONTAL<2
#define LCD_W 128
#define LCD_H 64
#else
#define LCD_W 64
#define LCD_H 128
#endif



//-----------------LCD�˿ڶ���---------------- 

#define LCD_SCLK_Clr() GPIO_ResetBits(GPIOG,GPIO_Pin_12)//SCL=SCLK
#define LCD_SCLK_Set() GPIO_SetBits(GPIOG,GPIO_Pin_12)

#define LCD_MOSI_Clr() GPIO_ResetBits(GPIOD,GPIO_Pin_5)//SDA=MOSI
#define LCD_MOSI_Set() GPIO_SetBits(GPIOD,GPIO_Pin_5)

#define LCD_RES_Clr()  GPIO_ResetBits(GPIOD,GPIO_Pin_4)//RES
#define LCD_RES_Set()  GPIO_SetBits(GPIOD,GPIO_Pin_4)

#define LCD_DC_Clr()   GPIO_ResetBits(GPIOD,GPIO_Pin_15)//DC
#define LCD_DC_Set()   GPIO_SetBits(GPIOD,GPIO_Pin_15)

#define LCD_BLK_Clr()  GPIO_ResetBits(GPIOD,GPIO_Pin_1)//BLK
#define LCD_BLK_Set()  GPIO_SetBits(GPIOD,GPIO_Pin_1)




void LCD_GPIO_Init(void);//��ʼ��GPIO
void LCD_Writ_Bus(u8 dat);//ģ��SPIʱ��
void LCD_WR_DATA8(u8 dat);//д��һ���ֽ�
void LCD_WR_DATA(u16 dat);//д�������ֽ�
void LCD_WR_REG(u8 dat);//д��һ��ָ��
void LCD_Address_Set(u16 x1,u16 y1,u16 x2,u16 y2);//�������꺯��
void LCD_Init(void);//LCD��ʼ��
void Bsp_TFT_Init(void);
#endif




