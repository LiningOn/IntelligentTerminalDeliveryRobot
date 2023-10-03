#ifndef __INCLUDE_H
#define __INCLUDE_H

/*Systerm相关*/
#include "sys.h"
#include "delay.h"
#include "usart.h"
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/*FreeRTOS相关*/
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"


/*Annlysis相关*/
#include <pid.h>
#include <fsm.h>
#include <speed_analysis.h>
#include "datascope.h"			//上位机


/*硬件驱动*/
#include <bsp_motor.h>
#include <bsp_usart.h>
#include <bsp_uart.h>
#include "bsp_hc_sr04.h"	//超声波模块
#include "bsp_steering.h"	//舵机
#include "bsp_lcd_init.h"
#include "bsp_lcd.h"
//#include "bsp_lcdfont.h"	//引用会引起很多error
#include "bsp_microswitch.h"	//微动开关

/*MPU6050*/
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "mpu6050.h"
#include "dmpKey.h"
#include "dmpmap.h"
#include "myiic.h"

/*外设模块*/
#include "voice_broadcast.h"	//语音播报


/*前进*/
#define V_Forward_Max 				400.0		
#define V_Forward_Little 			200.0		
/*右移*/
#define V_Turn_Right_Max 		   -400.0	
/*左移*/
#define V_Turn_Left_Max 		    400.0	
/*后退*/
#define V_Back_Off_Max 			   -400.0
/*撞墙*/
#define V_Strike_Max 			    100.0
/*离墙*/
#define V_Leave_Max 			   -150.0
/*复位*/
#define V_Turn_Max 			   	   -150.0

/*前进到红绿灯口过程*/
#define Time_Forward_Long   		2220		//前进
#define Time_Forward_Little 		680			//前进一小段	

/*PATH1*/
#define Time_PATH1_Left 			1350		//左移	
#define Time_PATH1_Strike 			1200		//撞墙
#define Time_PATH1_Leave 			 500		//离墙
#define Time_PATH1_Right 			1300		//右移		
#define Time_PATH1_Back 			2800		//退后
#define Time_PATH1_Turn				 175		//复位

/*PATH2*/
#define Time_PATH2_Right 			1390		//右移
#define Time_PATH2_Strike 			1200		//撞墙
#define Time_PATH2_Leave 			 580		//离墙
#define Time_PATH2_Left 			1470		//左移
#define Time_PATH2_Back 			2800		//退后
#define Time_PATH2_Turn				 165		//复位



/*全局变量*/
extern PID_TypeDef	motor_PID[4];	//电机PID参数

typedef struct
{
    float motor1;
	float motor2;
	float motor3;
	float motor4;
}Encoder;
extern Encoder Encoder_realvalue;		//电机编码器值

typedef struct
{
	float V1;
	float V2;
	float V3;
	float V4;
}Wheel_Movement;
extern Wheel_Movement Wheel_target;	//轮子的目标速度

typedef struct
{
    float Vx;
	float Vy;
	float acc;
}Robot_Movement;
extern Robot_Movement Robot_target;	//小车整体的目标速度

void All_Init(void);




#endif










