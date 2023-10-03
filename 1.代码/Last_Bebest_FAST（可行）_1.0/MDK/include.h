#ifndef __INCLUDE_H
#define __INCLUDE_H

/*Systerm���*/
#include "sys.h"
#include "delay.h"
#include "usart.h"
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/*FreeRTOS���*/
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"


/*Annlysis���*/
#include <pid.h>
#include <fsm.h>
#include <speed_analysis.h>
#include "datascope.h"			//��λ��


/*Ӳ������*/
#include <bsp_motor.h>
#include <bsp_usart.h>
#include <bsp_uart.h>
#include "bsp_hc_sr04.h"	//������ģ��
#include "bsp_steering.h"	//���
#include "bsp_lcd_init.h"
#include "bsp_lcd.h"
//#include "bsp_lcdfont.h"	//���û�����ܶ�error
#include "bsp_microswitch.h"	//΢������

/*MPU6050*/
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "mpu6050.h"
#include "dmpKey.h"
#include "dmpmap.h"
#include "myiic.h"

/*����ģ��*/
#include "voice_broadcast.h"	//��������


/*ǰ��*/
#define V_Forward_Max 				400.0		
#define V_Forward_Little 			200.0		
/*����*/
#define V_Turn_Right_Max 		   -400.0	
/*����*/
#define V_Turn_Left_Max 		    400.0	
/*����*/
#define V_Back_Off_Max 			   -400.0
/*ײǽ*/
#define V_Strike_Max 			    100.0
/*��ǽ*/
#define V_Leave_Max 			   -150.0
/*��λ*/
#define V_Turn_Max 			   	   -150.0

/*ǰ�������̵ƿڹ���*/
#define Time_Forward_Long   		2220		//ǰ��
#define Time_Forward_Little 		680			//ǰ��һС��	

/*PATH1*/
#define Time_PATH1_Left 			1350		//����	
#define Time_PATH1_Strike 			1200		//ײǽ
#define Time_PATH1_Leave 			 500		//��ǽ
#define Time_PATH1_Right 			1300		//����		
#define Time_PATH1_Back 			2800		//�˺�
#define Time_PATH1_Turn				 175		//��λ

/*PATH2*/
#define Time_PATH2_Right 			1390		//����
#define Time_PATH2_Strike 			1200		//ײǽ
#define Time_PATH2_Leave 			 580		//��ǽ
#define Time_PATH2_Left 			1470		//����
#define Time_PATH2_Back 			2800		//�˺�
#define Time_PATH2_Turn				 165		//��λ



/*ȫ�ֱ���*/
extern PID_TypeDef	motor_PID[4];	//���PID����

typedef struct
{
    float motor1;
	float motor2;
	float motor3;
	float motor4;
}Encoder;
extern Encoder Encoder_realvalue;		//���������ֵ

typedef struct
{
	float V1;
	float V2;
	float V3;
	float V4;
}Wheel_Movement;
extern Wheel_Movement Wheel_target;	//���ӵ�Ŀ���ٶ�

typedef struct
{
    float Vx;
	float Vy;
	float acc;
}Robot_Movement;
extern Robot_Movement Robot_target;	//С�������Ŀ���ٶ�

void All_Init(void);




#endif










