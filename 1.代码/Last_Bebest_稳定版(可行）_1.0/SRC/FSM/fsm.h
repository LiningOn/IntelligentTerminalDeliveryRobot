#ifndef __FSM_H
#define __FSM_H

//1.�ƶ�״̬
typedef enum
{
	INIT_STATE,				//0		��ʼ��				
	WAITING_STATE,			//1		��ʼ�ȴ�������ʼ�㣩
	FORWARD_STATE,
	CR0SSROADS_STATE,	
	TAKEING_STATE,
	TURNBACK_STATE,
	EMERGENCY,				//���ϰ���������״̬
	ROBOT_TASK_FINISHER		//		�������
}MOVE_STATE;

//2.������ʹ����̬
typedef enum
{
	NOT_USE,
	USING	// 1 ʹ��
}USE_MPU6050_STATE;

//3.���ſ���״̬
typedef enum
{
	CLOSED,
	OPEN							//����
}DOOR;

//4.��ǰ·��
typedef enum
{
	NO_PATH,
	PATH_1,
	PATH_2						
}PATH;

//5.��ǰ·��
typedef enum
{
	UNRECOGNIZED,
	RECOGNIZED
}CODE;

typedef struct{
	MOVE_STATE   NOW_STATE;
	USE_MPU6050_STATE MPU6050;
	DOOR  DOOR_STATE;
	PATH NOW_PATH;
	CODE QRCODE;
}ROBOT_STATE;
extern ROBOT_STATE ROBOT;



void Chassis_control(void *pvParameters);
void Datascope(void *pvParameters);
void Robot_FSM(void *pvParameters);
void Get_mpu6050_data(void *pvParameters);
void Get_distance(void *pvParameters);
void Key_Check(void *pvParameters);
void Voice_play(void);
float mpu_limit(float value);


#endif
