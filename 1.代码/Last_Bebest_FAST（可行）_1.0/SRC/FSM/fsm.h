#ifndef __FSM_H
#define __FSM_H

//1.移动状态
typedef enum
{
	INIT_STATE,				//0		初始化				
	WAITING_STATE,			//1		起始等待区（起始点）
	FORWARD_STATE,
	CR0SSROADS_STATE,	
	TAKEING_STATE,
	TURNBACK_STATE,
	EMERGENCY,				//有障碍物，进入紧急状态
	ROBOT_TASK_FINISHER		//		任务完成
}MOVE_STATE;

//2.陀螺仪使用姿态
typedef enum
{
	NOT_USE,
	USING	// 1 使用
}USE_MPU6050_STATE;

//3.仓门开闭状态
typedef enum
{
	CLOSED,
	OPEN							//开启
}DOOR;

//4.当前路径
typedef enum
{
	NO_PATH,
	PATH_1,
	PATH_2						
}PATH;

//5.当前路径
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
