#ifndef __PID_H
#define __PID_H

typedef struct 
{
    float P;
    float I;
    float D;
    float Limit;
    float Current_Error; //当前误差
    float Last_Error;    //前一次误差
    float Previous_Error;//前前一次误差
		float Sum_Error;			 //误差之和
    float Output;
}PID_TypeDef;

void PID_Init(PID_TypeDef *pid, float P, float I, float D, float Limit);
void PID_Clear(PID_TypeDef *pid);
void PID_Incremental_PID_Calculation(PID_TypeDef *pid, float NowPlace, float Point);
void PID_position_PID_calculation(PID_TypeDef *pid, float NowPlace, float Point);
void Motor_PID_Init(void);
float ANGLE_PID(float Angle_now,float Angle_target);//入口参数是陀螺仪得到的角度

#endif
