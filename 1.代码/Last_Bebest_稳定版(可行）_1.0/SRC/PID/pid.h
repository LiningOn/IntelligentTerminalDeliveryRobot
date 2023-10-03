#ifndef __PID_H
#define __PID_H

typedef struct 
{
    float P;
    float I;
    float D;
    float Limit;
    float Current_Error; //��ǰ���
    float Last_Error;    //ǰһ�����
    float Previous_Error;//ǰǰһ�����
		float Sum_Error;			 //���֮��
    float Output;
}PID_TypeDef;

void PID_Init(PID_TypeDef *pid, float P, float I, float D, float Limit);
void PID_Clear(PID_TypeDef *pid);
void PID_Incremental_PID_Calculation(PID_TypeDef *pid, float NowPlace, float Point);
void PID_position_PID_calculation(PID_TypeDef *pid, float NowPlace, float Point);
void Motor_PID_Init(void);
float ANGLE_PID(float Angle_now,float Angle_target);//��ڲ����������ǵõ��ĽǶ�

#endif
