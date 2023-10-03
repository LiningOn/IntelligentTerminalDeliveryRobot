#include "include.h"

PID_TypeDef	motor_PID[4];	//���PID����

void Motor_PID_Init(void)
{	
	PID_Init(&motor_PID[0], 150,   67, 0, 65000);	//motor1(���·���	--OK
	PID_Init(&motor_PID[1], 250, 88.8, 0, 65000);	//motor2(���·���##�����	----OKKK
	PID_Init(&motor_PID[2], 150, 88.8, 0, 65000);	//motor3(���·���	--veryOK
	PID_Init(&motor_PID[3], 250, 88.8, 0, 65000);	//motor4(���·���	--veryOK	

}

float ANGLE_PID(float Angle_now,float Angle_target)//��ڲ����������ǵõ��ĽǶ�
{
	static float Error,Error_Last,Error_Last_Last;
	static float acc_add,acc_out;
	float Kp = 6.0,Ki = 0.0,Kd = 0.0;		//��PD�ȣ��������
	   Error=   Angle_now- Angle_target;
       acc_add=Kp*(Error-Error_Last)+Ki*Error+Kd*(Error-2*Error_Last+Error_Last_Last);
       acc_out+=acc_add;
	   Error_Last_Last=Error_Last;
	   Error_Last=Error;
	return acc_out;	
}

/*��ʼ��PID*/
void PID_Init(PID_TypeDef *pid, float P, float I, float D, float Limit)
{
	pid->P = P;
	pid->I = I;
	pid->D = D;
	pid->Limit = Limit;
	pid->Current_Error = 0.0;
	pid->Last_Error = 0.0;
	pid->Previous_Error = 0.0;
	pid->Output = 0.0;
	pid->Sum_Error = 0.0;
}


/*����ʽPID����*/
void PID_Incremental_PID_Calculation(PID_TypeDef *pid, float NowPlace, float Point)
{
    pid->Current_Error = Point - NowPlace;//���㵱ǰ���
    pid->Output += pid->P * (pid->Current_Error - pid->Last_Error)\
	+ pid->I * pid->Current_Error + pid->D * (pid->Current_Error - 2.0 * pid->Last_Error +\
	pid->Previous_Error);

    //����޷�
    if(pid->Output > pid->Limit) pid->Output = pid->Limit;
    else if (pid->Output < -pid->Limit) pid->Output = -pid->Limit;

    //�������
    pid->Previous_Error = pid->Last_Error;
    pid->Last_Error = pid->Current_Error;
}


/*λ��ʽPID����*/ //δ�õ�
void PID_position_PID_calculation(PID_TypeDef *pid, float NowPlace, float Point)
{
	pid->Current_Error = Point - NowPlace;//���㵱ǰ���
	pid->Sum_Error += pid->Current_Error;				 //����ۻ�
	pid->Output = pid->P * pid->Current_Error + pid->I * pid->Sum_Error + pid->D * (pid->Current_Error - pid->Last_Error);
	
	//����޷�
  if(pid->Output > pid->Limit) pid->Output = pid->Limit;
	else if (pid->Output < -pid->Limit) pid->Output = -pid->Limit;
	
//	//����
//	if(pid->Output < 300.0 && pid->Output > 0) pid->Output = 300.0;
//	else if (pid->Output > -300.0 && pid->Output < 0) pid->Output = -300.0;

	//�������
  pid->Last_Error = pid->Current_Error;
}
