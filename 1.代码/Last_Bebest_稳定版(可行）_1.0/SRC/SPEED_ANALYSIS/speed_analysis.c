#include <include.h>

Wheel_Movement Wheel_target;	//轮子目标速度	
Robot_Movement Robot_target;			//小车整体目标速度
Encoder Encoder_realvalue;	//编码器读取值

float kp_angle=10.0f,pid_angle;	//陀螺仪矫正参数
void Speed_analysis(void)
{


	if(ROBOT.MPU6050 == USING){
		Wheel_target.V1 = -Robot_target.Vx*sqrt(2)/2 - Robot_target.Vy*sqrt(2)/2 + kp_angle*Robot_target.acc;
		Wheel_target.V2 =  Robot_target.Vx*sqrt(2)/2 - Robot_target.Vy*sqrt(2)/2 + kp_angle*Robot_target.acc;
		Wheel_target.V3 =  Robot_target.Vx*sqrt(2)/2 + Robot_target.Vy*sqrt(2)/2 + kp_angle*Robot_target.acc;
		Wheel_target.V4 = -Robot_target.Vx*sqrt(2)/2 + Robot_target.Vy*sqrt(2)/2 + kp_angle*Robot_target.acc;
	}else if (ROBOT.MPU6050 == NOT_USE){
		Wheel_target.V1 = -Robot_target.Vx*sqrt(2)/2 - Robot_target.Vy*sqrt(2)/2;
		Wheel_target.V2 =  Robot_target.Vx*sqrt(2)/2 - Robot_target.Vy*sqrt(2)/2;
		Wheel_target.V3 =  Robot_target.Vx*sqrt(2)/2 + Robot_target.Vy*sqrt(2)/2;
		Wheel_target.V4 = -Robot_target.Vx*sqrt(2)/2 + Robot_target.Vy*sqrt(2)/2;
	}
	
}


					//总时间			最大速度			加速比		减速比			
void Speed_programme(int time, float v_max, float acc_pro, float dec_pro, float *speed)
{
	int time_now = 0;
	int time_acc = acc_pro * time;		//加速时间
	int time_dec = dec_pro * time;		//减速时间
	int time_con = time - time_acc - time_dec;
	float v_acc_step = v_max / time_acc;
	float v_dec_step = v_max / time_dec;
	
	//加速阶段
	while(time_now < time_acc)
	{
		time_now++;
		*speed += v_acc_step;
		vTaskDelay(1);
	}
	*speed = v_max;
	
	//匀速阶段
	while(time_now < time_con + time_acc)
	{
		time_now++;
		vTaskDelay(1);
	}
	
	//减速阶段
	while(time_now < time)
	{
		time_now++;
		*speed -= v_dec_step;
		vTaskDelay(1);
	}
	*speed = 0;
}


void Encoder_Get_CNT(void)
{
	Encoder_realvalue.motor1 = -(short)TIM3 -> CNT;
	TIM3 -> CNT = 0;
	
	Encoder_realvalue.motor2 = -(short)TIM1 -> CNT;
	TIM1 -> CNT = 0;
	
	Encoder_realvalue.motor3 = -(short)TIM4 -> CNT;
	TIM4 -> CNT = 0;
	
	Encoder_realvalue.motor4 = -(short)TIM2 -> CNT;
	TIM2 -> CNT = 0;			
}

//设置pwm
void Set_PWM(void)
{
	//正反转控制
	//motorA
	if(motor_PID[0].Output>=0)
	{
		PEout(4)=1; 
		PCout(13)=0; 
		TIM9->CCR1=	motor_PID[0].Output;
		TIM_SetCompare1(TIM9,motor_PID[0].Output); 
	}else if(motor_PID[0].Output<0){
		PEout(4)=0; 
		PCout(13)=1;
		TIM9->CCR1=-motor_PID[0].Output;
		TIM_SetCompare1(TIM9, -motor_PID[0].Output);
	}
	//motorB	
	if(motor_PID[1].Output>=0)
	{
		PDout(8)=0; 
		PDout(10)=1; 
		TIM12->CCR1=motor_PID[1].Output;
		TIM_SetCompare1(TIM12, motor_PID[1].Output); 
	}else if(motor_PID[1].Output<0){
		PDout(8)=1; 
		PDout(10)=0;
		TIM12->CCR1=-motor_PID[1].Output;
		TIM_SetCompare1(TIM12, -motor_PID[1].Output);
	}
	//motorC	
	if(motor_PID[2].Output>=0)
	{
		PDout(9)=1; 
		PDout(11)=0; 
		TIM12->CCR2=motor_PID[2].Output;
		TIM_SetCompare2(TIM12, motor_PID[2].Output); 
	} 
	else if(motor_PID[2].Output<0)
	{
		PDout(9)=0; 
		PDout(11)=1;
		TIM12->CCR2=-motor_PID[2].Output;
		TIM_SetCompare2(TIM12, -motor_PID[2].Output);
	}
	//motorD	
	if(motor_PID[3].Output>=0)
	{
		PCout(14)=1; 
		PCout(15)=0; 
		TIM9->CCR2=motor_PID[3].Output;
		TIM_SetCompare2(TIM9, motor_PID[3].Output); 
	} 
	else if(motor_PID[3].Output<0)
	{
		PCout(14)=0; 
		PCout(15)=1;
		TIM9->CCR2=-motor_PID[3].Output;
		TIM_SetCompare2(TIM9, -motor_PID[3].Output);
	}
}

