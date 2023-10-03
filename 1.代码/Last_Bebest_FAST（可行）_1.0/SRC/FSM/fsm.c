#include <include.h>

extern int QRcode;
extern char SeeInfo;
extern float kp_angle;	//陀螺仪矫正参数
float value_max =20.0;

ROBOT_STATE ROBOT;

//底盘驱动任务
void Robot_FSM(void *pvParameters)
{
	Send_data(6,voice_3);	//“预热”----不要删
	ROBOT.NOW_STATE = INIT_STATE;	//初始状态
	while(1)
	{
		switch(ROBOT.NOW_STATE)
		{	
/*初始化状态*/
			case INIT_STATE:
				ROBOT.MPU6050 = NOT_USE;	//不使用MPU6050
				ROBOT.DOOR_STATE = CLOSED;		
				ROBOT.NOW_PATH = NO_PATH;
				ROBOT.QRCODE = UNRECOGNIZED;				
				ROBOT.NOW_STATE = WAITING_STATE;
				break;
			
			
/*初始等待*/
			case WAITING_STATE:
				LCD_ShowString(40,13,"0.0 g",GREEN,BLACK,32,0);
				ROBOT.MPU6050 = NOT_USE;	
				//路径1
				if(QRcode==11 && ROBOT.QRCODE == UNRECOGNIZED){	//装1号物料到1号点位
					ROBOT.QRCODE = RECOGNIZED;
					ROBOT.NOW_PATH = PATH_1;					
					ROBOT.DOOR_STATE = OPEN;
					open1();					
					Send_data(6,voice_1);	//播报：扫描完成，请装入1号物料，并关闭仓门
					vTaskDelay(100); 
					//等待物料放入
			
				}else if(QRcode==21 && ROBOT.QRCODE == UNRECOGNIZED){	//装2号物料到1号点位
					ROBOT.QRCODE = RECOGNIZED;
					ROBOT.NOW_PATH = PATH_1;					
					ROBOT.DOOR_STATE = OPEN;
					open2();					
					Send_data(6,voice_2);	//播报：扫描完成，请装入2号物料，并关闭仓门
					vTaskDelay(100); 
					//等待物料放入
				}
				//路径2
				else if(QRcode==12 && ROBOT.QRCODE == UNRECOGNIZED){
					ROBOT.QRCODE = RECOGNIZED;
					ROBOT.NOW_PATH = PATH_2;					
					ROBOT.DOOR_STATE = OPEN;
					open1();					
					Send_data(6,voice_1);	//播报：扫描完成，请装入1号物料，并关闭仓门
					vTaskDelay(100); 
				}else if(QRcode==22 && ROBOT.QRCODE == UNRECOGNIZED){
					ROBOT.QRCODE = RECOGNIZED;
					QRcode = 0;
					ROBOT.NOW_PATH = PATH_2;					
					ROBOT.DOOR_STATE = OPEN;
					open2();					
					Send_data(6,voice_2);	//播报：扫描完成，请装入2号物料，并关闭仓门
					vTaskDelay(100); 
				}
				break;
			
				
/*前进状态*/
			case FORWARD_STATE:
				LCD_ShowString(40,13,"13.5 g",GREEN,BLACK,32,0);
					vTaskDelay(1000);
					vTaskDelay(1000);
					vTaskDelay(1000);
					vTaskDelay(1000);					
					vTaskDelay(1000);
				
		////////////////这里的参数是前进的//////////////			 			
				kp_angle=10.;
				close1();
				close2();	
				ROBOT.DOOR_STATE=CLOSED;	
				ROBOT.MPU6050 = USING;	
			
				Speed_programme(Time_Forward_Long, V_Forward_Max , 0.2, 0.2, &Robot_target.Vy);

				SeeInfo = 0;	//防止之前的障碍物`颜色干扰
				ROBOT.NOW_STATE = CR0SSROADS_STATE;	//到达十字路口
				break;		

/*十字路口*/			
			case CR0SSROADS_STATE:
				ROBOT.MPU6050 = USING;						
//				SeeInfo = 0;
				if(SeeInfo==1&&ROBOT.NOW_PATH == PATH_1){		//识别到绿灯 and 路径1
	
					//前进到路中央
		////////////////这里的参数是从红绿灯出发的//////////////			 			
					kp_angle = 10;
					Robot_target.Vy=V_Forward_Little;			
					vTaskDelay(Time_Forward_Little+130);	
					Robot_target.Vy=0.0;
					Robot_target.Vx=0.0;
					vTaskDelay(100);	
					
					//左平移
		////////////////这里的参数是从红绿灯出发的//////////////			 			
					kp_angle = 10;
					Speed_programme(Time_PATH1_Left, V_Turn_Left_Max , 0.25, 0.2, &Robot_target.Vx);
					vTaskDelay(100);	
					Robot_target.Vy = 0.0;
					Robot_target.Vx = 0.0;
					vTaskDelay(100);
					
					//撞墙
					Robot_target.Vy = V_Strike_Max;			
					vTaskDelay(Time_PATH1_Strike);								
					Robot_target.Vy = 0.0;
					Robot_target.Vx = 0.0;
					vTaskDelay(100);
					
					Send_data(6,voice_3);	//播报：到达1号收货点，请出示提货码
					vTaskDelay(100);
					ROBOT.MPU6050 = NOT_USE;	//不使用MPU6050							
					Bsp_MPU6050_Init();	//重新初始化陀螺仪###############
					delay_ms(500);
					QRcode = 0;
					ROBOT.QRCODE = UNRECOGNIZED;
					ROBOT.NOW_STATE = TAKEING_STATE;	//到达取货区
				}
				else if(SeeInfo==1&&ROBOT.NOW_PATH == PATH_2){		//识别到绿灯 and 路径1	
	
					
					//前进到路中央	
			////////////////这里的参数是从红绿灯出发的//////////////			 			
					kp_angle = 10;
					Robot_target.Vy=V_Forward_Little;			
					vTaskDelay(Time_Forward_Little);										
					Robot_target.Vy=0.0;
					Robot_target.Vx=0.0;
					vTaskDelay(100);
					
					//右平移				
			////////////////这里的参数是从红绿灯出发的//////////////			 			
					kp_angle = 10;
					Speed_programme(Time_PATH2_Right, V_Turn_Right_Max , 0.25, 0.2, &Robot_target.Vx);
					vTaskDelay(100);	
					Robot_target.Vy = 0.0;
					Robot_target.Vx = 0.0;
					vTaskDelay(100);
					
					//撞墙
					Robot_target.Vy=V_Strike_Max;			
					vTaskDelay(Time_PATH2_Strike);			
					Robot_target.Vy=0.0;
					Robot_target.Vx=0.0;
					vTaskDelay(100);
					
					Send_data(6,voice_4);	//播报：到达2号收货点，请出示提货码
					vTaskDelay(100); 
					ROBOT.MPU6050 = NOT_USE;	//不使用MPU6050					
					Bsp_MPU6050_Init();	//重新初始化陀螺仪###############
					delay_ms(500);
					QRcode = 0;
					ROBOT.QRCODE = UNRECOGNIZED;
					ROBOT.NOW_STATE = TAKEING_STATE;	//到达取货区
				}
				break;
				
/*取货*/
			case TAKEING_STATE:
				ROBOT.MPU6050 = NOT_USE;	//不使用MPU6050					
				//路径1
				if(QRcode==11 && ROBOT.QRCODE == UNRECOGNIZED){	
					open1();	
					ROBOT.QRCODE = RECOGNIZED;
					ROBOT.DOOR_STATE = OPEN;					
					Send_data(6,voice_5);	//播报：请取出1号物料，并关闭仓门
					vTaskDelay(100); 				


				}else if(QRcode==21 && ROBOT.QRCODE == UNRECOGNIZED){
					open2();	
					ROBOT.QRCODE = RECOGNIZED;					
					ROBOT.DOOR_STATE = OPEN;					
					Send_data(6,voice_6);	//播报：请取出2号物料，并关闭仓门
					vTaskDelay(100); 					
				
				//路径2
				}else if(QRcode==12 && ROBOT.QRCODE == UNRECOGNIZED){
					open1();	
					ROBOT.QRCODE = RECOGNIZED;					
					ROBOT.DOOR_STATE = OPEN;	
					Send_data(6,voice_5);	//播报：请取出1号物料，并关闭仓门
					vTaskDelay(100); 				


				}else if(QRcode==22 && ROBOT.QRCODE == UNRECOGNIZED){	
					open2();	
					ROBOT.QRCODE = RECOGNIZED;
					ROBOT.DOOR_STATE = OPEN;					
					Send_data(6,voice_6);	//播报：请取出2号物料，并关闭仓门
					vTaskDelay(100);  					

				}
				break;
/*返程*/				
			case TURNBACK_STATE:
				close1();
				close2();
				ROBOT.DOOR_STATE=CLOSED;				
				ROBOT.MPU6050 = USING;	//使用MPU6050	
			
				//路径1
				if(ROBOT.NOW_PATH == PATH_1){		
					
					//离墙
					Robot_target.Vy = V_Leave_Max;			
					vTaskDelay(Time_PATH1_Leave);								
					Robot_target.Vy=0.0;
					Robot_target.Vx=0.0;
					vTaskDelay(100);
					
					//右移
			////////////////这里的参数是从红绿灯出发的//////////////			 			
					kp_angle = 12;
					Speed_programme(Time_PATH1_Right, V_Turn_Right_Max , 0.25, 0.2, &Robot_target.Vx);
					vTaskDelay(100); 	
					
					//后退
			////////////这里的参数是返程的////////////
					kp_angle=6;
					Speed_programme(Time_PATH1_Back, V_Back_Off_Max , 0.2, 0.2, &Robot_target.Vy);
					
					//回原点
					Robot_target.Vx = V_Turn_Max;
					vTaskDelay(Time_PATH1_Turn); 					
				}
				//路径2
				else if(ROBOT.NOW_PATH == PATH_2){		//识别到绿灯 and 路径1
					
					//离墙
					Robot_target.Vy = V_Leave_Max;			
					vTaskDelay(Time_PATH2_Leave);								
					Robot_target.Vy = 0.0;
					Robot_target.Vx = 0.0;
					vTaskDelay(100);
					
					//左移
			////////////这里的参数是返程的////////////
					kp_angle = 10;					
					Speed_programme(Time_PATH2_Left, V_Turn_Left_Max , 0.25, 0.2, &Robot_target.Vx);
					vTaskDelay(100); 			
					
					//后退
			////////////这里的参数是返程的////////////
					kp_angle=7;
					Speed_programme(Time_PATH2_Back, V_Back_Off_Max , 0.2, 0.2, &Robot_target.Vy);
					
					//回原点
					Robot_target.Vx = V_Turn_Max;
					vTaskDelay(Time_PATH2_Turn); 
				}				

				//停下	
				Robot_target.Vy=0.0;
				Robot_target.Vx=0.0;
				vTaskDelay(100);
				ROBOT.NOW_STATE = ROBOT_TASK_FINISHER;	//到达取货区				
				break;
			
			
/*任务完成状态*/
			case ROBOT_TASK_FINISHER:
				ROBOT.MPU6050 = NOT_USE;	//不使用MPU6050	
//				LCD_Fill(0,0,LCD_W,LCD_H,BLACK);	//清屏
//				LCD_ShowString(22,28,"TASK FINISH!",YELLOW,BLACK,16,0);
				break;
		}
		
		vTaskDelay(50); 
	}
}



//底盘驱动任务
void Chassis_control(void *pvParameters)
{
	Robot_target.Vx = 0;
	Robot_target.Vy = 0;
	vTaskDelay(100); 
	while(1)
	{	
		//运动学解算
		
		Speed_analysis();

		//读取编码器
		Encoder_Get_CNT();
//		//PID计算
		PID_Incremental_PID_Calculation(&motor_PID[2], Encoder_realvalue.motor3 ,Wheel_target.V3 );	//motor3
		PID_Incremental_PID_Calculation(&motor_PID[1], Encoder_realvalue.motor2 ,Wheel_target.V2 );
		PID_Incremental_PID_Calculation(&motor_PID[0], Encoder_realvalue.motor1 ,Wheel_target.V1 );
		PID_Incremental_PID_Calculation(&motor_PID[3], Encoder_realvalue.motor4 ,Wheel_target.V4 );	//motor3
		
//		//输出pwm
		Set_PWM();
		
		vTaskDelay(50); 		
	}
}



/*红外感应*/
void Key_Check(void *pvParameters)
{
	
	while(1)
	{	
		if(ROBOT.NOW_STATE==WAITING_STATE && ROBOT.DOOR_STATE==OPEN){	   
			if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_12)==1||GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_3)==1){
				vTaskDelay(1000);
				ROBOT.NOW_STATE = FORWARD_STATE;
			}
			else{
				vTaskDelay(2000); //防止没有感应到
				ROBOT.NOW_STATE = FORWARD_STATE;				
			}
		}
		else if(ROBOT.NOW_STATE==TAKEING_STATE && ROBOT.DOOR_STATE==OPEN ){
			GPIO_ResetBits(GPIOE, GPIO_Pin_3); 
			GPIO_ResetBits(GPIOC, GPIO_Pin_12); 
			if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_12)==1||GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_3)==1){
				vTaskDelay(1500);		
				ROBOT.NOW_STATE = TURNBACK_STATE;	//到达取货区
			}
			else{
				vTaskDelay(3500); 
				ROBOT.NOW_STATE = TURNBACK_STATE; //防止没有感应到				
			}
				
			
		}
		
		vTaskDelay(5); 	
	}
}






/*上位机*/	
void Datascope(void *pvParameters)
{
	unsigned char i; //计数变量
	unsigned char Send_Count; //串口需要发送的数据个数
	
	while(1)
	{
		DataScope_Get_Channel_Data(Encoder_realvalue.motor3, 1 );	//电机2的实际速度（编码器计数值）
		DataScope_Get_Channel_Data(Wheel_target.V3, 2 );			//电机2的目标速度（编码器计数值）
		DataScope_Get_Channel_Data(Encoder_realvalue.motor2, 3 );  //motor_pid_2.Output
		DataScope_Get_Channel_Data(Wheel_target.V2, 4 );
		DataScope_Get_Channel_Data(Encoder_realvalue.motor1, 5 );
		DataScope_Get_Channel_Data(Wheel_target.V1, 6 );
		Send_Count = DataScope_Data_Generate(6);
		for( i = 0 ; i < Send_Count; i++)
		{
			while((USART1->SR&0X40)==0);
			USART1->DR = DataScope_OutPut_Buffer[i];
		}
		vTaskDelay(20); 	
	}
}

/*陀螺仪限幅*/
float mpu_limit(float value)
{
	if(value > value_max){value =value_max;}
	if(value < -value_max){value =-value_max;}
	return value;
}

/*获取陀螺仪数据*/
void Get_mpu6050_data(void *pvParameters)
{
	float pitch,roll,yaw; 		//欧拉角		
	while(1)
	{
		while(mpu_dmp_get_data(&pitch,&roll,&yaw)!=0){} 	//更新数据		
		Robot_target.acc = mpu_limit(yaw);
		vTaskDelay(1); 				 
	} 	
}


/*测距*/
/*
int taggge = 0;
float Left_distance,Right_distance;
void Get_distance(void *pvParameters)
{
	int ROBOT_LAST_STATE,warn_sign = 0;	//距离警报标志位
	while(1)
	{	
		Right_distance = Hcsr04GetLength();
		Left_distance = Hcsr04GetLength2();
		
		if(Left_distance < 35 | Right_distance < 35){
			ROBOT_LAST_STATE = ROBOT.NOW_STATE;
			//停下	
			Robot_target.Vy=0.0;
			Robot_target.Vx=0.0;
			vTaskDelay(500);
			ROBOT.NOW_STATE = EMERGENCY;	
			taggge = 6;
		}
		vTaskDelay(5); 	

	}
}
*/
