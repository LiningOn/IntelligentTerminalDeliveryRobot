#include <include.h>

extern int QRcode;
extern char SeeInfo;
extern float kp_angle;	//�����ǽ�������
float value_max =20.0;

ROBOT_STATE ROBOT;

//������������
void Robot_FSM(void *pvParameters)
{
	Send_data(6,voice_3);	//��Ԥ�ȡ�----��Ҫɾ
	ROBOT.NOW_STATE = INIT_STATE;	//��ʼ״̬
	while(1)
	{
		switch(ROBOT.NOW_STATE)
		{	
/*��ʼ��״̬*/
			case INIT_STATE:
				ROBOT.MPU6050 = NOT_USE;	//��ʹ��MPU6050
				ROBOT.DOOR_STATE = CLOSED;		
				ROBOT.NOW_PATH = NO_PATH;
				ROBOT.QRCODE = UNRECOGNIZED;				
				ROBOT.NOW_STATE = WAITING_STATE;
				break;
			
			
/*��ʼ�ȴ�*/
			case WAITING_STATE:
				LCD_ShowString(40,13,"0.0 g",GREEN,BLACK,32,0);
				ROBOT.MPU6050 = NOT_USE;	
				//·��1
				if(QRcode==11 && ROBOT.QRCODE == UNRECOGNIZED){	//װ1�����ϵ�1�ŵ�λ
					ROBOT.QRCODE = RECOGNIZED;
					ROBOT.NOW_PATH = PATH_1;					
					ROBOT.DOOR_STATE = OPEN;
					open1();					
					Send_data(6,voice_1);	//������ɨ����ɣ���װ��1�����ϣ����رղ���
					vTaskDelay(100); 
					//�ȴ����Ϸ���
			
				}else if(QRcode==21 && ROBOT.QRCODE == UNRECOGNIZED){	//װ2�����ϵ�1�ŵ�λ
					ROBOT.QRCODE = RECOGNIZED;
					ROBOT.NOW_PATH = PATH_1;					
					ROBOT.DOOR_STATE = OPEN;
					open2();					
					Send_data(6,voice_2);	//������ɨ����ɣ���װ��2�����ϣ����رղ���
					vTaskDelay(100); 
					//�ȴ����Ϸ���
				}
				//·��2
				else if(QRcode==12 && ROBOT.QRCODE == UNRECOGNIZED){
					ROBOT.QRCODE = RECOGNIZED;
					ROBOT.NOW_PATH = PATH_2;					
					ROBOT.DOOR_STATE = OPEN;
					open1();					
					Send_data(6,voice_1);	//������ɨ����ɣ���װ��1�����ϣ����رղ���
					vTaskDelay(100); 
				}else if(QRcode==22 && ROBOT.QRCODE == UNRECOGNIZED){
					ROBOT.QRCODE = RECOGNIZED;
					QRcode = 0;
					ROBOT.NOW_PATH = PATH_2;					
					ROBOT.DOOR_STATE = OPEN;
					open2();					
					Send_data(6,voice_2);	//������ɨ����ɣ���װ��2�����ϣ����رղ���
					vTaskDelay(100); 
				}
				break;
			
				
/*ǰ��״̬*/
			case FORWARD_STATE:
				LCD_ShowString(40,13,"13.5 g",GREEN,BLACK,32,0);
					vTaskDelay(1000);
					vTaskDelay(1000);
					vTaskDelay(1000);
					vTaskDelay(1000);					
					vTaskDelay(1000);
				
		////////////////����Ĳ�����ǰ����//////////////			 			
				kp_angle=10.;
				close1();
				close2();	
				ROBOT.DOOR_STATE=CLOSED;	
				ROBOT.MPU6050 = USING;	
			
				Speed_programme(Time_Forward_Long, V_Forward_Max , 0.2, 0.2, &Robot_target.Vy);

				SeeInfo = 0;	//��ֹ֮ǰ���ϰ���`��ɫ����
				ROBOT.NOW_STATE = CR0SSROADS_STATE;	//����ʮ��·��
				break;		

/*ʮ��·��*/			
			case CR0SSROADS_STATE:
				ROBOT.MPU6050 = USING;						
//				SeeInfo = 0;
				if(SeeInfo==1&&ROBOT.NOW_PATH == PATH_1){		//ʶ���̵� and ·��1
	
					//ǰ����·����
		////////////////����Ĳ����ǴӺ��̵Ƴ�����//////////////			 			
					kp_angle = 10;
					Robot_target.Vy=V_Forward_Little;			
					vTaskDelay(Time_Forward_Little+130);	
					Robot_target.Vy=0.0;
					Robot_target.Vx=0.0;
					vTaskDelay(100);	
					
					//��ƽ��
		////////////////����Ĳ����ǴӺ��̵Ƴ�����//////////////			 			
					kp_angle = 10;
					Speed_programme(Time_PATH1_Left, V_Turn_Left_Max , 0.25, 0.2, &Robot_target.Vx);
					vTaskDelay(100);	
					Robot_target.Vy = 0.0;
					Robot_target.Vx = 0.0;
					vTaskDelay(100);
					
					//ײǽ
					Robot_target.Vy = V_Strike_Max;			
					vTaskDelay(Time_PATH1_Strike);								
					Robot_target.Vy = 0.0;
					Robot_target.Vx = 0.0;
					vTaskDelay(100);
					
					Send_data(6,voice_3);	//����������1���ջ��㣬���ʾ�����
					vTaskDelay(100);
					ROBOT.MPU6050 = NOT_USE;	//��ʹ��MPU6050							
					Bsp_MPU6050_Init();	//���³�ʼ��������###############
					delay_ms(500);
					QRcode = 0;
					ROBOT.QRCODE = UNRECOGNIZED;
					ROBOT.NOW_STATE = TAKEING_STATE;	//����ȡ����
				}
				else if(SeeInfo==1&&ROBOT.NOW_PATH == PATH_2){		//ʶ���̵� and ·��1	
	
					
					//ǰ����·����	
			////////////////����Ĳ����ǴӺ��̵Ƴ�����//////////////			 			
					kp_angle = 10;
					Robot_target.Vy=V_Forward_Little;			
					vTaskDelay(Time_Forward_Little);										
					Robot_target.Vy=0.0;
					Robot_target.Vx=0.0;
					vTaskDelay(100);
					
					//��ƽ��				
			////////////////����Ĳ����ǴӺ��̵Ƴ�����//////////////			 			
					kp_angle = 10;
					Speed_programme(Time_PATH2_Right, V_Turn_Right_Max , 0.25, 0.2, &Robot_target.Vx);
					vTaskDelay(100);	
					Robot_target.Vy = 0.0;
					Robot_target.Vx = 0.0;
					vTaskDelay(100);
					
					//ײǽ
					Robot_target.Vy=V_Strike_Max;			
					vTaskDelay(Time_PATH2_Strike);			
					Robot_target.Vy=0.0;
					Robot_target.Vx=0.0;
					vTaskDelay(100);
					
					Send_data(6,voice_4);	//����������2���ջ��㣬���ʾ�����
					vTaskDelay(100); 
					ROBOT.MPU6050 = NOT_USE;	//��ʹ��MPU6050					
					Bsp_MPU6050_Init();	//���³�ʼ��������###############
					delay_ms(500);
					QRcode = 0;
					ROBOT.QRCODE = UNRECOGNIZED;
					ROBOT.NOW_STATE = TAKEING_STATE;	//����ȡ����
				}
				break;
				
/*ȡ��*/
			case TAKEING_STATE:
				ROBOT.MPU6050 = NOT_USE;	//��ʹ��MPU6050					
				//·��1
				if(QRcode==11 && ROBOT.QRCODE == UNRECOGNIZED){	
					open1();	
					ROBOT.QRCODE = RECOGNIZED;
					ROBOT.DOOR_STATE = OPEN;					
					Send_data(6,voice_5);	//��������ȡ��1�����ϣ����رղ���
					vTaskDelay(100); 				


				}else if(QRcode==21 && ROBOT.QRCODE == UNRECOGNIZED){
					open2();	
					ROBOT.QRCODE = RECOGNIZED;					
					ROBOT.DOOR_STATE = OPEN;					
					Send_data(6,voice_6);	//��������ȡ��2�����ϣ����رղ���
					vTaskDelay(100); 					
				
				//·��2
				}else if(QRcode==12 && ROBOT.QRCODE == UNRECOGNIZED){
					open1();	
					ROBOT.QRCODE = RECOGNIZED;					
					ROBOT.DOOR_STATE = OPEN;	
					Send_data(6,voice_5);	//��������ȡ��1�����ϣ����رղ���
					vTaskDelay(100); 				


				}else if(QRcode==22 && ROBOT.QRCODE == UNRECOGNIZED){	
					open2();	
					ROBOT.QRCODE = RECOGNIZED;
					ROBOT.DOOR_STATE = OPEN;					
					Send_data(6,voice_6);	//��������ȡ��2�����ϣ����رղ���
					vTaskDelay(100);  					

				}
				break;
/*����*/				
			case TURNBACK_STATE:
				close1();
				close2();
				ROBOT.DOOR_STATE=CLOSED;				
				ROBOT.MPU6050 = USING;	//ʹ��MPU6050	
			
				//·��1
				if(ROBOT.NOW_PATH == PATH_1){		
					
					//��ǽ
					Robot_target.Vy = V_Leave_Max;			
					vTaskDelay(Time_PATH1_Leave);								
					Robot_target.Vy=0.0;
					Robot_target.Vx=0.0;
					vTaskDelay(100);
					
					//����
			////////////////����Ĳ����ǴӺ��̵Ƴ�����//////////////			 			
					kp_angle = 12;
					Speed_programme(Time_PATH1_Right, V_Turn_Right_Max , 0.25, 0.2, &Robot_target.Vx);
					vTaskDelay(100); 	
					
					//����
			////////////����Ĳ����Ƿ��̵�////////////
					kp_angle=6;
					Speed_programme(Time_PATH1_Back, V_Back_Off_Max , 0.2, 0.2, &Robot_target.Vy);
					
					//��ԭ��
					Robot_target.Vx = V_Turn_Max;
					vTaskDelay(Time_PATH1_Turn); 					
				}
				//·��2
				else if(ROBOT.NOW_PATH == PATH_2){		//ʶ���̵� and ·��1
					
					//��ǽ
					Robot_target.Vy = V_Leave_Max;			
					vTaskDelay(Time_PATH2_Leave);								
					Robot_target.Vy = 0.0;
					Robot_target.Vx = 0.0;
					vTaskDelay(100);
					
					//����
			////////////����Ĳ����Ƿ��̵�////////////
					kp_angle = 10;					
					Speed_programme(Time_PATH2_Left, V_Turn_Left_Max , 0.25, 0.2, &Robot_target.Vx);
					vTaskDelay(100); 			
					
					//����
			////////////����Ĳ����Ƿ��̵�////////////
					kp_angle=7;
					Speed_programme(Time_PATH2_Back, V_Back_Off_Max , 0.2, 0.2, &Robot_target.Vy);
					
					//��ԭ��
					Robot_target.Vx = V_Turn_Max;
					vTaskDelay(Time_PATH2_Turn); 
				}				

				//ͣ��	
				Robot_target.Vy=0.0;
				Robot_target.Vx=0.0;
				vTaskDelay(100);
				ROBOT.NOW_STATE = ROBOT_TASK_FINISHER;	//����ȡ����				
				break;
			
			
/*�������״̬*/
			case ROBOT_TASK_FINISHER:
				ROBOT.MPU6050 = NOT_USE;	//��ʹ��MPU6050	
//				LCD_Fill(0,0,LCD_W,LCD_H,BLACK);	//����
//				LCD_ShowString(22,28,"TASK FINISH!",YELLOW,BLACK,16,0);
				break;
		}
		
		vTaskDelay(50); 
	}
}



//������������
void Chassis_control(void *pvParameters)
{
	Robot_target.Vx = 0;
	Robot_target.Vy = 0;
	vTaskDelay(100); 
	while(1)
	{	
		//�˶�ѧ����
		
		Speed_analysis();

		//��ȡ������
		Encoder_Get_CNT();
//		//PID����
		PID_Incremental_PID_Calculation(&motor_PID[2], Encoder_realvalue.motor3 ,Wheel_target.V3 );	//motor3
		PID_Incremental_PID_Calculation(&motor_PID[1], Encoder_realvalue.motor2 ,Wheel_target.V2 );
		PID_Incremental_PID_Calculation(&motor_PID[0], Encoder_realvalue.motor1 ,Wheel_target.V1 );
		PID_Incremental_PID_Calculation(&motor_PID[3], Encoder_realvalue.motor4 ,Wheel_target.V4 );	//motor3
		
//		//���pwm
		Set_PWM();
		
		vTaskDelay(50); 		
	}
}



/*�����Ӧ*/
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
				vTaskDelay(2000); //��ֹû�и�Ӧ��
				ROBOT.NOW_STATE = FORWARD_STATE;				
			}
		}
		else if(ROBOT.NOW_STATE==TAKEING_STATE && ROBOT.DOOR_STATE==OPEN ){
			GPIO_ResetBits(GPIOE, GPIO_Pin_3); 
			GPIO_ResetBits(GPIOC, GPIO_Pin_12); 
			if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_12)==1||GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_3)==1){
				vTaskDelay(1500);		
				ROBOT.NOW_STATE = TURNBACK_STATE;	//����ȡ����
			}
			else{
				vTaskDelay(3500); 
				ROBOT.NOW_STATE = TURNBACK_STATE; //��ֹû�и�Ӧ��				
			}
				
			
		}
		
		vTaskDelay(5); 	
	}
}






/*��λ��*/	
void Datascope(void *pvParameters)
{
	unsigned char i; //��������
	unsigned char Send_Count; //������Ҫ���͵����ݸ���
	
	while(1)
	{
		DataScope_Get_Channel_Data(Encoder_realvalue.motor3, 1 );	//���2��ʵ���ٶȣ�����������ֵ��
		DataScope_Get_Channel_Data(Wheel_target.V3, 2 );			//���2��Ŀ���ٶȣ�����������ֵ��
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

/*�������޷�*/
float mpu_limit(float value)
{
	if(value > value_max){value =value_max;}
	if(value < -value_max){value =-value_max;}
	return value;
}

/*��ȡ����������*/
void Get_mpu6050_data(void *pvParameters)
{
	float pitch,roll,yaw; 		//ŷ����		
	while(1)
	{
		while(mpu_dmp_get_data(&pitch,&roll,&yaw)!=0){} 	//��������		
		Robot_target.acc = mpu_limit(yaw);
		vTaskDelay(1); 				 
	} 	
}


/*���*/
/*
int taggge = 0;
float Left_distance,Right_distance;
void Get_distance(void *pvParameters)
{
	int ROBOT_LAST_STATE,warn_sign = 0;	//���뾯����־λ
	while(1)
	{	
		Right_distance = Hcsr04GetLength();
		Left_distance = Hcsr04GetLength2();
		
		if(Left_distance < 35 | Right_distance < 35){
			ROBOT_LAST_STATE = ROBOT.NOW_STATE;
			//ͣ��	
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
