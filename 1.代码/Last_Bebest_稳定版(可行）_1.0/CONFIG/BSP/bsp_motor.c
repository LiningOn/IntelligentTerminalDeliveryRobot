#include <include.h>

//�����ʼ��
void Bsp_Motor_Init()	
{
	
	TIM9_PWM_Init(0xffff,0);		//PE5���PWM
	TIM12_PWM_Init(0xffff,0);		//PB14��PE15���PWM		
	TIM3_ENCODER_Init();	
	TIM1_ENCODER_Init();	
	TIM4_ENCODER_Init(); 	
	TIM2_ENCODER_Init();
	TB6612_Init();	
}




/**************************************************************************
�������ܣ� TB6612 ��ʼ������
��ڲ����� ��
���� ֵ�� ��
**************************************************************************/
void TB6612_Init(void)
{
	//����ṹ��
	GPIO_InitTypeDef GPIO_InitStructure; 
	
	//ʹ��ʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); 
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE); 
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	
	//GPIOC 
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT; 
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed=GPIO_High_Speed; 
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOC, GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15); 	
	//GPIOD 	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;  
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed=GPIO_High_Speed; 
	GPIO_Init(GPIOD, &GPIO_InitStructure); 
	GPIO_ResetBits(GPIOD, GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11); 
	//GPIOE 		
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT; 
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed=GPIO_High_Speed; 
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOE, GPIO_Pin_4); 
}

/**************************************************************************
�������ܣ� PWM�����ʼ������
��ڲ����� ��
���� ֵ�� ��
��ע��TIM9��TIM12���pwm
**************************************************************************/
void TIM9_PWM_Init(u32 arr,u32 psc)
{		 					 	
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9,ENABLE);  	//TIM9ʱ��ʹ��    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE); 	//ʹ��PORTFʱ��	
	
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource5,GPIO_AF_TIM9); //GPIO����λ��ʱ��9
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource6,GPIO_AF_TIM9);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6; //GPIO 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOE,&GPIO_InitStructure); //��ʼ��
	
	
	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //��ʱ����Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_Period=arr;   //�Զ���װ��ֵ
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM9,&TIM_TimeBaseStructure);
	
	//��ʼ��TIM9 Channel1 PWMģʽ	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //�������:TIM����Ƚϼ��Ը�
	TIM_OCInitStructure.TIM_Pulse=0;
 	//CH1 	
	TIM_OC1Init(TIM9, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM9 OC1
	TIM_OC1PreloadConfig(TIM9, TIM_OCPreload_Disable);  //ʹ��TIM9��CCR1�ϵ�Ԥװ�ؼĴ���
 	//CH2 
	TIM_OC2Init(TIM9, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM9, TIM_OCPreload_Disable);
	
	TIM_ARRPreloadConfig(TIM9,ENABLE);
	
	TIM_Cmd(TIM9, ENABLE);  //ʹ��TIM9		

}  



void TIM12_PWM_Init(u32 arr,u32 psc)
{		 					 
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12,ENABLE);  	//TIM12ʱ��ʹ��    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); 	//ʹ��PORTFʱ��	
	
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource14,GPIO_AF_TIM12); //GPIO����λ��ʱ��12
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource15,GPIO_AF_TIM12);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14|GPIO_Pin_15; //GPIO 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOB,&GPIO_InitStructure); //��ʼ��
	
	
	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //��ʱ����Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_Period=arr;   //�Զ���װ��ֵ
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM12,&TIM_TimeBaseStructure);
	
	//��ʼ��TIM12 Channel1 PWMģʽ	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //�������:TIM����Ƚϼ��Ը�
	TIM_OCInitStructure.TIM_Pulse=0;
	//CH1
	TIM_OC1Init(TIM12, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM12 OC1
	TIM_OC1PreloadConfig(TIM12, TIM_OCPreload_Disable);  //ʹ��TIM12��CCR1�ϵ�Ԥװ�ؼĴ���
	//CH2 
	TIM_OC2Init(TIM12, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM12, TIM_OCPreload_Disable);
 
	TIM_ARRPreloadConfig(TIM12,ENABLE);
	
	TIM_Cmd(TIM12, ENABLE);  //ʹ��TIM12		

}  


//TIM9�жϷ�����
void TIM9_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM9,TIM_IT_Update)!=0)
	{
		TIM_ClearITPendingBit(TIM9,TIM_IT_Update);
	}
}

//TIM12�жϷ�����
void TIM12_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM12,TIM_IT_Update)!=0)
	{
		TIM_ClearITPendingBit(TIM12,TIM_IT_Update);
	}
}


/**************************************************************************
�������ܣ�TIM3��TIM1��TIM4 ---> ������ģʽ
��ڲ�������
����  ֵ����
**************************************************************************/
void TIM3_ENCODER_Init(void)	//���ñ�����ģʽ   
{ 
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure; 
	TIM_ICInitTypeDef TIM_ICInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	
	GPIO_InitStructure.GPIO_Mode= GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_6| GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource6,GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_TIM3);
	
	TIM_TimeBaseStructure.TIM_Period= 65535;
	TIM_TimeBaseStructure.TIM_Prescaler=0;
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);
	
	TIM_EncoderInterfaceConfig(TIM3,TIM_EncoderMode_TI12,TIM_ICPolarity_Rising,TIM_ICPolarity_Rising);//���ñ�����ģʽ
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_ICFilter=0;
	TIM_ICInit(TIM3,&TIM_ICInitStructure);
	
	TIM_ClearFlag(TIM3,TIM_FLAG_Update);
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);
	TIM_SetCounter(TIM3, 0); 
	TIM_Cmd(TIM3,ENABLE);
} 


void TIM1_ENCODER_Init(void)	//���ñ�����ģʽ   
{ 
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure2;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure; 
	TIM_ICInitTypeDef TIM_ICInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE);

	//PA8	
	GPIO_InitStructure.GPIO_Mode= GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource8,GPIO_AF_TIM1);
//	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_TIM1);
	
	//PE11
	GPIO_InitStructure2.GPIO_Mode= GPIO_Mode_AF;
	GPIO_InitStructure2.GPIO_Pin=GPIO_Pin_11;
	GPIO_InitStructure2.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_InitStructure2.GPIO_OType=GPIO_OType_OD;
	GPIO_InitStructure2.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_Init(GPIOE,&GPIO_InitStructure2);
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource11,GPIO_AF_TIM1);
	
	TIM_TimeBaseStructure.TIM_Period= 65535;
	TIM_TimeBaseStructure.TIM_Prescaler=0;
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM1,&TIM_TimeBaseStructure);
	
	TIM_EncoderInterfaceConfig(TIM1,TIM_EncoderMode_TI12,TIM_ICPolarity_Rising,TIM_ICPolarity_Rising);//���ñ�����ģʽ
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_ICFilter=0;
	TIM_ICInit(TIM1,&TIM_ICInitStructure);
	
	TIM_ClearFlag(TIM1,TIM_FLAG_Update);
	TIM_ITConfig(TIM1,TIM_IT_Update,ENABLE);
	TIM_SetCounter(TIM1, 0); 
	TIM_Cmd(TIM1,ENABLE);
} 


void TIM4_ENCODER_Init(void)	//���ñ�����ģʽ   
{ 
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure; 
	TIM_ICInitTypeDef TIM_ICInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	
	GPIO_InitStructure.GPIO_Mode= GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_6| GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource7,GPIO_AF_TIM4);
	
	TIM_TimeBaseStructure.TIM_Period= 65535;
	TIM_TimeBaseStructure.TIM_Prescaler=0;
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseStructure);
	
	TIM_EncoderInterfaceConfig(TIM4,TIM_EncoderMode_TI12,TIM_ICPolarity_Rising,TIM_ICPolarity_Rising);//���ñ�����ģʽ
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_ICFilter=0;
	TIM_ICInit(TIM4,&TIM_ICInitStructure);
	
	TIM_ClearFlag(TIM4,TIM_FLAG_Update);
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);
	TIM_SetCounter(TIM4, 0); 
	TIM_Cmd(TIM4,ENABLE);
} 


void TIM2_ENCODER_Init(void)	//���ñ�����ģʽ   
{ 
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure; 
	TIM_ICInitTypeDef TIM_ICInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	
	GPIO_InitStructure.GPIO_Mode= GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0| GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource0,GPIO_AF_TIM2);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource1,GPIO_AF_TIM2);
	
	TIM_TimeBaseStructure.TIM_Period= 65535;
	TIM_TimeBaseStructure.TIM_Prescaler=0;
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);
	
	TIM_EncoderInterfaceConfig(TIM2,TIM_EncoderMode_TI12,TIM_ICPolarity_Rising,TIM_ICPolarity_Rising);//���ñ�����ģʽ
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_ICFilter=0;
	TIM_ICInit(TIM2,&TIM_ICInitStructure);
	
	TIM_ClearFlag(TIM2,TIM_FLAG_Update);
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
	TIM_SetCounter(TIM2, 0); 
	TIM_Cmd(TIM2,ENABLE);
} 



//TIM3�жϷ�����
void TIM3_IRQHandler(void)
{
	if(TIM3->SR&0X0001)//����ж�
	{ 

	} 
    TIM3->SR&=~(1<<0);//����жϱ�־λ
}

//TIM1�жϷ�����
void TIM1_IRQHandler(void)
{
	if(TIM1->SR&0X0001)//����ж�
	{ 

	} 
    TIM1->SR&=~(1<<0);//����жϱ�־λ
}

//TIM4�жϷ�����
void TIM4_IRQHandler(void)
{
	if(TIM4->SR&0X0001)//����ж�
	{ 

	} 
    TIM4->SR&=~(1<<0);//����жϱ�־λ
}

//TIM2�жϷ�����
void TIM2_IRQHandler(void)
{
	if(TIM2->SR&0X0001)//����ж�
	{ 

	} 
    TIM2->SR&=~(1<<0);//����жϱ�־λ
}





