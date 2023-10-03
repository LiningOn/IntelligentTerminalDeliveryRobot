#include <include.h>

/**
 *TX: PC6
 *RX: PC7
 **/
 
char SeeInfo=8;

void Bsp_USART6_Init(u32 bound_rate)
{
	// �����ṹ��
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE);  // ʹ��USART4ʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);   // ʹ��GPIOCʱ��
 
	// ����6��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_USART6);  // PC6����ΪUSART6 TX
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_USART6);  // PC7����ΪUSART6 RX
	
	// USART6�˿�����
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;       // ���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	 // �ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;     // �������
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;       // ����
	GPIO_Init(GPIOC,&GPIO_InitStructure); 
	
  // USART6��ʼ������
	USART_InitStructure.USART_BaudRate = bound_rate;                                  // ����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;                      // �ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;                           // һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;                              // ����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  // ��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	                 // �շ�ģʽ
	USART_Init(USART6, &USART_InitStructure);

	
	// �����ж�
	USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);  // ��������ж�
	NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;           // ����6�ж�ͨ��   
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;  // ��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;		     // �����ȼ�0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			       // IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	
	
	USART_ClearFlag(USART6, USART_FLAG_TC);
	USART_Cmd(USART6, ENABLE);  // ʹ�ܴ���6
}

void USART6_IRQHandler(void)                	//����6�жϷ������
{	
	if(USART_GetITStatus(USART6, USART_IT_RXNE) == SET)   
	{
//		if(USART_ReceiveData(USART6)!=0){
			SeeInfo =USART_ReceiveData(USART6);//(USART1->DR);	//��ȡ���յ�������
//		}
		if(USART_GetITStatus(USART6, USART_IT_RXNE) == SET)   
		{ 
			USART_ClearITPendingBit(USART6, USART_IT_RXNE);  // �����־λ
		}
	}
}



// ����3��ʼ������
void Bsp_USART2_Init(u32 baud_rate)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);  
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); 

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_USART2); 
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2); 

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;			
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;			
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;			 
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = baud_rate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;						
	USART_InitStructure.USART_StopBits = USART_StopBits_1;						
	USART_InitStructure.USART_Parity = USART_Parity_No;								
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; 
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;				
	USART_Init(USART2, &USART_InitStructure);									

	USART_Cmd(USART2, ENABLE); 

	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); 
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7; 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		  
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			  
	NVIC_Init(&NVIC_InitStructure);							 
}



char am;
void USART2_IRQHandler(void)                	//����6�жϷ������
{	
	if(USART_GetITStatus(USART2, USART_IT_RXNE) == SET)   
	{
		am =USART_ReceiveData(USART2);//(USART1->DR);	//��ȡ���յ�������

		if(USART_GetITStatus(USART2, USART_IT_RXNE) == SET)   
		{ 
			USART_ClearITPendingBit(USART2, USART_IT_RXNE);  // �����־λ
		}
	}
}



