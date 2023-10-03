#include <include.h>


/**
 *TX: PC10
 *RX: PC11
 **/
void Bsp_UART4_Init(u32 bound_rate)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);

	// 复用
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_UART4); 
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_UART4);

	// UART4端口
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	  
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	 
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	// UART4初始化
	USART_InitStructure.USART_BaudRate = bound_rate;									
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;						
	USART_InitStructure.USART_StopBits = USART_StopBits_1;							
	USART_InitStructure.USART_Parity = USART_Parity_No;								
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; 
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;					
	USART_Init(UART4, &USART_InitStructure);

	USART_Cmd(UART4, ENABLE); // 使能

	// 配置中断
	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;		 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		  
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			 
	NVIC_Init(&NVIC_InitStructure);

	USART_ClearFlag(UART4, USART_FLAG_TC);
	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE); 
}

int i = 0,Buffer[2]={0,0},QRcode = 0;
void UART4_IRQHandler(void)                	//串口4中断服务程序
{	
	
	if(USART_GetITStatus(UART4, USART_IT_RXNE) != RESET) {
		USART_ClearITPendingBit(UART4, USART_IT_RXNE);
		Buffer[i++]=USART_ReceiveData(UART4) - 48; //Buffesh是一个自己定义的接收数组
		if(i > 1)	i = 0;
		QRcode = Buffer[0]*10 + Buffer[1];
	}
}

void Bsp_UART5_Init(u32 bound_rate)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);

	// 复用
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_UART5); 
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource2, GPIO_AF_UART4);

	// UART4端口
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	  
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	 
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	// UART5初始化
	USART_InitStructure.USART_BaudRate = bound_rate;									
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;						
	USART_InitStructure.USART_StopBits = USART_StopBits_1;							
	USART_InitStructure.USART_Parity = USART_Parity_No;								
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; 
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;					
	USART_Init(UART5, &USART_InitStructure);

	USART_Cmd(UART5, ENABLE); // 使能

	// 配置中断
	NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;		 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		  
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			 
	NVIC_Init(&NVIC_InitStructure);

	USART_ClearFlag(UART5, USART_FLAG_TC);
	USART_ITConfig(UART5, USART_IT_RXNE, ENABLE); 
}

void UART5_IRQHandler(void)                	//串口6中断服务程序
{	
	if(USART_GetITStatus(UART5, USART_IT_RXNE) == SET)   
	{
//		sss =USART_ReceiveData(UART5);//(USART1->DR);	//读取接收到的数据

		if(USART_GetITStatus(UART5, USART_IT_RXNE) == SET)   
		{ 
			//VisionReceiveData(&ROBOT_POT_INFO.pos_x, &ROBOT_POT_INFO.pos_y, &ROBOT_POT_INFO.pos_angle, &ROBOT_POT_INFO.pot_num );			
			USART_ClearITPendingBit(UART5, USART_IT_RXNE);  // 清除标志位
		}
	}
}
