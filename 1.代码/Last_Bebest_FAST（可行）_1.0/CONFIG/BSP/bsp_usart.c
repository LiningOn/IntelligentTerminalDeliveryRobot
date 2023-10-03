#include <include.h>

/**
 *TX: PC6
 *RX: PC7
 **/
 
char SeeInfo=8;

void Bsp_USART6_Init(u32 bound_rate)
{
	// 声明结构体
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE);  // 使能USART4时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);   // 使能GPIOC时钟
 
	// 串口6对应引脚复用映射
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_USART6);  // PC6复用为USART6 TX
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_USART6);  // PC7复用为USART6 RX
	
	// USART6端口配置
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;       // 复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	 // 速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;     // 推挽输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;       // 上拉
	GPIO_Init(GPIOC,&GPIO_InitStructure); 
	
  // USART6初始化设置
	USART_InitStructure.USART_BaudRate = bound_rate;                                  // 波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;                      // 字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;                           // 一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;                              // 无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  // 无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	                 // 收发模式
	USART_Init(USART6, &USART_InitStructure);

	
	// 配置中断
	USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);  // 开启相关中断
	NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;           // 串口6中断通道   
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;  // 抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;		     // 子优先级0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			       // IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	
	
	USART_ClearFlag(USART6, USART_FLAG_TC);
	USART_Cmd(USART6, ENABLE);  // 使能串口6
}

void USART6_IRQHandler(void)                	//串口6中断服务程序
{	
	if(USART_GetITStatus(USART6, USART_IT_RXNE) == SET)   
	{
//		if(USART_ReceiveData(USART6)!=0){
			SeeInfo =USART_ReceiveData(USART6);//(USART1->DR);	//读取接收到的数据
//		}
		if(USART_GetITStatus(USART6, USART_IT_RXNE) == SET)   
		{ 
			USART_ClearITPendingBit(USART6, USART_IT_RXNE);  // 清除标志位
		}
	}
}



// 串口3初始化函数
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
void USART2_IRQHandler(void)                	//串口6中断服务程序
{	
	if(USART_GetITStatus(USART2, USART_IT_RXNE) == SET)   
	{
		am =USART_ReceiveData(USART2);//(USART1->DR);	//读取接收到的数据

		if(USART_GetITStatus(USART2, USART_IT_RXNE) == SET)   
		{ 
			USART_ClearITPendingBit(USART2, USART_IT_RXNE);  // 清除标志位
		}
	}
}



