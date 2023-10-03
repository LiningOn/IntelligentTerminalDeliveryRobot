#include "include.h"


void Bsp_HC_SR04_Init(void)
{
	//超声波模块1
	Hcsr04Init();
	//超声波模块2
	Hcsr04Init2();
}


////////////////////////////						超声波模块1						///////////////////////////
//定时器2设置
u16 msHcCount = 0;
void hcsr04_NVIC()
{
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
			
	NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;             
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;         
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;       
	NVIC_Init(&NVIC_InitStructure);
}

//IO口初始化 及其他初始化
void Hcsr04Init()
{      
	GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;   
    
	
	//输出和接收
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
     //PB11  
    GPIO_InitStructure.GPIO_Pin =GPIO_Pin_11;      
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;				//改
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOB,GPIO_Pin_11);
     //PB10
    GPIO_InitStructure.GPIO_Pin =   GPIO_Pin_10;  
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;				//改
    GPIO_Init(GPIOB, &GPIO_InitStructure);  
    GPIO_ResetBits(GPIOB,GPIO_Pin_10);    
     
	
	//计时
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);   
     
    TIM_DeInit(TIM7);
    TIM_TimeBaseStructure.TIM_Period = (1000-1); 	
    TIM_TimeBaseStructure.TIM_Prescaler =(84-1); 			//改
    TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
    TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);          
        
    TIM_ClearFlag(TIM7, TIM_FLAG_Update);  
    TIM_ITConfig(TIM7,TIM_IT_Update,ENABLE);    
    hcsr04_NVIC();
    TIM_Cmd(TIM7,DISABLE);     
}



//打开定时器2
static void OpenTimerForHc()  
{
   TIM_SetCounter(TIM7,0);
   msHcCount = 0;
   TIM_Cmd(TIM7, ENABLE); 
}

//关闭定时器2
static void CloseTimerForHc()    
{
   TIM_Cmd(TIM7, DISABLE); 
}

//定时器2终中断
void TIM7_IRQHandler(void)  
{
   if (TIM_GetITStatus(TIM7, TIM_IT_Update) != RESET)  
   {
       TIM_ClearITPendingBit(TIM7, TIM_IT_Update  ); 
       msHcCount++;
   }
}
 

//获取定时器2计数器值
u32 GetEchoTimer(void)
{
   u32 t = 0;
   t = msHcCount*1000;
   t += TIM_GetCounter(TIM7);
   TIM7->CNT = 0;  
   delay_ms(50);
   return t;
}
 
//通过定时器2计数器值推算距离
float Hcsr04GetLength(void )
{
   u32 t = 0;
   int i = 0;
   float lengthTemp = 0;
   float sum = 0;
   while(i!=5)
   {
      PBout(11) = 1;      
      delay_us(20);
      PBout(11) = 0;
      while(PBin(10) == 0);      
      OpenTimerForHc();        
      i = i + 1;
      while(PBin(10) == 1);
      CloseTimerForHc();        
      t = GetEchoTimer();        
      lengthTemp = ((float)t/58.0);//cm
      sum = lengthTemp + sum ;
        
    }
    lengthTemp = sum/5.0;
    return lengthTemp;
}


////////////////////////////						超声波模块2					///////////////////////////
u16 msHcCount2 = 0;
void hcsr04_NVIC2()
{
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
			
	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;             
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;         
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;       
	NVIC_Init(&NVIC_InitStructure);
}

void Hcsr04Init2()
{      
	GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;   
    
	//输出和接收
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
     //PB12  
    GPIO_InitStructure.GPIO_Pin =GPIO_Pin_13;      
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;				//改
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOB,GPIO_Pin_13);
     //PB13
    GPIO_InitStructure.GPIO_Pin =   GPIO_Pin_12;  
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;				//改
    GPIO_Init(GPIOB, &GPIO_InitStructure);  
    GPIO_ResetBits(GPIOB,GPIO_Pin_12);    
     
	
	//计时
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);   
     
    TIM_DeInit(TIM5);
    TIM_TimeBaseStructure.TIM_Period = (1000-1); 	
    TIM_TimeBaseStructure.TIM_Prescaler =(84-1); 			//改
    TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
    TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);          
        
    TIM_ClearFlag(TIM5, TIM_FLAG_Update);  
    TIM_ITConfig(TIM5,TIM_IT_Update,ENABLE);    
    hcsr04_NVIC2();
    TIM_Cmd(TIM5,DISABLE);     
}

//打开定时器5
static void OpenTimerForHc2()  
{
   TIM_SetCounter(TIM5,0);
   msHcCount2 = 0;
   TIM_Cmd(TIM5, ENABLE); 
}

//关闭定时器5
static void CloseTimerForHc2()    
{
   TIM_Cmd(TIM5, DISABLE); 
}

//定时器2终中断
void TIM5_IRQHandler(void)  
{
   if (TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET)  
   {
       TIM_ClearITPendingBit(TIM5, TIM_IT_Update  ); 
       msHcCount2++;
   }
}
 

//获取定时器5计数器值
u32 GetEchoTimer2(void)
{
   u32 t = 0;
   t = msHcCount2*1000;
   t += TIM_GetCounter(TIM5);
   TIM5->CNT = 0;  
   delay_ms(50);
   return t;
}
 
//通过定时器2计数器值推算距离
float Hcsr04GetLength2(void )
{
   u32 t = 0;
   int i = 0;
   float lengthTemp = 0;
   float sum = 0;
   while(i!=5)
   {
      PBout(13) = 1;      
      delay_us(20);
      PBout(13) = 0;
      while(PBin(12) == 0);      
      OpenTimerForHc2();        
      i = i + 1;
      while(PBin(12) == 1);
      CloseTimerForHc2();        
      t = GetEchoTimer2();        
      lengthTemp = ((float)t/58.0);//cm
      sum = lengthTemp + sum ;
        
    }
    lengthTemp = sum/5.0;
    return lengthTemp;
}


