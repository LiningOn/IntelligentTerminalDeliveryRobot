#include <include.h>
 

//按键初始化函数
void KEY_Init(void)
{
	
  GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);//使能GPIOE时钟
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);//使能

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3; //KEY0 KEY1 对应引脚
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//普通输入模式
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//下拉
  GPIO_Init(GPIOE, &GPIO_InitStructure);//初始化GPIOE3
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12; //KEY0 KEY1 对应引脚
  GPIO_Init(GPIOC, &GPIO_InitStructure);//初始化GPIOE3

} 


//按键处理函数
//返回按键值
//mode:0,不支持连续按;1,支持连续按;
//0，没有任何按键按下
//2，KEY1按下
//3，KxcEY2按下 
//注意此函数有响应优先级,KEY1>KEY2

u8 KEY_Scan(u8 mode)
{	 
	static u8 key_up=1;//按键按松开标志
	if(mode)key_up=1;  //支持连按		  
	if(key_up&&(PE3==1||PE2==1))
	{
		delay_ms(10);//去抖动 
		key_up=0;
		if(PE3==1)return 3;
		else if(PE2==1)return 2;
	}else if(PE3==0&&PE2==0)key_up=1; 	    
 	return 0;// 无按键按下
}










