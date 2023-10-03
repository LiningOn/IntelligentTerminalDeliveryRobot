#include <include.h>
 

//������ʼ������
void KEY_Init(void)
{
	
  GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);//ʹ��GPIOEʱ��
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);//ʹ��

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3; //KEY0 KEY1 ��Ӧ����
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//��ͨ����ģʽ
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//����
  GPIO_Init(GPIOE, &GPIO_InitStructure);//��ʼ��GPIOE3
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12; //KEY0 KEY1 ��Ӧ����
  GPIO_Init(GPIOC, &GPIO_InitStructure);//��ʼ��GPIOE3

} 


//����������
//���ذ���ֵ
//mode:0,��֧��������;1,֧��������;
//0��û���κΰ�������
//2��KEY1����
//3��KxcEY2���� 
//ע��˺�������Ӧ���ȼ�,KEY1>KEY2

u8 KEY_Scan(u8 mode)
{	 
	static u8 key_up=1;//�������ɿ���־
	if(mode)key_up=1;  //֧������		  
	if(key_up&&(PE3==1||PE2==1))
	{
		delay_ms(10);//ȥ���� 
		key_up=0;
		if(PE3==1)return 3;
		else if(PE2==1)return 2;
	}else if(PE3==0&&PE2==0)key_up=1; 	    
 	return 0;// �ް�������
}










