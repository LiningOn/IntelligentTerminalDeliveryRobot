#include "HX711.h"
#include "delay.h"

u32 HX711_Buffer;
u32 Weight_Maopi;
s32 Weight_Shiwu;
u8 Flag_Error = 0;

//У׼����
//��Ϊ��ͬ�Ĵ������������߲��Ǻ�һ�£���ˣ�ÿһ����������Ҫ�������������������ʹ����ֵ��׼ȷ��
//�����ֲ��Գ���������ƫ��ʱ�����Ӹ���ֵ��
//������Գ���������ƫСʱ����С����ֵ��
//��ֵ����ΪС��
#define GapValue 48.5


void Init_HX711pin(void)
{
  GPIO_Init(GPIOA, GPIO_PIN_3, GPIO_MODE_OUT_PP_HIGH_FAST);//HX711_SCK
  GPIO_Init(GPIOA, GPIO_PIN_2, GPIO_MODE_IN_PU_NO_IT);//HX711_DOUT
  HX711_SCK(0); 
}
//****************************************************
//��ȡHX711
//****************************************************
u32 HX711_Read(void)	//����128
{
  unsigned long count; 
  unsigned char i; 
  HX711_DOUT_SETOUT;
  HX711_DOUT_OUT(1); 
  HX711_DOUT_SETIN;
  delay_us(10);
  HX711_SCK(0); 
  count=0; 
  while(HX711_DOUT_GET); 
  for(i=0;i<24;i++)
  { 
    HX711_SCK(1); 
    count=count<<1; 
    delay_us(10);
    HX711_SCK(0); 
    if(HX711_DOUT_GET)
      count++; 
    delay_us(10);
  } 
  HX711_SCK(1); 
  count=count^0x800000;//��25�������½�����ʱ��ת������
  delay_us(10);
  HX711_SCK(0);  
  return(count);
}

//****************************************************
//��ȡëƤ����
//****************************************************
void Get_Maopi(void)
{
  Weight_Maopi = HX711_Read();	
} 

//****************************************************
//����
//****************************************************
void Get_Weight(void)
{
  HX711_Buffer = HX711_Read();
  if(HX711_Buffer > Weight_Maopi)			
  {
    Weight_Shiwu = HX711_Buffer;
    Weight_Shiwu = Weight_Shiwu - Weight_Maopi;				//��ȡʵ���AD������ֵ��
    
    Weight_Shiwu = (s32)((float)Weight_Shiwu/GapValue); 	//����ʵ���ʵ������
    //��Ϊ��ͬ�Ĵ������������߲�һ������ˣ�ÿһ����������Ҫ���������GapValue���������
    //�����ֲ��Գ���������ƫ��ʱ�����Ӹ���ֵ��
    //������Գ���������ƫСʱ����С����ֵ��
  }
  
  
}
