
#include "main.h"
#include "HX711.h"

unsigned long HX711_Buffer = 0;
unsigned long Weight_Maopi = 0;
long Weight_Shiwu = 0;

unsigned int Temp_Buffer = 0;
code unsigned char table[] = { 0x3f,0x06,0x5b,0x4f,0x66,0x6d,0x7d,0x07,0x7f,0x6f,0X00,0XFF,0x40};			//0,1,2,3,4,5,6,7,8,9,ȫ����ȫ��,g,e,r,o,h,l			//����
unsigned char COM1_DATA = 0,COM2_DATA = 0,COM3_DATA = 0,COM4_DATA = 0,COM5_DATA = 0,COM6_DATA = 0;
unsigned char flag = 0;

unsigned char  timer=0;

bit Flag_ERROR = 0;
bit Flag_yemian = 0;

sbit beep = P1^7;
	
sbit P22 = P2^2;
sbit P23 = P2^3;
sbit P24 = P2^4;
sbit P25 = P2^5;


unsigned int GapValue= 430;	  //У׼����
unsigned char num1,num2,num3; //num1 �� num2  �Ǵ洢�ڵ�Ƭ��eeprom�ڲ��ġ� num3��ˢ�±���
#include "eeprom52.h"




/******************�����ݱ��浽��Ƭ���ڲ�eeprom��******************/
void write_eeprom()
{
    num1=GapValue/256;
	num2=GapValue%256;
	SectorErase(0x2000);
	byte_write(0x2001,  num1);
	byte_write(0x2002,  num2);
	byte_write(0x2060, a_a);	
}

/******************�����ݴӵ�Ƭ���ڲ�eeprom�ж�����*****************/
void read_eeprom()
{
	num1   = byte_read(0x2001);
	num2   = byte_read(0x2002);
	a_a    = byte_read(0x2060);

	GapValue= num1*256+num2;
}

/**************�����Լ�eeprom��ʼ��*****************/
void init_eeprom()
{
	read_eeprom();		//�ȶ�
	if(a_a != 3)		//�µĵ�Ƭ����ʼ��Ƭ������eeprom
	{
		GapValue= 430;
		a_a = 3;
		write_eeprom();
	}	
}

//ɨ�谴��
void Scan_Key()
{
	if(KEY1 == 0)
	{
		Delay_ms(5);
		if(KEY1 == 0)
		{
			while(KEY1 == 0);
			Get_Maopi();			//ȥƤ
		}	
	}

	if(KEY2 == 0)
	{
		Delay_ms(25);
		if(KEY2 == 0)
		{
		//	while(KEY2 == 0);
	    	Flag_yemian=1; num3=0;
			
		    if(GapValue<999)  { 	GapValue++; }
			 write_eeprom();       //��������
			  
			  COM1_DATA =	12;	
			  COM2_DATA = GapValue/100;		
			  COM3_DATA = GapValue%100/10;		
			  COM4_DATA = GapValue%10;
		}	
	}

	if(KEY3 == 0)
	{
		Delay_ms(25);
		if(KEY3 == 0)
		{	 
		     Flag_yemian=1; num3=0;
			//while(KEY3 == 0);
	    	if(GapValue>1)  { 	GapValue--; }
			 write_eeprom();       //��������

			  COM1_DATA =	12;	
			  COM2_DATA = GapValue/100;		
			  COM3_DATA = GapValue%100/10;		
			  COM4_DATA = GapValue%10;
		}	
	}
}



//****************************************************
//������
//****************************************************
void main()
{
  init_eeprom();
	Init_Timer0();

	COM1_DATA =	12;				
	COM2_DATA = 12;					
	COM3_DATA = 12;					
	COM4_DATA = 12;					
	Delay_ms(1000);		 //��ʱ,�ȴ��������ȶ�
  Get_Maopi();				//��ëƤ����
	
	while(1)
	{ 
		
    EA = 0;
		Get_Weight();			//����
		EA = 1;

		Scan_Key();
	  num3++; 
    if(num3>=10)  {num3=0; Flag_yemian=0; }
    if(Flag_yemian==0)
		 {
		   //��ʾ��ǰ����
		    if( Flag_ERROR == 1)
	     	 {
           beep =0;	  //����������
         }
		
		    else
	      {
		      beep =1;
			    COM1_DATA = Weight_Shiwu/1000;		
			    COM2_DATA = Weight_Shiwu%1000/100;		
			    COM3_DATA = Weight_Shiwu%100/10;		
			    COM4_DATA = Weight_Shiwu%10;
				
        }
				Delay_ms(100);	
      
			
		}
	}
}



//****************************************************
//����
//****************************************************
void Get_Weight()
{
	Weight_Shiwu = HX711_Read();
	Weight_Shiwu = Weight_Shiwu - Weight_Maopi;		//��ȡ����
	if(Weight_Shiwu > 0)			
	{	
		Weight_Shiwu = (unsigned int)((float)Weight_Shiwu/GapValue); 	//����ʵ���ʵ������
																		
																		
		if(Weight_Shiwu > 5000)		//���ر���
		{
			Flag_ERROR = 1;
			COM1_DATA = 5;			//
			COM2_DATA = 0;			//
			COM3_DATA = 0;			//
			COM4_DATA = 0;			//
		}
		else
		{
			Flag_ERROR = 0;
		
		}
	}
	else
	{
		Weight_Shiwu = 0;
	//	Flag_ERROR = 1;				//���ر���
	}
	
}

//****************************************************
//��ȡëƤ����
//****************************************************
void Get_Maopi()
{
	Weight_Maopi = HX711_Read();	
} 


//****************************************************
//��ʼ����ʱ��0
//****************************************************
void Init_Timer0()
{
	TMOD = 0X01;			//T0, ����ģʽ1

	TH0 = (65536 - 1000)/256;
	TL0 = (65536 - 1000)%256;

	TR0 = 1;				//������ʱ��
	ET0 = 1;			    //������ʱ���ж�
	EA = 1;					//�������ж�		
}


//�жϺ���
//****************************************************
void Timer0() interrupt 1
{
	TH0 = (65536 - 1000)/256;
	TL0 = (65536 - 1000)%256;
	
  flag++;
	if(flag >= 4)
	{
		flag = 0;
		
	}
  
switch(flag)
	{ 
		
		case 0:
		    P23=P24=P25=1;P22=0;
        SEG_DATA = table[COM1_DATA];	
        break;

		case 1:
				P23=0;P22=P24=P25=1;
				SEG_DATA = table[COM2_DATA];		//��ʾ�ڶ�λֵ
				
				break;

							  
		case 2:
				P24=0;P22=P23=P25=1;
				SEG_DATA = table[COM3_DATA];		////��ʾ����λֵ
				
				break;


		case 3:
				P25=0;P22=P23=P24=1;
				SEG_DATA = table[COM4_DATA];			  //��ʾ����λֵ
				break;

	

	
	}


} 

//****************************************************
//MS��ʱ����(12M�����²���)
//****************************************************
void Delay_ms(unsigned int n)
{
	unsigned int  i,j;
	for(i=0;i<n;i++)
		for(j=0;j<123;j++);
}