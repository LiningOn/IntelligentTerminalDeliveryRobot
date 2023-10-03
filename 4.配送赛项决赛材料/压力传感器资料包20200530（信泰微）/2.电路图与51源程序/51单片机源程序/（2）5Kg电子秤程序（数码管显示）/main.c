
#include "main.h"
#include "HX711.h"

unsigned long HX711_Buffer = 0;
unsigned long Weight_Maopi = 0;
long Weight_Shiwu = 0;

unsigned int Temp_Buffer = 0;
code unsigned char table[] = { 0x3f,0x06,0x5b,0x4f,0x66,0x6d,0x7d,0x07,0x7f,0x6f,0X00,0XFF,0x40};			//0,1,2,3,4,5,6,7,8,9,全暗，全亮,g,e,r,o,h,l			//共阴
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


unsigned int GapValue= 430;	  //校准参数
unsigned char num1,num2,num3; //num1 和 num2  是存储在单片机eeprom内部的。 num3是刷新变量
#include "eeprom52.h"




/******************把数据保存到单片机内部eeprom中******************/
void write_eeprom()
{
    num1=GapValue/256;
	num2=GapValue%256;
	SectorErase(0x2000);
	byte_write(0x2001,  num1);
	byte_write(0x2002,  num2);
	byte_write(0x2060, a_a);	
}

/******************把数据从单片机内部eeprom中读出来*****************/
void read_eeprom()
{
	num1   = byte_read(0x2001);
	num2   = byte_read(0x2002);
	a_a    = byte_read(0x2060);

	GapValue= num1*256+num2;
}

/**************开机自检eeprom初始化*****************/
void init_eeprom()
{
	read_eeprom();		//先读
	if(a_a != 3)		//新的单片机初始单片机内问eeprom
	{
		GapValue= 430;
		a_a = 3;
		write_eeprom();
	}	
}

//扫描按键
void Scan_Key()
{
	if(KEY1 == 0)
	{
		Delay_ms(5);
		if(KEY1 == 0)
		{
			while(KEY1 == 0);
			Get_Maopi();			//去皮
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
			 write_eeprom();       //保存数据
			  
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
			 write_eeprom();       //保存数据

			  COM1_DATA =	12;	
			  COM2_DATA = GapValue/100;		
			  COM3_DATA = GapValue%100/10;		
			  COM4_DATA = GapValue%10;
		}	
	}
}



//****************************************************
//主函数
//****************************************************
void main()
{
  init_eeprom();
	Init_Timer0();

	COM1_DATA =	12;				
	COM2_DATA = 12;					
	COM3_DATA = 12;					
	COM4_DATA = 12;					
	Delay_ms(1000);		 //延时,等待传感器稳定
  Get_Maopi();				//称毛皮重量
	
	while(1)
	{ 
		
    EA = 0;
		Get_Weight();			//称重
		EA = 1;

		Scan_Key();
	  num3++; 
    if(num3>=10)  {num3=0; Flag_yemian=0; }
    if(Flag_yemian==0)
		 {
		   //显示当前重量
		    if( Flag_ERROR == 1)
	     	 {
           beep =0;	  //蜂鸣器报警
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
//称重
//****************************************************
void Get_Weight()
{
	Weight_Shiwu = HX711_Read();
	Weight_Shiwu = Weight_Shiwu - Weight_Maopi;		//获取净重
	if(Weight_Shiwu > 0)			
	{	
		Weight_Shiwu = (unsigned int)((float)Weight_Shiwu/GapValue); 	//计算实物的实际重量
																		
																		
		if(Weight_Shiwu > 5000)		//超重报警
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
	//	Flag_ERROR = 1;				//负重报警
	}
	
}

//****************************************************
//获取毛皮重量
//****************************************************
void Get_Maopi()
{
	Weight_Maopi = HX711_Read();	
} 


//****************************************************
//初始化定时器0
//****************************************************
void Init_Timer0()
{
	TMOD = 0X01;			//T0, 工作模式1

	TH0 = (65536 - 1000)/256;
	TL0 = (65536 - 1000)%256;

	TR0 = 1;				//开启定时器
	ET0 = 1;			    //开启定时器中断
	EA = 1;					//开启总中断		
}


//中断函数
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
				SEG_DATA = table[COM2_DATA];		//显示第二位值
				
				break;

							  
		case 2:
				P24=0;P22=P23=P25=1;
				SEG_DATA = table[COM3_DATA];		////显示第三位值
				
				break;


		case 3:
				P25=0;P22=P23=P24=1;
				SEG_DATA = table[COM4_DATA];			  //显示第四位值
				break;

	

	
	}


} 

//****************************************************
//MS延时函数(12M晶振下测试)
//****************************************************
void Delay_ms(unsigned int n)
{
	unsigned int  i,j;
	for(i=0;i<n;i++)
		for(j=0;j<123;j++);
}