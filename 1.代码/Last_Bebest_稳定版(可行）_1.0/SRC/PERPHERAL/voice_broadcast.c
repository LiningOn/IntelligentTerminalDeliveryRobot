#include "include.h"

//0xAAΪ��ʼ��  ��ʼ��-ָ������-���ݳ��ȣ� n�� -���� 1������ n���ͼ���(SM)
//Ϊ֮ǰ�����ֽ�֮�͵ĵ� 8 λ,����ʼ�뵽������Ӻ�ȡ�� 8 λ

/*   
ָ����Ŀ(07)
ָ� AA 07 02 ��Ŀ�� ��Ŀ�� SM
���أ���
����: AA 07 02 00 08 BB ָ�����ŵ�ǰ�̷��� 8 �ף���Ŀ���� 1�� 65535
��Ŀ������ɴ洢˳�������
*/

/*   
����(02)
ָ� AA 02 00 AC
���أ���
˵�������κ�ʱ�򷢴�������ͷ��ʼ���ŵ�ǰ��Ŀ
*/

u8 Next_song[4] = {0xAA,0x06,0x00,0xB0};  
u8 Last_song[4] = {0xAA,0x05,0x00,0xAF};  
u8 DATA[5] = {0xAA,0x18,0x01,0x02,0xC5};   //ѭ��ģʽ
u8 voice_1[6] = {0xAA,0x07,0x02,0x00,0x01,0xB4};   //
u8 voice_2[6] = {0xAA,0x07,0x02,0x00,0x02,0xB5};
u8 voice_3[6] = {0xAA,0x07,0x02,0x00,0x03,0xB6};
u8 voice_4[6] = {0xAA,0x07,0x02,0x00,0x04,0xB7};
u8 voice_5[6] = {0xAA,0x07,0x02,0x00,0x05,0xB8};
u8 voice_6[6] = {0xAA,0x07,0x02,0x00,0x06,0xB9};
u8 voice_7[6] = {0xAA,0x07,0x02,0x00,0x07,0xBA};
u8 voice_8[6] = {0xAA,0x07,0x02,0x00,0x08,0xBB}; //ָ����Ŀ
u8 voice_9[6] = {0xAA,0x07,0x02,0x00,0x09,0xBC};
u8 voice_10[6] = {0xAA,0x07,0x02,0x00,0x0a,0xBD};
u8 voice_11[6] = {0xAA,0x07,0x02,0x00,0x0b,0xBE};

u8 open[4] = {0xAA,0x02,0x00,0xAC};  
u8 stop[4] = {0xAA,0x03,0x00,0xAD};  //��ͣ

void Send_data(int num,u8 *Data)
{
	  u8 t;
	   for(t=0;t<num;t++)
		{
				USART_SendData(USART1, Data[t]);         //�򴮿�2��������
				while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//�ȴ����ͽ���
		}
}

//������ɨ����ɣ���װ��1�����ϣ����رղ���
//		delay_ms(50);	
//		Send_data(6,voice_1);  

//������ɨ����ɣ���װ��2�����ϣ����رղ���
//		delay_ms(50);
//	  Send_data(6,voice_2);

//����������1���ջ��㣬���ʾ�����
//		delay_ms(50);
//		Send_data(6,voice_3);

//����������2���ջ��㣬���ʾ�����
//		delay_ms(50);
//		Send_data(6,voice_4);

//��������ȡ��1�����ϣ����رղ���
//		delay_ms(50);
//		Send_data(6,voice_5);

//��������ȡ��2�����ϣ����رղ���
//		delay_ms(50);		
//		Send_data(6,voice_6);




