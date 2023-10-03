#include "include.h"	/*ͷ�ļ�����*/

TaskHandle_t startTaskHandle;
static void startTask(void *arg);

int main() 
{
	All_Init();	//��ʼ��
	Robot_target.Vy = 300.0;			
	xTaskCreate(startTask, "START_TASK", 300, NULL, 2, &startTaskHandle);	/*������ʼ����*/
	vTaskStartScheduler();	/*������v�����*/

	while(1){};

}

/*��������*/
void startTask(void *pvParameters)
{
	taskENTER_CRITICAL();	/*�����ٽ���*/
	
							// ���ȼ���ߣ�����״̬  ��ֵԽС�����ȼ�Խ��
	xTaskCreate(Chassis_control, "Chassis_control", 500, NULL, 5, NULL);	//���̿���
	xTaskCreate(Robot_FSM, "Robot_FSM", 500, NULL, 4, NULL);	// ״̬��
	xTaskCreate(Get_mpu6050_data, "Get_mpu6050_data", 500, NULL, 2, NULL);	// ������
	xTaskCreate(Key_Check, "Key_Check", 500, NULL, 1, NULL);	// ΢������		
//	xTaskCreate(Get_distance, "Get_distance", 500, NULL, 3, NULL);	// ���������
	
	
//	xTaskCreate(Datascope, "Datascope", 500, NULL, 2, NULL);	// ��λ��
//	xTaskCreate(Voice_play, "Voice_play", 500, NULL, 4, NULL);	// �Ӿ�		

	vTaskDelete(startTaskHandle);										/*ɾ����ʼ����*/
	taskEXIT_CRITICAL();	/*�˳��ٽ���*/
}


void All_Init(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	delay_init(168);
	
	Bsp_MPU6050_Init();
	delay_ms(1000);
	Bsp_HC_SR04_Init();	
	Bsp_STERRING_Init();	
	KEY_Init();
	
	uart_init(9600);  //��ʼ������1--��������	
	Bsp_UART4_Init(9600);	//����4--��ά��ʶ��
	Bsp_USART6_Init(115200);	//����6--openmv
	
	Bsp_Motor_Init();
	Motor_PID_Init();
	Bsp_TFT_Init();
//	LCD_ShowString(14,23,"INIT SUCCESS!",GREEN,BLACK,16,0);
}

/*	������־��
	1.mpu6050��ʼ��ʧ�ܣ������ϵͳ���ԣ����ڲ���ϵͳrtos����		2021/10/09
		�ѽ����IIC�ļ����Ǽ���
	2.openmv ʶ����е���
		ԭ�򣨲²⣩��һֱ����0����
		�ѽ����ԭ���ǳ�ʼ����ʱ����ʱ��
	3.����������ʱ��������������
		�ѽ����ԭ����û�С�Ԥ�ȡ�
	4.΢��������ʱ����������´򿪶��
		δ��������Ǻ������û������
	5.���������ϣ������������2��
		������Ҳ��
	6.����ȡ��û�����
		�ѽ�����ô����߼������
	7.ȡ�����Ϻ������е���

*/

