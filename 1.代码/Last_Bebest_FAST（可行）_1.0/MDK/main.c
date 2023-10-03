#include "include.h"	/*头文件集合*/

TaskHandle_t startTaskHandle;
static void startTask(void *arg);

int main() 
{
	All_Init();	//初始化
	Robot_target.Vy = 300.0;			
	xTaskCreate(startTask, "START_TASK", 300, NULL, 2, &startTaskHandle);	/*创建起始任务*/
	vTaskStartScheduler();	/*开启任v务调度*/

	while(1){};

}

/*创建任务*/
void startTask(void *pvParameters)
{
	taskENTER_CRITICAL();	/*进入临界区*/
	
							// 优先级最高，更新状态  数值越小，优先级越低
	xTaskCreate(Chassis_control, "Chassis_control", 500, NULL, 5, NULL);	//底盘控制
	xTaskCreate(Robot_FSM, "Robot_FSM", 500, NULL, 4, NULL);	// 状态机
	xTaskCreate(Get_mpu6050_data, "Get_mpu6050_data", 500, NULL, 2, NULL);	// 陀螺仪
	xTaskCreate(Key_Check, "Key_Check", 500, NULL, 1, NULL);	// 微动开关		
//	xTaskCreate(Get_distance, "Get_distance", 500, NULL, 3, NULL);	// 超声波测距
	
	
//	xTaskCreate(Datascope, "Datascope", 500, NULL, 2, NULL);	// 上位机
//	xTaskCreate(Voice_play, "Voice_play", 500, NULL, 4, NULL);	// 视觉		

	vTaskDelete(startTaskHandle);										/*删除开始任务*/
	taskEXIT_CRITICAL();	/*退出临界区*/
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
	
	uart_init(9600);  //初始化串口1--语音播报	
	Bsp_UART4_Init(9600);	//串口4--二维码识别
	Bsp_USART6_Init(115200);	//串口6--openmv
	
	Bsp_Motor_Init();
	Motor_PID_Init();
	Bsp_TFT_Init();
//	LCD_ShowString(14,23,"INIT SUCCESS!",GREEN,BLACK,16,0);
}

/*	问题日志：
	1.mpu6050初始化失败，在裸机系统可以，而在操作系统rtos不行		2021/10/09
		已解决，IIC文件忘记加了
	2.openmv 识别的有点慢
		原因（猜测）：一直发送0导致
		已解决，原因是初始化的时候延时了
	3.语音播报有时很慢，甚至不行
		已解决，原因是没有“预热”
	4.微动开关有时候关了又重新打开舵机
		未解决：但是好像后面没出现了
	5.不放入物料，语音播报会放2次
		不处理也行
	6.物料取出没法检测
		已解决，用代码逻辑解决了
	7.取出物料后，启动有点慢

*/

