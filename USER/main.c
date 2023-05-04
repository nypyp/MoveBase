#include "led.h"
#include "delay.h"
#include "sys.h"
#include "usart.h"
#include "dma.h"
#include "motor.h"
#include "encoder.h"
#include "timer.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
//#include "oled.h"
//#include "wtdg.h"

int main(void)
{
	delay_init();
	LED_Init();
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//�����ж����ȼ�����Ϊ��2��2λ��ռ���ȼ���2λ��Ӧ���ȼ�
	uart_init(115200);	//���ڳ�ʼ��
	DMA_USART1_Init(DMA1_Channel5,DMA1_Channel4,(u32)&USART1TXBuff,(u32)&USART1RXBuff);
	DMA_USART1_NVIC_config();
	MOTOR_PWM_Init(7199,0);
	Motor_Init();
	MPU_Init();
	while(mpu_dmp_init())
	{
		delay_ms(300);
		LED0 = 0;
		delay_ms(100);
		LED0 = 1;
		delay_ms(100);
		LED0 = 0;
		delay_ms(100);
		LED0 = 1;
	}
	PID_Init(sPID_Left,50,40);//����max���壺85
	PID_Init(sPID_Right,50,40);//�ҵ��max���壺83
	Encoder_Init_TIM2();
	Encoder_Init_TIM4();
	//printf("encoder init success");
//	OLED_Init();
	TIM1_Int_Init(99,7199);
//	IWDG_Init(4,63);
	//printf("��ʼ�����...");
	while(1)
	{
		LED0 = 0;
		RESUALT_FLAG = mpu_dmp_get_data(&pitch,&roll,&yaw);
		LED0 = 1;
//		My_OLED_Display();
	}
}   
