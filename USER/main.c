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
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置中断优先级分组为组2：2位抢占优先级，2位响应优先级
	uart_init(115200);	//串口初始化
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
	PID_Init(sPID_Left,50,40);//左电机max脉冲：85
	PID_Init(sPID_Right,50,40);//右电机max脉冲：83
	Encoder_Init_TIM2();
	Encoder_Init_TIM4();
	//printf("encoder init success");
//	OLED_Init();
	TIM1_Int_Init(99,7199);
//	IWDG_Init(4,63);
	//printf("初始化完成...");
	while(1)
	{
		LED0 = 0;
		RESUALT_FLAG = mpu_dmp_get_data(&pitch,&roll,&yaw);
		LED0 = 1;
//		My_OLED_Display();
	}
}   
