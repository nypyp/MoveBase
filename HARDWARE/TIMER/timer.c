#include "timer.h"
#include "encoder.h"
//#include "oled.h"
#include "led.h"
#include "usart.h"
#include "motor.h"
//#include "wtdg.h"
#include "stdlib.h"
#include "string.h"
#include "dma.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"


void TIM1_Int_Init(u16 arr,u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE); //时钟使能
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);  //使能GPIOA时钟
	
//	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_8;  //PA0 清除之前设置  
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; //PA0 输入  
//	GPIO_Init(GPIOA, &GPIO_InitStructure);
//	GPIO_ResetBits(GPIOA,GPIO_Pin_8);
	
	TIM_DeInit(TIM1);

	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	 计数到5000为500ms
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值  10Khz的计数频率  
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0x00;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
	
	TIM_ClearFlag(TIM1,TIM_FLAG_Update);
	TIM_ITConfig(TIM1, TIM_IT_Update,ENABLE);//
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;			//使能按键WK_UP所在的外部中断通道
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x03;	//抢占优先级1， 
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;					//子优先级4
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//使能外部中断通道
  	NVIC_Init(&NVIC_InitStructure); 
	
	TIM_Cmd(TIM1, ENABLE);  //使能TIMx外设
							 
}
void TIM1_UP_IRQHandler(void)   //TIM3中断
{
	//char encoder2str[16] = {0};
	if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET) //检查指定的TIM中断发生与否:TIM 中断源 
		{
			MotorVelocity PID_SetValue;
			TIM_ClearITPendingBit(TIM1, TIM_IT_Update);  //清除TIMx的中断待处理位:TIM 中断源.
//			DISPLAY_FLAG++;
			DATAPACK_FREQ ++;
			Encoder_Left = - Read_Encoder(4);	//正转时编码器读数为负，反转时为正，所以取相反数
			Encoder_Right=  Read_Encoder(8);
//			Lpuls_sum = Lpuls_sum - Encoder_Left;
//			Rpuls_sum = Rpuls_sum + Encoder_Right;
			PID_SetValue = get_puls_analyze(cmd_vel_data,sPID_Left->Max_speed,sPID_Left->Max_wheel_diff);
			PID_MOTOR_L=PID_Loc(PID_SetValue.v_left,Encoder_Left,sPID_Left);
			PID_MOTOR_R=PID_Loc(PID_SetValue.v_right,Encoder_Right,sPID_Right);
			//PID_MOTOR_L=PID_Loc(-30,Encoder_Left,sPID_Left);
			//PID_MOTOR_R=PID_Loc(-30,Encoder_Right,sPID_Right);
			Set_Pwm(PID_MOTOR_L,PID_MOTOR_R);
			//Set_Pwm(30,0);
			get_odom();
			if(DATAPACK_FREQ >= 10)
			{
				data_pack();
				DATAPACK_FREQ = 0;
			}
//			IWDG_Feed();
		}
}
