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

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE); //ʱ��ʹ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);  //ʹ��GPIOAʱ��
	
//	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_8;  //PA0 ���֮ǰ����  
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; //PA0 ����  
//	GPIO_Init(GPIOA, &GPIO_InitStructure);
//	GPIO_ResetBits(GPIOA,GPIO_Pin_8);
	
	TIM_DeInit(TIM1);

	TIM_TimeBaseStructure.TIM_Period = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	 ������5000Ϊ500ms
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ  10Khz�ļ���Ƶ��  
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0x00;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
	
	TIM_ClearFlag(TIM1,TIM_FLAG_Update);
	TIM_ITConfig(TIM1, TIM_IT_Update,ENABLE);//
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;			//ʹ�ܰ���WK_UP���ڵ��ⲿ�ж�ͨ��
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x03;	//��ռ���ȼ�1�� 
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;					//�����ȼ�4
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//ʹ���ⲿ�ж�ͨ��
  	NVIC_Init(&NVIC_InitStructure); 
	
	TIM_Cmd(TIM1, ENABLE);  //ʹ��TIMx����
							 
}
void TIM1_UP_IRQHandler(void)   //TIM3�ж�
{
	//char encoder2str[16] = {0};
	if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET) //���ָ����TIM�жϷ������:TIM �ж�Դ 
		{
			MotorVelocity PID_SetValue;
			TIM_ClearITPendingBit(TIM1, TIM_IT_Update);  //���TIMx���жϴ�����λ:TIM �ж�Դ.
//			DISPLAY_FLAG++;
			DATAPACK_FREQ ++;
			Encoder_Left = - Read_Encoder(4);	//��תʱ����������Ϊ������תʱΪ��������ȡ�෴��
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
