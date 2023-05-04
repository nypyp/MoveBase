#include "motor.h"
#include "delay.h"

void Motor_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); //ʹ�ܶ˿�ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); //ʹ�ܶ˿�ʱ��
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5;	//�˿�����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;      //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     //50M
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5;	//�˿�����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;      //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     //50M
	GPIO_Init(GPIOC, &GPIO_InitStructure);
}

void MOTOR_PWM_Init(u16 arr, u16 psc)
{
	//��ʼ���ṹ��
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	//ʹ��ʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA  | RCC_APB2Periph_AFIO, ENABLE);
	
	//����GPIO��ʼ���ṹ��
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7; //TIM3_CH1|TIM3_CH2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //�����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIO
	
	//��ʼ��TIM3
	TIM_TimeBaseStructure.TIM_Period = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
	
	//��ʼ��TIM3 Channel2 PWMģʽ	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //�������:TIM����Ƚϼ��Ը�
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);  //��ʼ��TIM3_OC1
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM3 OC2
	
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);  //ʹ��TIM3��CCR2�ϵ�Ԥװ�ؼĴ���
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);

	TIM_Cmd(TIM3, ENABLE);  //ʹ��TIM3
}

void Set_Pwm(int motor1, int motor2)
{
	if(motor1>0 && motor1>100) motor1 = 100;//�޷�
	if(motor1<0 && motor1<-100) motor1 = -100;
	if(motor1>0) AIN1 = 0, AIN2 = 1;
	else         AIN1 = 1, AIN2 = 0;
	PWMA = myabs(motor1)*69;
	if(motor2>0 && motor2>100) motor2 = 100;//�޷�
	if(motor2<0 && motor2<-100) motor2 = -100;
	if(motor2>0) BIN1 = 0, BIN2 = 1;
	else         BIN1 = 1, BIN2 = 0;
	PWMB = myabs(motor2)*69; //motor ��ΧΪ[0,100], ���ϡ�69���޷����Ӻ�PWM��ΧΪ[0,6900]
}

int myabs(int num)
{
	int temp;
	if(num<0) temp = -num;
	else temp = num;
	return temp;
}

//void Turn_Left(void)
//{
//	Set_Pwm(0,50);
//	delay_ms(500);
//	Set_Pwm(0,0);
//}

//void Turn_Right(void)
//{
//	Set_Pwm(50,0);
//	delay_ms(500);
//	Set_Pwm(0,0);
//}

//u16 MATLAB_FLAG = 0;
//u16 MATLAB_SET = 0;

//void Matlab_Draw(void)
//{
//	if(MATLAB_FLAG > 180)
//	{
//		MATLAB_FLAG = 0;
//	}else if(MATLAB_FLAG >= 120)
//	{
//		MATLAB_SET = 60;
//	}else if(MATLAB_FLAG >=60)
//	{
//		MATLAB_SET = 50;
//	}else
//	{
//		MATLAB_SET = 50;
//	}
//}
