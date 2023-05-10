#include "encoder.h"

 /**************************************************************************
 ��  �� ���������
 �Ա���ַ��https://shop119207236.taobao.com
 
 ΢�Ź��ںš���������ˡ�
 ��̨�ظ���ƽ��С��������ȡƽ��С��ȫ��DIY����
 ��̨�ظ������ӿ������ߡ�����ȡ���ӹ���ʦ�ر���������
 ��̨�ظ�������������ϡ�����ȡ����������ϰ�
 
 ֪���������� 
**************************************************************************/
/**************************************************************************
�������ܣ���TIM2��ʼ��Ϊ�������ӿ�ģʽ
��ڲ�������
����  ֵ����
**************************************************************************/
int	Encoder_Left;
int Encoder_Right;
int PID_MOTOR_R = 0;
int PID_MOTOR_L = 0;
PID_LocTypeDef PID_Left;
PID_LocTypeDef* sPID_Left = &PID_Left;
PID_LocTypeDef PID_Right;
PID_LocTypeDef* sPID_Right = &PID_Right;

void Encoder_Init_TIM8(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;	
	TIM_ICInitTypeDef TIM_ICInitStructure;  
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);//ʹ�ܶ�ʱ��4��ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);//ʹ��PB�˿�ʱ��

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;	//�˿�����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //��������
	GPIO_Init(GPIOA, &GPIO_InitStructure);					      //�����趨������ʼ��GPIOB

	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Prescaler = 0x0; // Ԥ��Ƶ�� 
	TIM_TimeBaseStructure.TIM_Period = ENCODER_TIM_PERIOD; //�趨�������Զ���װֵ
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//ѡ��ʱ�ӷ�Ƶ������Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;////TIM���ϼ���  
	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);
	
	TIM_EncoderInterfaceConfig(TIM8, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);//ʹ�ñ�����ģʽ3
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_ICFilter = 10;
	TIM_ICInit(TIM8, &TIM_ICInitStructure);
	
	TIM_ClearFlag(TIM8, TIM_FLAG_Update);//���TIM�ĸ��±�־λ
	TIM_ITConfig(TIM8, TIM_IT_Update, ENABLE);
	//Reset counter
	TIM_SetCounter(TIM8,0);
	TIM_Cmd(TIM8, ENABLE); 
}
/**************************************************************************
�������ܣ���TIM4��ʼ��Ϊ�������ӿ�ģʽ
��ڲ�������
����  ֵ����
**************************************************************************/
void Encoder_Init_TIM4(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;  
	TIM_ICInitTypeDef TIM_ICInitStructure;  
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);//ʹ�ܶ�ʱ��4��ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);//ʹ��PB�˿�ʱ��

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;	//�˿�����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; //��������
	GPIO_Init(GPIOB, &GPIO_InitStructure);					      //�����趨������ʼ��GPIOB

	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Prescaler = 0x0; // Ԥ��Ƶ�� 
	TIM_TimeBaseStructure.TIM_Period = ENCODER_TIM_PERIOD; //�趨�������Զ���װֵ
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//ѡ��ʱ�ӷ�Ƶ������Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;////TIM���ϼ���  
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	
	TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);//ʹ�ñ�����ģʽ3
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_ICFilter = 10;
	TIM_ICInit(TIM4, &TIM_ICInitStructure);
	
	TIM_ClearFlag(TIM4, TIM_FLAG_Update);//���TIM�ĸ��±�־λ
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
	
	TIM_SetCounter(TIM4,0);
	TIM_Cmd(TIM4, ENABLE); 
}

/**************************************************************************
�������ܣ���λʱ���ȡ����������
��ڲ�������ʱ��
����  ֵ���ٶ�ֵ
**************************************************************************/
int Read_Encoder(u8 TIMX)
{
   int Encoder_TIM;    
   switch(TIMX)
	 {
		case 8:  Encoder_TIM= (short)TIM8 -> CNT;  TIM8 -> CNT=0;break;
		case 3:  Encoder_TIM= (short)TIM3 -> CNT;  TIM3 -> CNT=0;break;	
		case 4:  Encoder_TIM= (short)TIM4 -> CNT;  TIM4 -> CNT=0;break;	
		default: Encoder_TIM=0;
	 }
	return Encoder_TIM;
}

/************************************************
�������� �� PID_Loc
��    �� �� PIDλ��(Location)����
��    �� �� SetValue ------ ����ֵ(����ֵ)��Ϊ10ms����������������������Ϊ�ٷ�����ʽ����ֵ��0~100֮��
            ActualValue --- ʵ��ֵ(����ֵ)
            PID ----------- PID���ݽṹ
�� �� ֵ �� PIDLoc -------- PIDλ��
��    �� �� strongerHuang
*************************************************/
float PID_Loc(float SetValue, float ActualValue, PID_LocTypeDef* PID)
{
	float PIDLoc;                                  //λ��
	//ActualValue = (ActualValue/430)*100;		//����430ת������һ������ֵ��0~100
	ActualValue = ActualValue;		//������������ת��Ϊת�٣���λΪrpm
	//SetValue = (PID->Max_speed)*SetValue/100;		//Max_speedΪ��ǰ��������ѹ���ת�٣���ʽΪ���ٷ�����ʽ��Ϊת����ʽ����λΪrpm,�������SetValueΪ�ٷ�����ʽʱ���ñ���
	
	PID->Ek = SetValue - ActualValue;
	if(SetValue != PID->SetValue_last)
	{
		PID->SetValue_last = SetValue;
		PID->LocSum = 0;
	}
	else
	{
		PID->LocSum += PID->Ek;                         //�ۼ����
		if(PID->LocSum >= 5000) PID->LocSum = 5000;
		else if(PID->LocSum <= -5000) PID->LocSum = -5000;
	}

	PIDLoc = PID->Kp * PID->Ek + (PID->Ki * PID->LocSum) + PID->Kd * (PID->Ek1 - PID->Ek);
	PIDLoc = (PIDLoc / PID->Max_speed) * 100;

	PID->Ek1 = PID->Ek;  return PIDLoc;
}
void PID_Init(PID_LocTypeDef* PID,int max_speed,int max_wheel_diff)
{
	PID->Kp = 0.61;
	PID->Ki = 0.01;		//1.0818
	PID->Kd = 0.2;			//1.4192
	PID->Ek = 0;                    //��ǰ���
	PID->Ek1 = 0;                     //ǰһ����� e(k-1)
	PID->Ek2 = 0;                      //��ǰһ����� e(k-2)
	PID->LocSum = 0;                 //�ۼƻ���λ��
	PID->SetValue_last = 0;
	PID->Max_speed = max_speed;
	PID->Max_wheel_diff = max_wheel_diff;
}
/**************************************************************************
�������ܣ�TIM4�жϷ�����
��ڲ�������
����  ֵ����
**************************************************************************/
void TIM4_IRQHandler(void)
{ 		    		  			    
	if(TIM4->SR&0X0001)//����ж�
	{    				   				     	    	
	}				   
	TIM4->SR&=~(1<<0);//����жϱ�־λ 	    
}

/**************************************************************************
�������ܣ�TIM2�жϷ�����
��ڲ�������
����  ֵ����
**************************************************************************/
void TIM8_IRQHandler(void)
{ 		    		  			    
	if(TIM8->SR&0X0001)//����ж�
	{    				   				     	    	
	}				   
	TIM8->SR&=~(1<<0);//����жϱ�־λ 	    
}

