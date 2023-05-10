#include "encoder.h"

 /**************************************************************************
 作  者 ：大鱼电子
 淘宝地址：https://shop119207236.taobao.com
 
 微信公众号【大鱼机器人】
 后台回复【平衡小车】：获取平衡小车全套DIY资料
 后台回复【电子开发工具】：获取电子工程师必备开发工具
 后台回复【电子设计资料】：获取电子设计资料包
 
 知乎：张巧龙 
**************************************************************************/
/**************************************************************************
函数功能：把TIM2初始化为编码器接口模式
入口参数：无
返回  值：无
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
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);//使能定时器4的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);//使能PB端口时钟

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;	//端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //浮空输入
	GPIO_Init(GPIOA, &GPIO_InitStructure);					      //根据设定参数初始化GPIOB

	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Prescaler = 0x0; // 预分频器 
	TIM_TimeBaseStructure.TIM_Period = ENCODER_TIM_PERIOD; //设定计数器自动重装值
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//选择时钟分频：不分频
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;////TIM向上计数  
	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);
	
	TIM_EncoderInterfaceConfig(TIM8, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);//使用编码器模式3
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_ICFilter = 10;
	TIM_ICInit(TIM8, &TIM_ICInitStructure);
	
	TIM_ClearFlag(TIM8, TIM_FLAG_Update);//清除TIM的更新标志位
	TIM_ITConfig(TIM8, TIM_IT_Update, ENABLE);
	//Reset counter
	TIM_SetCounter(TIM8,0);
	TIM_Cmd(TIM8, ENABLE); 
}
/**************************************************************************
函数功能：把TIM4初始化为编码器接口模式
入口参数：无
返回  值：无
**************************************************************************/
void Encoder_Init_TIM4(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;  
	TIM_ICInitTypeDef TIM_ICInitStructure;  
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);//使能定时器4的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);//使能PB端口时钟

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;	//端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; //浮空输入
	GPIO_Init(GPIOB, &GPIO_InitStructure);					      //根据设定参数初始化GPIOB

	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Prescaler = 0x0; // 预分频器 
	TIM_TimeBaseStructure.TIM_Period = ENCODER_TIM_PERIOD; //设定计数器自动重装值
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//选择时钟分频：不分频
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;////TIM向上计数  
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	
	TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);//使用编码器模式3
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_ICFilter = 10;
	TIM_ICInit(TIM4, &TIM_ICInitStructure);
	
	TIM_ClearFlag(TIM4, TIM_FLAG_Update);//清除TIM的更新标志位
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
	
	TIM_SetCounter(TIM4,0);
	TIM_Cmd(TIM4, ENABLE); 
}

/**************************************************************************
函数功能：单位时间读取编码器计数
入口参数：定时器
返回  值：速度值
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
函数名称 ： PID_Loc
功    能 ： PID位置(Location)计算
参    数 ： SetValue ------ 设置值(期望值)，为10ms采样周期内期望脉冲数，为百分数形式，数值在0~100之间
            ActualValue --- 实际值(反馈值)
            PID ----------- PID数据结构
返 回 值 ： PIDLoc -------- PID位置
作    者 ： strongerHuang
*************************************************/
float PID_Loc(float SetValue, float ActualValue, PID_LocTypeDef* PID)
{
	float PIDLoc;                                  //位置
	//ActualValue = (ActualValue/430)*100;		//满速430转，做归一化处理，值在0~100
	ActualValue = ActualValue;		//将编码器计数转化为转速，单位为rpm
	//SetValue = (PID->Max_speed)*SetValue/100;		//Max_speed为当前电机供电电压最大转速，本式为将百分数形式化为转速形式，单位为rpm,当输入的SetValue为百分数形式时启用本句
	
	PID->Ek = SetValue - ActualValue;
	if(SetValue != PID->SetValue_last)
	{
		PID->SetValue_last = SetValue;
		PID->LocSum = 0;
	}
	else
	{
		PID->LocSum += PID->Ek;                         //累计误差
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
	PID->Ek = 0;                    //当前误差
	PID->Ek1 = 0;                     //前一次误差 e(k-1)
	PID->Ek2 = 0;                      //再前一次误差 e(k-2)
	PID->LocSum = 0;                 //累计积分位置
	PID->SetValue_last = 0;
	PID->Max_speed = max_speed;
	PID->Max_wheel_diff = max_wheel_diff;
}
/**************************************************************************
函数功能：TIM4中断服务函数
入口参数：无
返回  值：无
**************************************************************************/
void TIM4_IRQHandler(void)
{ 		    		  			    
	if(TIM4->SR&0X0001)//溢出中断
	{    				   				     	    	
	}				   
	TIM4->SR&=~(1<<0);//清除中断标志位 	    
}

/**************************************************************************
函数功能：TIM2中断服务函数
入口参数：无
返回  值：无
**************************************************************************/
void TIM8_IRQHandler(void)
{ 		    		  			    
	if(TIM8->SR&0X0001)//溢出中断
	{    				   				     	    	
	}				   
	TIM8->SR&=~(1<<0);//清除中断标志位 	    
}

