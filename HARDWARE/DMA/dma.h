#ifndef __DMA_H
#define	__DMA_H	   
#include "sys.h"
#include "stdio.h"
#include "string.h"

#define odem_data_size 27
#define distance_wheels 0.1	//两轮间距的一半
#define radius_wheel 0.0242		//轮子半径
#define puls_round 3131			//编码器旋转一圈脉冲数
#define rate_encoder 0.0000425892 //(radius_wheel * 2 * 3.14159 / puls_round) 由于puls_round未知，通过旋转小车一周，测得左右轮脉冲数差值直接计算得到	
#define USART1TXBuff_Size 64
#define USART1RXBuff_Size 14


//浮点数与HEX快速获取
typedef	union{
		float fv;
		uint8_t cv[4];
}float_union;

//接收数据结构
typedef	struct{

		float_union		linear_vx;//线速度x
		float_union		linear_vy;//线速度y
		float_union		angular_v;//角速度
		
}rcv_data;

//发送数据结构
typedef	struct{
		
		float_union	x_pos;//x方向坐标
		float_union	y_pos;//y方向坐标
		float_union	x_v;//x方向速度
		float_union	y_v;//y方向速度
		float_union	angular_v;//角速度
		float_union	pose_angular;//角度
	
}send_data;

//左右电机速度结构
typedef struct{
	
	float v_left;	//左电机速度
	float v_right;	//右电机速度
}MotorVelocity;

extern u8 USART1TXBuff[USART1TXBuff_Size];
extern u8 USART1RXBuff[USART1RXBuff_Size];
extern send_data odem_data;
extern rcv_data cmd_vel_data;
extern u16 DATAPACK_FREQ;
extern float pitch,roll,yaw; 		//欧拉角
extern u16 RESUALT_FLAG;

void DMA_USART1_Init(DMA_Channel_TypeDef* DMA_Rx_CHx,DMA_Channel_TypeDef* DMA_Tx_CHx,u32 Tx_data_Base,u32 Rx_data_Base);//配置DMA1_CHx
void DMA_USART1_NVIC_config(void);
void DMA_USART1_Enable(DMA_Channel_TypeDef*DMA_CHx);//使能DMA1_CHx
void DMA1_Channel4_IRQHandler(void);
void get_odom(void);
void data_pack(void);
u8 get_data_analyze(uint8_t	*pdata);
MotorVelocity get_puls_analyze(rcv_data cmd_vel_data, int Max_Vx_puls, int Max_Angular_delta);
		   
#endif
