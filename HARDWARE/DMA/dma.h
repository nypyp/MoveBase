#ifndef __DMA_H
#define	__DMA_H	   
#include "sys.h"
#include "stdio.h"
#include "string.h"

#define odem_data_size 27
#define distance_wheels 0.1	//���ּ���һ��
#define radius_wheel 0.0242		//���Ӱ뾶
#define puls_round 3131			//��������תһȦ������
#define rate_encoder 0.0000425892 //(radius_wheel * 2 * 3.14159 / puls_round) ����puls_roundδ֪��ͨ����תС��һ�ܣ������������������ֱֵ�Ӽ���õ�	
#define USART1TXBuff_Size 64
#define USART1RXBuff_Size 14


//��������HEX���ٻ�ȡ
typedef	union{
		float fv;
		uint8_t cv[4];
}float_union;

//�������ݽṹ
typedef	struct{

		float_union		linear_vx;//���ٶ�x
		float_union		linear_vy;//���ٶ�y
		float_union		angular_v;//���ٶ�
		
}rcv_data;

//�������ݽṹ
typedef	struct{
		
		float_union	x_pos;//x��������
		float_union	y_pos;//y��������
		float_union	x_v;//x�����ٶ�
		float_union	y_v;//y�����ٶ�
		float_union	angular_v;//���ٶ�
		float_union	pose_angular;//�Ƕ�
	
}send_data;

//���ҵ���ٶȽṹ
typedef struct{
	
	float v_left;	//�����ٶ�
	float v_right;	//�ҵ���ٶ�
}MotorVelocity;

extern u8 USART1TXBuff[USART1TXBuff_Size];
extern u8 USART1RXBuff[USART1RXBuff_Size];
extern send_data odem_data;
extern rcv_data cmd_vel_data;
extern u16 DATAPACK_FREQ;
extern float pitch,roll,yaw; 		//ŷ����
extern u16 RESUALT_FLAG;

void DMA_USART1_Init(DMA_Channel_TypeDef* DMA_Rx_CHx,DMA_Channel_TypeDef* DMA_Tx_CHx,u32 Tx_data_Base,u32 Rx_data_Base);//����DMA1_CHx
void DMA_USART1_NVIC_config(void);
void DMA_USART1_Enable(DMA_Channel_TypeDef*DMA_CHx);//ʹ��DMA1_CHx
void DMA1_Channel4_IRQHandler(void);
void get_odom(void);
void data_pack(void);
u8 get_data_analyze(uint8_t	*pdata);
MotorVelocity get_puls_analyze(rcv_data cmd_vel_data, int Max_Vx_puls, int Max_Angular_delta);
		   
#endif
