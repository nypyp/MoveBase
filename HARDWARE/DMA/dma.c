#include "dma.h"
#include "encoder.h"
#include "math.h"
/**********************************************************	 
*ʵ�ֹ��ܣ�
*			1. ��ʼ��DMA1,ʹ��DMA1 ͨ��4 ���ڴ���odem_data�򴮿�1����
*			2. ����DMA1����,������ɺ��жϹر�DMA1����
*			3. ����odem����
***********************************************************/
/***********************************************************
 * ���ڽ������ݸ�ʽ��15�ֽ�
 * head head linear_v_x  linear_v_y angular_v  CRC
 * 0xff 0xff float       float      float      u8
 * ������Ϣ���ݰ����ӣ�
 * FF FF 00 00 80 3f 00 00 00 00 00 00 00 40 ff
 * FF FF 00 00 80 3f 00 00 00 00 00 00 00 00 bf
 * ********************************************************/
/**********************************************************
 * ���ڷ������ݸ�ʽ��27�ֽ�
 * head head x-position y-position x-speed y-speed angular-speed pose-angular CRC
 * 0xaa 0xaa float      float      float   float   float         float(yaw)   u8
 * ********************************************************/
float pitch,roll,yaw; 		//ŷ����
send_data odem_data;
float Vx_left = 0;
float Vx_right = 0;
float sum_Angular = 0;
float sum_Pose_x = 0;
float sum_Pose_y = 0;
float delta_t = 0.01;
u16 RESUALT_FLAG = -1;
u16 DATAPACK_FREQ = 0;
rcv_data cmd_vel_data;

u8 USART1TXBuff[USART1TXBuff_Size];
u8 USART1RXBuff[USART1RXBuff_Size];

DMA_InitTypeDef DMA_InitStructure;
NVIC_InitTypeDef NVIC_InitStructure;

void DMA_USART1_Init(DMA_Channel_TypeDef* DMA_Rx_CHx,DMA_Channel_TypeDef* DMA_Tx_CHx,u32 Tx_data_Base,u32 Rx_data_Base)
{
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);	//ʹ��DMA����
	
	DMA_DeInit(DMA_Tx_CHx);	//��DMA�ķ���ͨ���Ĵ�������Ϊȱʡֵ
	DMA_DeInit(DMA_Rx_CHx);//�ָ�ȱʡ����
	
	//DMA����ͨ����ʼ��
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART1->DR;  //DMA�������ַ
	DMA_InitStructure.DMA_MemoryBaseAddr = Tx_data_Base;  //DMA�ڴ����ַ
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;  //���ݴ��䷽�򣬴��ڴ��ȡ���͵�����
	DMA_InitStructure.DMA_BufferSize = USART1TXBuff_Size;  //DMAͨ����DMA����Ĵ�С
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  //�����ַ�Ĵ�������
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  //�ڴ��ַ�Ĵ�������
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  //���ݿ��Ϊ8λ
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; //���ݿ��Ϊ8λ
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;  //����������ģʽ
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium; //DMAͨ�� xӵ�������ȼ� 
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  //DMAͨ��xû������Ϊ�ڴ浽�ڴ洫��
	DMA_Init(DMA_Tx_CHx, &DMA_InitStructure);  //����DMA_InitStruct��ָ���Ĳ�����ʼ��DMA��ͨ��USART1_Tx_DMA_Channel����ʶ�ļĴ���
	
	//DMA����ͨ����ʼ��
	DMA_Cmd(DMA_Rx_CHx,DISABLE);//stop dma
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&USART1->DR);//���ô��ڽ������ݼĴ���
	DMA_InitStructure.DMA_MemoryBaseAddr = Rx_data_Base;//���ջ����ַ
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;//��������Ϊ����Դ
	DMA_InitStructure.DMA_BufferSize = USART1RXBuff_Size;//��Ҫ���յ��ֽ���
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; //�����ַ�������ӵ���       
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  //�ڴ滺������ַ���ӵ���              
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //�������ݿ�� һ���ֽ�
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; //�ڴ����ݿ��һ���ֽ�        
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                           //���δ���ģʽ
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh; //�����ȼ�                
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  //�ر��ڴ浽�ڴ��DMAģʽ 
	DMA_Init(DMA_Rx_CHx, &DMA_InitStructure);//д������	                         
	DMA_Cmd(DMA_Rx_CHx, ENABLE); //����DMA����ͨ��
}
void DMA_USART1_NVIC_config(void)
{
	/* Enable the DMA Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel4_IRQn;  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;     
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);  //����DMA����ͨ���ж�
	
	/* Enable the DMA Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel5_IRQn;  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;     
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	DMA_ITConfig(DMA1_Channel5, DMA_IT_TC, ENABLE);  //����DMA����ͨ���ж�
	DMA_ClearFlag(DMA1_FLAG_TC5);    //���DMA���б�־ 
	
}

void DMA1_Channel4_IRQHandler(void)//�ж�����������startup_stm32f10x.s�ļ���
{
    if(DMA_GetITStatus(DMA1_FLAG_TC4))
    {
        DMA_ClearFlag(DMA1_FLAG_TC4);    //���DMA���б�־    
		DMA_Cmd(DMA1_Channel4, DISABLE);  //�ر�DMA����ͨ��
    }
}
void DMA1_Channel5_IRQHandler(void)//�ж�����������startup_stm32f10x.s�ļ���
{
	u8 res;
    if(DMA_GetITStatus(DMA1_IT_TC5) != RESET)
    {
		DMA_Cmd(DMA1_Channel5, DISABLE);    // �ر� DMA1_Channel5 ͨ��
		DMA_SetCurrDataCounter(DMA1_Channel5, USART1RXBuff_Size);   // ����д��Ҫ�������������
		DMA_Cmd(DMA1_Channel5, ENABLE);	
		USART_DMACmd(USART1,USART_DMAReq_Rx,DISABLE); //ʹ�ܴ���1��DMA����
		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//�������ڽ����ж�
        res = get_data_analyze(USART1RXBuff);
		if(res != 0)
		{
			//printf("Fail to analyze data��lost one cmd_vel pack!\n");
			memset(&USART1RXBuff,0,USART1RXBuff_Size);
		}
		DMA_ClearFlag(DMA1_FLAG_TC5);    //���DMA���б�־
		DMA_ClearFlag(DMA1_FLAG_GL5);	//���DMAGL��־
		DMA_ClearFlag(DMA1_FLAG_HT5);	//���DMAHT��־
    }
}

void DMA_USART1_Enable(DMA_Channel_TypeDef* DMA_CHx)
{
	DMA_Cmd(DMA_CHx, DISABLE); //�ر�USARTx TX DMAx ��ָʾ��ͨ��
	DMA_SetCurrDataCounter(DMA_CHx,odem_data_size);//DMAͨ����DMA����Ĵ�С
 	DMA_Cmd(DMA_CHx, ENABLE);  //ʹ��USART1 TX DMA1 ��ָʾ��ͨ�� 
}

void get_odom(void)
{
	float X_left = Encoder_Left * rate_encoder;	//���ֵ�λʱ��λ��
	float X_right = Encoder_Right * rate_encoder; //���ֵ�λʱ��λ��
	float inc_Wheel_x = (X_left + X_right) / 2;	//����x�����ϵ�λ������
	//float inc_Angular = (X_right - X_left) / (2 * distance_wheels);	//�Ƕ���������λΪ����ֵ
	Vx_left = X_left / delta_t;
	Vx_right = X_right / delta_t;
	//sum_Angular += inc_Angular;
	//sum_Angular = fmod(sum_Angular,3.14159 * 2); 
	sum_Pose_x = sum_Pose_x + inc_Wheel_x * cos(yaw);
	sum_Pose_y = sum_Pose_y + inc_Wheel_x * sin(yaw);
}

void data_pack(void)
{
	odem_data.x_pos.fv = sum_Pose_x;//2.68;//x����
	odem_data.y_pos.fv = sum_Pose_y;//y����
	odem_data.x_v.fv = (Vx_left + Vx_right) / 2;	//x�����ٶȣ���һ��
	odem_data.y_v.fv = 0.0;//y �����ٶ�
	odem_data.angular_v.fv = (Vx_right - Vx_left) / (2 * distance_wheels);//���ٶ� ��z��
	odem_data.pose_angular.fv = yaw;//yawƫ���� 
	
	USART1TXBuff[0] = 0xaa;
	USART1TXBuff[1] = 0xaa;
	
	USART1TXBuff[2] = odem_data.x_pos.cv[0];
	USART1TXBuff[3] = odem_data.x_pos.cv[1];
	USART1TXBuff[4] = odem_data.x_pos.cv[2];
	USART1TXBuff[5] = odem_data.x_pos.cv[3];
	
	USART1TXBuff[6] = odem_data.y_pos.cv[0];
	USART1TXBuff[7] = odem_data.y_pos.cv[1];
	USART1TXBuff[8] = odem_data.y_pos.cv[2];
	USART1TXBuff[9] = odem_data.y_pos.cv[3];
	
	USART1TXBuff[10] = odem_data.x_v.cv[0];
	USART1TXBuff[11] = odem_data.x_v.cv[1];
	USART1TXBuff[12] = odem_data.x_v.cv[2];
	USART1TXBuff[13] = odem_data.x_v.cv[3];
	
	USART1TXBuff[14] = odem_data.y_v.cv[0];
	USART1TXBuff[15] = odem_data.y_v.cv[1];
	USART1TXBuff[16] = odem_data.y_v.cv[2];
	USART1TXBuff[17] = odem_data.y_v.cv[3];
	
	USART1TXBuff[18] = odem_data.angular_v.cv[0];
	USART1TXBuff[19] = odem_data.angular_v.cv[1];
	USART1TXBuff[20] = odem_data.angular_v.cv[2];
	USART1TXBuff[21] = odem_data.angular_v.cv[3];
	
	USART1TXBuff[22] = odem_data.pose_angular.cv[0];
	USART1TXBuff[23] = odem_data.pose_angular.cv[1];
	USART1TXBuff[24] = odem_data.pose_angular.cv[2];
	USART1TXBuff[25] = odem_data.pose_angular.cv[3];
	
	USART1TXBuff[26] = USART1TXBuff[2]^USART1TXBuff[3]^USART1TXBuff[4]^USART1TXBuff[5]^USART1TXBuff[6]^
												USART1TXBuff[7]^USART1TXBuff[8]^USART1TXBuff[9]^USART1TXBuff[10]^USART1TXBuff[11]^
												USART1TXBuff[12]^USART1TXBuff[13]^USART1TXBuff[14]^USART1TXBuff[15]^USART1TXBuff[16]^
												USART1TXBuff[17]^USART1TXBuff[18]^USART1TXBuff[19]^USART1TXBuff[20]^USART1TXBuff[21]^
												USART1TXBuff[22]^USART1TXBuff[23]^USART1TXBuff[24]^USART1TXBuff[25];
	
	DMA_USART1_Enable(DMA1_Channel4);
}
//���ݽ��շ���
u8 get_data_analyze(uint8_t	*pdata)
{
	
	int8_t ret=0;
	int8_t	crc = 0;
	int8_t  p_crc = 0;
	if( *(pdata + 0) == 0xff ){
		crc = (*(pdata + 1))^(*(pdata + 2))^(*(pdata + 3))^(*(pdata + 4))^(*(pdata + 5))^(*(pdata + 6))^(*(pdata + 7))^(*(pdata + 8))^(*(pdata + 9))^(*(pdata + 10))^(*(pdata + 11))^(*(pdata + 12));
		p_crc = (int8_t)(*(pdata + 13));//����������ת��������������
	}
	else{
		ret = -1;
		return ret;
	}
	if(p_crc != crc ){//У��ͷ�������
		ret = -1;
		return ret;
	}
	//���ݰ�������ȷ����ȡ����
	memset(&cmd_vel_data,0,sizeof(cmd_vel_data));
	cmd_vel_data.linear_vx.cv[0] = *(pdata + 1);
	cmd_vel_data.linear_vx.cv[1] = *(pdata + 2);
	cmd_vel_data.linear_vx.cv[2] = *(pdata + 3);
	cmd_vel_data.linear_vx.cv[3] = *(pdata + 4);
	
	cmd_vel_data.linear_vy.cv[0] = *(pdata + 5);
	cmd_vel_data.linear_vy.cv[1] = *(pdata + 6);
	cmd_vel_data.linear_vy.cv[2] = *(pdata + 7);
	cmd_vel_data.linear_vy.cv[3] = *(pdata + 8);
	
	cmd_vel_data.angular_v.cv[0] = *(pdata + 9);
	cmd_vel_data.angular_v.cv[1] = *(pdata + 10);
	cmd_vel_data.angular_v.cv[2] = *(pdata + 11);
	cmd_vel_data.angular_v.cv[3] = *(pdata + 12);
	
	return ret;
	
}
/************************************************
��������	��get_puls_analyze
��   ��	��������ָ�����Ϊ����������������������PID������ʵ�ֶ����ٶȺͽ��ٶȵĿ���
��   ��	��cmd_vel_data -------	��λ�������Ŀ���ָ����ٶ�����ٶ�Ŀ�������ѹ�һ������
          Max_Vx_puls --------	�趨�ĵ�ǰ��ѹ�²������������������������������ٶ�
          Max_Angular_delta---	���ҵ����λʱ������������ֵ�����������ٶ�
�� �� ֵ	��MotorVelocity motor-	���ҵ����λʱ��������������ʽ�� Vl = V + Omega * d, Vr = V - Omega * d����ó�
��   ��	��pyp
*************************************************/
MotorVelocity get_puls_analyze(rcv_data cmd_vel_data, int Max_Vx_puls, int Max_Angular_delta)
{
	float base_v;
	float angular_v;
	MotorVelocity motor;
	motor.v_left = 0.0;
	motor.v_right = 0.0;
	
	base_v = cmd_vel_data.linear_vx.fv*delta_t/rate_encoder;//�ٶ�*��������/�������������=������ //cmd_vel_data.linear_vx.fv* Max_Vx_puls;
	angular_v = cmd_vel_data.angular_v.fv * Max_Angular_delta / 2;
	motor.v_left = base_v - angular_v;
	motor.v_right = base_v + angular_v;
	
	return motor;
}
	
