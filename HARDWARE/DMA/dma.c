#include "dma.h"
#include "encoder.h"
#include "math.h"
/**********************************************************	 
*实现功能：
*			1. 初始化DMA1,使能DMA1 通道4 将内存中odem_data向串口1传输
*			2. 开启DMA1传输,传输完成后中断关闭DMA1传输
*			3. 整合odem数据
***********************************************************/
/***********************************************************
 * 串口接收数据格式共15字节
 * head head linear_v_x  linear_v_y angular_v  CRC
 * 0xff 0xff float       float      float      u8
 * 接收消息数据包例子：
 * FF FF 00 00 80 3f 00 00 00 00 00 00 00 40 ff
 * FF FF 00 00 80 3f 00 00 00 00 00 00 00 00 bf
 * ********************************************************/
/**********************************************************
 * 串口发送数据格式共27字节
 * head head x-position y-position x-speed y-speed angular-speed pose-angular CRC
 * 0xaa 0xaa float      float      float   float   float         float(yaw)   u8
 * ********************************************************/
float pitch,roll,yaw; 		//欧拉角
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
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);	//使能DMA传输
	
	DMA_DeInit(DMA_Tx_CHx);	//将DMA的发送通道寄存器重设为缺省值
	DMA_DeInit(DMA_Rx_CHx);//恢复缺省配置
	
	//DMA发送通道初始化
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART1->DR;  //DMA外设基地址
	DMA_InitStructure.DMA_MemoryBaseAddr = Tx_data_Base;  //DMA内存基地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;  //数据传输方向，从内存读取发送到外设
	DMA_InitStructure.DMA_BufferSize = USART1TXBuff_Size;  //DMA通道的DMA缓存的大小
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  //外设地址寄存器不变
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  //内存地址寄存器递增
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  //数据宽度为8位
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; //数据宽度为8位
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;  //工作在正常模式
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium; //DMA通道 x拥有中优先级 
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  //DMA通道x没有设置为内存到内存传输
	DMA_Init(DMA_Tx_CHx, &DMA_InitStructure);  //根据DMA_InitStruct中指定的参数初始化DMA的通道USART1_Tx_DMA_Channel所标识的寄存器
	
	//DMA接收通道初始化
	DMA_Cmd(DMA_Rx_CHx,DISABLE);//stop dma
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&USART1->DR);//设置串口接收数据寄存器
	DMA_InitStructure.DMA_MemoryBaseAddr = Rx_data_Base;//接收缓存地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;//设置外设为数据源
	DMA_InitStructure.DMA_BufferSize = USART1RXBuff_Size;//需要接收的字节数
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; //外设地址不做增加调整       
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  //内存缓冲区地址增加调整              
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //外设数据宽度 一个字节
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; //内存数据宽度一个字节        
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                           //单次传输模式
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh; //高优先级                
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  //关闭内存到内存的DMA模式 
	DMA_Init(DMA_Rx_CHx, &DMA_InitStructure);//写入配置	                         
	DMA_Cmd(DMA_Rx_CHx, ENABLE); //开启DMA接收通道
}
void DMA_USART1_NVIC_config(void)
{
	/* Enable the DMA Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel4_IRQn;  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;     
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);  //开启DMA发送通道中断
	
	/* Enable the DMA Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel5_IRQn;  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;     
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	DMA_ITConfig(DMA1_Channel5, DMA_IT_TC, ENABLE);  //开启DMA接收通道中断
	DMA_ClearFlag(DMA1_FLAG_TC5);    //清除DMA所有标志 
	
}

void DMA1_Channel4_IRQHandler(void)//中断向量定义在startup_stm32f10x.s文件中
{
    if(DMA_GetITStatus(DMA1_FLAG_TC4))
    {
        DMA_ClearFlag(DMA1_FLAG_TC4);    //清除DMA所有标志    
		DMA_Cmd(DMA1_Channel4, DISABLE);  //关闭DMA发送通道
    }
}
void DMA1_Channel5_IRQHandler(void)//中断向量定义在startup_stm32f10x.s文件中
{
	u8 res;
    if(DMA_GetITStatus(DMA1_IT_TC5) != RESET)
    {
		DMA_Cmd(DMA1_Channel5, DISABLE);    // 关闭 DMA1_Channel5 通道
		DMA_SetCurrDataCounter(DMA1_Channel5, USART1RXBuff_Size);   // 重新写入要传输的数据数量
		DMA_Cmd(DMA1_Channel5, ENABLE);	
		USART_DMACmd(USART1,USART_DMAReq_Rx,DISABLE); //使能串口1的DMA接收
		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启串口接受中断
        res = get_data_analyze(USART1RXBuff);
		if(res != 0)
		{
			//printf("Fail to analyze data，lost one cmd_vel pack!\n");
			memset(&USART1RXBuff,0,USART1RXBuff_Size);
		}
		DMA_ClearFlag(DMA1_FLAG_TC5);    //清除DMA所有标志
		DMA_ClearFlag(DMA1_FLAG_GL5);	//清除DMAGL标志
		DMA_ClearFlag(DMA1_FLAG_HT5);	//清除DMAHT标志
    }
}

void DMA_USART1_Enable(DMA_Channel_TypeDef* DMA_CHx)
{
	DMA_Cmd(DMA_CHx, DISABLE); //关闭USARTx TX DMAx 所指示的通道
	DMA_SetCurrDataCounter(DMA_CHx,odem_data_size);//DMA通道的DMA缓存的大小
 	DMA_Cmd(DMA_CHx, ENABLE);  //使能USART1 TX DMA1 所指示的通道 
}

void get_odom(void)
{
	float X_left = Encoder_Left * rate_encoder;	//左轮单位时间位移
	float X_right = Encoder_Right * rate_encoder; //右轮单位时间位移
	float inc_Wheel_x = (X_left + X_right) / 2;	//轮子x方向上的位移增量
	//float inc_Angular = (X_right - X_left) / (2 * distance_wheels);	//角度增量，单位为弧度值
	Vx_left = X_left / delta_t;
	Vx_right = X_right / delta_t;
	//sum_Angular += inc_Angular;
	//sum_Angular = fmod(sum_Angular,3.14159 * 2); 
	sum_Pose_x = sum_Pose_x + inc_Wheel_x * cos(yaw);
	sum_Pose_y = sum_Pose_y + inc_Wheel_x * sin(yaw);
}

void data_pack(void)
{
	odem_data.x_pos.fv = sum_Pose_x;//2.68;//x坐标
	odem_data.y_pos.fv = sum_Pose_y;//y坐标
	odem_data.x_v.fv = (Vx_left + Vx_right) / 2;	//x方向速度，归一化
	odem_data.y_v.fv = 0.0;//y 方向速度
	odem_data.angular_v.fv = (Vx_right - Vx_left) / (2 * distance_wheels);//角速度 绕z轴
	odem_data.pose_angular.fv = yaw;//yaw偏航角 
	
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
//数据接收分析
u8 get_data_analyze(uint8_t	*pdata)
{
	
	int8_t ret=0;
	int8_t	crc = 0;
	int8_t  p_crc = 0;
	if( *(pdata + 0) == 0xff ){
		crc = (*(pdata + 1))^(*(pdata + 2))^(*(pdata + 3))^(*(pdata + 4))^(*(pdata + 5))^(*(pdata + 6))^(*(pdata + 7))^(*(pdata + 8))^(*(pdata + 9))^(*(pdata + 10))^(*(pdata + 11))^(*(pdata + 12));
		p_crc = (int8_t)(*(pdata + 13));//不进行类型转换，负数不正常
	}
	else{
		ret = -1;
		return ret;
	}
	if(p_crc != crc ){//校验和分析有误
		ret = -1;
		return ret;
	}
	//数据包分析正确，提取数据
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
函数名称	：get_puls_analyze
功   能	：将控制指令解析为采样周期内脉冲数，送入PID控制器实现对线速度和角速度的控制
参   数	：cmd_vel_data -------	上位机传来的控制指令，线速度与角速度目标量，已归一化处理
          Max_Vx_puls --------	设定的当前电压下采样周期内最大脉冲数，代表最大线速度
          Max_Angular_delta---	左右电机单位时间脉冲数最大差值，代表最大角速度
返 回 值	：MotorVelocity motor-	左右电机单位时间脉冲数，根据式子 Vl = V + Omega * d, Vr = V - Omega * d计算得出
作   者	：pyp
*************************************************/
MotorVelocity get_puls_analyze(rcv_data cmd_vel_data, int Max_Vx_puls, int Max_Angular_delta)
{
	float base_v;
	float angular_v;
	MotorVelocity motor;
	motor.v_left = 0.0;
	motor.v_right = 0.0;
	
	base_v = cmd_vel_data.linear_vx.fv*delta_t/rate_encoder;//速度*采样周期/脉冲数距离比例=脉冲数 //cmd_vel_data.linear_vx.fv* Max_Vx_puls;
	angular_v = cmd_vel_data.angular_v.fv * Max_Angular_delta / 2;
	motor.v_left = base_v - angular_v;
	motor.v_right = base_v + angular_v;
	
	return motor;
}
	
