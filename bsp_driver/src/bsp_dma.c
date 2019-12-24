#include "bsp_dma.h"
void DMA2_Channel3_NVIC_Init(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_InitStructure.NVIC_IRQChannel = DMA2_Channel3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0 ; //抢占优先级3
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		//子优先级3
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
    NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
}
	
void dma2_channel3_init(uint32_t PeripheralBaseAddr, uint32_t MemoryBaseAddr, uint32_t BufferSize, uint32_t DIR)
{
    DMA_InitTypeDef DMA_InitStructure;
    /*开启DMA时钟*/
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);

    /*设置DMA源：串口数据寄存器地址*/
    DMA_InitStructure.DMA_PeripheralBaseAddr = PeripheralBaseAddr;

    /*内存地址(要传输的变量的指针)*/
    DMA_InitStructure.DMA_MemoryBaseAddr = MemoryBaseAddr;

    /*方向：从内存到外设*/
    DMA_InitStructure.DMA_DIR = DIR;

    /*传输大小DMA_BufferSize=SENDBUFF_SIZE*/
    DMA_InitStructure.DMA_BufferSize = BufferSize;

    /*外设地址不增*/
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;

    /*内存地址自增*/
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;

    /*外设数据单位*/
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;

    /*内存数据单位 8bit*/
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;

    /*DMA模式：不断循环*/
//    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal ;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;

    /*优先级：中*/
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;

    /*禁止内存到内存的传输	*/
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

    /*配置DMA1的4通道*/
    DMA_Init(DMA2_Channel3, &DMA_InitStructure);

    /*使能DMA*/
    DMA_Cmd (DMA2_Channel3, ENABLE);

//    DMA_ITConfig(DMA2_Channel3, DMA_IT_TC, ENABLE); //配置DMA发送完成后产生中断
	
    DMA_ClearFlag(DMA2_FLAG_GL3);
//	DMA2_Channel3_NVIC_Init();
}
void DMA2_Channel5_NVIC_Init(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_InitStructure.NVIC_IRQChannel = DMA2_Channel4_5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0 ; //抢占优先级3
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		//子优先级3
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
    NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
}
void dma2_channel5_init(uint32_t PeripheralBaseAddr, uint32_t MemoryBaseAddr, uint32_t BufferSize, uint32_t DIR)
{
    DMA_InitTypeDef DMA_InitStructure;
    /*开启DMA时钟*/
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);

    /*设置DMA源：串口数据寄存器地址*/
    DMA_InitStructure.DMA_PeripheralBaseAddr = PeripheralBaseAddr;

    /*内存地址(要传输的变量的指针)*/
    DMA_InitStructure.DMA_MemoryBaseAddr = MemoryBaseAddr;

    /*方向：从内存到外设*/
    DMA_InitStructure.DMA_DIR = DIR;

    /*传输大小DMA_BufferSize=SENDBUFF_SIZE*/
    DMA_InitStructure.DMA_BufferSize = BufferSize;

    /*外设地址不增*/
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;

    /*内存地址自增*/
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;

    /*外设数据单位*/
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;

    /*内存数据单位 8bit*/
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;

    /*DMA模式：不断循环*/
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal ;
//	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;

    /*优先级：中*/
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;

    /*禁止内存到内存的传输	*/
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

    /*配置DMA1的4通道*/
    DMA_Init(DMA2_Channel5, &DMA_InitStructure);

    /*使能DMA*/
    DMA_Cmd (DMA2_Channel5, ENABLE);

    DMA_ITConfig(DMA2_Channel5, DMA_IT_TC, ENABLE); //配置DMA发送完成后产生中断

    DMA_ClearFlag(DMA2_FLAG_GL5);
    DMA2_Channel5_NVIC_Init();
//    GPIO_SetBits(GPIOC, GPIO_Pin_8);
}
void DMA2_Channel3_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA2_IT_TC3) == SET) {

//        DMA_ClearFlag(DMA2_IT_TC3);
		printf("dma recv ok");
    }
}

void DMA2_Channel4_5_IRQHandler(void)
{
    if(DMA_GetITStatus(DMA2_IT_TC5) == SET) {

        DMA_ClearFlag(DMA2_IT_TC5);
//		printf("dma ok");
    }
}

