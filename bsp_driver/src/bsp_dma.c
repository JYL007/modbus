#include "bsp_dma.h"
void DMA2_Channel3_NVIC_Init(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_InitStructure.NVIC_IRQChannel = DMA2_Channel3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0 ; //��ռ���ȼ�3
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		//�����ȼ�3
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
    NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
}
	
void dma2_channel3_init(uint32_t PeripheralBaseAddr, uint32_t MemoryBaseAddr, uint32_t BufferSize, uint32_t DIR)
{
    DMA_InitTypeDef DMA_InitStructure;
    /*����DMAʱ��*/
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);

    /*����DMAԴ���������ݼĴ�����ַ*/
    DMA_InitStructure.DMA_PeripheralBaseAddr = PeripheralBaseAddr;

    /*�ڴ��ַ(Ҫ����ı�����ָ��)*/
    DMA_InitStructure.DMA_MemoryBaseAddr = MemoryBaseAddr;

    /*���򣺴��ڴ浽����*/
    DMA_InitStructure.DMA_DIR = DIR;

    /*�����СDMA_BufferSize=SENDBUFF_SIZE*/
    DMA_InitStructure.DMA_BufferSize = BufferSize;

    /*�����ַ����*/
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;

    /*�ڴ��ַ����*/
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;

    /*�������ݵ�λ*/
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;

    /*�ڴ����ݵ�λ 8bit*/
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;

    /*DMAģʽ������ѭ��*/
//    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal ;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;

    /*���ȼ�����*/
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;

    /*��ֹ�ڴ浽�ڴ�Ĵ���	*/
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

    /*����DMA1��4ͨ��*/
    DMA_Init(DMA2_Channel3, &DMA_InitStructure);

    /*ʹ��DMA*/
    DMA_Cmd (DMA2_Channel3, ENABLE);

//    DMA_ITConfig(DMA2_Channel3, DMA_IT_TC, ENABLE); //����DMA������ɺ�����ж�
	
    DMA_ClearFlag(DMA2_FLAG_GL3);
//	DMA2_Channel3_NVIC_Init();
}
void DMA2_Channel5_NVIC_Init(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_InitStructure.NVIC_IRQChannel = DMA2_Channel4_5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0 ; //��ռ���ȼ�3
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		//�����ȼ�3
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
    NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
}
void dma2_channel5_init(uint32_t PeripheralBaseAddr, uint32_t MemoryBaseAddr, uint32_t BufferSize, uint32_t DIR)
{
    DMA_InitTypeDef DMA_InitStructure;
    /*����DMAʱ��*/
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);

    /*����DMAԴ���������ݼĴ�����ַ*/
    DMA_InitStructure.DMA_PeripheralBaseAddr = PeripheralBaseAddr;

    /*�ڴ��ַ(Ҫ����ı�����ָ��)*/
    DMA_InitStructure.DMA_MemoryBaseAddr = MemoryBaseAddr;

    /*���򣺴��ڴ浽����*/
    DMA_InitStructure.DMA_DIR = DIR;

    /*�����СDMA_BufferSize=SENDBUFF_SIZE*/
    DMA_InitStructure.DMA_BufferSize = BufferSize;

    /*�����ַ����*/
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;

    /*�ڴ��ַ����*/
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;

    /*�������ݵ�λ*/
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;

    /*�ڴ����ݵ�λ 8bit*/
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;

    /*DMAģʽ������ѭ��*/
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal ;
//	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;

    /*���ȼ�����*/
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;

    /*��ֹ�ڴ浽�ڴ�Ĵ���	*/
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

    /*����DMA1��4ͨ��*/
    DMA_Init(DMA2_Channel5, &DMA_InitStructure);

    /*ʹ��DMA*/
    DMA_Cmd (DMA2_Channel5, ENABLE);

    DMA_ITConfig(DMA2_Channel5, DMA_IT_TC, ENABLE); //����DMA������ɺ�����ж�

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

