#include "bsp_usartx.h"
#include "bsp_rs485.h"
#include "bsp_led.h"
#include<stdio.h>
#include "modbus_host.h"


/* ����ÿ�����ڽṹ����� */
#if UART1_FIFO_EN == 1

static char g_TxBuf1[UART1_TX_BUF_SIZE];		/* ���ͻ����� */
static char g_RxBuf1[UART1_RX_BUF_SIZE];		/* ���ջ����� */
#endif

#if UART2_FIFO_EN == 1
static UART_T g_tUart2;
static char g_TxBuf2[UART2_TX_BUF_SIZE];		/* ���ͻ����� */
static char g_RxBuf2[UART2_RX_BUF_SIZE];		/* ���ջ����� */
#endif

#if UART3_FIFO_EN == 1
UART_T g_tUart3;
char g_TxBuf3[UART3_TX_BUF_SIZE];		/* ���ͻ����� */
char g_RxBuf3[UART3_RX_BUF_SIZE];		/* ���ջ����� */
#endif

#if UART4_FIFO_EN == 1
char g_TxBuf4[UART4_TX_BUF_SIZE];		/* ���ͻ����� */
char g_RxBuf4[UART4_RX_BUF_SIZE];		/* ���ջ����� */
#endif

#if UART5_FIFO_EN == 1

char g_TxBuf5[UART5_TX_BUF_SIZE];		/* ���ͻ����� */
char g_RxBuf5[UART5_RX_BUF_SIZE];		/* ���ջ����� */
#endif

void InitHardUart(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
#if UART1_FIFO_EN == 1		/* ����1 TX = PA9   RX = PA10 �� TX = PB6   RX = PB7*/
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

    /* ��2������USART Tx��GPIO����Ϊ���츴��ģʽ */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* ��3������USART Rx��GPIO����Ϊ��������ģʽ
    	����CPU��λ��GPIOȱʡ���Ǹ�������ģʽ���������������費�Ǳ����
    	���ǣ��һ��ǽ�����ϱ����Ķ������ҷ�ֹ�����ط��޸���������ߵ����ò���
    */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* ��4���� ���ô���Ӳ������ */
    USART_InitStructure.USART_BaudRate = UART1_BAUD;	/* ������ */
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No ;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);	/* ʹ�ܽ����ж� */
    /*
    	USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
    	ע��: ��Ҫ�ڴ˴��򿪷����ж�
    	�����ж�ʹ����SendUart()������
    */
    USART_Cmd(USART1, ENABLE);		/* ʹ�ܴ��� */

    /* CPU��Сȱ�ݣ��������úã����ֱ��Send�����1���ֽڷ��Ͳ���ȥ
    	�����������1���ֽ��޷���ȷ���ͳ�ȥ������ */
    USART_ClearFlag(USART1, USART_FLAG_TC);     /* �巢����ɱ�־��Transmission Complete flag */
#endif

#if UART2_FIFO_EN == 1		/* ����2 TX = PA2�� RX = PA3  */
    /* ��1������GPIO��USART������ʱ�� */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    /* ��2������USART Tx��GPIO����Ϊ���츴��ģʽ */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* ��3������USART Rx��GPIO����Ϊ��������ģʽ
    	����CPU��λ��GPIOȱʡ���Ǹ�������ģʽ���������������費�Ǳ����
    	���ǣ��һ��ǽ�����ϱ����Ķ������ҷ�ֹ�����ط��޸���������ߵ����ò���
    */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    /*  ��3���Ѿ����ˣ�����ⲽ���Բ���
    	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    */
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* ��4���� ���ô���Ӳ������ */
    USART_InitStructure.USART_BaudRate = UART2_BAUD;	/* ������ */
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No ;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;		/* ��ѡ�����ģʽ */
    USART_Init(USART2, &USART_InitStructure);

    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);	/* ʹ�ܽ����ж� */
    /*
    	USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
    	ע��: ��Ҫ�ڴ˴��򿪷����ж�
    	�����ж�ʹ����SendUart()������
    */
    USART_Cmd(USART2, ENABLE);		/* ʹ�ܴ��� */

    /* CPU��Сȱ�ݣ��������úã����ֱ��Send�����1���ֽڷ��Ͳ���ȥ
    	�����������1���ֽ��޷���ȷ���ͳ�ȥ������ */
    USART_ClearFlag(USART2, USART_FLAG_TC);     /* �巢����ɱ�־��Transmission Complete flag */
#endif

#if UART3_FIFO_EN == 1			/* ����3 TX = PB10   RX = PB11 */


    /* ��1���� ����GPIO��UARTʱ�� */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

    /* ��2������USART Tx��GPIO����Ϊ���츴��ģʽ */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* ��3������USART Rx��GPIO����Ϊ��������ģʽ
    	����CPU��λ��GPIOȱʡ���Ǹ�������ģʽ���������������費�Ǳ����
    	���ǣ��һ��ǽ�����ϱ����Ķ������ҷ�ֹ�����ط��޸���������ߵ����ò���
    */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    /*  ��3���Ѿ����ˣ�����ⲽ���Բ���
    	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    */
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* ��4���� ���ô���Ӳ������ */
    USART_InitStructure.USART_BaudRate = UART3_BAUD;	/* ������ */
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No ;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART3, &USART_InitStructure);

    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);	/* ʹ�ܽ����ж� */
    /*
    	USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
    	ע��: ��Ҫ�ڴ˴��򿪷����ж�
    	�����ж�ʹ����SendUart()������
    */
    USART_Cmd(USART3, ENABLE);		/* ʹ�ܴ��� */

    /* CPU��Сȱ�ݣ��������úã����ֱ��Send�����1���ֽڷ��Ͳ���ȥ
    	�����������1���ֽ��޷���ȷ���ͳ�ȥ������ */
    USART_ClearFlag(USART3, USART_FLAG_TC);     /* �巢����ɱ�־��Transmission Complete flag */
#endif

#if UART4_FIFO_EN == 1			/* ����4 TX = PC10   RX = PC11 */
    /* ��1���� ����GPIO��UARTʱ�� */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);

    /* ��2������USART Tx��GPIO����Ϊ���츴��ģʽ */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    /* ��3������USART Rx��GPIO����Ϊ��������ģʽ
    	����CPU��λ��GPIOȱʡ���Ǹ�������ģʽ���������������費�Ǳ����
    	���ǣ��һ��ǽ�����ϱ����Ķ������ҷ�ֹ�����ط��޸���������ߵ����ò���
    */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    /* ��4���� ���ô���Ӳ������ */
    USART_InitStructure.USART_BaudRate = UART4_BAUD;	/* ������ */
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No ;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(UART4, &USART_InitStructure);
	USART_DMACmd(UART4, USART_DMAReq_Rx, ENABLE);
//    USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);	/* ʹ�ܽ����ж� */
	USART_ITConfig(UART4, USART_IT_IDLE, ENABLE);	/* ʹ�ܿ����ж� */
    /*
    	USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
    	ע��: ��Ҫ�ڴ˴��򿪷����ж�
    	�����ж�ʹ����SendUart()������
    */
    USART_Cmd(UART4, ENABLE);		/* ʹ�ܴ��� */

    /* CPU��Сȱ�ݣ��������úã����ֱ��Send�����1���ֽڷ��Ͳ���ȥ
    	�����������1���ֽ��޷���ȷ���ͳ�ȥ������ */
    USART_ClearFlag(UART4, USART_FLAG_TC);     /* �巢����ɱ�־��Transmission Complete flag */
#endif

#if UART5_FIFO_EN == 1			/* ����5 TX = PC12   RX = PD2 */
    /* ��1���� ����GPIO��UARTʱ�� */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);

    /* ��2������USART Tx��GPIO����Ϊ���츴��ģʽ */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    /* ��3������USART Rx��GPIO����Ϊ��������ģʽ
    	����CPU��λ��GPIOȱʡ���Ǹ�������ģʽ���������������費�Ǳ����
    	���ǣ��һ��ǽ�����ϱ����Ķ������ҷ�ֹ�����ط��޸���������ߵ����ò���
    */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);


    /* ��4���� ���ô���Ӳ������ */
    USART_InitStructure.USART_BaudRate = UART5_BAUD;	/* ������ */
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No ;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(UART5, &USART_InitStructure);

    USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);	/* ʹ�ܽ����ж� */
    /*
    	USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
    	ע��: ��Ҫ�ڴ˴��򿪷����ж�
    	�����ж�ʹ����SendUart()������
    */
    USART_Cmd(UART5, ENABLE);		/* ʹ�ܴ��� */

    /* CPU��Сȱ�ݣ��������úã����ֱ��Send�����1���ֽڷ��Ͳ���ȥ
    	�����������1���ֽ��޷���ȷ���ͳ�ȥ������ */
    USART_ClearFlag(UART5, USART_FLAG_TC);     /* �巢����ɱ�־��Transmission Complete flag */
#endif

}

void ConfigUartNVIC(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Configure the NVIC Preemption Priority Bits */
    /*	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);  --- �� bsp.c �� bsp_Init() �������ж����ȼ��� */

#if UART1_FIFO_EN == 1
    /* ʹ�ܴ���1�ж� */
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
#endif

#if UART2_FIFO_EN == 1
    /* ʹ�ܴ���2�ж� */
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
#endif

#if UART3_FIFO_EN == 1
    /* ʹ�ܴ���3�ж�t */
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
#endif

#if UART4_FIFO_EN == 1
    /* ʹ�ܴ���4�ж�t */
    NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
#endif

#if UART5_FIFO_EN == 1
    /* ʹ�ܴ���5�ж�t */
    NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
#endif

#if UART6_FIFO_EN == 1
    /* ʹ�ܴ���6�ж�t */
    NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 5;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
#endif
}

void uart4_tx_dma_init(void)
{
	
}
void uart4_rx_dma_init(void)
{

}

void bsp_usart_init(void)
{
    InitHardUart();
    ConfigUartNVIC();
}
void uart4_dma_sendmsg(char *msg,uint16_t len)
{
	DMA_ClearFlag(DMA2_FLAG_TC5);
	USART_ClearITPendingBit(UART4,USART_IT_TC);
	DMA_Cmd(DMA2_Channel5,DISABLE);
	DMA2_Channel5->CMAR=(uint32_t)msg;
	DMA2_Channel5->CNDTR=len;
	USART_ITConfig(UART4,USART_IT_TC,ENABLE);
	DMA_Cmd(DMA2_Channel5,ENABLE);
	USART_DMACmd(UART4, USART_DMAReq_Tx, ENABLE);	
}
void uart4_dma_recvmsg()
{
	DMA_Cmd(DMA2_Channel3, DISABLE);
	DMA_ClearFlag( DMA2_FLAG_GL3 );
    //  ���¸�ֵ����ֵ��������ڵ��������ܽ��յ�������֡��Ŀ
    DMA2_Channel3->CNDTR = 1000;
    DMA_Cmd(DMA2_Channel3, ENABLE);
	
}
void uart5_send_buf()
{
	
}
int fputc(int ch, FILE *f)
{
#if 0	/* ����Ҫprintf���ַ�ͨ�������ж�FIFO���ͳ�ȥ��printf�������������� */
    comSendChar(COM1, ch);

    return ch;
#else	/* ����������ʽ����ÿ���ַ�,�ȴ����ݷ������ */
    /* дһ���ֽڵ�USART1 */
    USART_SendData(USART1, (uint8_t) ch);

    /* �ȴ����ͽ��� */
    while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
    {}

    return ch;
#endif
}
int fgetc(FILE *f)
{

#if 0	/* �Ӵ��ڽ���FIFO��ȡ1������, ֻ��ȡ�����ݲŷ��� */
    uint8_t ucData;

    while(comGetChar(COM1, &ucData) == 0);

    return ucData;
#else
    /* �ȴ�����1�������� */
    while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET);

    return (int)USART_ReceiveData(USART1);
#endif
}

#if UART1_FIFO_EN==1
void USART1_IRQHandler(void)
{


}
#endif
#if UART2_FIFO_EN==1
void USART2_IRQHandler(void)
{
	
}
#endif
#if UART3_FIFO_EN==1
void USART3_IRQHandler(void)
{

}
#endif
#if UART4_FIFO_EN==1
extern uint8_t g_modh_timeout ;
extern MODH_T g_tModH;

void UART4_IRQHandler(void)
{
//	uint8_t ch;
//	if(USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)
//	{
//		ch=USART_ReceiveData(UART4);
//		printf("%02x ",ch);
//		USART_ClearFlag(UART4,USART_IT_RXNE);
//		RGB_Light(RED);
//	}
    if(USART_GetITStatus(UART4, USART_IT_IDLE) != RESET)
    {
		printf("recv cnt:%d",DMA_GetCurrDataCounter(DMA2_Channel3));
		g_modh_timeout=1;
		USART_ReceiveData(UART4);
		printf("idle");
		RGB_Light(YELLOW);
		
		{
			uint8_t i;
			for(i=0;i<1000-DMA_GetCurrDataCounter(DMA2_Channel3);i++)
			{
				printf("%d:%02X ",i,g_RxBuf4[i]);
			}
		}
		g_tModH.RxCount=1000-DMA_GetCurrDataCounter(DMA2_Channel3);
		uart4_dma_recvmsg();
		
    }
	if(USART_GetITStatus(UART4,USART_IT_TC))
	{
		USART_ClearITPendingBit(UART4,USART_IT_TC);
		USART_ITConfig(UART4,USART_IT_TC,DISABLE);
		printf("send ok");
		RS485_Host_SendOver();
	}
	
}
#endif
#if UART5_FIFO_EN==1
void UART5_IRQHandler(void)
{
	uint8_t ch;
	if(USART_GetITStatus(UART5,USART_IT_RXNE)!=RESET)
	{
		ch=USART_ReceiveData(UART5);
		printf("%c",ch);
	}
}
#endif

