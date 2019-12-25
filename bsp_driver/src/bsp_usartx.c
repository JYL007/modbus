#include "bsp_usartx.h"
#include "bsp_rs485.h"
#include "bsp_led.h"
#include<stdio.h>
#include "modbus_host.h"
#include "modbus_slave.h"
#include "bsp.h"


/* 定义每个串口结构体变量 */
#if UART1_FIFO_EN == 1

static char g_TxBuf1[UART1_TX_BUF_SIZE];		/* 发送缓冲区 */
static char g_RxBuf1[UART1_RX_BUF_SIZE];		/* 接收缓冲区 */
#endif

#if UART2_FIFO_EN == 1
static UART_T g_tUart2;
static char g_TxBuf2[UART2_TX_BUF_SIZE];		/* 发送缓冲区 */
static char g_RxBuf2[UART2_RX_BUF_SIZE];		/* 接收缓冲区 */
#endif

#if UART3_FIFO_EN == 1
UART_T g_tUart3;
char g_TxBuf3[UART3_TX_BUF_SIZE];		/* 发送缓冲区 */
char g_RxBuf3[UART3_RX_BUF_SIZE];		/* 接收缓冲区 */
#endif

#if UART4_FIFO_EN == 1
char g_TxBuf4[UART4_TX_BUF_SIZE];		/* 发送缓冲区 */
char g_RxBuf4[UART4_RX_BUF_SIZE];		/* 接收缓冲区 */
#endif

#if UART5_FIFO_EN == 1
UART_T g_tUart5;
char g_TxBuf5[UART5_TX_BUF_SIZE];		/* 发送缓冲区 */
char g_RxBuf5[UART5_RX_BUF_SIZE];		/* 接收缓冲区 */
#endif
void UartVarInit(void)
{
#if UART5_FIFO_EN == 1
    g_tUart5.uart = UART5;						/* STM32 串口设备 */
    g_tUart5.pTxBuf = g_TxBuf5;					/* 发送缓冲区指针 */
    g_tUart5.pRxBuf = g_RxBuf5;					/* 接收缓冲区指针 */
    g_tUart5.usTxBufSize = UART5_TX_BUF_SIZE;	/* 发送缓冲区大小 */
    g_tUart5.usRxBufSize = UART5_RX_BUF_SIZE;	/* 接收缓冲区大小 */
    g_tUart5.usTxWrite = 0;						/* 发送FIFO写索引 */
    g_tUart5.usTxRead = 0;						/* 发送FIFO读索引 */
    g_tUart5.usRxWrite = 0;						/* 接收FIFO写索引 */
    g_tUart5.usRxRead = 0;						/* 接收FIFO读索引 */
    g_tUart5.usRxCount = 0;						/* 接收到的新数据个数 */
    g_tUart5.usTxCount = 0;						/* 待发送的数据个数 */
    g_tUart5.SendBefor = RS485_Slave_SendBefor;	/* 发送数据前的回调函数 */
    g_tUart5.SendOver = RS485_Slave_SendOver;	/* 发送完毕后的回调函数 */
    g_tUart5.ReciveNew = MODS_ReciveNew;	/* 接收到新数据后的回调函数 */
#endif
}
void InitHardUart(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
#if UART1_FIFO_EN == 1		/* 串口1 TX = PA9   RX = PA10 或 TX = PB6   RX = PB7*/
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

    /* 第2步：将USART Tx的GPIO配置为推挽复用模式 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* 第3步：将USART Rx的GPIO配置为浮空输入模式
    	由于CPU复位后，GPIO缺省都是浮空输入模式，因此下面这个步骤不是必须的
    	但是，我还是建议加上便于阅读，并且防止其它地方修改了这个口线的设置参数
    */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* 第4步： 配置串口硬件参数 */
    USART_InitStructure.USART_BaudRate = UART1_BAUD;	/* 波特率 */
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No ;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);	/* 使能接收中断 */
    /*
    	USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
    	注意: 不要在此处打开发送中断
    	发送中断使能在SendUart()函数打开
    */
    USART_Cmd(USART1, ENABLE);		/* 使能串口 */

    /* CPU的小缺陷：串口配置好，如果直接Send，则第1个字节发送不出去
    	如下语句解决第1个字节无法正确发送出去的问题 */
    USART_ClearFlag(USART1, USART_FLAG_TC);     /* 清发送完成标志，Transmission Complete flag */
#endif

#if UART2_FIFO_EN == 1		/* 串口2 TX = PA2， RX = PA3  */
    /* 第1步：打开GPIO和USART部件的时钟 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    /* 第2步：将USART Tx的GPIO配置为推挽复用模式 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* 第3步：将USART Rx的GPIO配置为浮空输入模式
    	由于CPU复位后，GPIO缺省都是浮空输入模式，因此下面这个步骤不是必须的
    	但是，我还是建议加上便于阅读，并且防止其它地方修改了这个口线的设置参数
    */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    /*  第3步已经做了，因此这步可以不做
    	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    */
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* 第4步： 配置串口硬件参数 */
    USART_InitStructure.USART_BaudRate = UART2_BAUD;	/* 波特率 */
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No ;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;		/* 仅选择接收模式 */
    USART_Init(USART2, &USART_InitStructure);

    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);	/* 使能接收中断 */
    /*
    	USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
    	注意: 不要在此处打开发送中断
    	发送中断使能在SendUart()函数打开
    */
    USART_Cmd(USART2, ENABLE);		/* 使能串口 */

    /* CPU的小缺陷：串口配置好，如果直接Send，则第1个字节发送不出去
    	如下语句解决第1个字节无法正确发送出去的问题 */
    USART_ClearFlag(USART2, USART_FLAG_TC);     /* 清发送完成标志，Transmission Complete flag */
#endif

#if UART3_FIFO_EN == 1			/* 串口3 TX = PB10   RX = PB11 */


    /* 第1步： 开启GPIO和UART时钟 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

    /* 第2步：将USART Tx的GPIO配置为推挽复用模式 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* 第3步：将USART Rx的GPIO配置为浮空输入模式
    	由于CPU复位后，GPIO缺省都是浮空输入模式，因此下面这个步骤不是必须的
    	但是，我还是建议加上便于阅读，并且防止其它地方修改了这个口线的设置参数
    */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    /*  第3步已经做了，因此这步可以不做
    	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    */
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* 第4步： 配置串口硬件参数 */
    USART_InitStructure.USART_BaudRate = UART3_BAUD;	/* 波特率 */
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No ;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART3, &USART_InitStructure);
	USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);
//    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);	/* 使能接收中断 */
	USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);	/* 使能空闲中断 */
    /*
    	USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
    	注意: 不要在此处打开发送中断
    	发送中断使能在SendUart()函数打开
    */
    USART_Cmd(USART3, ENABLE);		/* 使能串口 */

    /* CPU的小缺陷：串口配置好，如果直接Send，则第1个字节发送不出去
    	如下语句解决第1个字节无法正确发送出去的问题 */
    USART_ClearFlag(USART3, USART_FLAG_TC);     /* 清发送完成标志，Transmission Complete flag */
#endif

#if UART4_FIFO_EN == 1			/* 串口4 TX = PC10   RX = PC11 */
    /* 第1步： 开启GPIO和UART时钟 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);

    /* 第2步：将USART Tx的GPIO配置为推挽复用模式 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    /* 第3步：将USART Rx的GPIO配置为浮空输入模式
    	由于CPU复位后，GPIO缺省都是浮空输入模式，因此下面这个步骤不是必须的
    	但是，我还是建议加上便于阅读，并且防止其它地方修改了这个口线的设置参数
    */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    /* 第4步： 配置串口硬件参数 */
    USART_InitStructure.USART_BaudRate = UART4_BAUD;	/* 波特率 */
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No ;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(UART4, &USART_InitStructure);
    USART_DMACmd(UART4, USART_DMAReq_Rx, ENABLE);
//    USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);	/* 使能接收中断 */
    USART_ITConfig(UART4, USART_IT_IDLE, ENABLE);	/* 使能空闲中断 */
    /*
    	USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
    	注意: 不要在此处打开发送中断
    	发送中断使能在SendUart()函数打开
    */
    USART_Cmd(UART4, ENABLE);		/* 使能串口 */

    /* CPU的小缺陷：串口配置好，如果直接Send，则第1个字节发送不出去
    	如下语句解决第1个字节无法正确发送出去的问题 */
    USART_ClearFlag(UART4, USART_FLAG_TC);     /* 清发送完成标志，Transmission Complete flag */
#endif

#if UART5_FIFO_EN == 1			/* 串口5 TX = PC12   RX = PD2 */
    /* 第1步： 开启GPIO和UART时钟 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);

    /* 第2步：将USART Tx的GPIO配置为推挽复用模式 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    /* 第3步：将USART Rx的GPIO配置为浮空输入模式
    	由于CPU复位后，GPIO缺省都是浮空输入模式，因此下面这个步骤不是必须的
    	但是，我还是建议加上便于阅读，并且防止其它地方修改了这个口线的设置参数
    */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);


    /* 第4步： 配置串口硬件参数 */
    USART_InitStructure.USART_BaudRate = UART5_BAUD;	/* 波特率 */
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No ;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(UART5, &USART_InitStructure);

    USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);	/* 使能接收中断 */
    /*
    	USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
    	注意: 不要在此处打开发送中断
    	发送中断使能在SendUart()函数打开
    */
    USART_Cmd(UART5, ENABLE);		/* 使能串口 */

    /* CPU的小缺陷：串口配置好，如果直接Send，则第1个字节发送不出去
    	如下语句解决第1个字节无法正确发送出去的问题 */
    USART_ClearFlag(UART5, USART_FLAG_TC);     /* 清发送完成标志，Transmission Complete flag */
#endif

}

void ConfigUartNVIC(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Configure the NVIC Preemption Priority Bits */
    /*	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);  --- 在 bsp.c 中 bsp_Init() 中配置中断优先级组 */

#if UART1_FIFO_EN == 1
    /* 使能串口1中断 */
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
#endif

#if UART2_FIFO_EN == 1
    /* 使能串口2中断 */
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
#endif

#if UART3_FIFO_EN == 1
    /* 使能串口3中断t */
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
#endif

#if UART4_FIFO_EN == 1
    /* 使能串口4中断t */
    NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
#endif

#if UART5_FIFO_EN == 1
    /* 使能串口5中断t */
    NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
#endif

#if UART6_FIFO_EN == 1
    /* 使能串口6中断t */
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
    UartVarInit();
    InitHardUart();
    ConfigUartNVIC();
}
void uart3_dma_sendmsg(char *msg, uint16_t len)
{
	DMA_ClearFlag(DMA1_FLAG_TC2);
    USART_ClearITPendingBit(USART3, USART_IT_TC);
    DMA_Cmd(DMA1_Channel2, DISABLE);
    DMA1_Channel2->CMAR = (uint32_t)msg;
    DMA1_Channel2->CNDTR = len;
    USART_ITConfig(USART3, USART_IT_TC, ENABLE);
    DMA_Cmd(DMA1_Channel2, ENABLE);
    USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE);
}
void uart3_dma_recvmsg(void)
{
	DMA_Cmd(DMA1_Channel3,DISABLE);
	DMA_ClearFlag(DMA1_FLAG_GL3);
	DMA1_Channel3->CNDTR=1024;
	DMA_Cmd(DMA1_Channel3,ENABLE);
}
void uart4_dma_sendmsg(char *msg, uint16_t len)
{
    DMA_ClearFlag(DMA2_FLAG_TC5);
    USART_ClearITPendingBit(UART4, USART_IT_TC);
    DMA_Cmd(DMA2_Channel5, DISABLE);
    DMA2_Channel5->CMAR = (uint32_t)msg;
    DMA2_Channel5->CNDTR = len;
    USART_ITConfig(UART4, USART_IT_TC, ENABLE);
    DMA_Cmd(DMA2_Channel5, ENABLE);
    USART_DMACmd(UART4, USART_DMAReq_Tx, ENABLE);
}
void uart4_dma_recvmsg(void)
{
    DMA_Cmd(DMA2_Channel3, DISABLE);
    DMA_ClearFlag( DMA2_FLAG_GL3 );
    //  重新赋值计数值，必须大于等于最大可能接收到的数据帧数目
    DMA2_Channel3->CNDTR = 1000;
    DMA_Cmd(DMA2_Channel3, ENABLE);

}
void uart5_send_buf(char *msg, uint16_t len)
{
    uint16_t i;
if (g_tUart5.SendBefor != 0)
    {
        g_tUart5.SendBefor();		/* 如果是RS485通信，可以在这个函数中将RS485设置为发送模式 */
    }
    for (i = 0; i < len; i++)
    {
        while (1)
        {
            __IO uint16_t usCount;

            DISABLE_INT();
            usCount = g_tUart5.usTxCount;
            ENABLE_INT();

            if (usCount < g_tUart5.usTxBufSize)
            {
                break;
            }
        }
        g_tUart5.pTxBuf[g_tUart5.usTxWrite] = msg[i];

        DISABLE_INT();
        if (++g_tUart5.usTxWrite >= g_tUart5.usTxBufSize)
        {
            g_tUart5.usTxWrite = 0;
        }
        g_tUart5.usTxCount++;
        ENABLE_INT();

    }
	{
		uint8_t j;
		for(j=0;j<g_tUart5.usTxCount;j++)
		{
			printf("%02X ",g_tUart5.pTxBuf[j]);
		}
	}
    USART_ITConfig(g_tUart5.uart, USART_IT_TXE, ENABLE);
}
int fputc(int ch, FILE *f)
{
#if 0	/* 将需要printf的字符通过串口中断FIFO发送出去，printf函数会立即返回 */
    comSendChar(COM1, ch);

    return ch;
#else	/* 采用阻塞方式发送每个字符,等待数据发送完毕 */
    /* 写一个字节到USART1 */
    USART_SendData(USART1, (uint8_t) ch);

    /* 等待发送结束 */
    while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
    {}

    return ch;
#endif
}
int fgetc(FILE *f)
{

#if 0	/* 从串口接收FIFO中取1个数据, 只有取到数据才返回 */
    uint8_t ucData;

    while(comGetChar(COM1, &ucData) == 0);

    return ucData;
#else
    /* 等待串口1输入数据 */
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
	uint8_t ch;
//	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
//    {
//        ch = USART_ReceiveData(USART3);
//		printf("%c",ch);
//	}
	if(USART_GetITStatus(USART3, USART_IT_IDLE) != RESET)
	{
		
		printf("recv cnt:%d", 1024-DMA_GetCurrDataCounter(DMA1_Channel3));
		USART_ReceiveData(USART3);
		uart3_dma_recvmsg();
	}
	if(USART_GetITStatus(USART3, USART_IT_TC) != RESET)
	{
		USART_ClearITPendingBit(USART3, USART_IT_TC);
        USART_ITConfig(USART3, USART_IT_TC, DISABLE);
	}
	
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
        printf("recv cnt:%d", DMA_GetCurrDataCounter(DMA2_Channel3));
        g_modh_timeout = 1;
        USART_ReceiveData(UART4);
        printf("idle");
        RGB_Light(YELLOW);

        {
            uint8_t i;
            for(i = 0; i < 1000 - DMA_GetCurrDataCounter(DMA2_Channel3); i++)
            {
                printf("%d:%02X ", i, g_RxBuf4[i]);
            }
        }
        g_tModH.RxCount = 1000 - DMA_GetCurrDataCounter(DMA2_Channel3);
        uart4_dma_recvmsg();

    }
    if(USART_GetITStatus(UART4, USART_IT_TC))
    {
        USART_ClearITPendingBit(UART4, USART_IT_TC);
        USART_ITConfig(UART4, USART_IT_TC, DISABLE);
        printf("send ok");
        RS485_Host_SendOver();
    }

}
#endif
#if UART5_FIFO_EN==1
void UART5_IRQHandler(void)
{
    uint8_t ch;
    if(USART_GetITStatus(UART5, USART_IT_RXNE) != RESET)
    {
        ch = USART_ReceiveData(g_tUart5.uart);
        g_tUart5.pRxBuf[g_tUart5.usRxWrite] = ch;
        if (++g_tUart5.usRxWrite >= g_tUart5.usRxBufSize)
        {
            g_tUart5.usRxWrite = 0;
        }
        if (g_tUart5.usRxCount < g_tUart5.usRxBufSize)
        {
            g_tUart5.usRxCount++;
        }
        {
            if (g_tUart5.ReciveNew)
            {
                g_tUart5.ReciveNew(ch);
            }
        }

    }
    /* 处理发送缓冲区空中断 */
    if (USART_GetITStatus(g_tUart5.uart, USART_IT_TXE) != RESET)
    {
//		printf("uart5 tx irq");
        //if (_pUart->usTxRead == _pUart->usTxWrite)
        if (g_tUart5.usTxCount == 0)
        {
            /* 发送缓冲区的数据已取完时， 禁止发送缓冲区空中断 （注意：此时最后1个数据还未真正发送完毕）*/
            USART_ITConfig(g_tUart5.uart, USART_IT_TXE, DISABLE);

            /* 使能数据发送完毕中断 */
            USART_ITConfig(g_tUart5.uart, USART_IT_TC, ENABLE);
        }
        else
        {
			printf("uart5 send\n");
            /* 从发送FIFO取1个字节写入串口发送数据寄存器 */
            USART_SendData(g_tUart5.uart, g_tUart5.pTxBuf[g_tUart5.usTxRead]);
            if (++g_tUart5.usTxRead >= g_tUart5.usTxBufSize)
            {
                g_tUart5.usTxRead = 0;
            }
            g_tUart5.usTxCount--;
        }
    }
    /* 数据bit位全部发送完毕的中断 */
    else if (USART_GetITStatus(g_tUart5.uart, USART_IT_TC) != RESET)
    {
        //if (_pUart->usTxRead == _pUart->usTxWrite)
        if (g_tUart5.usTxCount == 0)
        {
            /* 如果发送FIFO的数据全部发送完毕，禁止数据发送完毕中断 */
            USART_ITConfig(g_tUart5.uart, USART_IT_TC, DISABLE);

            /* 回调函数, 一般用来处理RS485通信，将RS485芯片设置为接收模式，避免抢占总线 */
            if (g_tUart5.SendOver)
            {
                g_tUart5.SendOver();
            }
        }
        else
        {
            /* 正常情况下，不会进入此分支 */

            /* 如果发送FIFO的数据还未完毕，则从发送FIFO取1个数据写入发送数据寄存器 */
            USART_SendData(g_tUart5.uart, g_tUart5.pTxBuf[g_tUart5.usTxRead]);
            if (++g_tUart5.usTxRead >= g_tUart5.usTxBufSize)
            {
                g_tUart5.usTxRead = 0;
            }
            g_tUart5.usTxCount--;
        }
    }
}
#endif

