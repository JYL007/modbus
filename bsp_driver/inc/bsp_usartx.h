#ifndef __BSP_USARTX_H
#define __BSP_USARTX_H
#include "stm32f10x.h"


#define	UART1_FIFO_EN	1
#define	UART2_FIFO_EN	0
#define	UART3_FIFO_EN	0
#define	UART4_FIFO_EN	1
#define	UART5_FIFO_EN	1

#define USART4_DR_Base  (UART4_BASE+0x04)		// 0x40013800 + 0x04 = 0x40013804


typedef enum
{
	COM1 = 0,	/* USART1  PA9, PA10 */
	COM2 = 1,	/* USART2, PA2, PA3 */
	COM3 = 2,	/* USART3, PB10, PB11 */
	COM4 = 3,	/* UART4, PC10, PC11 */
	COM5 = 4	/* UART5, PC12, PD2 */
}COM_PORT_E;

#if UART1_FIFO_EN == 1
	#define UART1_BAUD			115200
	#define UART1_TX_BUF_SIZE	1*256
	#define UART1_RX_BUF_SIZE	1*256
#endif

#if UART2_FIFO_EN == 1
	#define UART2_BAUD			115200
	#define UART2_TX_BUF_SIZE	1*1024
	#define UART2_RX_BUF_SIZE	1*1024
#endif

#if UART3_FIFO_EN == 1
	#define UART3_BAUD			9600
	#define UART3_TX_BUF_SIZE	1*256
	#define UART3_RX_BUF_SIZE	1*256
#endif


#define UART4_BAUD			9600
#if UART4_FIFO_EN == 1
	#define UART4_TX_BUF_SIZE	1*256
	#define UART4_RX_BUF_SIZE	1*256
#endif

#if UART5_FIFO_EN == 1
	#define UART5_BAUD			9600
	#define UART5_TX_BUF_SIZE	1*256
	#define UART5_RX_BUF_SIZE	1*256
#endif


void bsp_usart_init(void);

void uart4_dma_sendmsg(char *msg,uint16_t len);

#endif


