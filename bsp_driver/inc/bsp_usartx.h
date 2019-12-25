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

typedef struct
{
	USART_TypeDef *uart;		/* STM32�ڲ������豸ָ�� */
	char *pTxBuf;			/* ���ͻ����� */
	char *pRxBuf;			/* ���ջ����� */
	uint16_t usTxBufSize;		/* ���ͻ�������С */
	uint16_t usRxBufSize;		/* ���ջ�������С */
	__IO uint16_t usTxWrite;	/* ���ͻ�����дָ�� */
	__IO uint16_t usTxRead;		/* ���ͻ�������ָ�� */
	__IO uint16_t usTxCount;	/* �ȴ����͵����ݸ��� */

	__IO uint16_t usRxWrite;	/* ���ջ�����дָ�� */
	__IO uint16_t usRxRead;		/* ���ջ�������ָ�� */
	__IO uint16_t usRxCount;	/* ��δ��ȡ�������ݸ��� */

	void (*SendBefor)(void); 	/* ��ʼ����֮ǰ�Ļص�����ָ�루��Ҫ����RS485�л�������ģʽ�� */
	void (*SendOver)(void); 	/* ������ϵĻص�����ָ�루��Ҫ����RS485������ģʽ�л�Ϊ����ģʽ�� */
	void (*ReciveNew)(uint8_t _byte);	/* �����յ����ݵĻص�����ָ�� */
}UART_T;

void bsp_usart_init(void);

void uart4_dma_sendmsg(char *msg,uint16_t len);
void uart5_send_buf(char *msg,uint16_t len);
#endif


