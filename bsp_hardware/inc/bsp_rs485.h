#ifndef __BSP_RS485_H
#define __BSP_RS485_H
#include "stm32f10x.h"


#define RCC_RS485_SLAVE_TXEN 	 	RCC_APB2Periph_GPIOC
#define PORT_RS485_SLAVE_TXEN  		GPIOC
#define PIN_RS485_SLAVE_TXEN	 	GPIO_Pin_9

#define RCC_RS485_HOST_TXEN 	 	RCC_APB2Periph_GPIOC
#define PORT_RS485_HOST_TXEN  		GPIOC
#define PIN_RS485_HOST_TXEN	 		GPIO_Pin_8

#define RS485_SLAVE_RX_EN()	PORT_RS485_SLAVE_TXEN->BRR = PIN_RS485_SLAVE_TXEN
#define RS485_SLAVE_TX_EN()	PORT_RS485_SLAVE_TXEN->BSRR = PIN_RS485_SLAVE_TXEN

#define RS485_HOST_RX_EN()	PORT_RS485_HOST_TXEN->BRR=PIN_RS485_HOST_TXEN
#define RS485_HOST_TX_EN()	PORT_RS485_HOST_TXEN->BSRR=PIN_RS485_HOST_TXEN

void RS485_Host_InitTXE(void);
void RS485_Slave_InitTXE(void);

void RS485_Host_SendBefor(void);
void RS485_Host_SendOver(void);

void RS485_Slave_SendBefor(void);
void RS485_Slave_SendOver(void);

void RS485_Slave_SendStr(char *_pBuf);
void RS485_Host_SendStr(char *_pBuf);
void RS485_Host_SendBuf(uint8_t *_ucaBuf, uint16_t _usLen);
void RS485_Slave_SendBuf(uint8_t *_ucaBuf, uint16_t _usLen);
#endif
