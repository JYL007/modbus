#include "bsp_rs485.h"
#include "bsp_usartx.h"
#include "bsp_led.h"
#include<stdio.h>
#include<string.h>

void RS485_Slave_SendBefor(void)
{
    RS485_SLAVE_TX_EN();
}
void RS485_Slave_SendOver(void)
{
    RS485_SLAVE_RX_EN();
}
void RS485_Host_SendBefor(void)
{
	RGB_Light(BLUE);
    RS485_HOST_TX_EN();
}
void RS485_Host_SendOver(void)
{
    RS485_HOST_RX_EN();
	RGB_Light(GREEN);
}
void RS485_Slave_InitTXE(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_RS485_SLAVE_TXEN, ENABLE);	/* 打开GPIO时钟 */

    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	/* 推挽输出模式 */
    GPIO_InitStructure.GPIO_Pin = PIN_RS485_SLAVE_TXEN;
    GPIO_Init(PORT_RS485_SLAVE_TXEN, &GPIO_InitStructure);
    RS485_Slave_SendOver();
}
void RS485_Host_InitTXE(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_RS485_HOST_TXEN, ENABLE);	/* 打开GPIO时钟 */

    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	/* 推挽输出模式 */
    GPIO_InitStructure.GPIO_Pin = PIN_RS485_HOST_TXEN;
    GPIO_Init(PORT_RS485_HOST_TXEN, &GPIO_InitStructure);
	RS485_Host_SendOver();
		
}

void RS485_Slave_SendBuf(uint8_t *_ucaBuf, uint16_t _usLen)
{
//    printf("\r\nslave ack");
	uart5_send_buf(_ucaBuf,_usLen);
	
}
void RS485_Host_SendBuf(uint8_t *_ucaBuf, uint16_t _usLen)
{
	RS485_Host_SendBefor();
	uart4_dma_sendmsg((char*)_ucaBuf,_usLen);
}
void RS485_Slave_SendStr(char *_pBuf)
{
    RS485_Slave_SendBuf((uint8_t *)_pBuf, strlen(_pBuf));
}
void RS485_Host_SendStr(char *_pBuf)
{
    RS485_Host_SendBuf((uint8_t *)_pBuf, strlen(_pBuf));
}
