#include "stm32f10x.h"
#include "bsp.h"
#include<stdio.h>
#include "bsp_timer.h"
#include "modbus_host.h"
#include "modbus_slave.h"
#include "bsp_led.h"
#include "bsp_usartx.h"
#include<stdlib.h>
#include<string.h>
extern char g_TxBuf4[UART4_TX_BUF_SIZE];		/* 发送缓冲区 */
extern char g_RxBuf4[UART4_RX_BUF_SIZE];		/* 发送缓冲区 */
char buf[6] = { 1, 2, 3, 4, 5, 6 };
unsigned short dr[3];
int main()
{

    bsp_board_init();
    printf("init\n");
    RGB_Light(RED);

//	memcpy(dr, buf, 6);
//	{
//		int i;
//		for (i = 0; i < 3; i++)
//		{
//			printf("%0004X ", dr[i]);
//		}
//	}

//	bsp_StartHardTimer(1,30000,led_demo);
//	MODH_Send04H(1,0,1);
//	RGB_Light(GREEN);
    if(MODH_ReadParam_04H(1, 0, 1)) {
        printf("host ok");
    } else {
        printf("host err");
    }

    uart3_dma_sendmsg("AT\r\n", 4);

    while(1)
    {
        MODS_Poll();
        if(MODH_ReadParam_04H(1, 0, 1)) {
            printf("host ok");
        } else {
            printf("host err");
        }
		uart3_dma_sendmsg("AT\r\n", 4);

    }

}
