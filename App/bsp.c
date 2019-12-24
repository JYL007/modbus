#include "bsp.h"
#include "bsp_usartx.h"
#include "bsp_timer.h"
#include "bsp_led.h"
#include "bsp_rs485.h"
#include "bsp_dma.h"
#include "modbus_host.h"

extern char g_TxBuf4[UART4_TX_BUF_SIZE];		/* 发送缓冲区 */
extern char g_RxBuf4[UART4_RX_BUF_SIZE];		/* 发送缓冲区 */
extern MODH_T g_tModH;	//modbus host protocol data buff
void bsp_board_init(void)
{
	bsp_usart_init();
	RGB_PWM_Init();
	bsp_systick_init();
	bsp_timer_init();
	RS485_Host_InitTXE();
	dma2_channel5_init(USART4_DR_Base,(uint32_t)g_TxBuf4,1000,DMA_DIR_PeripheralDST);
	dma2_channel3_init(USART4_DR_Base,(uint32_t)g_tModH.RxBuf,1000,DMA_DIR_PeripheralSRC);
}
void MODH_AnalyzeApp(void);
void bsp_Idle(void)
{
	MODH_AnalyzeApp();
}