#ifndef __BSP_H
#define __BSP_H

#include "stm32f10x.h"

#define ENABLE_INT()	__set_PRIMASK(0)	/* 使能全局中断 */
#define DISABLE_INT()	__set_PRIMASK(1)	/* 禁止全局中断 */



void bsp_board_init(void);

#endif
