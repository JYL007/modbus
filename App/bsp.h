#ifndef __BSP_H
#define __BSP_H

#include "stm32f10x.h"

#define ENABLE_INT()	__set_PRIMASK(0)	/* ʹ��ȫ���ж� */
#define DISABLE_INT()	__set_PRIMASK(1)	/* ��ֹȫ���ж� */



void bsp_board_init(void);

#endif
