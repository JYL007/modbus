#ifndef __BSP_TIMER_H
#define __BSP_TIMER_H
#include "stm32f10x.h"

#define TMR_COUNT	4		/* 软件定时器的个数 （定时器ID范围 0 - 3) */

/* 定时器结构体，成员变量必须是 volatile, 否则C编译器优化时可能有问题 */
typedef enum
{
	TMR_ONCE_MODE = 0,		/* 一次工作模式 */
	TMR_AUTO_MODE = 1		/* 自动定时工作模式 */
}TMR_MODE_E;

/* 定时器结构体，成员变量必须是 volatile, 否则C编译器优化时可能有问题 */
typedef struct
{
	volatile uint8_t Mode;		/* 计数器模式，1次性 */
	volatile uint8_t Flag;		/* 定时到达标志  */
	volatile uint32_t Count;	/* 计数器 */
	volatile uint32_t PreLoad;	/* 计数器预装值 */
}SOFT_TMR;
void bsp_timer_init(void);
void bsp_StartHardTimer(uint8_t _CC, uint32_t _uiTimeOut, void * _pCallBack);
int32_t bsp_GetRunTime(void);
uint8_t bsp_CheckTimer(uint8_t _id);
uint32_t bsp_CheckRunTime(uint32_t _LastTime);
void bsp_systick_init(void);
void bsp_DelayUS(uint32_t n);
void bsp_DelayMS(uint32_t n);

#endif
