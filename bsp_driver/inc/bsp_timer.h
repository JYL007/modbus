#ifndef __BSP_TIMER_H
#define __BSP_TIMER_H
#include "stm32f10x.h"

#define TMR_COUNT	4		/* �����ʱ���ĸ��� ����ʱ��ID��Χ 0 - 3) */

/* ��ʱ���ṹ�壬��Ա���������� volatile, ����C�������Ż�ʱ���������� */
typedef enum
{
	TMR_ONCE_MODE = 0,		/* һ�ι���ģʽ */
	TMR_AUTO_MODE = 1		/* �Զ���ʱ����ģʽ */
}TMR_MODE_E;

/* ��ʱ���ṹ�壬��Ա���������� volatile, ����C�������Ż�ʱ���������� */
typedef struct
{
	volatile uint8_t Mode;		/* ������ģʽ��1���� */
	volatile uint8_t Flag;		/* ��ʱ�����־  */
	volatile uint32_t Count;	/* ������ */
	volatile uint32_t PreLoad;	/* ������Ԥװֵ */
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
