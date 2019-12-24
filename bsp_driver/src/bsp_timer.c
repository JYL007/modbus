#include "bsp_timer.h"
#include "bsp.h"

__IO uint32_t g_iRunTime = 0;
static volatile uint32_t s_uiDelayCount = 0;
static volatile uint8_t s_ucTimeOutFlag = 0;

/* ���������ʱ���ṹ����� */
static SOFT_TMR s_tTmr[TMR_COUNT];
/* ���� TIM��ʱ�жϵ���ִ�еĻص�����ָ�� */
static void (*s_TIM_CallBack1)(void);
static void (*s_TIM_CallBack2)(void);
static void (*s_TIM_CallBack3)(void);
static void (*s_TIM_CallBack4)(void);

void bsp_timer_init(void)
{
    uint8_t i;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;	/* �жϽṹ���� misc.h �ж��� */
    uint32_t usPeriod;
    uint16_t usPrescaler;
    uint32_t uiTIMxCLK;
    /* �������е������ʱ�� */
    for (i = 0; i < TMR_COUNT; i++)
    {
        s_tTmr[i].Count = 0;
        s_tTmr[i].PreLoad = 0;
        s_tTmr[i].Flag = 0;
        s_tTmr[i].Mode = TMR_ONCE_MODE;	/* ȱʡ��1���Թ���ģʽ */
    }
	
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    uiTIMxCLK = SystemCoreClock / 2;
    usPrescaler = uiTIMxCLK / 1000000 ;	/* ��Ƶ������ 1us */
    usPeriod = 0xFFFF;

    TIM_TimeBaseStructure.TIM_Period = usPeriod;
    TIM_TimeBaseStructure.TIM_Prescaler = usPrescaler;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    //TIM_ARRPreloadConfig(TIMx, ENABLE);

    /* TIMx enable counter */
    TIM_Cmd(TIM3, ENABLE);
	
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;	/* �ȴ������ȼ��� */
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

}

/*
*********************************************************************************************************
*	�� �� ��: bsp_StartHardTimer
*	����˵��: ʹ��TIM2-5�����ζ�ʱ��ʹ��, ��ʱʱ�䵽��ִ�лص�����������ͬʱ����4����ʱ�����������š�
*             ��ʱ��������10us ����Ҫ�ķ��ڵ��ñ�������ִ��ʱ�䣬�����ڲ������˲�����С��
*			 TIM2��TIM5 ��16λ��ʱ����
*			 TIM3��TIM4 ��16λ��ʱ����
*	��    ��: _CC : ����ͨ������1��2��3, 4
*             _uiTimeOut : ��ʱʱ��, ��λ 1us.       ����16λ��ʱ������� 65.5ms; ����32λ��ʱ������� 4294��
*             _pCallBack : ��ʱʱ�䵽�󣬱�ִ�еĺ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_StartHardTimer(uint8_t _CC, uint32_t _uiTimeOut, void * _pCallBack)
{
	uint32_t cnt_now;
    uint32_t cnt_tar;
	
	/*
        ִ�����������䣬ʱ�� = 18us (ͨ���߼������ǲ���IO��ת)
        bsp_StartTimer2(3, 500, (void *)test1);
    */
    if (_uiTimeOut < 5)
    {
        ;
    }
    else
    {
        _uiTimeOut -= 5;
    }

    cnt_now = TIM_GetCounter(TIM3);    	/* ��ȡ��ǰ�ļ�����ֵ */
    cnt_tar = cnt_now + _uiTimeOut;			/* ���㲶��ļ�����ֵ */
    if (_CC == 1)
    {
        s_TIM_CallBack1 = (void (*)(void))_pCallBack;

        TIM_SetCompare1(TIM3, cnt_tar);      	/* ���ò���Ƚϼ�����CC1 */
        TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);
        TIM_ITConfig(TIM3, TIM_IT_CC1, ENABLE);	/* ʹ��CC1�ж� */

    }
    else if (_CC == 2)
    {
        s_TIM_CallBack2 = (void (*)(void))_pCallBack;

        TIM_SetCompare2(TIM3, cnt_tar);      	/* ���ò���Ƚϼ�����CC2 */
        TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);
        TIM_ITConfig(TIM3, TIM_IT_CC2, ENABLE);	/* ʹ��CC2�ж� */
    }
    else if (_CC == 3)
    {
        s_TIM_CallBack3 = (void (*)(void))_pCallBack;

        TIM_SetCompare3(TIM3, cnt_tar);      	/* ���ò���Ƚϼ�����CC3 */
        TIM_ClearITPendingBit(TIM3, TIM_IT_CC3);
        TIM_ITConfig(TIM3, TIM_IT_CC3, ENABLE);	/* ʹ��CC3�ж� */
    }
    else if (_CC == 4)
    {
        s_TIM_CallBack4 = (void (*)(void))_pCallBack;

        TIM_SetCompare4(TIM3, cnt_tar);      	/* ���ò���Ƚϼ�����CC4 */
        TIM_ClearITPendingBit(TIM3, TIM_IT_CC4);
        TIM_ITConfig(TIM3, TIM_IT_CC4, ENABLE);	/* ʹ��CC4�ж� */
    }
    else
    {
        return;
    }
}
/*
*********************************************************************************************************
*	�� �� ��: bsp_StartAutoTimer
*	����˵��: ����һ���Զ���ʱ���������ö�ʱ���ڡ�
*	��    ��:  	_id     : ��ʱ��ID��ֵ��0,TMR_COUNT-1�����û���������ά����ʱ��ID���Ա��ⶨʱ��ID��ͻ��
*				_period : ��ʱ���ڣ���λ10ms
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_StartAutoTimer(uint8_t _id, uint32_t _period)
{
    if (_id >= TMR_COUNT)
    {
        /* ��ӡ�����Դ�����ļ������������� */
//        BSP_Printf("Error: file %s, function %s()\r\n", __FILE__, __FUNCTION__);
        while(1); /* �����쳣�������ȴ����Ź���λ */
    }

    DISABLE_INT();  		/* ���ж� */

    s_tTmr[_id].Count = _period;			/* ʵʱ��������ֵ */
    s_tTmr[_id].PreLoad = _period;		/* �������Զ���װֵ�����Զ�ģʽ������ */
    s_tTmr[_id].Flag = 0;				/* ��ʱʱ�䵽��־ */
    s_tTmr[_id].Mode = TMR_AUTO_MODE;	/* �Զ�����ģʽ */

    ENABLE_INT();  			/* ���ж� */
}
/*
*********************************************************************************************************
*	�� �� ��: bsp_StopTimer
*	����˵��: ֹͣһ����ʱ��
*	��    ��:  	_id     : ��ʱ��ID��ֵ��0,TMR_COUNT-1�����û���������ά����ʱ��ID���Ա��ⶨʱ��ID��ͻ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_StopTimer(uint8_t _id)
{
    if (_id >= TMR_COUNT)
    {
        /* ��ӡ�����Դ�����ļ������������� */
//        BSP_Printf("Error: file %s, function %s()\r\n", __FILE__, __FUNCTION__);
        while(1); /* �����쳣�������ȴ����Ź���λ */
    }

    DISABLE_INT();  	/* ���ж� */

    s_tTmr[_id].Count = 0;				/* ʵʱ��������ֵ */
    s_tTmr[_id].Flag = 0;				/* ��ʱʱ�䵽��־ */
    s_tTmr[_id].Mode = TMR_ONCE_MODE;	/* �Զ�����ģʽ */

    ENABLE_INT();  		/* ���ж� */
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_CheckTimer
*	����˵��: ��ⶨʱ���Ƿ�ʱ
*	��    ��:  	_id     : ��ʱ��ID��ֵ��0,TMR_COUNT-1�����û���������ά����ʱ��ID���Ա��ⶨʱ��ID��ͻ��
*				_period : ��ʱ���ڣ���λ1ms
*	�� �� ֵ: ���� 0 ��ʾ��ʱδ���� 1��ʾ��ʱ��
*********************************************************************************************************
*/
uint8_t bsp_CheckTimer(uint8_t _id)
{
    if (_id >= TMR_COUNT)
    {
        return 0;
    }

    if (s_tTmr[_id].Flag == 1)
    {
        s_tTmr[_id].Flag = 0;
        return 1;
    }
    else
    {
        return 0;
    }
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_GetRunTime
*	����˵��: ��ȡCPU����ʱ�䣬��λ1ms������Ա�ʾ 24.85�죬�����Ĳ�Ʒ��������ʱ�䳬�������������뿼���������
*	��    ��:  ��
*	�� �� ֵ: CPU����ʱ�䣬��λ1ms
*********************************************************************************************************
*/
int32_t bsp_GetRunTime(void)
{
    int32_t runtime;

    DISABLE_INT();  	/* ���ж� */

    runtime = g_iRunTime;	/* ���������Systick�ж��б���д�������Ҫ���жϽ��б��� */

    ENABLE_INT();  		/* ���ж� */

    return runtime;
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_CheckRunTime
*	����˵��: ���㵱ǰ����ʱ��͸���ʱ��֮��Ĳ�ֵ�������˼�����ѭ����
*	��    ��:  _LastTime �ϸ�ʱ��
*	�� �� ֵ: ��ǰʱ��͹�ȥʱ��Ĳ�ֵ����λ1ms
*********************************************************************************************************
*/
uint32_t bsp_CheckRunTime(uint32_t _LastTime)
{
    uint32_t now_time;
    uint32_t time_diff;

    DISABLE_INT();  	/* ���ж� */

    now_time = g_iRunTime;	/* ���������Systick�ж��б���д�������Ҫ���жϽ��б��� */

    ENABLE_INT();  		/* ���ж� */

    if (now_time >= _LastTime)
    {
        time_diff = now_time - _LastTime;
    }
    else
    {
        time_diff = 0xFFFFFFFF - _LastTime + now_time;
    }

    return time_diff;
}


void TIM3_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM3, TIM_IT_CC1))
    {
        TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);
        TIM_ITConfig(TIM3, TIM_IT_CC1, DISABLE);	/* ����CC1�ж� */

        /* �ȹر��жϣ���ִ�лص���������Ϊ�ص�����������Ҫ������ʱ�� */
        s_TIM_CallBack1();
    }

    if (TIM_GetITStatus(TIM3, TIM_IT_CC2))
    {
        TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);
        TIM_ITConfig(TIM3, TIM_IT_CC2, DISABLE);	/* ����CC2�ж� */

        /* �ȹر��жϣ���ִ�лص���������Ϊ�ص�����������Ҫ������ʱ�� */
        s_TIM_CallBack2();
    }

    if (TIM_GetITStatus(TIM3, TIM_IT_CC3))
    {
        TIM_ClearITPendingBit(TIM3, TIM_IT_CC3);
        TIM_ITConfig(TIM3, TIM_IT_CC3, DISABLE);	/* ����CC3�ж� */

        /* �ȹر��жϣ���ִ�лص���������Ϊ�ص�����������Ҫ������ʱ�� */
        s_TIM_CallBack3();
    }

    if (TIM_GetITStatus(TIM3, TIM_IT_CC4))
    {
        TIM_ClearITPendingBit(TIM3, TIM_IT_CC4);
        TIM_ITConfig(TIM3, TIM_IT_CC4, DISABLE);	/* ����CC4�ж� */

        /* �ȹر��жϣ���ִ�лص���������Ϊ�ص�����������Ҫ������ʱ�� */
        s_TIM_CallBack4();
    }
}
	


void bsp_systick_init(void)
{
    SysTick_Config(SystemCoreClock / 10000);
}
/*
*********************************************************************************************************
*	�� �� ��: bsp_SoftTimerDec
*	����˵��: ÿ��1ms�����ж�ʱ��������1�����뱻SysTick_ISR�����Ե��á�
*	��    ��:  _tmr : ��ʱ������ָ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void bsp_SoftTimerDec(SOFT_TMR *_tmr)
{
    if (_tmr->Count > 0)
    {
        /* �����ʱ����������1�����ö�ʱ�������־ */
        if (--_tmr->Count == 0)
        {
            _tmr->Flag = 1;

            /* ������Զ�ģʽ�����Զ���װ������ */
            if(_tmr->Mode == TMR_AUTO_MODE)
            {
                _tmr->Count = _tmr->PreLoad;
            }
        }
    }
}
void SysTick_ISR(void)
{
	uint8_t i;
    /* ÿ��1ms����1�� �������� bsp_DelayMS�� */
    if (s_uiDelayCount > 0)
    {
        if (--s_uiDelayCount == 0)
        {
            s_ucTimeOutFlag = 1;
        }
    }

    /* ÿ��1ms���������ʱ���ļ��������м�һ���� */
    for (i = 0; i < TMR_COUNT; i++)
    {
        bsp_SoftTimerDec(&s_tTmr[i]);
    }

    /* ȫ������ʱ��ÿ1ms��1 */
    g_iRunTime++;

    if (g_iRunTime == 0xFFFFFFFF)	/* ��������� int32_t ���ͣ������Ϊ 0x7FFFFFFF */
    {
        g_iRunTime = 0;
    }

//    bsp_RunPer1ms();		/* ÿ��1ms����һ�δ˺������˺����� bsp.c */

//    if (s_count % 10==0)
//    {
//        bsp_RunPer10ms();	/* ÿ��10ms����һ�δ˺������˺����� bsp.c */
//    }
//    if(s_count %1000==0){
//        bsp_RunPer1s();
//        s_count=1;
//    }
}
void SysTick_Handler(void)
{
    SysTick_ISR();

}

void bsp_DelayUS(uint32_t n)
{
    uint32_t ticks;
    uint32_t told;
    uint32_t tnow;
    uint32_t tcnt = 0;
    uint32_t reload;

    reload = SysTick->LOAD;
    ticks = n * (SystemCoreClock / 1000000);	 /* ��Ҫ�Ľ����� */
    tcnt = 0;
    told = SysTick->VAL;             /* �ս���ʱ�ļ�����ֵ */
    while (1)
    {
        tnow = SysTick->VAL;
        if (tnow != told)
        {
            /* SYSTICK��һ���ݼ��ļ����� */
            if (tnow < told)
            {
                tcnt += told - tnow;
            }
            /* ����װ�صݼ� */
            else
            {
                tcnt += reload - tnow + told;
            }
            told = tnow;
            /* ʱ�䳬��/����Ҫ�ӳٵ�ʱ��,���˳� */
            if (tcnt >= ticks)
            {
                break;
            }
        }
    }
}
void bsp_DelayMS(uint32_t n)
{
    if (n == 0)
    {
        return;
    }
    else if (n == 1)
    {
        n = 2;
    }

    DISABLE_INT();  			/* ���ж� */

    s_uiDelayCount = n;
    s_ucTimeOutFlag = 0;

    ENABLE_INT();  				/* ���ж� */

    while (1)
    {
//        bsp_Idle();				/* CPU����ִ�еĲ����� �� bsp.c �� bsp.h �ļ� */
        /*
        	�ȴ��ӳ�ʱ�䵽
        	ע�⣺��������Ϊ s_ucTimeOutFlag = 0�����Կ����Ż�������� s_ucTimeOutFlag ������������Ϊ volatile
        */
        if (s_ucTimeOutFlag == 1)
        {
            break;
        }
    }

}


