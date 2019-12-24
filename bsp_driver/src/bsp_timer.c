#include "bsp_timer.h"
#include "bsp.h"

__IO uint32_t g_iRunTime = 0;
static volatile uint32_t s_uiDelayCount = 0;
static volatile uint8_t s_ucTimeOutFlag = 0;

/* 定于软件定时器结构体变量 */
static SOFT_TMR s_tTmr[TMR_COUNT];
/* 保存 TIM定时中断到后执行的回调函数指针 */
static void (*s_TIM_CallBack1)(void);
static void (*s_TIM_CallBack2)(void);
static void (*s_TIM_CallBack3)(void);
static void (*s_TIM_CallBack4)(void);

void bsp_timer_init(void)
{
    uint8_t i;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;	/* 中断结构体在 misc.h 中定义 */
    uint32_t usPeriod;
    uint16_t usPrescaler;
    uint32_t uiTIMxCLK;
    /* 清零所有的软件定时器 */
    for (i = 0; i < TMR_COUNT; i++)
    {
        s_tTmr[i].Count = 0;
        s_tTmr[i].PreLoad = 0;
        s_tTmr[i].Flag = 0;
        s_tTmr[i].Mode = TMR_ONCE_MODE;	/* 缺省是1次性工作模式 */
    }
	
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    uiTIMxCLK = SystemCoreClock / 2;
    usPrescaler = uiTIMxCLK / 1000000 ;	/* 分频到周期 1us */
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
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;	/* 比串口优先级低 */
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

}

/*
*********************************************************************************************************
*	函 数 名: bsp_StartHardTimer
*	功能说明: 使用TIM2-5做单次定时器使用, 定时时间到后执行回调函数。可以同时启动4个定时器，互不干扰。
*             定时精度正负10us （主要耗费在调用本函数的执行时间，函数内部进行了补偿减小误差）
*			 TIM2和TIM5 是16位定时器。
*			 TIM3和TIM4 是16位定时器。
*	形    参: _CC : 捕获通道几，1，2，3, 4
*             _uiTimeOut : 超时时间, 单位 1us.       对于16位定时器，最大 65.5ms; 对于32位定时器，最大 4294秒
*             _pCallBack : 定时时间到后，被执行的函数
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_StartHardTimer(uint8_t _CC, uint32_t _uiTimeOut, void * _pCallBack)
{
	uint32_t cnt_now;
    uint32_t cnt_tar;
	
	/*
        执行下面这个语句，时长 = 18us (通过逻辑分析仪测量IO翻转)
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

    cnt_now = TIM_GetCounter(TIM3);    	/* 读取当前的计数器值 */
    cnt_tar = cnt_now + _uiTimeOut;			/* 计算捕获的计数器值 */
    if (_CC == 1)
    {
        s_TIM_CallBack1 = (void (*)(void))_pCallBack;

        TIM_SetCompare1(TIM3, cnt_tar);      	/* 设置捕获比较计数器CC1 */
        TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);
        TIM_ITConfig(TIM3, TIM_IT_CC1, ENABLE);	/* 使能CC1中断 */

    }
    else if (_CC == 2)
    {
        s_TIM_CallBack2 = (void (*)(void))_pCallBack;

        TIM_SetCompare2(TIM3, cnt_tar);      	/* 设置捕获比较计数器CC2 */
        TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);
        TIM_ITConfig(TIM3, TIM_IT_CC2, ENABLE);	/* 使能CC2中断 */
    }
    else if (_CC == 3)
    {
        s_TIM_CallBack3 = (void (*)(void))_pCallBack;

        TIM_SetCompare3(TIM3, cnt_tar);      	/* 设置捕获比较计数器CC3 */
        TIM_ClearITPendingBit(TIM3, TIM_IT_CC3);
        TIM_ITConfig(TIM3, TIM_IT_CC3, ENABLE);	/* 使能CC3中断 */
    }
    else if (_CC == 4)
    {
        s_TIM_CallBack4 = (void (*)(void))_pCallBack;

        TIM_SetCompare4(TIM3, cnt_tar);      	/* 设置捕获比较计数器CC4 */
        TIM_ClearITPendingBit(TIM3, TIM_IT_CC4);
        TIM_ITConfig(TIM3, TIM_IT_CC4, ENABLE);	/* 使能CC4中断 */
    }
    else
    {
        return;
    }
}
/*
*********************************************************************************************************
*	函 数 名: bsp_StartAutoTimer
*	功能说明: 启动一个自动定时器，并设置定时周期。
*	形    参:  	_id     : 定时器ID，值域【0,TMR_COUNT-1】。用户必须自行维护定时器ID，以避免定时器ID冲突。
*				_period : 定时周期，单位10ms
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_StartAutoTimer(uint8_t _id, uint32_t _period)
{
    if (_id >= TMR_COUNT)
    {
        /* 打印出错的源代码文件名、函数名称 */
//        BSP_Printf("Error: file %s, function %s()\r\n", __FILE__, __FUNCTION__);
        while(1); /* 参数异常，死机等待看门狗复位 */
    }

    DISABLE_INT();  		/* 关中断 */

    s_tTmr[_id].Count = _period;			/* 实时计数器初值 */
    s_tTmr[_id].PreLoad = _period;		/* 计数器自动重装值，仅自动模式起作用 */
    s_tTmr[_id].Flag = 0;				/* 定时时间到标志 */
    s_tTmr[_id].Mode = TMR_AUTO_MODE;	/* 自动工作模式 */

    ENABLE_INT();  			/* 开中断 */
}
/*
*********************************************************************************************************
*	函 数 名: bsp_StopTimer
*	功能说明: 停止一个定时器
*	形    参:  	_id     : 定时器ID，值域【0,TMR_COUNT-1】。用户必须自行维护定时器ID，以避免定时器ID冲突。
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_StopTimer(uint8_t _id)
{
    if (_id >= TMR_COUNT)
    {
        /* 打印出错的源代码文件名、函数名称 */
//        BSP_Printf("Error: file %s, function %s()\r\n", __FILE__, __FUNCTION__);
        while(1); /* 参数异常，死机等待看门狗复位 */
    }

    DISABLE_INT();  	/* 关中断 */

    s_tTmr[_id].Count = 0;				/* 实时计数器初值 */
    s_tTmr[_id].Flag = 0;				/* 定时时间到标志 */
    s_tTmr[_id].Mode = TMR_ONCE_MODE;	/* 自动工作模式 */

    ENABLE_INT();  		/* 开中断 */
}

/*
*********************************************************************************************************
*	函 数 名: bsp_CheckTimer
*	功能说明: 检测定时器是否超时
*	形    参:  	_id     : 定时器ID，值域【0,TMR_COUNT-1】。用户必须自行维护定时器ID，以避免定时器ID冲突。
*				_period : 定时周期，单位1ms
*	返 回 值: 返回 0 表示定时未到， 1表示定时到
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
*	函 数 名: bsp_GetRunTime
*	功能说明: 获取CPU运行时间，单位1ms。最长可以表示 24.85天，如果你的产品连续运行时间超过这个数，则必须考虑溢出问题
*	形    参:  无
*	返 回 值: CPU运行时间，单位1ms
*********************************************************************************************************
*/
int32_t bsp_GetRunTime(void)
{
    int32_t runtime;

    DISABLE_INT();  	/* 关中断 */

    runtime = g_iRunTime;	/* 这个变量在Systick中断中被改写，因此需要关中断进行保护 */

    ENABLE_INT();  		/* 开中断 */

    return runtime;
}

/*
*********************************************************************************************************
*	函 数 名: bsp_CheckRunTime
*	功能说明: 计算当前运行时间和给定时刻之间的差值。处理了计数器循环。
*	形    参:  _LastTime 上个时刻
*	返 回 值: 当前时间和过去时间的差值，单位1ms
*********************************************************************************************************
*/
uint32_t bsp_CheckRunTime(uint32_t _LastTime)
{
    uint32_t now_time;
    uint32_t time_diff;

    DISABLE_INT();  	/* 关中断 */

    now_time = g_iRunTime;	/* 这个变量在Systick中断中被改写，因此需要关中断进行保护 */

    ENABLE_INT();  		/* 开中断 */

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
        TIM_ITConfig(TIM3, TIM_IT_CC1, DISABLE);	/* 禁能CC1中断 */

        /* 先关闭中断，再执行回调函数。因为回调函数可能需要重启定时器 */
        s_TIM_CallBack1();
    }

    if (TIM_GetITStatus(TIM3, TIM_IT_CC2))
    {
        TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);
        TIM_ITConfig(TIM3, TIM_IT_CC2, DISABLE);	/* 禁能CC2中断 */

        /* 先关闭中断，再执行回调函数。因为回调函数可能需要重启定时器 */
        s_TIM_CallBack2();
    }

    if (TIM_GetITStatus(TIM3, TIM_IT_CC3))
    {
        TIM_ClearITPendingBit(TIM3, TIM_IT_CC3);
        TIM_ITConfig(TIM3, TIM_IT_CC3, DISABLE);	/* 禁能CC3中断 */

        /* 先关闭中断，再执行回调函数。因为回调函数可能需要重启定时器 */
        s_TIM_CallBack3();
    }

    if (TIM_GetITStatus(TIM3, TIM_IT_CC4))
    {
        TIM_ClearITPendingBit(TIM3, TIM_IT_CC4);
        TIM_ITConfig(TIM3, TIM_IT_CC4, DISABLE);	/* 禁能CC4中断 */

        /* 先关闭中断，再执行回调函数。因为回调函数可能需要重启定时器 */
        s_TIM_CallBack4();
    }
}
	


void bsp_systick_init(void)
{
    SysTick_Config(SystemCoreClock / 10000);
}
/*
*********************************************************************************************************
*	函 数 名: bsp_SoftTimerDec
*	功能说明: 每隔1ms对所有定时器变量减1。必须被SysTick_ISR周期性调用。
*	形    参:  _tmr : 定时器变量指针
*	返 回 值: 无
*********************************************************************************************************
*/
static void bsp_SoftTimerDec(SOFT_TMR *_tmr)
{
    if (_tmr->Count > 0)
    {
        /* 如果定时器变量减到1则设置定时器到达标志 */
        if (--_tmr->Count == 0)
        {
            _tmr->Flag = 1;

            /* 如果是自动模式，则自动重装计数器 */
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
    /* 每隔1ms进来1次 （仅用于 bsp_DelayMS） */
    if (s_uiDelayCount > 0)
    {
        if (--s_uiDelayCount == 0)
        {
            s_ucTimeOutFlag = 1;
        }
    }

    /* 每隔1ms，对软件定时器的计数器进行减一操作 */
    for (i = 0; i < TMR_COUNT; i++)
    {
        bsp_SoftTimerDec(&s_tTmr[i]);
    }

    /* 全局运行时间每1ms增1 */
    g_iRunTime++;

    if (g_iRunTime == 0xFFFFFFFF)	/* 这个变量是 int32_t 类型，最大数为 0x7FFFFFFF */
    {
        g_iRunTime = 0;
    }

//    bsp_RunPer1ms();		/* 每隔1ms调用一次此函数，此函数在 bsp.c */

//    if (s_count % 10==0)
//    {
//        bsp_RunPer10ms();	/* 每隔10ms调用一次此函数，此函数在 bsp.c */
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
    ticks = n * (SystemCoreClock / 1000000);	 /* 需要的节拍数 */
    tcnt = 0;
    told = SysTick->VAL;             /* 刚进入时的计数器值 */
    while (1)
    {
        tnow = SysTick->VAL;
        if (tnow != told)
        {
            /* SYSTICK是一个递减的计数器 */
            if (tnow < told)
            {
                tcnt += told - tnow;
            }
            /* 重新装载递减 */
            else
            {
                tcnt += reload - tnow + told;
            }
            told = tnow;
            /* 时间超过/等于要延迟的时间,则退出 */
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

    DISABLE_INT();  			/* 关中断 */

    s_uiDelayCount = n;
    s_ucTimeOutFlag = 0;

    ENABLE_INT();  				/* 开中断 */

    while (1)
    {
//        bsp_Idle();				/* CPU空闲执行的操作， 见 bsp.c 和 bsp.h 文件 */
        /*
        	等待延迟时间到
        	注意：编译器认为 s_ucTimeOutFlag = 0，所以可能优化错误，因此 s_ucTimeOutFlag 变量必须申明为 volatile
        */
        if (s_ucTimeOutFlag == 1)
        {
            break;
        }
    }

}


