#include "bsp_led.h"
uint16_t Color[10][3] =
{
    {0, 1000, 1000},
    {1000, 0, 1000},
    {1000, 1000, 0},
    {0, 550, 1000}, //黄色

};
void RGB_GPIO_PWM_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		    // 复用推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = LEDR_Pin;
    GPIO_Init(LEDR_PORT, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = LEDG_Pin;
    GPIO_Init(LEDR_PORT, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = LEDB_Pin;
    GPIO_Init(LEDR_PORT, &GPIO_InitStructure);

}
void RGB_TIM_PWM_Init(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;

    uint16_t CCR1_Val = 1000;      //R
    uint16_t CCR2_Val = 1000;      //G
    uint16_t CCR3_Val = 1000;     //B
    uint16_t CCR4_Val = 1000;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    /* Time base configuration */
    TIM_TimeBaseStructure.TIM_Period = 999;       //当定时器从0计数到999，即为1000次，为一个定时周期
    TIM_TimeBaseStructure.TIM_Prescaler = 71;	    //设置预分频：不预分频，即为72MHz
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1 ;	//设置时钟分频系数：不分频(这里用不到)
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //向上计数模式
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    /* PWM1 Mode configuration: Channel1 */
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;	    //配置为PWM模式1
    
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = CCR1_Val;	   //设置跳变值，当计数器计数到这个值时，电平发生跳变
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;  //当定时器计数值小于CCR1_Val时为高电平
    TIM_OC1Init(TIM2, &TIM_OCInitStructure);	 //使能通道1
    TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);

    /* PWM1 Mode configuration: Channel2 */
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = CCR2_Val;	  //设置通道2的电平跳变值，输出另外一个占空比的PWM
    TIM_OC2Init(TIM2, &TIM_OCInitStructure);	  //使能通道2
    TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);

    /* PWM1 Mode configuration: Channel3 */
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = CCR3_Val;	//设置通道3的电平跳变值，输出另外一个占空比的PWM
    TIM_OC3Init(TIM2, &TIM_OCInitStructure);	 //使能通道3
    TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);

    /* PWM1 Mode configuration: Channel4 */
//    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
//    TIM_OCInitStructure.TIM_Pulse = CCR4_Val;	//设置通道4的电平跳变值，输出另外一个占空比的PWM
//    TIM_OC4Init(TIM2, &TIM_OCInitStructure);	//使能通道4
//    TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM2, ENABLE);			 // 使能TIM3重载寄存器ARR

    /* TIM3 enable counter */
    TIM_Cmd(TIM2, ENABLE);                   //使能定时器3
}

void RGB_PWM_Init(void)
{
    RGB_GPIO_PWM_Init();
    RGB_TIM_PWM_Init();
    TIM2->CCR1 = 500;
    TIM2->CCR2 = 1000;
    TIM2->CCR3 = 1000;
}
void RGB_Light(uint16_t color)
{
    TIM2->CCR1 = Color[color][0];
    TIM2->CCR2 = Color[color][1];
    TIM2->CCR3 = Color[color][2];
}
void led_demo(void)
{
	RGB_Light(GREEN);
	printf("led_demo");
}