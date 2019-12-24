#include "bsp_led.h"
uint16_t Color[10][3] =
{
    {0, 1000, 1000},
    {1000, 0, 1000},
    {1000, 1000, 0},
    {0, 550, 1000}, //��ɫ

};
void RGB_GPIO_PWM_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		    // �����������
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
    TIM_TimeBaseStructure.TIM_Period = 999;       //����ʱ����0������999����Ϊ1000�Σ�Ϊһ����ʱ����
    TIM_TimeBaseStructure.TIM_Prescaler = 71;	    //����Ԥ��Ƶ����Ԥ��Ƶ����Ϊ72MHz
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1 ;	//����ʱ�ӷ�Ƶϵ��������Ƶ(�����ò���)
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //���ϼ���ģʽ
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    /* PWM1 Mode configuration: Channel1 */
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;	    //����ΪPWMģʽ1
    
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = CCR1_Val;	   //��������ֵ�������������������ֵʱ����ƽ��������
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;  //����ʱ������ֵС��CCR1_ValʱΪ�ߵ�ƽ
    TIM_OC1Init(TIM2, &TIM_OCInitStructure);	 //ʹ��ͨ��1
    TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);

    /* PWM1 Mode configuration: Channel2 */
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = CCR2_Val;	  //����ͨ��2�ĵ�ƽ����ֵ���������һ��ռ�ձȵ�PWM
    TIM_OC2Init(TIM2, &TIM_OCInitStructure);	  //ʹ��ͨ��2
    TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);

    /* PWM1 Mode configuration: Channel3 */
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = CCR3_Val;	//����ͨ��3�ĵ�ƽ����ֵ���������һ��ռ�ձȵ�PWM
    TIM_OC3Init(TIM2, &TIM_OCInitStructure);	 //ʹ��ͨ��3
    TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);

    /* PWM1 Mode configuration: Channel4 */
//    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
//    TIM_OCInitStructure.TIM_Pulse = CCR4_Val;	//����ͨ��4�ĵ�ƽ����ֵ���������һ��ռ�ձȵ�PWM
//    TIM_OC4Init(TIM2, &TIM_OCInitStructure);	//ʹ��ͨ��4
//    TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM2, ENABLE);			 // ʹ��TIM3���ؼĴ���ARR

    /* TIM3 enable counter */
    TIM_Cmd(TIM2, ENABLE);                   //ʹ�ܶ�ʱ��3
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