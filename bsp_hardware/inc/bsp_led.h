#ifndef __BSP_LED_H
#define __BSP_LED_H

#include "stm32f10x.h"


#define LEDR_CLK  RCC_APB2Periph_GPIOA
#define LEDR_PORT GPIOA
#define LEDR_Pin  GPIO_Pin_0

#define LEDG_CLK  RCC_APB2Periph_GPIOA
#define LEDG_PORT GPIOA
#define LEDG_Pin  GPIO_Pin_1

#define LEDB_CLK  RCC_APB2Periph_GPIOA
#define LEDB_PORT GPIOA
#define LEDB_Pin  GPIO_Pin_2

typedef enum
{
    RED=0,
    GREEN,
    BLUE,
    YELLOW
}RGB_Color;

void RGB_PWM_Init(void);
void RGB_Light(uint16_t color);
void led_demo(void);
#endif
