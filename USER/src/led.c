#include "led.h"
#include "stm32f10x.h"
#include "delay.h"

void LED_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_SetBits(GPIOC, GPIO_Pin_13); // 熄灭LED
}

void LED_Blink(uint8_t times)
{
    uint8_t i;
    for(i=0; i<times; i++)
    {
        GPIO_ResetBits(GPIOC, GPIO_Pin_13); // 点亮
        delay_ms(300);
        GPIO_SetBits(GPIOC, GPIO_Pin_13);   // 熄灭
        delay_ms(300);
    }
} 
