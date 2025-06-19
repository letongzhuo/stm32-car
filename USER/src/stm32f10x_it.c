#include "stm32f10x_it.h"
#include "stm32f10x.h"

// 外部变量声明
extern volatile uint32_t g_systick_count;

void SysTick_Handler(void)
{
    // 系统滴答定时器中断处理
    // 每1ms触发一次
    g_systick_count++;
} 