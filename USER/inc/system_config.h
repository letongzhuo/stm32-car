#ifndef __SYSTEM_CONFIG_H
#define __SYSTEM_CONFIG_H

#include "stm32f10x.h"

// 系统时钟频率（72MHz）
#define SYSTEM_CLOCK_FREQ    72000000

// 系统时钟变量
extern uint32_t SystemCoreClock;

// 系统时钟配置函数
void SystemClock_Config(void);

// 系统滴答定时器中断处理函数
void SysTick_Handler(void);

#endif /* __SYSTEM_CONFIG_H */ 
