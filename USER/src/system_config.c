#include "system_config.h"
#include "stm32f10x_flash.h"  // 添加FLASH相关头文件

// 系统时钟变量
uint32_t SystemCoreClock = SYSTEM_CLOCK_FREQ;

// 系统时钟配置
void SystemClock_Config(void)
{
    // 复位RCC时钟配置
    RCC_DeInit();
    
    // 使能HSE
    RCC_HSEConfig(RCC_HSE_ON);
    
    // 等待HSE就绪
    if(RCC_WaitForHSEStartUp() == SUCCESS)
    {
        // 配置FLASH等待状态
        FLASH_SetLatency(FLASH_Latency_2);
        FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
        
        // 配置PLL
        RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);
        
        // 使能PLL
        RCC_PLLCmd(ENABLE);
        
        // 等待PLL就绪
        while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);
        
        // 选择PLL作为系统时钟
        RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
        
        // 等待PLL作为系统时钟
        while(RCC_GetSYSCLKSource() != 0x08);
        
        // 配置AHB、APB1、APB2分频
        RCC_HCLKConfig(RCC_SYSCLK_Div1);
        RCC_PCLK1Config(RCC_HCLK_Div2);
        RCC_PCLK2Config(RCC_HCLK_Div1);
        
        // 更新系统时钟频率变量
        SystemCoreClock = SYSTEM_CLOCK_FREQ;
    }
    else
    {
        // HSE启动失败，使用内部HSI
        while(1);
    }
}
