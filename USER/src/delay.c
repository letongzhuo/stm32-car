#include "delay.h"

// 初始化延迟函数
// SYSTICK的时钟固定为HCLK时钟的1/8
// SYSCLK:系统时钟
void delay_init(u8 SYSCLK)
{
    // 不需要配置SysTick，因为main函数已经配置了
    (void)SYSCLK;  // 防止编译警告
}

// 延时nms
// 注意nms的范围
// SysTick->LOAD为24位寄存器,所以,最大延时为:
// nms<=0xffffff*8*1000/SYSCLK
// SYSCLK单位为Hz,nms单位为ms
// 对72M条件下,nms<=1864 
void delay_ms(u16 nms)
{	 		  	  
    uint32_t start = g_systick_count;
    while(g_systick_count - start < nms)
    {
        // 等待时间到达
    }
}

// 延时nus
// nus为要延时的us数.		    								   
void delay_us(u32 nus)
{		
    uint32_t start, now, load, count;
    
    // 关闭中断，确保计时准确
    __disable_irq();
    
    load = SysTick->LOAD;
    start = SysTick->VAL;
    
    // 计算需要的计数值，考虑到72MHz主频，每1us计数72次
    count = nus * (SystemCoreClock / 1000000);
    
    do
    {
        now = SysTick->VAL;
        // 如果发生了重载
        if(now > start)
        {
            now = now - start;
        }
        else
        {
            now = load - start + now;
        }
    } while(now < count);
    
    // 重新使能中断
    __enable_irq();
}
