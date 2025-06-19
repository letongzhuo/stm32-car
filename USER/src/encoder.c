#include "encoder.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_exti.h"
#include "misc.h"
#include "core_cm3.h"
#include "system_config.h"

// 编码器计数 (仅左轮)
static int32_t encoder_count[1] = {0};

// 初始化编码器 (仅左轮)
void Encoder_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    
    // 使能GPIOB时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
    
    // 配置GPIO引脚为输入模式 (仅左轮)
    GPIO_InitStructure.GPIO_Pin = ENCODER_LEFT_PIN; // 只配置左轮引脚
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(ENCODER_LEFT_PORT, &GPIO_InitStructure);
    
    // 配置外部中断 (仅左轮)
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource8); // 只配置左轮中断线
    
    EXTI_InitStructure.EXTI_Line = EXTI_Line8; // 只使能左轮中断线
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
    
    // 配置NVIC (EXTI Line 5-9共用中断，所以还是EXTI9_5_IRQn)
    NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    

    // 重置编码器计数
    encoder_count[0] = 0;
}

// 重置编码器计数 (仅左轮)
void Encoder_Reset(void)
{
    encoder_count[0] = 0;
}

// 获取编码器计数值 (仅左轮)
int16_t Encoder_GetCount(uint8_t encoder)
{
    if(encoder == 0) // 只处理左轮
        return encoder_count[0];
    return 0; // 右轮返回0
}

// 更新编码器计数（中断处理函数调用）(仅左轮)
void Encoder_Update(uint8_t encoder)
{
    if(encoder == 0) // 只处理左轮
    {
        // 根据引脚状态判断方向
        if(GPIO_ReadInputDataBit(ENCODER_LEFT_PORT, ENCODER_LEFT_PIN)) // 左轮引脚
            encoder_count[0]++;
    }
}

// 外部中断服务函数 (EXTI Line 5-9)
void EXTI9_5_IRQHandler(void)
{
    // 判断是哪个中断线触发
    if(EXTI_GetITStatus(EXTI_Line8) != RESET) // 左轮编码器中断
    {
        Encoder_Update(0); // 更新左轮计数
        EXTI_ClearITPendingBit(EXTI_Line8); // 清除中断标志位
    }
    // 如果有其他中断线共用此服务函数，可以在这里添加判断
}
