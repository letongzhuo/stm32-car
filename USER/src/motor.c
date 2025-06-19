#include "motor.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"

// 初始化电机控制引脚
void Motor_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
    
    // 使能GPIOA时钟和复用功能时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
    // 使能TIM3时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    
    // 配置GPIO引脚为推挽输出（方向控制引脚）
    GPIO_InitStructure.GPIO_Pin = MOTOR_LEFT_IN1_PIN | MOTOR_LEFT_IN2_PIN |
                                 MOTOR_RIGHT_IN1_PIN | MOTOR_RIGHT_IN2_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  // 普通推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // 重新配置使能引脚为复用推挽输出（PWM输出）
    GPIO_InitStructure.GPIO_Pin = MOTOR_LEFT_EN_PIN | MOTOR_RIGHT_EN_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  // 复用推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // 配置定时器基本参数
    TIM_TimeBaseStructure.TIM_Period = 999;  // ARR = 999
    TIM_TimeBaseStructure.TIM_Prescaler = 71;  // PSC = 71
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
    
    // 配置PWM模式
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;  // 设置初始占空比为0%
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    
    // 配置两个通道（用于使能引脚）
    TIM_OC1Init(TIM3, &TIM_OCInitStructure);  // PA6 (TIM3_CH1)
    TIM_OC2Init(TIM3, &TIM_OCInitStructure);  // PA7 (TIM3_CH2)
    
    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM3, ENABLE);
    
    // 启动定时器
    TIM_Cmd(TIM3, ENABLE);
}

// 设置电机速度
void Motor_SetSpeed(uint8_t motor, int8_t speed)
{
    // 限制速度范围
    if(speed > 100) speed = 100;
    if(speed < -100) speed = -100;
    
    // 设置电机方向和PWM占空比
    if(motor == 0)  // 左电机
    {
        if(speed > 0) {
            GPIO_SetBits(GPIOA, MOTOR_LEFT_IN1_PIN);
            GPIO_ResetBits(GPIOA, MOTOR_LEFT_IN2_PIN);
            TIM_SetCompare1(TIM3, speed * 10);  // PA6 (TIM3_CH1)
        } else if(speed < 0) {
            GPIO_ResetBits(GPIOA, MOTOR_LEFT_IN1_PIN);
            GPIO_SetBits(GPIOA, MOTOR_LEFT_IN2_PIN);
            TIM_SetCompare1(TIM3, -speed * 10);  // PA6 (TIM3_CH1)
        } else {
            GPIO_ResetBits(GPIOA, MOTOR_LEFT_IN1_PIN);
            GPIO_ResetBits(GPIOA, MOTOR_LEFT_IN2_PIN);
            TIM_SetCompare1(TIM3, 0);
        }
    }
    else  // 右电机
    {
        if(speed > 0) {
            GPIO_SetBits(GPIOA, MOTOR_RIGHT_IN1_PIN);
            GPIO_ResetBits(GPIOA, MOTOR_RIGHT_IN2_PIN);
            TIM_SetCompare2(TIM3, speed * 10);  // PA7 (TIM3_CH2)
        } else if(speed < 0) {
            GPIO_ResetBits(GPIOA, MOTOR_RIGHT_IN1_PIN);
            GPIO_SetBits(GPIOA, MOTOR_RIGHT_IN2_PIN);
            TIM_SetCompare2(TIM3, -speed * 10);  // PA7 (TIM3_CH2)
        } else {
            GPIO_ResetBits(GPIOA, MOTOR_RIGHT_IN1_PIN);
            GPIO_ResetBits(GPIOA, MOTOR_RIGHT_IN2_PIN);
            TIM_SetCompare2(TIM3, 0);
        }
    }
}

// 停止电机
void Motor_Stop(uint8_t motor)
{
    Motor_SetSpeed(motor, 0);
} 
