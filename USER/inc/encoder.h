#ifndef __ENCODER_H
#define __ENCODER_H

#include "stm32f10x.h"

// 编码器引脚定义 (仅左轮)
#define ENCODER_LEFT_PORT      GPIOB
#define ENCODER_LEFT_PIN       GPIO_Pin_8
// #define ENCODER_RIGHT_PORT     GPIOB  // 右轮编码器已移除
// #define ENCODER_RIGHT_PIN      GPIO_Pin_9 // 右轮编码器已移除

// 编码器相关定义
#define WHEEL_DIAMETER_MM      65    // 轮子直径（毫米）
#define ENCODER_RESOLUTION     20    // 编码器分辨率（每圈脉冲数）
#define WHEEL_CIRCUMFERENCE    (WHEEL_DIAMETER_MM * 3.14159)  // 轮子周长

// 函数声明
void Encoder_Init(void);
void Encoder_Reset(void);
/**
 * @brief 获取编码器计数值 (仅左轮)
 * @param encoder  编码器选择：0-左电机，1-右电机 (右轮始终返回0)
 * @return int16_t  编码器计数值
 */
int16_t Encoder_GetCount(uint8_t encoder);  // 获取编码器计数值
/**
 * @brief 更新编码器计数（中断处理函数调用）(仅左轮)
 * @param encoder  编码器选择：0-左电机，1-右电机 (右轮不进行更新)
 */
void Encoder_Update(uint8_t encoder);       // 更新编码器计数

#endif 
