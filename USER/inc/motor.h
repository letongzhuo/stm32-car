#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f10x.h"

// 左电机引脚定义
#define MOTOR_LEFT_IN1_PORT    GPIOA
#define MOTOR_LEFT_IN1_PIN     GPIO_Pin_3  // TIM3_CH1
#define MOTOR_LEFT_IN2_PORT    GPIOA
#define MOTOR_LEFT_IN2_PIN     GPIO_Pin_2 // TIM3_CH2
#define MOTOR_LEFT_EN_PORT     GPIOA
#define MOTOR_LEFT_EN_PIN      GPIO_Pin_6  // 使能引脚

// 右电机引脚定义
#define MOTOR_RIGHT_IN1_PORT   GPIOA
#define MOTOR_RIGHT_IN1_PIN    GPIO_Pin_4  // TIM3_CH3
#define MOTOR_RIGHT_IN2_PORT   GPIOA
#define MOTOR_RIGHT_IN2_PIN    GPIO_Pin_5  // TIM3_CH4
#define MOTOR_RIGHT_EN_PORT    GPIOA
#define MOTOR_RIGHT_EN_PIN     GPIO_Pin_7  // 使能引脚

// 电机控制方向定义
#define MOTOR_FORWARD     1
#define MOTOR_STOP        0

// 函数声明
/**
 * @brief 初始化电机控制相关的GPIO和定时器
 * @note  配置IN1/IN2为推挽输出，ENA/ENB为PWM输出
 *        初始化后电机默认全速前进
 */
void Motor_Init(void);

/**
 * @brief 设置电机速度
 * @param motor  电机选择：0-左电机，1-右电机
 * @param speed  速度值：-100到100
 *              - 正值：电机正向旋转，值越大速度越快
 *              - 负值：电机反向旋转，绝对值越大速度越快
 *              - 0：电机停止
 * @note  内部会自动设置电机方向和PWM占空比
 */
void Motor_SetSpeed(uint8_t motor, int8_t speed);

/**
 * @brief 停止指定电机
 * @param motor 电机选择：0-左电机，1-右电机
 * @note  内部会调用Motor_SetSpeed(0)实现停止
 */
void Motor_Stop(uint8_t motor);

#endif 
