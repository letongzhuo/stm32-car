#ifndef __SENSOR_H
#define __SENSOR_H

#include "stm32f10x.h"

// 红外传感器引脚定义
#define IR_SENSOR_PORT    GPIOA
#define IR_SENSOR1_PIN    GPIO_Pin_8  // 最左侧传感器
#define IR_SENSOR2_PIN    GPIO_Pin_9
#define IR_SENSOR3_PIN    GPIO_Pin_10  // 中间传感器
#define IR_SENSOR4_PIN    GPIO_Pin_11
#define IR_SENSOR5_PIN    GPIO_Pin_15 // 最右侧传感器

// 传感器状态定义
#define IR_SENSOR_BLACK   0   // 检测到黑线
#define IR_SENSOR_WHITE   1   // 检测到白色

// 函数声明
void IR_Sensor_Init(void);
uint8_t IR_Sensor_Read(void);  // 返回5位二进制数，表示5个传感器的状态
uint8_t IR_Sensor_GetPosition(void);  // 返回当前位置信息

#endif 
