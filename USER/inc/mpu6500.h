#ifndef __MPU6500_H
#define __MPU6500_H

// MPU6500驱动头文件

#include "stm32f10x.h"
#include <math.h>

// MPU6500 I2C地址
#define MPU6500_ADDR            0xD0  // 0x68<<1

// MPU6500寄存器地址
#define MPU_PWR_MGMT1_REG       0x6B    // 电源管理寄存器1
#define MPU_WHO_AM_I_REG        0x75    // 器件ID寄存器
#define MPU_GYRO_ZOUTH_REG      0x47    // 陀螺仪值,Z轴高8位寄存器
#define MPU_GYRO_ZOUTL_REG      0x48    // 陀螺仪值,Z轴低8位寄存器
#define MPU_GYRO_CONFIG_REG     0x1B    // 陀螺仪配置寄存器
#define MPU_SMPLRT_DIV_REG      0x19    // 采样频率分频器
#define MPU_CONFIG_REG          0x1A    // 配置寄存器

// 配置值
#define MPU_RESET_VALUE         0x80    // 复位值
#define MPU_PWR_MGMT1_VALUE     0x09    // 禁用温度传感器，使用X轴陀螺仪参考
#define MPU_GYRO_CONFIG_VALUE   0x08    // ±500°/s量程
#define MPU_SMPLRT_DIV_VALUE    0x00    // 采样率1kHz
#define MPU_CONFIG_VALUE        0x03    // 184Hz低通滤波

// 数据结构定义
typedef struct
{
    int16_t gx;
    int16_t gy;
    int16_t gz; // 三个方向的角速度
} MPU6500_Data_t;

// 全局变量用于存储角度
extern float g_angle_z; // 绕Z轴角度

// 函数声明
/**
 * @brief 初始化MPU6500传感器
 * @return uint8_t 初始化状态：0-失败，1-成功
 */
uint8_t MPU6500_Init(void);

/**
 * @brief 计算MPU6500的Z轴角度
 * @note 通过积分角速度计算角度，需要定期调用
 */
void MPU6500_CalculateAngle(void);

void MPU6500_Calibrate(void);
#endif /* __MPU6500_H */
