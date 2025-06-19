#include "mpu6500.h"
#include "stm32f10x.h"
#include <math.h>
#include "led.h"
#include "I2C.h"
#include "delay.h"

float g_angle_z = 0.0f;        // 绕Z轴角度

// 全局变量用于存储零偏
float g_gyro_z_offset = 0.0f;

// 初始化MPU6500
uint8_t MPU6500_Init(void)
{
    uint8_t id;
    
    // 初始化I2C
    IIC_Init();
    delay_ms(10);
    
    // 复位设备
    Single_WriteI2C(MPU_PWR_MGMT1_REG, MPU_RESET_VALUE);
    delay_ms(50);
    
    // 配置设备
    Single_WriteI2C(MPU_PWR_MGMT1_REG, MPU_PWR_MGMT1_VALUE);
    Single_WriteI2C(MPU_SMPLRT_DIV_REG, MPU_SMPLRT_DIV_VALUE);
    Single_WriteI2C(MPU_CONFIG_REG, MPU_CONFIG_VALUE);
    Single_WriteI2C(MPU_GYRO_CONFIG_REG, MPU_GYRO_CONFIG_VALUE);
    
    // 检查设备ID
    id = Single_ReadI2C(MPU_WHO_AM_I_REG);
    if(id != 0x70)
    {
        return 0; // 初始化失败
    }
    return 1;
}

// 校准MPU6500，消除零偏
void MPU6500_Calibrate(void)
{
    int32_t sum_z = 0;
    uint16_t i;
    const uint16_t sample_count = 200; // 2秒内采样200次
    
    // 等待2秒，同时进行采样
    for( i = 0; i < sample_count; i++)
    {
        int16_t gyro_z = GetData(MPU_GYRO_ZOUTH_REG);
        sum_z += gyro_z;
        delay_ms(10); // 10ms采样间隔
    }
    
    // 计算平均值作为零偏
    g_gyro_z_offset = (float)sum_z / sample_count;
}

// 计算MPU6500角度
void MPU6500_CalculateAngle(void)
{
    int16_t gyro_z;
    float gyro_z_dps;
    float dt = 0.0115f; // 调用间隔是10ms,0.012
    
    // 读取Z轴角速度数据
    gyro_z = GetData(MPU_GYRO_ZOUTH_REG);
    
    // 将原始数据转换为度/秒，并减去零偏
    gyro_z_dps = ((float)gyro_z - g_gyro_z_offset) / 65.5f;
    
    // 积分角速度获取角度
    g_angle_z += gyro_z_dps * dt;
}
