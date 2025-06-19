#include "sensor.h"

// 初始化红外传感器
void IR_Sensor_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    // 使能GPIOA时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    
    // 配置GPIO引脚为输入模式
    GPIO_InitStructure.GPIO_Pin = IR_SENSOR1_PIN | IR_SENSOR2_PIN | 
                                 IR_SENSOR3_PIN | IR_SENSOR4_PIN | 
                                 IR_SENSOR5_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;  // 上拉输入
    GPIO_Init(IR_SENSOR_PORT, &GPIO_InitStructure);
}

// 读取所有传感器状态
uint8_t IR_Sensor_Read(void)
{
    uint8_t sensor_state = 0;
    
    // 读取每个传感器的状态并组合成一个字节
    if(GPIO_ReadInputDataBit(IR_SENSOR_PORT, IR_SENSOR1_PIN) == IR_SENSOR_BLACK)
        sensor_state |= 0x01;
    if(GPIO_ReadInputDataBit(IR_SENSOR_PORT, IR_SENSOR2_PIN) == IR_SENSOR_BLACK)
        sensor_state |= 0x02;
    if(GPIO_ReadInputDataBit(IR_SENSOR_PORT, IR_SENSOR3_PIN) == IR_SENSOR_BLACK)
        sensor_state |= 0x04;
    if(GPIO_ReadInputDataBit(IR_SENSOR_PORT, IR_SENSOR4_PIN) == IR_SENSOR_BLACK)
        sensor_state |= 0x08;
    if(GPIO_ReadInputDataBit(IR_SENSOR_PORT, IR_SENSOR5_PIN) == IR_SENSOR_BLACK)
        sensor_state |= 0x10;
        
    return sensor_state;
}

// 获取当前位置信息
uint8_t IR_Sensor_GetPosition(void)
{
    uint8_t sensor_state = IR_Sensor_Read();
    
    // 根据传感器状态返回位置信息
    switch(sensor_state)
    {
        case 0x00:  // 00000 - 全部检测到白色
            return 0;  // 丢失黑线
            
        case 0x01:  // 00001 - 最左侧传感器检测到黑线
            return 1;

        case 0x02:  // 00010  - 左侧第二个传感器检测到黑线
            return 2;

        case 0x04:  // 00100 - 中间传感器检测到黑线
            return 3;

        case 0x08:  // 01000 - 最右侧第二个传感器检测到黑线
            return 4;

        case 0x10:  // 10000 - 最右侧传感器检测到黑线
            return 5;
            
        case 0x0a:  // 01010 - 留作模式切换
            return 10;
            
        default:
            return 0;  // 其他情况视为丢失黑线
    }
} 
