#ifndef __I2C_H
#define __I2C_H

#include "stm32f10x.h"

// IO方向设置
void SDA_IN(void);
void SDA_OUT(void);

// IO操作函数
#define IIC_SCL_HIGH()  GPIO_SetBits(GPIOB, GPIO_Pin_6)
#define IIC_SCL_LOW()   GPIO_ResetBits(GPIOB, GPIO_Pin_6)
#define IIC_SDA_HIGH()  GPIO_SetBits(GPIOB, GPIO_Pin_7)
#define IIC_SDA_LOW()   GPIO_ResetBits(GPIOB, GPIO_Pin_7)
#define READ_SDA()      GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_7)

// IIC所有操作函数
void IIC_Init(void);                //初始化IIC的IO口
void IIC_Start(void);               //发送IIC开始信号
void IIC_Stop(void);                //发送IIC停止信号
void IIC_Send_Byte(u8 txd);         //IIC发送一个字节
u8 IIC_Read_Byte(unsigned char ack);//IIC读取一个字节
u8 IIC_Wait_Ack(void);              //IIC等待ACK信号
void IIC_Ack(void);                 //IIC发送ACK信号
void IIC_NAck(void);                //IIC发送NACK信号

// MPU6050专用I2C操作函数
void Single_WriteI2C(u8 REG_Address, u8 REG_data);
u8 Single_ReadI2C(u8 REG_Address);
u16 GetData(u8 REG_Address);

#endif
