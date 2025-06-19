#include "I2C.h"
#include "delay.h"
#include "mpu6500.h"

// 设置SDA为输入
void SDA_IN(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}

// 设置SDA为输出
void SDA_OUT(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}

// 初始化IIC
void IIC_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    // 使能GPIOB时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    
    // 配置PB6(SCL)和PB7(SDA)为推挽输出
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    // 初始状态设置为高电平
    IIC_SCL_HIGH();
    IIC_SDA_HIGH();
}

// 产生IIC起始信号
void IIC_Start(void)
{
    SDA_OUT();     //sda线输出
    IIC_SDA_HIGH();
    IIC_SCL_HIGH();
    delay_us(4);
    IIC_SDA_LOW();//START:when CLK is high,DATA change form high to low
    delay_us(4);
    IIC_SCL_LOW();//钳住I2C总线，准备发送或接收数据
}

// 产生IIC停止信号
void IIC_Stop(void)
{
    SDA_OUT();//sda线输出
    IIC_SCL_LOW();
    IIC_SDA_LOW();//STOP:when CLK is high DATA change form low to high
    delay_us(4);
    IIC_SCL_HIGH();
    IIC_SDA_HIGH();//发送I2C总线结束信号
    delay_us(4);
}

// 等待应答信号到来
// 返回值：1，接收应答失败
//         0，接收应答成功
u8 IIC_Wait_Ack(void)
{
    u8 ucErrTime=0;
    SDA_IN();      //SDA设置为输入
    IIC_SDA_HIGH();delay_us(1);
    IIC_SCL_HIGH();delay_us(1);
    while(READ_SDA())
    {
        ucErrTime++;
        if(ucErrTime>250)
        {
            IIC_Stop();
            return 1;
        }
    }
    IIC_SCL_LOW();//时钟输出0
    return 0;
}

// 产生ACK应答
void IIC_Ack(void)
{
    IIC_SCL_LOW();
    SDA_OUT();
    IIC_SDA_LOW();
    delay_us(2);
    IIC_SCL_HIGH();
    delay_us(2);
    IIC_SCL_LOW();
}

// 不产生ACK应答
void IIC_NAck(void)
{
    IIC_SCL_LOW();
    SDA_OUT();
    IIC_SDA_HIGH();
    delay_us(2);
    IIC_SCL_HIGH();
    delay_us(2);
    IIC_SCL_LOW();
}

// IIC发送一个字节
// 返回从机有无应答
// 1，有应答
// 0，无应答
void IIC_Send_Byte(u8 txd)
{
    u8 t;
    SDA_OUT();
    IIC_SCL_LOW();//拉低时钟开始数据传输
    for(t=0; t<8; t++)
    {
        if((txd&0x80)>>7)
            IIC_SDA_HIGH();
        else
            IIC_SDA_LOW();
        txd<<=1;
        delay_us(2);   //对TEA5767这三个延时都是必须的
        IIC_SCL_HIGH();
        delay_us(2);
        IIC_SCL_LOW();
        delay_us(2);
    }
}

// 读1个字节，ack=1时，发送ACK，ack=0，发送nACK
u8 IIC_Read_Byte(unsigned char ack)
{
    unsigned char i,receive=0;
    SDA_IN();//SDA设置为输入
    for(i=0;i<8;i++ )
    {
        IIC_SCL_LOW();
        delay_us(2);
        IIC_SCL_HIGH();
        receive<<=1;
        if(READ_SDA())receive++;
        delay_us(1);
    }
    if (!ack)
        IIC_NAck();//发送nACK
    else
        IIC_Ack(); //发送ACK
    return receive;
}

// 写I2C一个字节
// reg:寄存器地址
// data:数据
void Single_WriteI2C(u8 REG_Address,u8 REG_data)
{
    IIC_Start();
    IIC_Send_Byte(MPU6500_ADDR);     //发送设备地址+写命令
    IIC_Wait_Ack();
    IIC_Send_Byte(REG_Address);  //发送寄存器地址
    IIC_Wait_Ack();
    IIC_Send_Byte(REG_data);     //发送数据
    IIC_Wait_Ack();
    IIC_Stop();
}

// 读I2C一个字节
// reg:寄存器地址
// 返回值:读到的数据
u8 Single_ReadI2C(u8 REG_Address)
{
    u8 REG_data;
    IIC_Start();
    IIC_Send_Byte(MPU6500_ADDR);     //发送设备地址+写命令
    IIC_Wait_Ack();
    IIC_Send_Byte(REG_Address);  //发送寄存器地址
    IIC_Wait_Ack();
    IIC_Start();
    IIC_Send_Byte(MPU6500_ADDR+1);   //发送设备地址+读命令
    IIC_Wait_Ack();
    REG_data= IIC_Read_Byte(0);
    IIC_Stop();
    return REG_data;
}

// 读取16位数据
u16 GetData(u8 REG_Address)
{
    u8 H,L;
    H=Single_ReadI2C(REG_Address);
    L=Single_ReadI2C(REG_Address+1);
    return (H<<8)+L;   //合成数据
} 
