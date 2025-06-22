/**
  ******************************************************************************
  * @file    Project/Template/main.c 
  * @author  MCD Application Team
  * @version V3.1.2
  * @date    09/28/2009
  * @brief   Main program body
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2009 STMicroelectronics</center></h2>
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "system_config.h"
#include "sensor.h"
#include "motor.h"
#include "encoder.h"
#include "mpu6500.h"
#include "car_control.h"
#include "led.h"
#include "delay.h"  // 添加delay.h头文件

// 系统滴答定时器计数
// 注意：定义为全局变量，供delay.c使用
volatile uint32_t g_systick_count = 0;

int main(void)
{
	uint8_t init_result;
	
	// 配置系统时钟
	SystemClock_Config();
	
	// 初始化延时函数
	delay_init(72);  // 系统时钟72MHz
	
	// 配置SysTick定时器，产生1ms中断
	if(SysTick_Config(SystemCoreClock / 1000))
	{
		// 如果配置失败，可以在这里添加错误处理
		while(1);
	}
	
	// 设置SysTick中断优先级为最高
	NVIC_SetPriority(SysTick_IRQn, 0);
	
	// 初始化各个模块
	IR_Sensor_Init();
	Motor_Init();
	Encoder_Init();
	LED_Init();
	// MPU6500初始化
	init_result = MPU6500_Init();
	if(init_result == 0)
	{
		// 初始化失败，可以在这里添加错误处理
		// 比如点亮错误指示灯
		LED_Blink(4);  // 闪烁LED表示错误
		while(1);  // 停止执行
	}
	else
	{
		LED_Blink(3);  // 初始化成功
	}
	MPU6500_Calibrate();  // 校准MPU6500，消除零偏
	
	Car_Init();
	
	// 设置初始模式为循迹模式
	Car_SetMode(MODE_AUTO_SEQUENCE);
	// 主循环
	while(1)
	{
		// 更新小车状态
		Car_Update();
		delay_ms(5);
		
	}
}

// 编码器中断处理函数
void EXTI0_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line0) != RESET)
	{
		Encoder_Update(0);
		EXTI_ClearITPendingBit(EXTI_Line0);
	}
}

void EXTI1_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line1) != RESET)
	{
		Encoder_Update(1);
		EXTI_ClearITPendingBit(EXTI_Line1);
	}
}
