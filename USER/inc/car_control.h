#ifndef __CAR_CONTROL_H
#define __CAR_CONTROL_H

#include "stm32f10x.h"
#include "sensor.h"
#include "motor.h"
#include "encoder.h"
#include "mpu6500.h"

// 小车控制模式
#define MODE_LINE_FOLLOW    0   // 循迹模式
#define MODE_PATH_AB        1   // 路径A->B
#define MODE_PATH_ABCD      2   // 路径A->B->C->D->A
#define MODE_PATH_ACBD      3   // 路径A->C->B->D->A
#define MODE_PATH_ACBD_LOOP 4   // 路径A->C->B->D->A，循环4圈

// 定义转向控制参数
#define ANGLE_DEADZONE 2.0f        // 死区范围（度）

// 定义误差阈值（度）
#define ERROR_THRESHOLD_1 90.0f    // 最大误差阈值
#define ERROR_THRESHOLD_2 70.0f    // 大误差阈值
#define ERROR_THRESHOLD_3 50.0f    // 中等误差阈值
#define ERROR_THRESHOLD_4 30.0f    // 小误差阈值

// 定义不同等级的速度值
#define SPEED_LEVEL_1 90          // 最大速度
#define SPEED_LEVEL_2 80           // 大速度
#define SPEED_LEVEL_3 70           // 中高速度
#define SPEED_LEVEL_4 60           // 中速度
#define SPEED_LEVEL_5 55           // 小速度

// 时间控制参数（毫秒）
#define TURN_START_BOOST_TIME 100  // 启动加速时间
#define TURN_DECEL_TIME 300        // 减速时间

// PID控制参数
typedef struct {
    float kp;        // 比例系数
    float ki;        // 积分系数
    float kd;        // 微分系数
    float target;    // 目标值
    float error;     // 误差
    float last_error;// 上次误差
    float integral;  // 积分项
    float output;    // 输出值
} PID_TypeDef;

// 小车状态结构体
typedef struct {
    uint8_t mode;            // 当前模式
    PID_TypeDef line_pid;    // 循迹PID
    uint8_t path_point;      // 当前路径点编号（0:A, 1:B, 2:C, 3:D）
    uint8_t loop_count;      // 圈数计数
    uint8_t need_reset_angle;// 是否需要重置直线行驶的初始角度
} Car_State_t;

// 函数声明
void Car_Init(void);
void Car_SetMode(uint8_t mode);
void Car_Update(void);           // 主控制循环
void Car_LineFollow(void);       // 循迹控制
void Car_Stop(void);             // 停止小车
void Car_Path_AB(void);         // 路径A->B
void Car_Path_ABCD(void);       // 路径A->B->C->D->A
void Car_Path_ACBD(void);       // 路径A->C->B->D->A
void Car_Path_ACBD_Loop(void);  // 路径A->C->B->D->A，循环4圈
void Car_TurnAngle(float target_angle); // 新增角度转弯函数
void Car_StraightLine(uint8_t base_speed); // 基于MPU6500角度控制的直线行驶
void Car_ResetStraightLine(void);

// PID控制函数
void PID_Init(PID_TypeDef *pid, float kp, float ki, float kd);
float PID_Calculate(PID_TypeDef *pid, float current);

#endif
