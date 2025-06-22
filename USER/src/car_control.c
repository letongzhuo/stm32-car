#include "encoder.h"
#include "car_control.h"
#include <math.h>
#include "led.h"
#include <string.h>
#include "delay.h"

// 小车状态
static Car_State_t car_state;

// PID参数初始化
void PID_Init(PID_TypeDef *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->error = 0;
    pid->last_error = 0;
    pid->integral = 0;
    pid->output = 0;
}

// PID计算
float PID_Calculate(PID_TypeDef *pid, float current)
{
    float error = current;  // 直接使用 current 作为误差，因为我们需要正值
    pid->integral += error;
    
    // 积分限幅
    if(pid->integral > 100) pid->integral = 100;
    if(pid->integral < -100) pid->integral = -100;
    
    pid->output = pid->kp * error + 40.0f;
    
    pid->last_error = error;
    
    // 输出限幅，确保为正值
    if(pid->output > 100) pid->output = 100;
    if(pid->output < 0) pid->output = 0;  // 不允许负值
    
    return pid->output;
}

// 初始化小车
void Car_Init(void)
{
    // 初始化PID参数
    PID_Init(&car_state.line_pid, 15.5f, 0.0f, 1.0f);     // 循迹PID
}

// 设置控制模式
void Car_SetMode(uint8_t mode)
{
    car_state.mode = mode;
    Car_Stop();
}

// 停止小车
void Car_Stop(void)
{
    Motor_Stop(0);  // 停止左电机
    Motor_Stop(1);  // 停止右电机
}

// 主控制循环
void Car_Update(void)
{
    static uint8_t last_mode = MODE_LINE_FOLLOW;  // 修改初始模式
    static uint8_t last_switch = 0;
    uint8_t pos;
    
    pos = IR_Sensor_GetPosition();
    // 检测到切换点且之前未切换
    if(pos == 10 && last_switch == 0) {
        // 停止当前动作
        Car_Stop();
        delay_ms(500);  // 等待一段时间确保稳定
        
        // 更新模式
        last_mode++;
        if(last_mode > MODE_AUTO_SEQUENCE) {
            last_mode = MODE_PATH_AB;
        }
        
        // 设置新模式
        car_state.mode = last_mode;
        LED_Blink(last_mode - MODE_PATH_AB + 1); // 闪烁1~6次（支持到MODE_AUTO_SEQUENCE）
        
        // 标记已切换
        last_switch = 1;
    }
    
    // 重置切换标志
    if(pos != 10) {
        last_switch = 0;
    }
    
    // 执行当前模式的控制
    switch(car_state.mode)
    {
        case MODE_LINE_FOLLOW:
            Car_LineFollow();
            break;
        case MODE_PATH_AB:
            Car_Path_AB();
            break;
        case MODE_PATH_ABCD:
            Car_Path_ABCD();
            break;
        case MODE_PATH_ACBD:
            Car_Path_ACBD();
            break;
        case MODE_PATH_ACBD_LOOP:
            Car_Path_ACBD_Loop();
            break;
        case MODE_AUTO_SEQUENCE:
            Car_AutoSequence();
            break;
        default:
            Car_Stop();
            break;
    }
}
// 循迹控制
void Car_LineFollow(void)
{
    uint8_t position;
    float base_speed = 70.0f;  // 基础速度
    
    position = IR_Sensor_GetPosition();
    
    // 如果检测到模式切换点，直接返回让Car_Update处理
    if (position == 10) {
        LED_Blink(1);
        return;
    }
    // 设置电机速度
    if(position>0 && position < 3) {  // 黑线在左侧，需要左转
        Motor_SetSpeed(0, -base_speed);  // 左轮减速
        Motor_SetSpeed(1, base_speed);  // 右轮保持全速
    } else if(position<6 && position > 3) {  // 黑线在右侧，需要右转
        Motor_SetSpeed(0, base_speed);  // 左轮保持全速
        Motor_SetSpeed(1, -base_speed);  // 右轮减速
    } else {  // position == 5，直线行驶
        Motor_SetSpeed(0, base_speed);  // 左轮全速
        Motor_SetSpeed(1, base_speed);  // 右轮全速
    }
}

// 全局变量用于存储角度
extern float g_angle_z; // 绕Z轴角度，左加右减
PID_TypeDef turn_pid = {1.0f, 0.0f, 0.5f, 0.0f, 0.0f}; // 可调参数

// 角度转弯控制
// target_angle: 目标角度 (度)
void Car_TurnAngle(float target_angle)
{
    static int8_t last_turn_direction = 0;
    static uint32_t turn_start_time = 0;
    float angle_error;
    float pid_output;
    uint32_t current_time;
    int8_t turn_direction;
    int16_t left_speed, right_speed;
    float min_speed = 45.0f,max_speed = 70.0f;
    float start_boost_speed = 70.0f; // 小角度初始加速
    float abs_error, speed;

    angle_error = target_angle - g_angle_z;
    abs_error = fabsf(angle_error);

    // PID计算
    pid_output = PID_Calculate(&turn_pid, angle_error);

    // 速度限幅
    speed = fabsf(pid_output);
    if (speed < min_speed) speed = min_speed;
    if (speed > max_speed) speed = max_speed;

    // 小角度初始加速
    current_time = g_systick_count;
    if (abs_error < 40.0f && last_turn_direction == 0) { // 20度以内的转向和第一帧转向
        speed = start_boost_speed;
        turn_start_time = current_time;
    }
    // 启动后短时间内保持boost
    if (current_time - turn_start_time < 100) { // 100ms内
        if (speed < start_boost_speed) speed = start_boost_speed;
    }

    // 死区处理
    if (abs_error <= ANGLE_DEADZONE) {
        Car_Stop();
        last_turn_direction = 0;
        turn_pid.integral = 0; // 防止积分累积
        return;
    }

    // 设置电机速度
    turn_direction = (angle_error > 0) ? 1 : -1;
    last_turn_direction = turn_direction;

    left_speed = (turn_direction > 0) ? -speed : speed;
    right_speed = (turn_direction > 0) ? speed : -speed;

    Motor_SetSpeed(0, left_speed);
    Motor_SetSpeed(1, right_speed);
}

// 路径A->B
void Car_Path_AB(void)
{
    static int32_t start_encoder = 0, start_time = 0;
    static uint8_t inited = 0;  // 静态变量，只在第一次调用时初始化为0
    int32_t cur_encoder = 0, cur_time = 0;
    float distance = 0.0f;
    
    if (!inited) {  // 第一次调用时，inited为0
        start_encoder = Encoder_GetCount(0);    // 记录起始位置
        start_time = g_systick_count;
        car_state.path_point = 0;   //设置路径点
        inited = 1; //标记已初始化
        g_angle_z = 0; // 重置角度
        Car_ResetStraightLine(); // 重置直线行驶状态
    }
    
    cur_encoder = Encoder_GetCount(0);
    distance = (cur_encoder - start_encoder) * ((WHEEL_CIRCUMFERENCE / 10.0f) / ENCODER_RESOLUTION); // 距离计算，单位：cm
    
    cur_time = g_systick_count; // 获取当前时间

    switch(car_state.path_point)
    {
        case 0: // A->B 直线
            // 更新MPU6500角度数据
            

            // 使用基于MPU6500角度控制的直线行驶，而不是简单的全速行驶
            Car_StraightLine(100);  // 使用100作为基础速度（全速）
            if (cur_time - start_time >= 2100 && distance >= 75) { // 直线100cm
                Car_Stop();
                car_state.path_point = 1; // 路径完成
            }
            break;
        case 1: // 路径完成，停止
            Car_Stop();
            break;
    }
}

// 路径A->B->C->D->A
void Car_Path_ABCD(void)
{
    static int32_t start_encoder = 0, start_time = 0;
    static uint8_t inited = 0;
    int32_t cur_encoder = 0, cur_time =0;
    float  duration = 0.0f;
    
    if (!inited) {
        start_encoder = Encoder_GetCount(0);
        start_time = g_systick_count;
        car_state.path_point = 0;
        inited = 1;
        g_angle_z = 0; // 重置角度
        Car_ResetStraightLine(); // 重置直线行驶状态
    }
    
    cur_time = g_systick_count; // 获取当前时间
    duration = cur_time - start_time; // 计算经过的时间，单位：ms
    switch(car_state.path_point) {
        case 0: // A->B 直线
            // 更新MPU6500角度数据
            
            
            Car_StraightLine(70);  // 使用基于MPU6500角度控制的直线行驶
            
            if (IR_Sensor_GetPosition() != 0) { // 行驶100cm
                Car_Stop();
                LED_Blink(1);
                start_encoder = cur_encoder;
                start_time = cur_time;
                car_state.path_point = 1; // 切换到B->C右半圆循迹
            }
            break;
            
        case 1: // B->C 右半圆循迹
            Car_LineFollow();  // 沿着黑线循迹
            
            // 同时判断距离和连续丢失次数
            if (fabsf(g_angle_z - (-180.0f)) < 10.0f && duration >= 1500) { // 半圆弧长 = π * 40cm 或 连续10次丢失黑线
                Car_Stop();
                LED_Blink(2);
                start_encoder = cur_encoder;
                start_time = cur_time;
                car_state.path_point = 2; // 切换到C->D直线
            }
            break;
            
        case 2:// C点直行前检查角度
            // 更新MPU6500角度数据
            Car_TurnAngle(-180);
            // 判断是否转弯到位
            if (fabsf(g_angle_z - (-180.0f)) < 1.0f) {
                Car_Stop();
                Car_ResetStraightLine();  // 重置直线行驶状态
                car_state.path_point = 3; // 角度正确，进入直行阶段
                LED_Blink(1);
                start_encoder = cur_encoder;
                start_time = cur_time;
            }
            break;
            
        case 3: // C->D 直线
            // 更新MPU6500角度数据
            
            
            Car_StraightLine(100);  // 使用基于MPU6500角度控制的直线行驶

            if (IR_Sensor_GetPosition() != 0 && duration >= 1500) { // 行驶100cm
                Car_Stop();
                LED_Blink(3);
                start_encoder = cur_encoder;
                start_time = cur_time;
                car_state.path_point = 4; // 切换到D->A左半圆循迹
            }
            break;
            
        case 4: // D->A 左半圆循迹
            Car_LineFollow();  // 沿着黑线循迹
            
            if (fabsf(g_angle_z - (-360.0f)) < 10.0f) { // 半圆弧长 = π * 40cm 且 连续30次丢失黑线
                Car_Stop();
                LED_Blink(4);
                start_encoder = cur_encoder;
                start_time = cur_time;
                car_state.path_point = 5; // 切换到路径完成
            }
            break;
            
        case 5: // 路径完成，停止
            Car_Stop();
            break;
    }
}

// 路径A->C->B->D->A
void Car_Path_ACBD(void)
{
    static int32_t start_encoder = 0, start_time = 0;
    static uint8_t inited = 0;
    int32_t cur_encoder = 0, cur_time = 0;
    float duration = 0.0f;
    
    if (!inited) {
        start_encoder = Encoder_GetCount(0);
        start_time = g_systick_count;
        car_state.path_point = 0;
        inited = 1;
        g_angle_z = 0; // 重置角度
        Car_ResetStraightLine(); // 重置直线行驶状态
    }
    
    
    cur_time = g_systick_count; // 获取当前时间
    duration = cur_time - start_time; // 计算经过的时间，单位：ms
    switch(car_state.path_point) {
        case 0: // A点转弯 (向右转38.66度)
            Car_TurnAngle(-30.00f);
            // 判断是否转弯到位
            if (fabsf(g_angle_z + (30.00f)) < 1.0f) { // 允许2度误差，需要调整
                 Car_Stop();
                 Car_ResetStraightLine(); // 重置直线行驶状态
                 start_encoder = cur_encoder;
                 start_time = cur_time;
                 car_state.path_point = 1; // 切换到A->C直线
                //  car_state.need_reset_angle = 1;  // 在路径点切换时设置重置标志位
                //  g_angle_z = 0; // 重置角度
            }
            break;
        case 1: // A->C 直线
            // 更新MPU6500角度数据
            
            
            Car_StraightLine(70);  // 使用基于MPU6500角度控制的直线行驶
            //cur_time - start_time >= 4250 || 
            if (IR_Sensor_GetPosition() == 1 && duration >= 1500) {  // A到C的直线距离
                Car_Stop();
                LED_Blink(1);
                start_encoder = cur_encoder;
                start_time = cur_time;
                car_state.path_point = 2; // 切换到C->B半圆
            }
            break;
        case 2: //C点转弯
            // A点转弯 (向左转38.66度)
            Car_TurnAngle(0.0f);
            // 判断是否转弯到位
            if (fabsf(g_angle_z - (00.00f)) < 1.0f) { // 允许2度误差，需要调整
                 Car_Stop();
                 delay_ms(1000); // 等待稳定
                 start_encoder = cur_encoder;
                 start_time = cur_time;
                 car_state.path_point = 3; // 切换到A->C直线
                //  g_angle_z = 0; // 重置角度
                //  car_state.need_reset_angle = 1;  // 设置重置标志位
            
            }
            break;

        case 3: // C->B 半圆
            Car_LineFollow();  // 半圆循迹

            if (fabsf(g_angle_z - 180.0f) < 5.0f) {  // C到B的半圆弧长
                Car_Stop();
                LED_Blink(2);
                start_encoder = cur_encoder;
                start_time = cur_time;
                car_state.path_point = 5; // 切换到B点转弯
                // g_angle_z = 0; // 重置角度
            }
            break;
        case 4: // B点转弯前检查角度,,,!!!已弃用
            // 更新MPU6500角度数据
            Car_TurnAngle(-180);
            
            // 检查当前角度是否接近180度（允许±5度误差）
            if (fabsf(g_angle_z - 180.0f) < 2.5f) {
                Car_Stop();
                start_encoder = cur_encoder;
                start_time = cur_time;
                car_state.path_point = 5; // 角度正确，进入转弯阶段
            }
            break;
        case 5: // B点转弯 (向左转38.66度)
            Car_TurnAngle(205.00f); // 向左转，目标角度为负
            // 判断是否转弯到位
            if (fabsf(g_angle_z - (205.00f)) < 1.0f) { // 允许5度误差，需要调整
                 Car_Stop();
                 LED_Blink(3);
                 Car_ResetStraightLine(); // 重置直线行驶状态
                 start_encoder = cur_encoder;
                 start_time = cur_time;
                 car_state.path_point = 6; // 切换到B->D直线
                //  car_state.need_reset_angle = 1;  // 在路径点切换时设置重置标志位
                //  g_angle_z = 0; // 重置角度
            }
            break;
        case 6: // B->D 直线
            
            Car_StraightLine(70);  // 使用基于MPU6500角度控制的直线行驶
            
            if (IR_Sensor_GetPosition() == 5 && duration >= 1500) {  // B到D的直线距离
                Car_Stop();
                LED_Blink(4);
                start_encoder = cur_encoder;
                start_time = cur_time;
                car_state.path_point = 7; // 切换到D点转弯
                // g_angle_z = 0; // 重置角度
            }
            break;
        case 7: // D点转弯 (向右转38.66度)
             Car_TurnAngle(180.00f);
            // 判断是否转弯到位
            if (fabsf(g_angle_z - 180.00f) < 1.0f) { // 允许5度误差，需要调整
                 Car_Stop();
                 delay_ms(1000); // 等待稳定
                 start_encoder = cur_encoder;
                 start_time = cur_time;
                 car_state.path_point = 8; // 切换到D->A半圆
                //  g_angle_z = 0; // 重置角度
            }
            break;
        case 8: // D->A 半圆
            Car_LineFollow();  // D到A半圆循迹
            if (fabsf(g_angle_z - 0.0f) < 10.0f) {  // D到A的半圆弧长
                Car_Stop();
                LED_Blink(5);
                car_state.path_point = 9; // 路径完成
            }
            break;
        case 9:
            Car_Stop();
            break;
    }
}

// 路径A->C->B->D->A，循环4圈
void Car_Path_ACBD_Loop(void)
{
    static int32_t start_encoder = 0, start_time = 0;
    static uint8_t inited = 0;
    int32_t cur_encoder = 0, cur_time = 0;
    float duration = 0.0f;
    
    if (!inited) {
        start_encoder = Encoder_GetCount(0);
        start_time = g_systick_count;
        car_state.path_point = 0;
        car_state.loop_count = 0;
        inited = 1;
        g_angle_z = 0; // 重置角度
        Car_ResetStraightLine(); // 重置直线行驶状态
    }
    
    cur_time = g_systick_count; // 获取当前时间
    duration = cur_time - start_time; // 计算经过的时间，单位：ms
    switch(car_state.path_point) {
        case 0: // A点转弯 (向右转38.66度)
            Car_TurnAngle(-30.00f);
            // 判断是否转弯到位
            if (fabsf(g_angle_z - (-30.00f)) < 1.0f) { // 允许2度误差，需要调整
                 Car_Stop();
                 Car_ResetStraightLine(); // 重置直线行驶状态
                 start_encoder = cur_encoder;
                 start_time = cur_time;
                 car_state.path_point = 1; // 切换到A->C直线
                //  g_angle_z = 0; // 重置角度
                //  car_state.need_reset_angle = 1;  // 设置重置标志位
            
            }
            break;
        case 1: // A->C 直线
            // 更新MPU6500角度数据
            
            
            Car_StraightLine(70);  // 使用基于MPU6500角度控制的直线行驶
            
            if (IR_Sensor_GetPosition() == 1 && duration >= 1500) {  // A到C的直线距离
                Car_Stop();
                LED_Blink(1);
                start_encoder = cur_encoder;
                start_time = cur_time;
                car_state.path_point = 2; // 切换到C->B半圆
                // g_angle_z = 0; // 重置角度
            }
            break;

        case 2: //C点转弯
            // C点转弯 (向左转38.66度)
            Car_TurnAngle(0.00f);
            // 判断是否转弯到位
            if (fabsf(g_angle_z - (0.00f)) < 1.0f) { // 允许2度误差，需要调整
                 Car_Stop();
                 delay_ms(1000); // 等待稳定
                 start_encoder = cur_encoder;
                 start_time = cur_time;
                 car_state.path_point = 3; // 切换到C->B半圆
                //  g_angle_z = 0; // 重置角度
                //  car_state.need_reset_angle = 1;  // 设置重置标志位
            
            }
            break;
        case 3: // C->B 半圆
            Car_LineFollow();  // C到B半圆循迹
            // 同时判断距离和连续丢失次数
            if (fabsf(g_angle_z - 180.0f) < 5.0f) {  // C到B的半圆弧长
                Car_Stop();
                LED_Blink(2);
                start_encoder = cur_encoder;
                start_time = cur_time;
                car_state.path_point = 5; // 切换到B点转弯
                // g_angle_z = 0; // 重置角度
            }
            break;
        case 4: // B点转弯前检查角度.!!已弃用！！！！！！！！
            Car_TurnAngle(-180);
            // 检查当前角度是否接近180度（允许±5度误差）
            if (fabsf(g_angle_z - 180.0f) < 5.0f) {
                car_state.path_point = 5; // 角度正确，进入转弯阶段
            }
            break;
        case 5: // B点转弯 (向左转38.66度)
            Car_TurnAngle(205.00f); // 向左转，目标角度为负
            // 判断是否转弯到位
            if (fabsf(g_angle_z - (205.00f)) < 1.0f) { // 允许5度误差，需要调整
                 Car_Stop();
                 LED_Blink(3);
                 Car_ResetStraightLine(); // 重置直线行驶状态
                 start_encoder = cur_encoder;
                 start_time = cur_time;
                 car_state.path_point = 6; // 切换到B->D直线
                //  car_state.need_reset_angle = 1;  // 在路径点切换时设置重置标志位
                //  g_angle_z = 0; // 重置角度
            }
            break;
        case 6: // B->D 直线

            Car_StraightLine(70);  // 使用基于MPU6500角度控制的直线行驶
            
            if (IR_Sensor_GetPosition() == 5 && duration >= 2000) {  // B到D的直线距离
                Car_Stop();
                LED_Blink(4);
                start_encoder = cur_encoder;
                start_time = cur_time;
                car_state.path_point = 7; // 切换到D点转弯
                // g_angle_z = 0; // 重置角度
            }
            break;
        case 7: // D点转弯 (向右转38.66度)
             Car_TurnAngle(180.00f);
            // 判断是否转弯到位
            if (fabsf(g_angle_z - 180.00f) < 1.0f) { // 允许5度误差，需要调整
                 Car_Stop();
                 delay_ms(1000); // 等待稳定
                 start_encoder = cur_encoder;
                 start_time = cur_time;
                 car_state.path_point = 8; // 切换到D->A半圆
                //  g_angle_z = 0; // 重置角度
            }
            break;
        case 8: // D->A 半圆

            Car_LineFollow();  // D到A半圆循迹

            if (fabsf(g_angle_z - 0.0f) < 10.0f) {  // D到A的半圆弧长
                 car_state.loop_count++;
                if (car_state.loop_count >= 4) {
                    Car_Stop();
                    car_state.path_point = 9; // 路径完成
                } else {
                    Car_Stop();
                    delay_ms(1000); // 等待稳定
                    Car_ResetStraightLine(); // 重置直线行驶状态
                    start_encoder = cur_encoder;
                    start_time = cur_time;
                    car_state.path_point = 0; // 回到A->C直线
                    // g_angle_z = 0; // 重置角度
                }
            }
            break;
        case 9:
            Car_Stop();
            break;

    }
}

// 基于MPU6500角度控制的直线行驶
// 参数：base_speed - 基础速度（0-100）
void Car_StraightLine(uint8_t base_speed)
{
    static float initial_angle = 0.0f;
    static uint8_t initialized = 0;
    float angle_error;
    float correction;
    float left_speed, right_speed;
    
    // 如果需要重置角度或第一次调用时记录初始角度
    if (car_state.need_reset_angle || !initialized) {
        initial_angle = g_angle_z;
        initialized = 1;
        car_state.need_reset_angle = 0;  // 重置标志位
    }
    
    // 计算角度误差（当前角度与初始角度的差值）
    angle_error = g_angle_z - initial_angle;
    // 根据角度误差计算速度修正值
    // 使用简单的比例控制，可以根据需要调整系数
    correction = angle_error * 2.0f;  // 比例系数可以调整
    
    // 限制修正值范围
    if (correction > 30.0f) correction = 30.0f;
    if (correction < -30.0f) correction = -30.0f;
    
    // 根据角度误差调整左右电机速度
    if (angle_error < 0) {  // 向右偏，需要左转修正
        left_speed = base_speed + correction;
        right_speed = base_speed;
    } else if (angle_error > 0) {  // 向左偏，需要右转修正
        left_speed = base_speed;
        right_speed = base_speed - correction;
    } else {  // 无偏差
        left_speed = base_speed;
        right_speed = base_speed;
    }
    // 速度限幅
    if (left_speed > 100) left_speed = 100;
    if (left_speed < 0) left_speed = 0;
    if (right_speed > 100) right_speed = 100;
    if (right_speed < 0) right_speed = 0;
    
    // 设置电机速度
    Motor_SetSpeed(0, (uint8_t)left_speed);
    Motor_SetSpeed(1, (uint8_t)right_speed);
}

// 重置直线行驶状态（在转弯或停止后调用）
void Car_ResetStraightLine(void)
{
    car_state.need_reset_angle = 1;  // 设置重置标志位
    Car_StraightLine(0);  // 调用一次以触发重置
}

// 自动连续执行：ABCD -> ACBD -> ACBD_LOOP
void Car_AutoSequence(void)
{
    static uint8_t current_sequence = 0;  // 当前执行的序列：0=ABCD, 1=ACBD, 2=ACBD_LOOP
    static uint8_t inited = 0;
    static uint8_t sequence_completed = 0;  // 当前序列是否完成
    
    if (!inited) {
        current_sequence = 0;
        sequence_completed = 0;
        inited = 1;
        // 重置所有路径函数的状态
        car_state.path_point = 0;
        car_state.loop_count = 0;
        g_angle_z = 0;
        Car_ResetStraightLine();
    }
    
    // 根据当前序列执行相应的路径
    switch (current_sequence) {
        case 0:  // 执行ABCD路径
            Car_Path_ABCD();
            // 检查ABCD是否完成（通过检查path_point是否为5）
            if (car_state.path_point == 5) {
                while(1){
                    Car_TurnAngle(-360.0f);
                    if(fabsf(g_angle_z - (-360.0f)) < 1.0f){
                        Car_Stop();
                        break;
                    }
                }

                sequence_completed = 1;
            }
            break;
            
        case 1:  // 执行ACBD路径
            Car_Path_ACBD();
            // 检查ACBD是否完成（通过检查path_point是否为9）
            if (car_state.path_point == 9) {
                sequence_completed = 1;
            }
            break;
            
        case 2:  // 执行ACBD_LOOP路径
            Car_Path_ACBD_Loop();
            // 检查ACBD_LOOP是否完成（通过检查path_point是否为9且loop_count>=4）
            if (car_state.path_point == 9 && car_state.loop_count >= 4) {
                sequence_completed = 1;
            }
            break;
    }
    
    // 如果当前序列完成，切换到下一个序列
    if (sequence_completed) {
        Car_Stop();
        delay_ms(1000);  // 等待1秒
        
        current_sequence++;
        if (current_sequence > 2) {
            // 所有序列完成，停止
            Car_Stop();
            return;
        }
        
        // 重置状态，准备执行下一个序列
        sequence_completed = 0;
        car_state.path_point = 0;
        car_state.loop_count = 0;
        g_angle_z = 0;
        Car_ResetStraightLine();
        
        // 指示当前执行的序列
        LED_Blink(current_sequence + 1);
    }
}
