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
    
    pid->output = pid->kp * error + 
                 pid->ki * pid->integral + 
                 pid->kd * (error - pid->last_error);
    
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
    delay_ms(1000);
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
        if(last_mode > MODE_PATH_ACBD_LOOP) {
            last_mode = MODE_PATH_AB;
        }
        
        // 设置新模式
        car_state.mode = last_mode;
        LED_Blink(last_mode - MODE_PATH_AB + 1); // 闪烁1~4次
        
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
        default:
            Car_Stop();
            break;
    }
}

// 循迹控制
void Car_LineFollow(void)
{
    uint8_t position;
    float line_error;
    float output;
    float base_speed = 70.0f;  // 基础速度
    static uint8_t last_position = 0;  // 记录上一次的位置
    static uint8_t lost_count = 0;     // 丢失黑线计数器
    
    // 更新MPU6500角度数据，虽然循迹用不上角度，但是后续的直线需要更新后的角度
    
    
    position = IR_Sensor_GetPosition();
    
    // 如果检测到模式切换点，直接返回让Car_Update处理
    if (position == 10) {
        LED_Blink(1);
        return;
    }
    
    // 处理丢失黑线的情况
    if (position == 0) {
        lost_count++;
        // 如果连续多次丢失黑线，根据上一次的位置决定转向
        if (lost_count > 0) {
            // if (last_position > 0 && last_position < 3) {  // 上一次在左边
            //     // Motor_SetSpeed(0, -base_speed*ks);
            //     // Motor_SetSpeed(1, base_speed*ks);  // 强制左转
            //     position = last_position;
            //     // return;  // 直接返回，不执行后面的PID控制
            // } else if (last_position > 3 && last_position < 5) {  // 上一次在右边
            //     // Motor_SetSpeed(0, base_speed*ks);
            //     // Motor_SetSpeed(1, -base_speed*ks);  // 强制右转
            //     // return;  // 直接返回，不执行后面的PID控制
            //     position = last_position;
            // }
            position = last_position;  // 保持上一次的位置
        }
    } else {
        lost_count = 0;
        last_position = position;
    }
    
    // 计算位置误差
    switch(position)
    {
        case 0:  // 丢失黑线
            line_error = 0.0f;  // 保持直线行驶
            break;
            
        case 1:  // 最左侧
            line_error = 4.5f;  // 需要左转
            break;
            
        case 2:
            line_error = 4.5f;
            break;
            
        case 3:  // 中间
            line_error = 0.0f;  // 直线行驶
            break;
            
        case 4:
            line_error = 4.7f;
            break;
            
        case 5:  // 最右侧
            line_error = 4.5f;  // 需要右转
            break;

        default:
            line_error = 0.0f;
            break;
    }
    
    // 计算PID输出
    output = PID_Calculate(&car_state.line_pid, line_error);
    //LED_Blink(position);
    // 设置电机速度
    if(position>0 && position < 3) {  // 黑线在左侧，需要左转
        Motor_SetSpeed(0, base_speed - output);  // 左轮减速
        Motor_SetSpeed(1, base_speed);  // 右轮保持全速
    } else if(position<6 && position > 3) {  // 黑线在右侧，需要右转
        Motor_SetSpeed(0, base_speed);  // 左轮保持全速
        Motor_SetSpeed(1, base_speed - output);  // 右轮减速
    } else {  // position == 5，直线行驶
        Motor_SetSpeed(0, base_speed-5);  // 左轮全速
        Motor_SetSpeed(1, base_speed-10);  // 右轮全速
    }
}

// 全局变量用于存储角度
extern float g_angle_z; // 绕Z轴角度，左加右减


// 角度转弯控制
// target_angle: 目标角度 (度)
void Car_TurnAngle(float target_angle)
{
    float angle_error = 0.0f;
    static uint32_t turn_start_time = 0;
    static uint32_t turn_duration = 0;
    static int8_t last_turn_direction = 0;  // 记录上次转向方向
    uint32_t current_time;
    int8_t turn_direction;
    int16_t left_speed = 0, right_speed = 0;
    
    // 更新MPU6500角度数据
    
    
    // 计算角度误差
    angle_error = target_angle + g_angle_z;
    
    // 分级控制转向
    if (fabsf(angle_error) <= ANGLE_DEADZONE) {
        // 在死区范围内，停止转向
        Motor_Stop(0);
        Motor_Stop(1);
        last_turn_direction = 0;
    } else {
        // 获取当前时间
        current_time = g_systick_count;
        
        // 如果是新的转向动作，记录开始时间
        if (last_turn_direction == 0) {
            turn_start_time = current_time;
        }
        turn_duration = current_time - turn_start_time;
        
        // 确定转向方向
        turn_direction = (angle_error > 0) ? 1 : -1;
        last_turn_direction = turn_direction;
        
        // 根据误差大小选择基础速度
        if (fabsf(angle_error) > ERROR_THRESHOLD_1) {
            left_speed = right_speed = SPEED_LEVEL_1;
        } else if (fabsf(angle_error) > ERROR_THRESHOLD_2) {
            left_speed = right_speed = SPEED_LEVEL_2;
        } else if (fabsf(angle_error) > ERROR_THRESHOLD_3) {
            left_speed = right_speed = SPEED_LEVEL_3;
        } else if (fabsf(angle_error) > ERROR_THRESHOLD_4) {
            left_speed = right_speed = SPEED_LEVEL_4;
        } else {
            left_speed = right_speed = SPEED_LEVEL_5;
        }
        
        // 启动加速阶段
        if (turn_duration < TURN_START_BOOST_TIME) {
            // 在启动阶段增加20%的速度
            left_speed = 80;
            right_speed = 80;
        }
        // 减速阶段
        else if (fabsf(angle_error) < ERROR_THRESHOLD_3 && turn_duration > TURN_DECEL_TIME) {
            // 在接近目标角度时逐渐降低速度
            float decel_factor = 1.0f - (float)(turn_duration - TURN_DECEL_TIME) / 1000.0f;
            if (decel_factor < 0.5f) decel_factor = 0.5f;
            left_speed = (int16_t)(left_speed * decel_factor);
            right_speed = (int16_t)(right_speed * decel_factor);
        }
        
        // 设置电机速度
        if (turn_direction > 0) { // 向右转
            Motor_SetSpeed(0, left_speed);
            Motor_SetSpeed(1, -right_speed);
        } else { // 向左转
            Motor_SetSpeed(0, -left_speed);
            Motor_SetSpeed(1, right_speed);
        }
    }
}

// 路径A->B
void Car_Path_AB(void)
{
    static int32_t start_encoder = 0;
    static uint8_t inited = 0;  // 静态变量，只在第一次调用时初始化为0
    int32_t cur_encoder = 0;
    float distance = 0.0f;
    
    if (!inited) {  // 第一次调用时，inited为0
        start_encoder = Encoder_GetCount(0);    // 记录起始位置
        car_state.path_point = 0;   //设置路径点
        inited = 1; //标记已初始化
        g_angle_z = 0; // 重置角度
        Car_ResetStraightLine(); // 重置直线行驶状态
    }
    
    cur_encoder = Encoder_GetCount(0);
    distance = (cur_encoder - start_encoder) * ((WHEEL_CIRCUMFERENCE / 10.0f) / ENCODER_RESOLUTION); // 距离计算，单位：cm
    
    switch(car_state.path_point)
    {
        case 0: // A->B 直线
            // 更新MPU6500角度数据
            

            // 使用基于MPU6500角度控制的直线行驶，而不是简单的全速行驶
            Car_StraightLine(100);  // 使用100作为基础速度（全速）
            
            if (distance >= 85.0f) { // 直线100cm
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
    static int32_t start_encoder = 0;
    static uint8_t inited = 0;
    static uint8_t lost_count = 0;  // 丢失黑线计数器
    int32_t cur_encoder = 0;
    float distance = 0.0f;
    uint8_t position = 0;
    
    if (!inited) {
        start_encoder = Encoder_GetCount(0);
        car_state.path_point = 0;
        inited = 1;
        g_angle_z = 0; // 重置角度
        Car_ResetStraightLine(); // 重置直线行驶状态
    }
    
    cur_encoder = Encoder_GetCount(0);
    distance = (cur_encoder - start_encoder) * ((WHEEL_CIRCUMFERENCE / 10.0f) / ENCODER_RESOLUTION);
    
    switch(car_state.path_point) {
        case 0: // A->B 直线
            // 更新MPU6500角度数据
            
            
            Car_StraightLine(100);  // 使用基于MPU6500角度控制的直线行驶
            
            if (distance >= 85.0f || IR_Sensor_GetPosition() != 0) { // 行驶100cm
                Car_Stop();
                LED_Blink(1);
                start_encoder = cur_encoder;
                car_state.path_point = 1; // 切换到B->C右半圆循迹
            }
            break;
            
        case 1: // B->C 右半圆循迹
            position = IR_Sensor_GetPosition();
            
            if (position == 0) {  // 丢失黑线
                lost_count++;
            } else {
                lost_count = 0;  // 找到黑线就重置计数器
            }
            
            Car_LineFollow();  // 沿着黑线循迹
            
            // 同时判断距离和连续丢失次数
            if (distance >= 105.66f && lost_count >= 60) { // 半圆弧长 = π * 40cm 或 连续10次丢失黑线
                Car_Stop();
                LED_Blink(2);
                start_encoder = cur_encoder;
                car_state.path_point = 2; // 切换到C->D直线
                lost_count = 0;  // 重置丢失计数器
            }
            break;
            
        case 2:// C点直行前检查角度
            // 更新MPU6500角度数据
            Car_TurnAngle(180);
            // 判断是否转弯到位
            if (fabsf(g_angle_z + 180.0f) < 2.5f) {
                Car_ResetStraightLine();  // 重置直线行驶状态
                car_state.path_point = 3; // 角度正确，进入直行阶段
                LED_Blink(1);
                Car_Stop();
                start_encoder = cur_encoder;
            }
            break;
            
        case 3: // C->D 直线
            // 更新MPU6500角度数据
            
            
            Car_StraightLine(100);  // 使用基于MPU6500角度控制的直线行驶
            
            if (distance >= 90.0f || IR_Sensor_GetPosition() != 0) { // 行驶100cm
                Car_Stop();
                LED_Blink(3);
                start_encoder = cur_encoder;
                car_state.path_point = 4; // 切换到D->A左半圆循迹
            }
            break;
            
        case 4: // D->A 左半圆循迹
            position = IR_Sensor_GetPosition();
            
            if (position == 0) {  // 丢失黑线
                lost_count++;
            } else {
                lost_count = 0;  // 找到黑线就重置计数器
            }
            
            Car_LineFollow();  // 沿着黑线循迹
            
            // 必须同时满足距离和黑线丢失条件
            if (distance >= 105.66f && lost_count >= 60) { // 半圆弧长 = π * 40cm 且 连续30次丢失黑线
                Car_Stop();
                LED_Blink(4);
                start_encoder = cur_encoder;
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
    static int32_t start_encoder = 0;
    static uint8_t inited = 0;
    static uint8_t lost_count = 0;  // 丢失黑线计数器
    int32_t cur_encoder = 0;
    float distance = 0.0f;
    uint8_t position = 0;
    
    if (!inited) {
        start_encoder = Encoder_GetCount(0);
        car_state.path_point = 0;
        inited = 1;
        g_angle_z = 0; // 重置角度
    }
    
    cur_encoder = Encoder_GetCount(0);
    distance = (cur_encoder - start_encoder) * ((WHEEL_CIRCUMFERENCE / 10.0f) / ENCODER_RESOLUTION); // 距离计算，单位：cm
    
    switch(car_state.path_point) {
        case 0: // A点转弯 (向右转38.66度)
            Car_TurnAngle(35.66f);
            // 判断是否转弯到位
            if (fabsf(g_angle_z + (35.66f)) < 2.0f) { // 允许2度误差，需要调整
                 Car_Stop();
                 Car_ResetStraightLine(); // 重置直线行驶状态
                 start_encoder = cur_encoder;
                 car_state.path_point = 1; // 切换到A->C直线
                //  car_state.need_reset_angle = 1;  // 在路径点切换时设置重置标志位
                //  g_angle_z = 0; // 重置角度
            }
            break;
        case 1: // A->C 直线
            // 更新MPU6500角度数据
            
            
            Car_StraightLine(100);  // 使用基于MPU6500角度控制的直线行驶
            
            if (distance >= 95.06f || IR_Sensor_GetPosition() != 0) {  // A到C的直线距离
                Car_Stop();
                LED_Blink(1);
                start_encoder = cur_encoder;
                car_state.path_point = 2; // 切换到C->B半圆
            }
            break;
        case 2: //C点转弯
            // A点转弯 (向左转38.66度)
            Car_TurnAngle(0.0f);
            // 判断是否转弯到位
            if (fabsf(g_angle_z - (00.00f)) < 2.0f) { // 允许2度误差，需要调整
                 Car_Stop();
                 start_encoder = cur_encoder;
                 car_state.path_point = 3; // 切换到A->C直线
                //  g_angle_z = 0; // 重置角度
                //  car_state.need_reset_angle = 1;  // 设置重置标志位
            
            }
            break;

        case 3: // C->B 半圆
            position = IR_Sensor_GetPosition();

            if (position == 0) {  // 丢失黑线
                lost_count++;
            } else {
                lost_count = 0;  // 找到黑线就重置计数器
            }

            Car_LineFollow();  // 半圆循迹

            if (distance >= 103.66f && lost_count >=60) {  // C到B的半圆弧长
                Car_Stop();
                LED_Blink(2);
                start_encoder = cur_encoder;
                car_state.path_point = 4; // 切换到B点转弯
                lost_count = 0;  // 重置丢失计数器
                // g_angle_z = 0; // 重置角度
            }
            break;
        case 4: // B点转弯前检查角度
            // 更新MPU6500角度数据
            Car_TurnAngle(-180);
            
            // 检查当前角度是否接近180度（允许±5度误差）
            if (fabsf(g_angle_z - 180.0f) < 2.5f) {
                Car_Stop();
                start_encoder = cur_encoder;
                car_state.path_point = 5; // 角度正确，进入转弯阶段
            }
            break;
        case 5: // B点转弯 (向左转38.66度)
            Car_TurnAngle(-240.66f); // 向左转，目标角度为负
            // 判断是否转弯到位
            if (fabsf(g_angle_z - (240.66f)) < 2.5f) { // 允许5度误差，需要调整
                 Car_Stop();
                 LED_Blink(3);
                 Car_ResetStraightLine(); // 重置直线行驶状态
                 start_encoder = cur_encoder;
                 car_state.path_point = 6; // 切换到B->D直线
                //  car_state.need_reset_angle = 1;  // 在路径点切换时设置重置标志位
                //  g_angle_z = 0; // 重置角度
            }
            break;
        case 6: // B->D 直线
            // 更新MPU6500角度数据
            
            
            Car_StraightLine(100);  // 使用基于MPU6500角度控制的直线行驶
            
            if (distance >= 110.06f || IR_Sensor_GetPosition() != 0) {  // B到D的直线距离
                Car_Stop();
                LED_Blink(4);
                start_encoder = cur_encoder;
                car_state.path_point = 7; // 切换到D点转弯
                // g_angle_z = 0; // 重置角度
            }
            break;
        case 7: // D点转弯 (向右转38.66度)
             Car_TurnAngle(-180.00f);
            // 判断是否转弯到位
            if (fabsf(g_angle_z - 180.00f) < 2.5f) { // 允许5度误差，需要调整
                 Car_Stop();
                 start_encoder = cur_encoder;
                 car_state.path_point = 8; // 切换到D->A半圆
                //  g_angle_z = 0; // 重置角度
            }
            break;
        case 8: // D->A 半圆
            position = IR_Sensor_GetPosition();

            if (position == 0) {  // 丢失黑线
                lost_count++;
            } else {
                lost_count = 0;  // 找到黑线就重置计数器
            }

            Car_LineFollow();  // D到A半圆循迹
            if (distance >= 105.66f && lost_count >= 40) {  // D到A的半圆弧长
                Car_Stop();
                LED_Blink(5);
                car_state.path_point = 9; // 路径完成
                lost_count = 0;  // 重置丢失计数器
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
    static int32_t start_encoder = 0;
    static uint8_t inited = 0;
    static uint8_t lost_count = 0;  // 丢失黑线计数器
    int32_t cur_encoder = 0;
    float distance = 0.0f;
    uint8_t position = 0;
    
    if (!inited) {
        start_encoder = Encoder_GetCount(0);
        car_state.path_point = 0;
        car_state.loop_count = 0;
        inited = 1;
        g_angle_z = 0; // 重置角度
        Car_ResetStraightLine(); // 重置直线行驶状态
    }
    
    cur_encoder = Encoder_GetCount(0);
    distance = (cur_encoder - start_encoder) * ((WHEEL_CIRCUMFERENCE / 10.0f) / ENCODER_RESOLUTION); // 距离计算，单位：cm
    
    switch(car_state.path_point) {
        case 0: // A点转弯 (向右转38.66度)
            Car_TurnAngle(38.66f);
            // 判断是否转弯到位
            if (fabsf(g_angle_z + (38.66f)) < 2.0f) { // 允许2度误差，需要调整
                 Car_Stop();
                 Car_ResetStraightLine(); // 重置直线行驶状态
                 start_encoder = cur_encoder;
                 car_state.path_point = 1; // 切换到A->C直线
                //  g_angle_z = 0; // 重置角度
                //  car_state.need_reset_angle = 1;  // 设置重置标志位
            
            }
            break;
        case 1: // A->C 直线
            // 更新MPU6500角度数据
            
            
            Car_StraightLine(100);  // 使用基于MPU6500角度控制的直线行驶
            
            if (distance >= 95.06f) {  // A到C的直线距离
                Car_Stop();
                LED_Blink(1);
                start_encoder = cur_encoder;
                car_state.path_point = 2; // 切换到C->B半圆
                // g_angle_z = 0; // 重置角度
            }
            break;

        case 2: //C点转弯
            // C点转弯 (向左转38.66度)
            Car_TurnAngle(0.00f);
            // 判断是否转弯到位
            if (fabsf(g_angle_z - (0.00f)) < 2.0f) { // 允许2度误差，需要调整
                 Car_Stop();
                 start_encoder = cur_encoder;
                 car_state.path_point = 3; // 切换到C->B半圆
                //  g_angle_z = 0; // 重置角度
                //  car_state.need_reset_angle = 1;  // 设置重置标志位
            
            }
            break;
        case 3: // C->B 半圆
            position = IR_Sensor_GetPosition();

            if (position == 0) {  // 丢失黑线
                lost_count++;
            } else {
                lost_count = 0;  // 找到黑线就重置计数器
            }

            Car_LineFollow();  // C到B半圆循迹
            // 同时判断距离和连续丢失次数
            if (distance >= 103.66f && lost_count >= 40) {  // C到B的半圆弧长
                Car_Stop();
                start_encoder = cur_encoder;
                car_state.path_point = 4; // 切换到B点转弯
                // g_angle_z = 0; // 重置角度
            }
            break;
        case 4: // B点转弯前检查角度
            Car_TurnAngle(-180);
            // 检查当前角度是否接近180度（允许±5度误差）
            if (fabsf(g_angle_z - 180.0f) < 5.0f) {
                car_state.path_point = 5; // 角度正确，进入转弯阶段
            }
            break;
        case 5: // B点转弯 (向左转38.66度)
            Car_TurnAngle(-218.66f); // 向左转，目标角度为负
            // 判断是否转弯到位
            if (fabsf(g_angle_z - (218.66f)) < 2.5f) { // 允许5度误差，需要调整
                 Car_Stop();
                 Car_ResetStraightLine(); // 重置直线行驶状态
                 start_encoder = cur_encoder;
                 car_state.path_point = 6; // 切换到B->D直线
                //  car_state.need_reset_angle = 1;  // 在路径点切换时设置重置标志位
                //  g_angle_z = 0; // 重置角度
            }
            break;
        case 6: // B->D 直线
            // 更新MPU6500角度数据
            
            
            Car_StraightLine(100);  // 使用基于MPU6500角度控制的直线行驶
            
            if (distance >= 110.06f) {  // B到D的直线距离
                Car_Stop();
                LED_Blink(3);
                start_encoder = cur_encoder;
                car_state.path_point = 7; // 切换到D点转弯
                // g_angle_z = 0; // 重置角度
            }
            break;
        case 7: // D点转弯 (向右转38.66度)
             Car_TurnAngle(-180.00f);
            // 判断是否转弯到位
            if (fabsf(g_angle_z - 180.00f) < 2.5f) { // 允许5度误差，需要调整
                 Car_Stop();
                 start_encoder = cur_encoder;
                 car_state.path_point = 8; // 切换到D->A半圆
                 g_angle_z = 0; // 重置角度
            }
            break;
        case 8: // D->A 半圆
            position = IR_Sensor_GetPosition();

            if (position == 0) {  // 丢失黑线
                lost_count++;
            } else {
                lost_count = 0;  // 找到黑线就重置计数器
            }

            Car_LineFollow();  // D到A半圆循迹

            if (distance >= 110.66f && lost_count >= 40) {  // D到A的半圆弧长
                car_state.loop_count++;
                if (car_state.loop_count >= 4) {
                    Car_Stop();
                    car_state.path_point = 9; // 路径完成
                } else {
                    start_encoder = cur_encoder;
                    car_state.path_point = 0; // 回到A->C直线
                    lost_count = 0;  // 重置丢失计数器
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
