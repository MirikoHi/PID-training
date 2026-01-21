#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "main.h"

/**
 * @brief 电机类型结构体
 * 
 * @param ID 电调ID
 * @param current 电机电流
 * @param speed 电机转速
 * @param raw_angle 机械转子角度
 * @param angle 电机角度
 * @param last_raw_angle 上一次原始角度值
 * @param turn_count 转数计数
 */
typedef struct {
    uint8_t ID;
    float current;
    float speed;
    uint16_t raw_angle;
    float angle;

    // 多圈处理
    uint16_t last_raw_angle;
    int32_t turn_count;

    uint8_t temperature;
    uint8_t err_code;
} motor_t;

extern motor_t motor;
void MOTOR_Init(motor_t *motor, uint8_t ID);
void MOTOR_Update(motor_t *motor, uint8_t *data);
float Angle_Calc(motor_t *motor, uint16_t raw);
#endif