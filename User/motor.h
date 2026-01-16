#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "main.h"

/**
 * @brief 电机类型结构体
 * 
 * @param ID 电调ID
 * @param current 电机电流
 * @param speed 电机转速
 * @param angle 机械转子角度
 */
typedef struct {
    uint8_t ID;
    float current;
    float speed;
    float angle;

    uint8_t temperature;
    uint8_t err_code;
} motor_t;

extern motor_t motor;
void MOTOR_Init(motor_t motor, uint8_t ID);
void MOTOR_Update(motor_t motor, uint8_t *data);
#endif