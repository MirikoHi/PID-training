#include "motor.h"
#include "can.h"
#include <math.h>

#define GEAR_RATIO 19
#define ENCODER_MAX 8191

void MOTOR_Init(motor_t *motor, uint8_t ID){
    motor->ID = ID;
}

void MOTOR_Update(motor_t *motor, uint8_t *data){
    uint16_t rotor_angle = (data[0] << 8 | data[1]);
    int16_t speed = (data[2] << 8 | data[3]);
    int16_t current = (data[4] << 8 | data[5]);

    motor->raw_angle = rotor_angle;
    motor->angle = Angle_Calc(motor, rotor_angle);
    motor->speed = speed;
    motor->current = map(current, -16384, 16384, -20, 20);
    motor->temperature = data[6];
    motor->err_code = data[7];
}

float Angle_Calc(motor_t *motor, uint16_t raw) {
    static const float DEG_PER_COUNT = 360.0f / ENCODER_MAX;
    
    // 初始化
    if (motor->turn_count == 0 && motor->last_raw_angle == 0) {
        motor->last_raw_angle = raw;
        return raw * DEG_PER_COUNT / GEAR_RATIO;
    }
    
    int32_t raw_diff = (int32_t)raw - (int32_t)motor->last_raw_angle;
    
    // 处理回绕
    if(raw_diff > (ENCODER_MAX / 2)){
        raw_diff -= ENCODER_MAX;
        motor->turn_count--;
    }
    else if(raw_diff < -(ENCODER_MAX / 2)){
        raw_diff += ENCODER_MAX;
        motor->turn_count++;
    }
    
    // 更新并计算
    motor->last_raw_angle = raw;
    
    // 使用公式：输出角度 = (raw/8191 + turn_count) * 360 / GEAR_RATIO
    float output_angle = ((float)raw / ENCODER_MAX + (float)motor->turn_count) * 360.0f / GEAR_RATIO;
    
    output_angle = fmodf(output_angle, 360.0f);
    if(output_angle < 0){
        output_angle += 360.0f;
    }
    
    return output_angle;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
    if(hcan->Instance == CAN1){
        CAN_RxHeaderTypeDef RxHeader;
        uint8_t rx_data[8] = {0};
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, rx_data) == HAL_OK){
            if(RxHeader.StdId - 0x200 == motor.ID){
                MOTOR_Update(&motor, rx_data);
            }
        }
    }
}