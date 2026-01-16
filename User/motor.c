#include "motor.h"
#include "can.h"

motor_t motor;

void MOTOR_Init(motor_t motor, uint16_t ID){
    motor.ID = ID;
}

void MOTOR_Update(motor_t motor, uint8_t *data){
    uint16_t angle = (data[0] << 8 | data[1]);
    uint16_t speed = (data[2] << 8 | data[3]);
    int16_t current = (data[4] << 8 | data[5]);

    motor.angle = map(angle, 0, 8191, 0, 360);
    motor.speed = speed;
    motor.current = map(current, -16384, 16384, -20, 20);
    motor.temperature = data[6];
    motor.err_code = data[7];
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
    if(hcan->Instance == CAN1){
        CAN_RxHeaderTypeDef RxHeader;
        uint8_t rx_data[8] = {0};
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, rx_data) == HAL_OK){
            if(RxHeader.StdId - 0x200 == MOTOR_ID){
                MOTOR_Update(motor, rx_data);
            }
        }
    }
}