#include "pid.h"
#include "can.h"
#include <string.h>
#include <math.h>

// PID_t pid;

/**
 * @param pid
 * @param Kp 比例项系数
 * @param Ki 积分项系数
 * @param Kd 微分项系数
 * @param Kf 前馈系数
 * @param min_out pid输出最小值
 * @param max_out pid输出最大值
 * @param max_iout 积分限幅上下界
 * @param itrigger 积分分离误差阈值
 */
void PID_init(PID_t *pid, float Kp, float Ki, float Kd, float Kf, float min_out, float max_out, float max_iout, float itrigger){
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->Kf = Kf;

    pid->min_out = min_out;
    pid->max_out = max_out;
    pid->max_iout = max_iout;
    pid->itrigger = itrigger;

    PID_clear(pid);
}

void PID_clear(PID_t *pid){
    pid->out = 0.0F;
    pid->Pout = 0.0F;
    pid->Iout = 0.0F;
    pid->Dout = 0.0F;
    pid->Fout = 0.0F;
    pid->Fbuf = 0.0F;

    memset(pid->fb, 0, sizeof(pid->fb));
    memset(pid->Dbuf, 0, sizeof(pid->Dbuf));
    memset(pid->error, 0, sizeof(pid->error));
}

float PID_calc(PID_t *pid, float target, float fb){
    pid->target[1] = pid->target[0];
    pid->target[0] = target;
    pid->fb[1] = pid->fb[0];
    pid->fb[0] = fb;

    // 误差传递
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->error[0] = pid->target[0] - pid->fb[0];

    // P比例项
    pid->Pout = pid->Kp * pid->error[0];

    // I积分项
    if(select & 0b0000100){ // 梯形积分
        pid->Iout += pid->Ki * (pid->error[0] + pid->error[1]) / 2.0F;
    }
    else{
        pid->Iout += pid->Ki * pid->error[0];
    }

    if(select & 0b0010000){ // 积分分离
        if(fabs(pid->error[0]) > pid->itrigger){
            pid->Iout = 0;
        }
    }

    if(select & 0b0100000){ // 积分限幅
        if(pid->Iout > pid->max_iout){
            pid->Iout = pid->max_iout;
        }
        else if(pid->Iout < -pid->max_iout){
            pid->Iout = -pid->max_iout;
        }
    }

    // D微分项
    pid->Dbuf[2] = pid->Dbuf[1];
    pid->Dbuf[1] = pid->Dbuf[0];
    if(select & 0b0001000){ // 微分先行
        pid->Dbuf[0] = pid->fb[0] - pid->fb[1];
    }
    else{
        pid->Dbuf[0] = pid->error[0] - pid->error[1];
    }
    pid->Dout = pid->Kd * pid->Dbuf[0];

    // F前馈项
    if(select & 0b1000000){  // 前馈
        pid->Fbuf = pid->target[0] - pid->target[1];
        pid->Fout = pid->Kf * pid->Fbuf;
        
        // 前馈限幅
        if(pid->Fout > pid->max_out * 0.5F){  // 总输出的50%
            pid->Fout = pid->max_out * 0.5F;
        }
        else if(pid->Fout < -pid->max_out * 0.5F){
            pid->Fout = -pid->max_out * 0.5F;
        }
    }

    float out = pid->Pout + pid->Iout + pid->Dout + pid->Fout;
    if(out > pid->max_out){
        out = pid->max_out;
    }
    else if(out < pid->min_out){
        out = pid->min_out;
    }

    pid->out = out;
    return pid->out;
}

// 1khz TIM2
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
    if(htim->Instance == TIM2){
        float cur = 0;
        if((select & 0b11) == 0b10){
            // 位置环
            cur = PID_calc(&Lpid, diration_target, motor.angle);
        }

        if((select & 0b11) == 0b01){
            // 速度环
            cur = PID_calc(&Spid, speed_target, motor.speed);
        }
        int16_t out = map(cur, -20, 20, -16384, 16384);

        if(motor.ID <= 4){
            uint8_t data[8] = {0};
            data[(motor.ID - 1) * 2] = out >> 8;
            data[(motor.ID * 2 - 1)] = out;
            CAN_Send_Data(&hcan1, 0x200, data, 8);
        }
        else if(motor.ID >= 5){
            uint8_t data[8] = {0};
            data[(motor.ID - 5) * 2] = out >> 8;
            data[(motor.ID * 2 - 9)] = out;
            CAN_Send_Data(&hcan1, 0x1FF, data, 8);
        }
    }
}


