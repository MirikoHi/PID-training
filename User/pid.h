#ifndef __PID_H__
#define __PID_H__

#include "main.h"
#include "motor.h"

typedef struct{ 

    float Kp;
    float Ki;
    float Kd;

    float max_out; // 最大输出
	float min_out; // 最小输出
    float max_iout; // 最大积分输出
    float itrigger; // 积分阈值

    float target;
    float fb[2];

    float out;
    float Pout;
    float Iout;
    float Dout;
    float Dbuf[3];  // 微分项 0最新 1上一次 2上上次
    float error[3]; // 误差项 0最新 1上一次 2上上次
}PID_t;

extern PID_t Spid;
extern PID_t Lpid;

void PID_init(PID_t *pid, float Kp, float Ki, float Kd, float max_out, float min_out, float max_iout, float itrigger);
void PID_clear(PID_t *pid);
float PID_calc(PID_t *pid, float target, float fb);

#endif