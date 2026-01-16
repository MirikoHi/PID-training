#include "pid.h"
#include "can.h"
#include <string.h>

PID_t pid;

void PID_init(PID_t *pid, float Kp, float Ki, float Kd, float max_out, float min_out, float max_iout)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;

    pid->max_out = max_out;
    pid->min_out = min_out;
    pid->max_iout = max_iout;

    PID_clear(pid);
}

void PID_clear(PID_t *pid)
{
    pid->out = 0.0F;
    pid->Pout = 0.0F;
    pid->Iout = 0.0F;
    pid->Dout = 0.0F;
    pid->target = 0.0F;

    memset(pid->fb, 0.0F, 2);
    memset(pid->Dbuf, 0.0F, 3);
    memset(pid->error, 0.0F, 3);
}

float PID_calc(PID_t *pid, float target, float fb)
{
    pid->target = target;
    pid->fb[1] = pid->fb[0];
    pid->fb[0] = fb;

    // 误差传递
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->error[0] = pid->target - pid->fb[0];

    // P比例项
    pid->Pout = pid->Kp * pid->error[0];

    // I积分项
    pid->Iout += pid->Ki * pid->error[0];
    if(pid->Iout > pid->max_iout){
        pid->Iout = pid->max_iout;
    }
    else if(pid->Iout < -pid->max_iout){
        pid->Iout = -pid->max_iout;
    }

    // D微分项
    pid->Dbuf[2] = pid->Dbuf[1];
    pid->Dbuf[1] = pid->Dbuf[0];
    pid->Dbuf[0] = pid->error[0] - pid->error[1];
    pid->Dout = pid->Kd * pid->Dbuf[0];

    float out = pid->Pout + pid->Iout + pid->Dout;
    if(out > pid->max_out){
        out = pid->max_out;
    }
    else if(out < pid->min_out){
        out = pid->min_out;
    }

    pid->out = out;
    return pid->out;
}
