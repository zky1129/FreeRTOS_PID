#include "pid.h"

void PID_Init(PID_t *p, float kp, float ki, float kd, float int_max, float out_max)
{
    p->Kp = kp;  p->Ki = ki;  p->Kd = kd;
    p->IntMax = int_max;  p->OutMax = out_max;
    p->ErrorInt = 0;  p->Error0 = 0;  p->Error1 = 0;
    p->Dout = 0;  p->DoutLast = 0;
    p->Target = 0;  p->Actual = 0;  p->Out = 0;
}

void PID_Update(PID_t *p)
{
    float d;

    p->Error0 = p->Target - p->Actual;

    
    if (p->Error0 < p->IntMax && p->Error0 > -p->IntMax)
        p->ErrorInt += p->Ki * p->Error0;

    
    if (p->ErrorInt >  p->OutMax) p->ErrorInt =  p->OutMax;
    if (p->ErrorInt < -p->OutMax) p->ErrorInt = -p->OutMax;

    
    d = p->Kd * (p->Error0 - p->Error1);
    p->Dout = 0.7f * d + 0.3f * p->DoutLast;
    p->DoutLast = p->Dout;

    
    p->Out = p->Kp * p->Error0 + p->ErrorInt + p->Dout;
    if (p->Out >  p->OutMax) p->Out =  p->OutMax;
    if (p->Out < -p->OutMax) p->Out = -p->OutMax;

    p->Error1 = p->Error0;
}
