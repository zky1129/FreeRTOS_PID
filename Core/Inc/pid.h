#ifndef __PID_H
#define __PID_H

typedef struct {
    float Target, Actual, Out;
    float Kp, Ki, Kd;
    float Error0, Error1;
    float ErrorInt;
    float Dout, DoutLast;
    float IntMax;
    float OutMax;
} PID_t;

void PID_Init(PID_t *p, float kp, float ki, float kd, float int_max, float out_max);
void PID_Update(PID_t *p);

#endif
