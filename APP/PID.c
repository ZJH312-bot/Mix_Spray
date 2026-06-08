#include "PID.h"

Intergrator_Handle_S Spray_Current_Inter;
Intergrator_Handle_S Mix_Current_Inter;
/*输出限幅*/
void BasicFuncMathLimiter(float f_u, float uplim, float downlim, float *f_y)
{
    if(f_u > uplim){
        *f_y = uplim;
    }
    else if (f_u < downlim){
        *f_y = downlim;
    }
    else{
        *f_y = f_u;
    }
}

/*积分器核心 y0=(T/2)(u0+u1)+y1*/
void BasicFuncMathIntergrator(Intergrator_Handle_S *Intergrator_Handle)
{
    Intergrator_Handle->output = (Intergrator_Handle->y1 + (Intergrator_Handle->u0 + Intergrator_Handle->u1)
                                * Intergrator_Handle->t / 2.f);
    
    BasicFuncMathLimiter(Intergrator_Handle->output, Intergrator_Handle->uplim, Intergrator_Handle->downlim,
                        &Intergrator_Handle->output);
}

/*积分器*/
void Intergrator(Intergrator_Handle_S *Intergrator_Handle, float u, float *output)
{
    Intergrator_Handle->u0 = u;
    BasicFuncMathIntergrator(Intergrator_Handle);
    Intergrator_Handle->u1 = u;
    Intergrator_Handle->y1 = Intergrator_Handle->output;
    *output = Intergrator_Handle->output;
}
/*积分器初值配置*/
void Intergrator_Status_init(Intergrator_Handle_S *Intergrator_Handle, float uplim, float downlim, float t, float kp, float ki)
{
    Intergrator_Handle->downlim = downlim;
    Intergrator_Handle->uplim   = uplim;
    Intergrator_Handle->t       = t;
    Intergrator_Handle->kp      = kp;
    Intergrator_Handle->ki      = ki;
}
void Intergrator_Reset(Intergrator_Handle_S *Intergrator_Handle)
{
    Intergrator_Handle->y1 = 0.f;
    Intergrator_Handle->u0 = 0.f;
    Intergrator_Handle->u1 = 0.f;
    Intergrator_Handle->output = 0.f;
}
void Motor_Control_Init(void)
{
    Intergrator_Status_init(&Spray_Current_Inter, 2399.f, 0.f, 0.001f, 5000.f, 0.0f);
    Intergrator_Status_init(&Mix_Current_Inter, 2399.f, 0.f, 0.001f, 5000.f, 10.0f);
}   