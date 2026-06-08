/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PID_H__
#define __PID_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct
{
	float y1;
    float u0;
	float u1;
    float uplim;
    float downlim;
    float t;
    float output;   //y0
    float kp;
	float ki;
}Intergrator_Handle_S;  //积分器中间变量

extern Intergrator_Handle_S Spray_Current_Inter;
extern Intergrator_Handle_S Mix_Current_Inter;

void Intergrator(Intergrator_Handle_S *Intergrator_Handle, float u, float *output);
void Motor_Control_Init(void);
void Intergrator_Reset(Intergrator_Handle_S *Intergrator_Handle);

#ifdef __cplusplus
}
#endif
#endif 