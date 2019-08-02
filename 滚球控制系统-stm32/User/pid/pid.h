#ifndef __PID_H
#define	__PID_H
#include "sys.h"

typedef struct Kpid {
	float ki;
	float kp;
	float kd;
}kpid_t; 

#define X_AngleBias         0
#define Y_AngleBias         0
#define Angle2Compare(x)  (u16)(100 * ((float)x/(float)90+0.5))  //将想设置的舵机角度转换成pwm的比较值
#define CTRL_X(angle)	    TIM_SetCompare1(TIM3, Angle2Compare((float)(angle+X_AngleBias)))
#define CTRL_Y(angle)     TIM_SetCompare2(TIM3, Angle2Compare((float)(angle+Y_AngleBias)))

extern void PID_Control(int position_x, int position_y, int aim_x, int aim_y);
extern void Control_Reset(void);

#endif /* __PID_H */

