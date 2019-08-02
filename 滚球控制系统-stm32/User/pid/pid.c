#include "pid.h" 
#include "math.h"
#include "bsp_usart.h"


static float X_Position_KP = 0;     
static float X_Position_KI = 0; 
static float X_Position_KD = 0;      

static float Y_Position_KP = 0; 
static float Y_Position_KI = 0;  
static float Y_Position_KD = 0;

static float last_x_bias = 0, last_y_bias = 0;
static int Integral_x_bias = 0, Integral_y_bias = 0;
static int limit = 4; 
int bFirst_pid = 1;


float limit_angle(float angle);

void PID_Control(int position_x, int position_y, int aim_x, int aim_y)
{
	float x_bias, y_bias;
	float x_angle, y_angle;
	
	x_bias = (float)(position_x - aim_x);  	     // 计算偏差值
	y_bias = (float)(position_y - aim_y);
	printf("%f%f",x_bias,y_bias);
	
	if(fabs(x_bias) < 25) 
		Integral_x_bias = 0;
	if(fabs(y_bias) < 25)
		Integral_y_bias = 0;
	
	Integral_x_bias += x_bias;
	Integral_y_bias += y_bias;
	
	if(bFirst_pid)
	{
		
		x_angle = X_Position_KP * x_bias;  // 第一次时避免last_bias
		y_angle = Y_Position_KP * y_bias;
		bFirst_pid = 0;
	}
	else
	{
		if(fabs(x_bias) < 150 && fabs(y_bias) < 150)
		{
			X_Position_KP = 0.05;    
			X_Position_KI = 0;  
			X_Position_KD = 0;      

			Y_Position_KP = 0.03;  
			Y_Position_KI = 0;  
			Y_Position_KD = 0;
		}

    else if(fabs(x_bias) < 550 && fabs(y_bias) < 550)   // 45
		{
			X_Position_KP = 0.25;  // 0.25     
			X_Position_KI = 0.0001;  
			X_Position_KD = 5;      

			Y_Position_KP = 0.25;  
			Y_Position_KI = 0.0001;  
			Y_Position_KD = 5;			
		}
			
		x_angle = X_Position_KP*x_bias + X_Position_KI*Integral_x_bias + X_Position_KD*(x_bias-last_x_bias);  
		y_angle = Y_Position_KP*y_bias + Y_Position_KI*Integral_y_bias + Y_Position_KD*(y_bias-last_y_bias);
	}
	
	last_x_bias = x_bias;            // 保存上一次偏差
	last_y_bias = y_bias;
	
	x_angle = limit_angle(x_angle);  // 限幅并转换量程 85-95
	y_angle = limit_angle(y_angle);
	//printf("%f%f",x_angle,y_angle);
	

	
	CTRL_X(x_angle);
	CTRL_Y(y_angle);
}


void Control_Reset(void)
{
	CTRL_X(90);
	CTRL_Y(90);
	
	last_x_bias = 0;
	last_y_bias = 0;
	Integral_x_bias = 0;
	Integral_y_bias = 0;
	bFirst_pid = 1;
}
	

float limit_angle(float angle)
{
	if(angle <= -limit)
		angle = -limit;
	if(angle >= limit)
		angle = limit;

	angle = 90 - angle;   //85~95
	
	return angle;
}




