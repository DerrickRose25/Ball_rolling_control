#include "stm32f10x.h"
#include "pwm.h"
#include "sys.h"
#include "delay.h"
#include "bsp_usart.h"
#include "pid.h"

int main(void)
{
	extern uint8_t flag;
	extern uint16_t data[8];
	
	u16 destination[9][2] = {{125, 125},{325, 125},{525, 125},
													 {125, 325},{325, 325},{525, 325},	
													 {125, 525},{325, 525},{525, 525}};
	
	u8 i=0;
	u16 position_x = 0;
	u16 position_y = 0;
	//u16 led0pwmval;
	delay_init();
	TIM3_PWM_Init(1999,719);
	USART_Config();
	Control_Reset();											 
	
	printf("欢迎使用秉火STM32开发板\n\n\n\n"); 

				
                                                                                               
	while(1)
	{
		if(flag)
		{
			flag = 0;
			if(data[1] == 'n')						//如果没有定位到球，板子置于水平
			{
				Control_Reset();
				for(i = 0; i < 8; i++)
				{
					data[i] = ' ';
				}
			}
			else if(data[1] == 'y')				//如果接收到数据
			{
				position_x = 100 * (data[2] - '0') + 10 * (data[3] - '0') + (data[4] - '0');
				position_y = 100 * (data[5] - '0') + 10 * (data[6] - '0') + (data[7] - '0');
				printf("%d%d\n",position_x,position_y);
				PID_Control(position_x, position_y, destination[1][0], destination[1][1]);
				printf("%d%d\n",destination[1][0],destination[1][1]);
				for(i = 0; i < 8; i++)
				{
					data[i] = ' ';
				}
			}
		}


	}
}
