#include "bsp_usart.h"

static void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
 
  NVIC_InitStructure.NVIC_IRQChannel = DEBUG_USART_IRQ;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}


void USART_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	DEBUG_USART_GPIO_APBxClkCmd(DEBUG_USART_GPIO_CLK, ENABLE);
	
	DEBUG_USART_APBxClkCmd(DEBUG_USART_CLK, ENABLE);

	GPIO_InitStructure.GPIO_Pin = DEBUG_USART_TX_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(DEBUG_USART_TX_GPIO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = DEBUG_USART_RX_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(DEBUG_USART_RX_GPIO_PORT, &GPIO_InitStructure);
	USART_InitStructure.USART_BaudRate = DEBUG_USART_BAUDRATE;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(DEBUG_USARTx, &USART_InitStructure);
	
	NVIC_Configuration();
	
	USART_ITConfig(DEBUG_USARTx, USART_IT_RXNE, ENABLE);	

	USART_Cmd(DEBUG_USARTx, ENABLE);	    
}


int fputc(int ch, FILE *f)
{
		USART_SendData(DEBUG_USARTx, (uint8_t) ch);
		
		while (USART_GetFlagStatus(DEBUG_USARTx, USART_FLAG_TXE) == RESET);		
	
		return (ch);
}


uint16_t data[8];
uint8_t flag;

void Read_data(void)
{
	extern uint16_t data[8];
	extern uint8_t flag;
	static int i = 0;
	data[i] = USART_ReceiveData(DEBUG_USARTx);
//	printf("ok\n");
	if(data[0] == 'p')
	{
		i++;
		//printf("%d\n", i);
	}
	if(i >= 8)
	{
		//printf("OK");
		flag = 1;
		i = 0;
	}

}


void USART1_IRQHandler(void)
{
	if(USART_GetFlagStatus(DEBUG_USARTx, USART_FLAG_RXNE))
	{
		Read_data();
	}
	
}

