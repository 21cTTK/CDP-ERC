#include "stm32f10x.h"                  // Device header
#include <string.h>
#include "HC05.h"

static char 	HC05_RxPacket[HC05_UART_PACKET_MAXLEN];
static uint8_t  HC05_RxFlag = 0U;       // `1`: Full Packet received and ready to be read; `0`: No packet received

typedef enum __HC05_RX_STATE_ENUM
{
	HC05_RX_STATE_PENDING_HEADER = 0,
	HC05_RX_STATE_PENDING_TAIL
} HC05_RxState;

/// @brief Helper fn used in @ref `HC05_SendString(char *String)`; Send a byte over HC-05 Bluetooth module
/// @note just a wrapper for periph fn @ref `USART_SendData(USART_TypeDef* USARTx, uint16_t Data)` following a wait for TXE
/// @param Byte The byte to send
static void HC05_SendByte(uint8_t byte)
{
	USART_SendData(USART3, byte);
	while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
}

void HC05_Init(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	
	
	//GPIO
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);					
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_Init(GPIOB, &GPIO_InitStructure);					
	
	//UART
	USART_InitTypeDef USART_InitStructure;					
	USART_InitStructure.USART_BaudRate = 9600;				
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_Init(USART3, &USART_InitStructure);

    //RX IT
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	
	//NVIC
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitStructure);	
	
	USART_Cmd(USART3, ENABLE);
}

/// @brief Send a string over HC-05 Bluetooth module; API fn
/// @param string The string to send
void HC05_SendString(char *string)
{
	uint8_t i;
	for (i = 0; string[i] != '\0'; i ++)
		HC05_SendByte(string[i]);
}

/// @brief - `Update()` fn of HC05 module;
/// @brief - interface fn for reading @ref `HC05_RxFlag`;
/// @note Check whether there's a received packet or not.
uint8_t HC05_GetRxFlag(void)
{
	return HC05_RxFlag;
}

/// @brief - `Update()` fn of HC05 module;
/// @brief - interface fn for reading @ref `HC05_RxFlag`;
/// @note - this set @ref `HC05_RxFlag` to 0 after reading
/// @param buffer Pointer to the buffer to store the received packet
void HC05_GetRxPacket(char *buffer)
{
	if (HC05_RxFlag)
	{
		strcpy(buffer, HC05_RxPacket);
		HC05_RxFlag = 0;
	}
}

void HC05_ISR(void)
{
	static HC05_RxState s_RxState = HC05_RX_STATE_PENDING_HEADER; // Rx state machine
	static uint8_t s_RxPacket_i = 0; 							  // Rx packet idx

	uint8_t RxData = USART_ReceiveData(USART3);

	switch (s_RxState)
	{
	case HC05_RX_STATE_PENDING_HEADER:
		if(RxData == HC05_UART_PACKET_HEAD && HC05_RxFlag == 0)
		{
			s_RxState = HC05_RX_STATE_PENDING_TAIL;
			s_RxPacket_i = 0;	// starts receiving packet
		}
		//else;
		break;
	case HC05_RX_STATE_PENDING_TAIL:
		if(RxData == HC05_UART_PACKET_TAIL)
		{
			HC05_RxPacket[s_RxPacket_i] = '\0';	// ends the packet
			HC05_RxFlag = 1;					// full packet received
			s_RxState = HC05_RX_STATE_PENDING_HEADER;
		}
		else
		{// read in and advance the idx
			HC05_RxPacket[s_RxPacket_i] = RxData;
			s_RxPacket_i++;	
		}
		break;
	default:
		break;
	}
}
