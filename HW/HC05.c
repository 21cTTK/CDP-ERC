#include "stm32f10x.h"                  // Device header
#include "HC05.h"
#include <string.h>
#include <stdarg.h>
#include <math.h>

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

/// @brief Format a floating-point number as a string up to 7 char long for sending over HC-05
/// @note - **AI GENERATED**.
/// @note - used exclusively in @ref `HC05_SendStringFormatted(const char *str_f, ...)` for formatting `%f` values
///	@note - display is preceded by a '+' or '-' sign; display with minimal digits possible for the input
/// @param dest "Output" val
/// @param value input a float value
static void HC05_FormatFloat7(char *dest, float value)
{
    float absVal = fabsf(value);
    char suffix = '\0';

    if (absVal >= 1000000.0f) {
        absVal /= 1000000.0f;
        suffix = 'M';
    } else if (absVal >= 1000.0f) {
        absVal /= 1000.0f;
        suffix = 'K';
    }

    char sign = (value < 0.0f) ? '-' : '+';

    int decimals;

    if (absVal >= 100.0f)
        decimals = 1;
    else if (absVal >= 10.0f)
        decimals = 2;
    else if (absVal >= 1.0f)
        decimals = 3;
    else
        decimals = 4;

    uint32_t scale = 1;
    for (int i = 0; i < decimals; i++)
        scale *= 10;

    uint32_t scaled = (uint32_t)floorf(absVal * (float)scale + 0.5f);

    uint32_t integerPart = scaled / scale;
    uint32_t fractionalPart = scaled % scale;

    char *p = dest;

    /* Sign */
    *p++ = sign;

    /* Integer part */
    if (integerPart == 0) {
        *p++ = '0';
    } else {
        char temp[10];
        int count = 0;

        while (integerPart > 0) {
            temp[count++] = (char)('0' + integerPart % 10);
            integerPart /= 10;
        }

        while (count > 0)
            *p++ = temp[--count];
    }

    /* Fractional part */
    if (fractionalPart != 0) {
        *p++ = '.';

        char *fracStart = p;

        for (int i = decimals - 1; i >= 0; i--) {
            uint32_t divisor = 1;

            for (int j = 0; j < i; j++)
                divisor *= 10;

            *p++ = (char)('0' + (fractionalPart / divisor) % 10);
        }

        while (p > fracStart && p[-1] == '0')
            --p;

        if (p == fracStart)
            --p;    /* Remove '.' too */
    }

    /* Suffix */
    if (suffix)
        *p++ = suffix;

    *p = '\0';
}

/// @brief Format a signed integer as a string up to 5 char long for sending over HC-05
/// @note - used exclusively in @ref `HC05_SendStringFormatted(const char *str_f, ...)` for formatting `%d` values
///	@note - display is preceded by a '+' or '-' sign; display with minimal digits possible for the input
/// @param dest "Output" val
/// @param value input a signed integer value
static void HC05_FormatInt5(char *dest, int16_t value)
{
	// sign
    dest[0] = (value < 0) ? '-' : '+';

    uint16_t absVal = (value < 0) ? (uint16_t)(-value) : (uint16_t)value;

	// overflow case
    if (absVal > 9999) {
        strcpy(dest + 1, "OVF!");
        return;
    }

	// decide how many digits to display (tries to minimize it)
    if (absVal >= 1000) {
        dest[1] = '0' + absVal / 1000;
        dest[2] = '0' + (absVal / 100) % 10;
        dest[3] = '0' + (absVal / 10) % 10;
        dest[4] = '0' + absVal % 10;
        dest[5] = '\0';
    }
    else if (absVal >= 100) {
        dest[1] = '0' + absVal / 100;
        dest[2] = '0' + (absVal / 10) % 10;
        dest[3] = '0' + absVal % 10;
        dest[4] = '\0';
    }
    else if (absVal >= 10) {
        dest[1] = '0' + absVal / 10;
        dest[2] = '0' + absVal % 10;
        dest[3] = '\0';
    }
    else {
        dest[1] = '0' + absVal;
        dest[2] = '\0';
    }
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

/// @brief Send a formatted string over HC-05 Bluetooth module; API fn
/// @note - supports only `%d` for signed integers; other format specifiers are ignored
/// @note - currently DOESN'T check if the buffer overflows; do not enter str that are too long and mind ur '%d'
/// @param str_f The format string (`printf`-style)
void HC05_SendStringFormatted(const char *str_f, ...)
{
	char buffer[HC05_UART_PACKET_MAXLEN] = "";
	
	va_list args;
	va_start(args, str_f);
	
		uint16_t end = 0;
		for(const char *p = str_f; *p != '\0'; p++)
		{
			if(*p == '%')
			{// is formatted char; skip the '%...' and swap in the formatted value 
				p++;
				switch(*p)
				{
					case 'd':
					{
						int val = va_arg(args, int);
                    
						char valStr[6];		// 1 * sign char + 4 * possible digits + 1 * '\0'
						HC05_FormatInt5(valStr, (int16_t)val);
						
						for (const char *q = valStr; *q != '\0'; q++)
		                    buffer[end++] = *q;

						break;
					}
					case 'f':
					{
						float val = va_arg(args, double);
						char valStr[8];		// 1 * sign char + (decimal pt, K or M magnitude suffix, digits) <= 6 char + 1 * '\0'
						HC05_FormatFloat7(valStr, val);

						for (const char *q = valStr; *q != '\0'; q++)
							buffer[end++] = *q;

						break;
					}
					default:
						break;
				}
			}
			else
			{
				buffer[end++] = *p;
			}
		}
		
		buffer[end] = '\0';

	va_end(args);

	HC05_SendString(buffer);
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
