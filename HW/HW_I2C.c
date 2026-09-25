#include "LEDs.h"
#include "Delay.h"

#define HW_I2C_CLOCK_SPEED          399999U
#define HW_I2C_TIMEOUT_INT          114514U  // Timeout allowance in INT; mainly used in /** Waiting Room **/ fn
#define HW_I2C_WRITE_SCL(x)         GPIO_WriteBit(GPIOB, GPIO_Pin_8, x ? Bit_SET : Bit_RESET);\
                                    Delay_us(1U);
#define HW_I2C_WRITE_SDA(x)         GPIO_WriteBit(GPIOB, GPIO_Pin_9, x ? Bit_SET : Bit_RESET);\
                                    Delay_us(1U);
#define HW_I2C_READ_SDA             GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_9)
#define HW_I2C_READ_SCL             GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_8)

typedef enum __HW_I2C_ERROR_ENUM {
    HW_I2C_ERROR_NONE          = 0,
    HW_I2C_ERROR_SCL_LOW       = (uint8_t)(1 << 0), /* 0b00000001 */ // -- this implies `HW_I2C_ERROR_BUS_STUCK`
    HW_I2C_ERROR_SDA_LOW       = (uint8_t)(1 << 1), /* 0b00000010 */ // -- this implies `HW_I2C_ERROR_BUS_STUCK`
    HW_I2C_ERROR_BUS_BUZY      = (uint8_t)(1 << 2), /* 0b00000100 */ // -- this implies `HW_I2C_ERROR_INIT_FAILED`
    HW_I2C_ERROR_BUS_STUCK     = (uint8_t)(1 << 3), /* 0b00001000 */ // -- this implies `HW_I2C_ERROR_INIT_FAILED`
    HW_I2C_ERROR_INIT_FAILED   = (uint8_t)(1 << 4), /* 0b00010000 */
    HW_I2C_ERROR_TIMEOUT       = (uint8_t)(1 << 5), /* 0b00100000 */
    HW_I2C_ERROR_NO_EVENT      = (uint8_t)(1 << 6), /* 0b01000000 */
    HW_I2C_ERROR_NO_FLAG       = (uint8_t)(1 << 7), /* 0b10000000 */
    HW_I2C_ERROR_COUNT
} HW_I2C_errors;

static uint8_t g_errVec;            // error vector of I2C module for debugging; see `HW_I2C_ERROR_...` macros for bitfield info
static uint8_t g_HW_I2C_isInit;

/* I2C module Error Handlers */

/// @brief controls writing to `g_errVec`
/// @note errors are set by fn failures and clear when some recovery work takes fx or starts a new round
/// @param error error bitfield to set. can be any combination of `HW_I2C_ERROR_...` macros
/// @param state state to set; can be either `SET`(set) or `RESET` (clear)
static void HW_I2C_WriteError(HW_I2C_errors error, FlagStatus state)
{
    if (state == SET)
    {
        g_errVec |= error;
    }
    else
    {
        g_errVec &= ~error;
    }
}

/// @brief check if certain errors occurs in this round
/// @param error error bitfield to check. can be any combination of `HW_I2C_ERROR_...` macros
/// @return - `1` if error is detected
/// @return - `0` otherwise
static uint8_t HW_I2C_hasError(HW_I2C_errors error)
{
    uint8_t i;
    for (i = 0U; i < HW_I2C_ERROR_COUNT; i++)
    {
        if ((error & (1 << i)) && (g_errVec & (1 << i)))
        {
            return 1U;
        }
    }

    return 0U;
}

/* Waitings' Room */

/// @brief Wait for SCL line to go high; reports error if timed out
/// @note failure of this fn relates to a direct failure of @ref `HW_I2C_ClearBus()` and may ripple upwards
/// @return - `1` if SCL is high in time
/// @return - `0` if timeout occurs; set `HW_I2C_ERROR_TIMEOUT` and `HW_I2C_ERROR_SCL_LOW`
static uint8_t HW_I2C_WaitSCLHigh(void)
{
    uint32_t timeout;

    timeout = HW_I2C_TIMEOUT_INT;
    while (HW_I2C_READ_SCL == Bit_RESET)
    {// SCL is holding low by Slave(s)
        if (--timeout == 0U)
        {
            HW_I2C_WriteError(HW_I2C_ERROR_SCL_LOW | HW_I2C_ERROR_TIMEOUT, SET);
            return 0U;
        }
    }

    return 1U;
}

/// @brief Wait for I2C bus free (idle state); reports error if timed out
/// @note failure of this fn relates to a direct failure of @ref `HW_I2C_Reinit()`; an indirect failure of @ref `HW_I2C_Init()` and may ripple upwards
/// @return - `1` if I2C bus become idle in time
/// @return - `0` if timeout occurs; and set `HW_I2C_ERROR_TIMEOUT` and `HW_I2C_ERROR_BUS_BUZY`
static uint8_t HW_I2C_WaitBusFree(void)
{
    uint32_t timeout;

    timeout = HW_I2C_TIMEOUT_INT;
    while (I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY) == SET)
    {// Transfer in progress
        if (--timeout == 0U)
        {
            HW_I2C_WriteError(HW_I2C_ERROR_TIMEOUT | HW_I2C_ERROR_BUS_BUZY, SET);
            return 0U;
        }
    }

    return 1U;
}

/* Service Fn */

/// @brief clear I2C bus;
/// @note - GPIO mode is restored to `AF_OD` later in `HW_I2C_Reinit()`; we call this fn before config GPIO there
/// @note - failure of this fn relates to a direct failure of @ref `HW_I2C_Reinit()` and may ripple upwards
/// @return - `1` if successful
/// @return - `0` if failed
static uint8_t HW_I2C_ClearBus()
{
	uint8_t pulse; // used for 9-clock recovery routine
    I2C_Cmd(I2C1, DISABLE);

    //switch GPIO to manual OD for potential 9-clock recovery
	GPIO_InitTypeDef gpio;
	gpio.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	gpio.GPIO_Mode = GPIO_Mode_Out_OD;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &gpio);

    //SCL/SDA rls attempt
    HW_I2C_WRITE_SCL(1U);
    HW_I2C_WRITE_SDA(1U);
    
    //SCL rls confirmation; this should normally pass since we're master
	if (HW_I2C_WaitSCLHigh() == 0U)
	{// return with failure
		return 0U;
	}

    //SDA rls confirmation; will enter 9-clock recovery routine if SDA is held low
	if (HW_I2C_READ_SDA == Bit_RESET)
	{
		//!IMP: 9-clock recovery routine: tick SCL up to 9 times and gen a STOP to ensure a SDA rls by Slave(s) and ready to comm 
		for (pulse = 0U; pulse < 9U; pulse++)
		{
            // gen a SCL pulse
			HW_I2C_WRITE_SCL(0U);
			HW_I2C_WRITE_SCL(1U);

            // reassure a high SCL; this should normally pass since we're master
            if (HW_I2C_WaitSCLHigh() == 0U)
            {// return with failure
                return 0U;
            }
            // finish the routine once SDA is freed
			if (HW_I2C_READ_SDA != Bit_RESET) break;
		}
	}

	// manual STOP ensues
	HW_I2C_WRITE_SCL(0U);
	HW_I2C_WRITE_SDA(0U);
	HW_I2C_WRITE_SCL(1U);
	HW_I2C_WRITE_SDA(1U);

    I2C_Cmd(I2C1, ENABLE);
    // check for healthy SDA and SCL (both rls)
    if (HW_I2C_READ_SCL != Bit_RESET)
	{
        // Need this error clear because the fn maybe called multiple times a round
        HW_I2C_WriteError(HW_I2C_ERROR_SCL_LOW, RESET);

		if (HW_I2C_READ_SDA != Bit_RESET)
        {// confident that I2C bus is totally freed when return with success
            // Need this error clear because the fn maybe called multiple times a round
            HW_I2C_WriteError(HW_I2C_ERROR_SDA_LOW, RESET); 
            return 1U;
        }
        else
        {
            HW_I2C_WriteError(HW_I2C_ERROR_SDA_LOW, SET);
        }
	}
    else
    {
        HW_I2C_WriteError(HW_I2C_ERROR_SCL_LOW, SET);
    }

    // failed with error set
    return 0U;
}

/* Service Fn */

/// @brief Reinitialize the whole `HW_I2C` module, including I2C peripheral and GPIOs
/// @note - failure of this fn relates to a direct failure of @ref `HW_I2C_Init()` and may ripple upwards
/// @return - `1` if successful
/// @return - `0` if failed
static uint8_t HW_I2C_Reinit()
{
	g_HW_I2C_isInit = 0U;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |
                           RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1,  ENABLE);

    /* Ensure a clear bus */
    
    if (HW_I2C_ClearBus() == 0U)
	{// set error and return with failure
        HW_I2C_WriteError(HW_I2C_ERROR_BUS_STUCK, SET);
        return 0U;
	}

    /* Config I2C periph */

    // GPIO: PB8 Remaps-to I2C1_SCL; PB9 Remaps-to I2C1_SDA
   	GPIO_InitTypeDef gpio;
	gpio.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	gpio.GPIO_Mode = GPIO_Mode_AF_OD;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &gpio);

    GPIO_PinRemapConfig(GPIO_Remap_I2C1, ENABLE); // Remaps

    // I2C

    //force RST
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, ENABLE);
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, DISABLE);

    I2C_InitTypeDef i2c;
	I2C_StructInit(&i2c); // set to default value    
	i2c.I2C_Mode = I2C_Mode_I2C;
	i2c.I2C_OwnAddress1 = 0x00U;
	i2c.I2C_Ack = I2C_Ack_Enable;
	i2c.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	i2c.I2C_DutyCycle = I2C_DutyCycle_2;        // faaast
	i2c.I2C_ClockSpeed = HW_I2C_CLOCK_SPEED;    // max speed
	I2C_Init(I2C1, &i2c);

    I2C_NACKPositionConfig(I2C1, I2C_NACKPosition_Current);
	I2C_AcknowledgeConfig(I2C1, ENABLE);

    I2C_Cmd(I2C1, ENABLE);

    /* Re-check if the bus is really cleared */

	if (HW_I2C_WaitBusFree() == 0U)
	{// return with failure
		return 0U;
	}

	g_HW_I2C_isInit = 1U;
	return 1U;
}

/* API Fn */

/// @note - this is just a wrapper for `static uint8_t HW_I2C_Reinit()` which do the majority of the initialization
/// @note - failure of this fn results in a not-ready state of the whole HW I2C module and may paralyse related modules that use it as well
/// @return - `1` if successful and clear `HW_I2C_ERROR_INIT_FAILED`
/// @return - `0` if failed and set `HW_I2C_ERROR_INIT_FAILED`
uint8_t HW_I2C_Init()
{
	if ((g_HW_I2C_isInit == 0U) ||
		(HW_I2C_WaitBusFree() == 0U))
	{
        if (HW_I2C_Reinit() == 0U)
        {
            HW_I2C_WriteError(HW_I2C_ERROR_INIT_FAILED, SET);
            return 0U;
        }
        return 1U;
	}
	return 1U;
}

/// @brief Check I2C errors for external use, stores it and clear it right after. `Update()` fn of HW I2C module
/// @param nowMs: Current timestamp in milliseconds
void HW_I2C_UpdateError(uint32_t nowMs)
{
    static uint8_t s_lastErrors = 0U;
    
    s_lastErrors = g_errVec;

    if( (g_errVec != 0U) &&
        (LED_IsFlickering(LED_I2C_ERROR) == 0U) )
    {
        LED_SetFlicker(nowMs, LED_I2C_ERROR, 3, 114);
    }

    HW_I2C_WriteError(s_lastErrors, RESET); // clear error vector after reporting
}
