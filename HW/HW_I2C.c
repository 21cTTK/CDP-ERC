#include "LEDs.h"
#include "Delay.h"

#define HW_I2C_CLOCK_SPEED          399999U  //!< I2C clock speed in Hz; curr at max speed; remember to adjust @c I2C_DutyCycle_... of @ref HW_I2C_Reinit if changed 
#define HW_I2C_TIMEOUT_INT          114514U  // Timeout allowance in INT; mainly used in /** Waiting Room **/ fn
#define HW_I2C_WRITE_SCL(x)         GPIO_WriteBit(GPIOB, GPIO_Pin_8, x ? Bit_SET : Bit_RESET);\
                                    Delay_us(1U);
#define HW_I2C_WRITE_SDA(x)         GPIO_WriteBit(GPIOB, GPIO_Pin_9, x ? Bit_SET : Bit_RESET);\
                                    Delay_us(1U);
#define HW_I2C_READ_SDA             GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_9)
#define HW_I2C_READ_SCL             GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_8)
#define HW_I2C_PERIPH_ERROR_MASK    (I2C_SR1_BERR | I2C_SR1_ARLO | \
								     I2C_SR1_AF | I2C_SR1_OVR)

/**
 * @brief Error definition of @ref g_errReg
 */
typedef enum __HW_I2C_ERROR_ENUM {
    HW_I2C_ERROR_NONE          = 0,         /* 0b 0000 0000 0000 0000 (X) */    //!< NO error occurs, congrats
    HW_I2C_ERROR_SCL_LOW       = (uint16_t)(1 << 0),  /* 0b 0000 0001 (L) */    //!< error from @ref HW_I2C_WaitSCLHigh (rarely seen)
    //HW_I2C_ERROR_SDA_LOW     = (uint16_t)(1 << 1),  /* 0b 0000 0010 (L) */    //   reserved error bit; was syn to HW_I2C_ERROR_BUS_STUCK, so it was dropped
    HW_I2C_ERROR_BUS_BUSY      = (uint16_t)(1 << 2),  /* 0b 0000 0100 (L) */    //!< error from @ref HW_I2C_WaitBusFree
    HW_I2C_ERROR_BUS_STUCK     = (uint16_t)(1 << 3),  /* 0b 0000 1000 (L) */    //!< error from @ref HW_I2C_ClearBus
    HW_I2C_ERROR_REINIT_FAILED = (uint16_t)(1 << 4),  /* 0b 0001 0000 (L) */    //!< error from @ref HW_I2C_Reinit
    HW_I2C_ERROR_TIMEOUT       = (uint16_t)(1 << 5),  /* 0b 0010 0000 (L) */    //!< error from waiting room functions (@ref HW_I2C_WaitEvent, @ref HW_I2C_WaitFlagSet, @ref HW_I2C_WaitBusFree, @ref HW_I2C_WaitSTOPComplete )
    HW_I2C_ERROR_NO_EVENT      = (uint16_t)(1 << 6),  /* 0b 0100 0000 (L) */    //!< error from @ref HW_I2C_WaitEvent
    HW_I2C_ERROR_NO_FLAG       = (uint16_t)(1 << 7),  /* 0b 1000 0000 (L) */    //!< error from @ref HW_I2C_WaitFlagSet
    HW_I2C_ERROR_NO_START      = (uint16_t)(1 << 8),  /* 0b 0000 0001 (H) */    //!< error from @ref HW_I2C_StartTransfer
    HW_I2C_ERROR_BYTE_NACK     = (uint16_t)(1 << 10), /* 0b 0000 0100 (H) */    //!< error from @ref HW_I2C_SendByte
    HW_I2C_ERROR_TX_FAILED     = (uint16_t)(1 << 11), /* 0b 0000 1000 (H) */    //!< error from @ref HW_I2C_MasterTx
    HW_I2C_ERROR_NO_STOP       = (uint16_t)(1 << 12), /* 0b 0001 0000 (H) */    //!< error from @ref HW_I2C_WaitSTOPComplete
    HW_I2C_ERROR_RX_FAILED     = (uint16_t)(1 << 13), /* 0b 0010 0000 (H) */    //!< error from @ref HW_I2C_MasterRx
    //HW_I2C_ERROR_RERSERVED   = (uint16_t)(1 << 14), /* 0b 0100 0000 (H) */    //!< reserved error bit
    HW_I2C_ERROR_PERIPH        = (uint16_t)(1 << 15), /* 0b 1000 0000 (H) */    //!< error from I2C peripheral; set by @ref HW_I2C_UpdateError; see @ref HW_I2C_PERIPH_ERROR_MASK for details
} HW_I2C_errorsDef_t;

/// @brief Return codes of HW_I2C module for inner activities
typedef enum __HW_I2C_RETURN_CODES_ENUM
{
    HW_I2C_SUCCESS = 0U,             //!< universal I2C fn return with no error
    HW_I2C_FAILURE = !HW_I2C_SUCCESS //!< universal I2C fn return with some error(s)
} HW_I2C_RetDef_t;

static uint16_t g_errReg;           //!< 16-bit error register of I2C module for debugging; see @ref __HW_I2C_ERROR_ENUM for bitfield info
static uint8_t  g_priErr;           //!< primary error register. stores the first "culprit" error
static uint8_t  g_HW_I2C_hasInit;    //!< `1` if HW_I2C module is initialized; `0` otherwise

/* I2C module Error Handlers */
    
static HW_I2C_RetDef_t HW_I2C_AssertFailure(HW_I2C_errorsDef_t error);
static HW_I2C_RetDef_t HW_I2C_AssertCleared(HW_I2C_errorsDef_t error);

/// @brief write errors to `g_errReg` and return @ref HW_I2C_FAILURE indicating the caller function failed
/// @param error error bitfield to set. can be any combination of @ref __HW_I2C_ERROR_ENUM enum-macro
/// @return - @ref HW_I2C_FAILURE always
/// @note - errors are set by fn failures (calling @ref HW_I2C_AssertFailure ) and clear when some recovery work takes fx or starts a new round (calling @ref HW_I2C_AssertCleared )
/// @note - Use case schema: `if (fn() == HW_I2C_FAILURE) return HW_I2C_AssertFailure(HW_I2C_ERROR_...);` or with a `goto` line for centralized error handling 
static HW_I2C_RetDef_t HW_I2C_AssertFailure(HW_I2C_errorsDef_t error)
{
    g_errReg |= error;
    return HW_I2C_FAILURE;
}

/// @brief clear errors in `g_errReg` and return @ref HW_I2C_SUCCESS indicating the caller function succeeded
/// @param error error bitfield to clear. can be any combination of @ref __HW_I2C_ERROR_ENUM enum-macro
/// @return - @ref HW_I2C_SUCCESS always
static HW_I2C_RetDef_t HW_I2C_AssertCleared(HW_I2C_errorsDef_t error)
{
    g_errReg &= ~error;
    return HW_I2C_SUCCESS;
}

/* Waitings' Room */

static HW_I2C_RetDef_t HW_I2C_WaitSCLHigh(void);
static HW_I2C_RetDef_t HW_I2C_WaitEvent(uint32_t event);
static HW_I2C_RetDef_t HW_I2C_WaitFlagSet(uint32_t flag);
static HW_I2C_RetDef_t HW_I2C_WaitBusFree(void);
static HW_I2C_RetDef_t HW_I2C_WaitSTOPComplete(void);

/// @brief Wait for SCL line to go high; reports error if timed out
/// @note failure of this fn causes a direct failure of @ref HW_I2C_ClearBus and may ripple upwards
/// @return - @ref HW_I2C_SUCCESS if SCL is high in time; clear @ref HW_I2C_ERROR_SCL_LOW thru @ref HW_I2C_AssertCleared
/// @return - @ref HW_I2C_FAILURE if timeout occurs; set @ref HW_I2C_ERROR_TIMEOUT and @ref HW_I2C_ERROR_SCL_LOW thru @ref HW_I2C_AssertFailure
static HW_I2C_RetDef_t HW_I2C_WaitSCLHigh(void)
{
    uint32_t timeout;

    timeout = HW_I2C_TIMEOUT_INT;
    while (HW_I2C_READ_SCL == Bit_RESET)
    {// SCL is holding low by Slave(s)
        if (--timeout == 0U)
        {// SCL stuck low; set error and return with failure
            return HW_I2C_AssertFailure(HW_I2C_ERROR_SCL_LOW | HW_I2C_ERROR_TIMEOUT);
        }
    }

    return HW_I2C_AssertCleared(HW_I2C_ERROR_SCL_LOW); // SCL is high
}

/// @brief Wait for a specific I2C periph event to occur; reports error if timed out
/// @param event I2C event to wait for
/// @return - @ref HW_I2C_SUCCESS if event occurred in time
/// @return - @ref HW_I2C_FAILURE if timeout occurs; set @ref HW_I2C_ERROR_TIMEOUT and @ref HW_I2C_ERROR_NO_EVENT thru @ref HW_I2C_AssertFailure
/// @note - failure of this fn causes a direct failure of @ref HW_I2C_StartTransfer or @ref HW_I2C_SendByte and may ripple upwards
/// @note - this reads SR1,2 so it also serves as an ADDR bit (EV6) clearing fn (ref to RM0008 Fig.273-277)
static HW_I2C_RetDef_t HW_I2C_WaitEvent(uint32_t event)
{
	uint32_t timeout;

	timeout = HW_I2C_TIMEOUT_INT;
	while (I2C_CheckEvent(I2C1, event) == ERROR) // reads I2C1->SR1,2; loops until the event shows up
	{
		if (--timeout == 0U)
		{// Event not occurred; set error and return with failure
            return HW_I2C_AssertFailure(HW_I2C_ERROR_TIMEOUT | HW_I2C_ERROR_NO_EVENT);
		}
	}
	return HW_I2C_SUCCESS; // Event occurred
}

/// @brief Wait for a specific I2C periph flag to be set; reports error if timed out
/// @param flag I2C periph flag to wait for
/// @note - failure of this fn causes a direct failure of @ref HW_I2C_MasterRx and may ripple upwards
/// @return - @ref HW_I2C_SUCCESS if flag is set in time
/// @return - @ref HW_I2C_FAILURE if timeout occurs; set @ref HW_I2C_ERROR_TIMEOUT and @ref HW_I2C_ERROR_NO_FLAG thru @ref HW_I2C_AssertFailure
static HW_I2C_RetDef_t HW_I2C_WaitFlagSet(uint32_t flag)
{
	uint32_t timeout;

	timeout = HW_I2C_TIMEOUT_INT;
	while (I2C_GetFlagStatus(I2C1, flag) == RESET)
	{
		if (--timeout == 0U)
		{// flag not set; set error and return with failure
			return HW_I2C_AssertFailure(HW_I2C_ERROR_TIMEOUT | HW_I2C_ERROR_NO_FLAG);
		}
	}
	return HW_I2C_SUCCESS; // Flag is set
}

/// @brief Wait for I2C bus free (idle state); reports error if timed out
/// @note - failure of this fn relates to a direct failure of @ref HW_I2C_Reinit , an indirect failure of @ref HW_I2C_Init and may ripple upwards
/// @return - @ref HW_I2C_SUCCESS if I2C bus become idle in time
/// @return - @ref HW_I2C_FAILURE if timeout occurs; and set @ref HW_I2C_ERROR_TIMEOUT and @ref HW_I2C_ERROR_BUS_BUSY thru @ref HW_I2C_AssertFailure
static HW_I2C_RetDef_t HW_I2C_WaitBusFree(void)
{
    uint32_t timeout;

    timeout = HW_I2C_TIMEOUT_INT;
    while (I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY) == SET)
    {// Transfer in progress
        if (--timeout == 0U)
        {// Bus never became free (transfer in progress or a slave is holding SCL low); set error and return with failure
            return HW_I2C_AssertFailure(HW_I2C_ERROR_TIMEOUT | HW_I2C_ERROR_BUS_BUSY);
        }
    }

    return HW_I2C_SUCCESS;  // Bus is free
}

/// @brief Wait for STOP condition to complete; reports error if timed out
/// @note - failure of this fn causes a direct failure of @ref HW_I2C_MasterTx or @ref HW_I2C_MasterRx and may ripple upwards
/// @return - @ref HW_I2C_SUCCESS if STOP condition is complete in time
/// @return - @ref HW_I2C_FAILURE if timeout occurs; set @ref HW_I2C_ERROR_TIMEOUT and @ref HW_I2C_ERROR_NO_STOP thru @ref HW_I2C_AssertFailure
static HW_I2C_RetDef_t HW_I2C_WaitSTOPComplete(void)
{
	uint32_t timeout;

	timeout = HW_I2C_TIMEOUT_INT;
	while ((I2C1->CR1 & I2C_CR1_STOP) != 0U)
	{
		if (--timeout == 0U)
		{
            // STOP not completed; set error and return with failure
            return HW_I2C_AssertFailure(HW_I2C_ERROR_TIMEOUT | HW_I2C_ERROR_NO_STOP);
		}
	}
	return HW_I2C_SUCCESS; // STOP confirmed
}

/* Service Fn */

static void            HW_I2C_ClearADDR(void);
static HW_I2C_RetDef_t HW_I2C_ClearBus(void);
static HW_I2C_RetDef_t HW_I2C_Reinit(void);
static HW_I2C_RetDef_t HW_I2C_StartTransfer(uint8_t addr7);
static HW_I2C_RetDef_t HW_I2C_SendByte(uint8_t value);

/// @brief Clear the ADDR flag by reading SR1 and SR2
/// @note - currently called from @ref HW_I2C_MasterRx only, where ADDR has to be cleared under explicit control (Method2, RM0008 p.761-764)
static void HW_I2C_ClearADDR(void)
{
    (void)I2C1->SR1;
    (void)I2C1->SR2;
}

/// @brief clear I2C bus;
/// @note - GPIO mode is changed here and restored to `AF_OD` later in @ref HW_I2C_Reinit as part of the procedure
/// @note - failure of this fn causes a direct failure of @ref HW_I2C_Reinit and may ripple upwards
/// @return - @ref HW_I2C_SUCCESS if successful
/// @return - @ref HW_I2C_FAILURE if failed; set @ref HW_I2C_ERROR_BUS_STUCK thru @ref HW_I2C_AssertFailure
static HW_I2C_RetDef_t HW_I2C_ClearBus(void)
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
	if (HW_I2C_WaitSCLHigh() == HW_I2C_FAILURE)
	{// SCL stuck low; set error and return with failure
        goto err;
	}

    //SDA rls confirmation
	if (HW_I2C_READ_SDA == Bit_RESET)
	{//enter 9-clock recovery routine if SDA is held low
		//!IMP: 9-clock recovery routine: tick SCL up to 9 times and gen a STOP to ensure a SDA rls by Slave(s) and ready to comm 
		for (pulse = 0U; pulse < 9U; pulse++)
		{
            // gen a SCL pulse
			HW_I2C_WRITE_SCL(0U);
			HW_I2C_WRITE_SCL(1U);

            // reassure a high SCL; this should normally pass since we're master
            if (HW_I2C_WaitSCLHigh() == HW_I2C_FAILURE) 
            {// SCL stuck low; set error and return with failure
                goto err;
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
    // health check for lines (both SDA and SCL rls, idle state)
    if (HW_I2C_WaitSCLHigh() == HW_I2C_SUCCESS)
	{// re-affirmed a HIGH SCL
        //next check SDA
		if (HW_I2C_READ_SDA != Bit_RESET)
        {// confident that I2C bus is totally freed when return with success
            goto okk;
        }
        else
        {// I2C bus recovery routine failed and SDA still stuck; set error and return with failure
            goto err;
        }
	}
    else
    {// SCL stuck, which is rarely seen
        goto err;
    }

// legit single-exit goto lines for error handling im not abusing goto here do not call the police

err:
    return HW_I2C_AssertFailure(HW_I2C_ERROR_BUS_STUCK);    // Bus stuck
okk:
    return HW_I2C_AssertCleared(HW_I2C_ERROR_BUS_STUCK);    // Bus cleared
}

/// @brief Reinitialize the whole `HW_I2C` module, including I2C peripheral and GPIOs
/// @note - failure of this fn causes a direct failure of @ref HW_I2C_Init and may ripple upwards
/// @return - @ref HW_I2C_SUCCESS if successful; clear @ref HW_I2C_ERROR_REINIT_FAILED thru @ref HW_I2C_AssertCleared
/// @return - @ref HW_I2C_FAILURE if failed; set @ref HW_I2C_ERROR_REINIT_FAILED thru @ref HW_I2C_AssertFailure
static HW_I2C_RetDef_t HW_I2C_Reinit(void)
{
	g_HW_I2C_hasInit = 0U;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB |
                           RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1,  ENABLE);

    /* Ensure a clear bus */
    
    if (HW_I2C_ClearBus() == HW_I2C_FAILURE)
	{// set error and return with failure
        goto err;
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

	if (HW_I2C_WaitBusFree() == HW_I2C_FAILURE)
	{// bus stuck; return with failure
		goto err;
	}
    else
    {// all good, finally
        goto okk;
    }

// legit single-exit goto lines for error handling im not abusing goto here do not call the police

err:
    g_HW_I2C_hasInit = 0U;
    return HW_I2C_AssertFailure(HW_I2C_ERROR_REINIT_FAILED);    // Reinit failed
okk:
    g_HW_I2C_hasInit = 1U;
    return HW_I2C_AssertCleared(HW_I2C_ERROR_REINIT_FAILED);    // Reinit succeeded
}

/// @brief Send HW I2C address to start a Tx/Rx transfer based on further actions
/// @note - failure of this fn causes a direct failure of @ref HW_I2C_MasterTx or @ref HW_I2C_MasterRx and may ripple upwards
/// @note - DO NOT mistake it for Tx-only, this is also used for Rx to start transfer
/// @param addr7 7-bit address to transmit
/// @return - @ref HW_I2C_SUCCESS if successful; clear @ref HW_I2C_ERROR_NO_START thru @ref HW_I2C_AssertCleared
/// @return - @ref HW_I2C_FAILURE if failed; set @ref HW_I2C_ERROR_NO_START thru @ref HW_I2C_AssertFailure
static HW_I2C_RetDef_t HW_I2C_StartTransfer(uint8_t addr7)
{
    //! @warning TODO: modify the comments so it fits both Tx and Rx(not curr considered)
    // note: step-0/step-1 is shared by @ref HW_I2C_MasterTx and @ref HW_I2C_MasterRx; the address phase itself is always sent in Tx mode (RM0008 p.759), the Rx mode is then selected by the repeated START inside @ref HW_I2C_MasterRx
	/// step-0. S --> SB = 1(EV5) (belongs to procedure @ref HW_I2C_MasterTx based on RM0008 Figure 273.)
    
	I2C_GenerateSTART(I2C1, ENABLE);

    /**
     * Waiting EV5: SB=1; will be cleared later
     * 
     * `I2C_EVENT_MASTER_MODE_SELECT` means these new flags are set:
     * - BUSY : I2C transmission started and is working
     * - MSL  :  is master mode; note that this is set with SB
     * - SB   :   Start Bit
     */
	// this reads SR1 register
	if (HW_I2C_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT) == HW_I2C_FAILURE)
	{// set error and return with failure
        goto err;
	}// EV5 confirmed; advances to step-1.

    /// step-1. Address (procedure @ref `HW_I2C_MasterTx()` and RM0008 Figure 273.)

	// this writes DR register with Address --> clear SB
    /** 
      * The LSB of the 2nd arg is the flag for master Tx/Rx mode selection (ref to RM0008 p.759).
      * To enter Tx mode, we need to reset it to 0.
      * The 3rd arg helps to do this. (refer to the implementation of this fn)
      * And we need to left-shift the address (`addr7 << 1`) to leave the LSB for it.
      */
	I2C_Send7bitAddress(I2C1, (uint8_t)(addr7 << 1),
					    I2C_Direction_Transmitter);

    /**
     * Waiting EV6 (ADDR=1) and EV8_1 (TXE=1) (ref to RM0008 Figure 273.)
     * 
     * `I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED` means these new flags are set:
     * - ADDR :  address sent and acknowledged by the slave (A)
     * - TXE  :  DR emptied, the address moved into the shift register
     * - TRA  :  master is in transmitter mode
     */
    if (HW_I2C_WaitEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) == HW_I2C_FAILURE)
    {// set error and return with failure
        goto err;
    }
    else
    {
        goto okk;
    }

// legit single-exit goto lines for error handling im not abusing goto here do not call the police

err:
    return HW_I2C_AssertFailure(HW_I2C_ERROR_NO_START);    // Address failed to send
okk:
    return HW_I2C_AssertCleared(HW_I2C_ERROR_NO_START);    // Address sent
}

/// @brief Send a byte over I2C
/// @details wraps `void I2C_SendData(I2C_TypeDef *I2Cx, uint8_t Data)` and then waits for @ref I2C_EVENT_MASTER_BYTE_TRANSMITTED thru @ref HW_I2C_WaitEvent
/// @param value The byte value to send
/// @return - @ref HW_I2C_SUCCESS if successful; clear @ref HW_I2C_ERROR_BYTE_NACK thru @ref HW_I2C_AssertCleared
/// @return - @ref HW_I2C_FAILURE if failed; set @ref HW_I2C_ERROR_BYTE_NACK thru @ref HW_I2C_AssertFailure
static HW_I2C_RetDef_t HW_I2C_SendByte(uint8_t value)
{
	/// step-2_n. Data1 --> TXE = 1(EV8) --> ACK/BTF --> Data2 --> TXE = 1(EV8) --> ... (procedure @ref HW_I2C_MasterTx and RM0008 Figure 273.)

	I2C_SendData(I2C1, value);
    /**
     * Waiting EV8: TXE=1 (ref to RM0008 Figure 273.)
     * 
     * `I2C_EVENT_MASTER_BYTE_TRANSMITTED` means these new flags are set:
     * - TXE : DR emptied, ready for the next byte
     * - BTF : the byte left the shift register and was acknowledged, so both DR and the shift register are
     *         empty for that byte; it is NOT set when the slave NACKs (then AF is set instead)
     */
    if (HW_I2C_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED) == HW_I2C_FAILURE)
	{//set error and return with failure 
        return HW_I2C_AssertFailure(HW_I2C_ERROR_BYTE_NACK);
	}
    // EV8_2: TXE, BTF.
	return HW_I2C_AssertCleared(HW_I2C_ERROR_BYTE_NACK); // Byte sent
}

/* API Fn */
//the public API returns a plain boolean (`uint8_t`), not @ref HW_I2C_RetDef_t, so that the caller cannot mistake it for an error bitfield; the prototypes live in HW_I2C.h

/// @note - this is just a wrapper for @ref HW_I2C_Reinit, which does the majority of the initialization
/// @note - failure of this fn results in a not-ready state of the whole HW I2C module and may paralyse related modules that use it as well
/// @note - no error bit of its own; it propagates the child errors of @ref HW_I2C_Reinit (@ref HW_I2C_ERROR_REINIT_FAILED and, when the bus could not be freed, @ref HW_I2C_ERROR_BUS_STUCK and @ref HW_I2C_ERROR_SCL_LOW)
/// @note - neither @ref HW_I2C_MasterTx nor @ref HW_I2C_MasterRx call this fn by themselves: the caller owns the contract of calling it once before the first transfer and after any failure, which is also the only way the bus gets recovered
/// @return - boolean `0` if successful
/// @return - boolean `1` if failed
uint8_t HW_I2C_Init(void)
{
	if ((g_HW_I2C_hasInit == 0U) ||
		(HW_I2C_WaitBusFree() == HW_I2C_FAILURE))
	{
        if (HW_I2C_Reinit() == HW_I2C_FAILURE)
        {
            return 1U;
        }
        else
        {
            return 0U; // re-init succeeded
        }
	}
    else
    {
        return 0U; // already initialized and bus is free
    }
}

/// @brief I2C Master Transmitter
/// @param addr7 7-bit address to transmit to
/// @param data Pointer to data buffer
/// @param len Length of data to transmit
/// @note - Can use this fn independently but recommended to add some wrapper like @ref HW_I2C_WriteReg1Byte and call the wrapper instead
/// @note - `len == 0` is legal and then @p data may be `0`: only the address (+ STOP) is sent, may be used to probe whether a slave answers
/// @return - boolean `0` if successful
/// @return - boolean `1` if failed; set @ref HW_I2C_ERROR_TX_FAILED
uint8_t HW_I2C_MasterTx(uint8_t addr7, uint8_t *data, uint16_t len)
{
    uint16_t index;

    //scq
	if ((addr7 > 127U) || ((data == 0) && (len != 0U)))
	{// invalid parameter; set error and return with failure
        goto err;
	}

    // step-0_1.: this covers S to EV8_1 (ref to RM0008 Figure 273.)
	if (HW_I2C_StartTransfer(addr7) == HW_I2C_FAILURE)
	{// address failed to send; set error and return with failure
        goto err;
	}

    // step-2_n.: this covers Data1 to DataN, and EV8_2 (ref to RM0008 Figure 273.)
    for (index = 0U; index < len; index++)
	{
		if (HW_I2C_SendByte(data[index]) == HW_I2C_FAILURE)
		{// data byte failed to send; set error and return with failure
			goto err;
		}
	}

    // step-fin.: this covers P and finish the transmission (ref to RM0008 Figure 273.)
	I2C_GenerateSTOP(I2C1, ENABLE);
	if (HW_I2C_WaitSTOPComplete() == HW_I2C_FAILURE)
    {// failed to detect a STOP condition; set error and return with failure
        goto err;
    }
    else
    {// all good, finally
        goto okk;
    }

// legit single-exit goto lines for error handling im not abusing goto here do not call the police

err:
    (void)HW_I2C_AssertFailure(HW_I2C_ERROR_TX_FAILED);
    return 1U; // Transmission failed
okk:
    return 0U; // Transmission succeeded
}

/// @brief I2C Master Receive
/// @details - The fn follows the Master Rx Method2 described in RM0008 p.761-764
/// @note - This fn also specifies the Reg addr, since an I2C register read always starts with a write of @p reg
/// @note - Can use this fn independently but recommended to add some wrapper like @ref HW_I2C_ReadReg1Byte and call the wrapper instead
/// @param addr7 7-bit address to receive from
/// @param reg Register address to read from
/// @param data Pointer to data buffer
/// @param len Length of data to receive
/// @return - boolean `0` if successful
/// @return - boolean `1` if failed; set @ref HW_I2C_ERROR_RX_FAILED
uint8_t HW_I2C_MasterRx(uint8_t addr7, uint8_t reg, uint8_t *data, uint16_t len)
{
    uint16_t index;

    //sanity check of the caller's parameters
	if ((addr7 > 127U) || (data == 0) || (len == 0U))
	{// invalid parameter; set error and return with failure
        goto err;
	}

    // step-0_1.: this covers S to EV6 (ref to RM0008 Figure 274.)
	if (HW_I2C_StartTransfer(addr7) == HW_I2C_FAILURE)
	{// address failed to send; set error and return with failure
        goto err;
	}

    // step-2.: this covers Data1 (as Reg addr) and its ACK  
    if (HW_I2C_SendByte(reg) == HW_I2C_FAILURE)
    {// reg addr failed to send; set error and return with failure
        goto err;
    }

    // step-3_4.: this covers Sr to Data1 (as Start repeat Addr);
    // fyi, these steps are from `HW_I2C_StartTransfer`

    I2C_GenerateSTART(I2C1, ENABLE);
    /*
        Waiting EV5: SB=1; will be cleared later
        
        `I2C_EVENT_MASTER_MODE_SELECT` means this very-important new flag is set:
        
          - SB   : Start Bit
        
        these new flags, which doesn't actually matter, are also set:

          - BUSY : I2C transmission started and is working
          - MSL  : is master mode; note that this is set with SB
    */
    if (HW_I2C_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT) == HW_I2C_FAILURE)
    {// Start failed to generate; set error and return with failure
        goto err;
    }

    /* 
        The LSB of the 2nd arg is the flag for master Tx/Rx mode selection (ref to RM0008 p.759).
        To enter Rx mode, we need to set it to 1.
        The 3rd arg helps to do this. (refer to the implementation of this fn)
        And we need to left-shift the address (`addr7 << 1`) to leave the LSB for it.
    */
    I2C_Send7bitAddress(I2C1, (uint8_t)(addr7 << 1), I2C_Direction_Receiver);
    /*
        Waiting EV6: ADDR=1; will be cleared later

          - ADDR : address sent and acknowledged by the slave (A)
    */
    if (HW_I2C_WaitFlagSet(I2C_FLAG_ADDR) == HW_I2C_FAILURE)
    {// Address failed to send; set error and return with failure
        goto err;
    }

    /*
        Branch below reflects 3 different Method2 procedures as described in RM0008:

          - Fig.277(Method2,N=1)
          - Fig.276(Method2,N=2)
          - Fig.275(Method2,N>2)
    */

    if (len == 1)       // Fig.277(Method2,N=1)
    {
        //EV6_3: program ACK=0
        I2C_AcknowledgeConfig(I2C1, DISABLE);

        //EV6_3: clear ADDR by reading SR1 then SR2
        HW_I2C_ClearADDR();

        //EV6_3: program STOP=1 just after ADDR is cleared
        I2C_GenerateSTOP(I2C1, ENABLE);

        //EV7: wait for RXNE=1; data byte received
        if (HW_I2C_WaitFlagSet(I2C_FLAG_RXNE) == HW_I2C_FAILURE)
        {// failed to receive data byte; set error and return with failure
            goto err;
        }

        data[0] = I2C_ReceiveData(I2C1);
    }
    else if (len == 2)  // Fig.276(Method2,N=2)
    {
        //Exclusive fn for N=2 MasterRx transfer sequence; see fn brief
        I2C_NACKPositionConfig(I2C1, I2C_NACKPosition_Next);

        //EV6_1: acknowledge disabled
        I2C_AcknowledgeConfig(I2C1, DISABLE);

        //EV6_1: ADDR is cleared 
        HW_I2C_ClearADDR();

        //EV7_3: wait for BTF = 1
        if (HW_I2C_WaitFlagSet(I2C_FLAG_BTF) == HW_I2C_FAILURE)
        {// failed to receive data bytes; set error and return with failure
            goto err;
        }

        //EV7_3: program STOP = 1
        I2C_GenerateSTOP(I2C1, ENABLE);

        //EV7_3: read DR twice (Read Data1 and Data2) just after programming the STOP
        data[0] = I2C_ReceiveData(I2C1);
        data[1] = I2C_ReceiveData(I2C1);
    }
    else if (len > 2)   // Fig.275(Method2,N>2)
    {
        I2C_AcknowledgeConfig(I2C1, ENABLE);
        
        // Below handles Data1 to DataN-2
        
        //after EV6: Clear ADDR
        HW_I2C_ClearADDR();

        //Read normally until only 3 bytes remain (all the way to that DataN-2 legend)
        for (index = 0U; index < len - 3; index++)
        {
            //EV7: RXNE=1, cleared by reading DR register
            if (HW_I2C_WaitFlagSet(I2C_FLAG_RXNE) == HW_I2C_FAILURE)
            {//failed to receive data byte; set error and return with failure
                goto err;
            }

            data[index] = I2C_ReceiveData(I2C1);
        }

        // below handles DataN-2 thru P (ref to "When 3 bytes remain to be read" sequence below Fig.275)

        // - wait for DataN-1 received; - BTF = 1
        if (HW_I2C_WaitFlagSet(I2C_FLAG_BTF) == HW_I2C_FAILURE)
        {//failed to receive data byte; set error and return with failure
            goto err;
        }

        // - Clear ACK bit
        I2C_AcknowledgeConfig(I2C1, DISABLE);

        // - Read DataN-2 in DR => this will launch the DataN reception in the shift register
        data[index++] = I2C_ReceiveData(I2C1);

        // - wait for DataN received (BTF = 1)
        if (HW_I2C_WaitFlagSet(I2C_FLAG_BTF) == HW_I2C_FAILURE)
        {//failed to receive data byte; set error and return with failure
            goto err;
        }// - DataN received (with a NACK)

        //- Program STOP
        I2C_GenerateSTOP(I2C1, ENABLE);

        // - Read DataN-1 --> RXNE
        data[index++] = I2C_ReceiveData(I2C1);
        // - Read DataN
        data[index]   = I2C_ReceiveData(I2C1);
    }

    // Restore normal configuration for the next transaction
    I2C_AcknowledgeConfig(I2C1, ENABLE);
    I2C_NACKPositionConfig(I2C1, I2C_NACKPosition_Current);

    // wait for STOP to complete
    if (HW_I2C_WaitSTOPComplete() == HW_I2C_FAILURE)
    {// failed to detect a STOP condition; set error and return with failure
        goto err;
    }
    else
    {// allll good, finally
        goto okk;
    }

// legit single-exit goto lines for error handling im not abusing goto here do not call the police

err:
    (void)HW_I2C_AssertFailure(HW_I2C_ERROR_RX_FAILED);
    return 1U; // Reception failed
okk:
    return 0U; // Reception succeeded
}

/// @brief Write a single byte to a specified reg of a certain device; Wraps @ref HW_I2C_MasterTx
/// @param addr7 7-bit address to transmit to
/// @param reg Register address to write to
/// @param byte Data byte to write
/// @return passes the return value of @ref HW_I2C_MasterTx
/// @return - boolean `0` if successful
/// @return - boolean `1` if failed;
uint8_t HW_I2C_WriteReg1Byte(uint8_t addr7, uint8_t reg, uint8_t byte)
{
    uint8_t payload[2];

    payload[0] = reg;
    payload[1] = byte;

    return HW_I2C_MasterTx(addr7, payload, 2);
}

/// @brief Read a single byte from a specified reg of a certain device; Wraps @ref HW_I2C_ReadReg
/// @param addr7 7-bit address to receive from
/// @param reg Register address to read from
/// @param data Pointer to data buffer
/// @param len Length of data to receive
/// @return directly passes the return value of @ref HW_I2C_MasterRx
/// @return - boolean `0` if successful
/// @return - boolean `1` if failed; set @ref HW_I2C_ERROR_RX_FAILED
uint8_t HW_I2C_ReadReg1Byte(uint8_t addr7, uint8_t reg, uint8_t *byte)
{
    return HW_I2C_MasterRx(addr7, reg, byte, 1);
}

/// @brief Check if the HW I2C module has been initialized
/// @return boolean `1` if initialized, `0` otherwise
uint8_t HW_I2C_HasInitialized(void)
{
    return g_HW_I2C_hasInit;
}

/// @brief Check I2C errors for external use, store them and clear them right after. `Update()` fn of HW I2C module
/// @note - this fn should be called at the end of a round, i.e., after I2C module's other actions in the main loop 
/// @details the fn first latches the peripheral faults, then snapshots and reports the error vector of the round, and then resets @ref g_errReg and @ref g_priErr to @ref HW_I2C_ERROR_NONE
/// @param nowMs: Current timestamp in milliseconds
void HW_I2C_UpdateError(uint32_t nowMs)
{
    uint16_t pendingErrors;
    
    // check for peripheral errors
    if ((I2C1->SR1 & (uint16_t)HW_I2C_PERIPH_ERROR_MASK) != 0U)
    {// set peripheral error indicator bit
        (void)HW_I2C_AssertFailure(HW_I2C_ERROR_PERIPH);

        // BERR/ARLO/AF/OVR stay latched in SR1 until software clears them (a NACK sets AF as well), so a
        // single fault would otherwise be re-reported in every following round; clear them once latched
        I2C_ClearFlag(I2C1, HW_I2C_PERIPH_ERROR_MASK);
    }
    
    //snapshot the error vector of this round; it is cleared at the end of the fn
    pendingErrors = g_errReg;

    //Error Report Routine: currently just flash some light to indicate the error
    if ((pendingErrors != 0U) &&
        (LED_IsFlickering(LED_I2C_ERROR) == 0U))
    {
        LED_SetFlicker(nowMs, LED_I2C_ERROR, 3, 114);
    }

    // clear error vector after reporting
    (void)HW_I2C_AssertCleared(pendingErrors);
    (void)HW_I2C_AssertCleared(g_priErr);
}
