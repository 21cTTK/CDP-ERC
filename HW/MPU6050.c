#include "stm32f10x.h"
#include "HW_I2C.h"
#include <math.h>
#include "MPU6050.h"

#define MPU6050_SAMPLE_INTVL_MS			15U
#define MPU6050_GYRO_INIT_OFFSET_DPS	0.94f
#define MPU6050_PID_INITIAL_KP      	3.1415926f
#define MPU6050_PID_INITIAL_KI      	0.52f
#define MPU6050_PID_PTERM_CLAMP_PRM     114.0f
#define MPU6050_PID_ITERM_CLAMP_RPM     114.0f
#define MPU6050_PID_RAMP_SIZE_DPS  		5.0f
#define MPU6050_YAW_FILTER_COEF         0.78f

/* MPU6050 Constants */
#define MPU6050_GYRO_RAW_PER_DPS    16.4f
#define MPU6050_ID                  0x68U
#define MPU6050_ADDR_7BIT           0x68U
#define MPU6050_SMPLRT_DIV          0x19U
#define MPU6050_CONFIG              0x1AU
#define MPU6050_GYRO_CONFIG         0x1BU
#define MPU6050_ACCEL_CONFIG        0x1CU
#define MPU6050_ACCEL_XOUT_H        0x3BU
#define MPU6050_ACCEL_XOUT_L        0x3CU
#define MPU6050_ACCEL_YOUT_H        0x3DU
#define MPU6050_ACCEL_YOUT_L        0x3EU
#define MPU6050_ACCEL_ZOUT_H        0x3FU
#define MPU6050_ACCEL_ZOUT_L        0x40U
#define MPU6050_GYRO_XOUT_H         0x43U
#define MPU6050_GYRO_XOUT_L         0x44U
#define MPU6050_GYRO_YOUT_H         0x45U
#define MPU6050_GYRO_YOUT_L         0x46U
#define MPU6050_GYRO_ZOUT_H         0x47U
#define MPU6050_GYRO_ZOUT_L         0x48U
#define MPU6050_PWR_MGMT_1          0x6BU
#define MPU6050_PWR_MGMT_2          0x6CU
#define MPU6050_WHO_AM_I            0x75U

static uint8_t  	g_MPU6050_ready = 0U; // `1` if MPU6050 module is ready to be used; `0` otherwise
static int8_t		g_MPU6050_msgReady = 0U; // `1` if MPU6050 module has a new message ready; `0` if not yet; `-1` if error occurred
static uint32_t 	g_MPU6050_lastSampleMs = 0U; // last update timestamp in milliseconds
static float		g_MPU6050_gyroOffsetDps = 0.0f;
static PidParams_t	g_offsetPID; // PID params for yaw control

volatile float tg_yawDps;

/* I2C Helper fn */

static uint8_t MPU6050_WriteReg(uint8_t reg, uint8_t value)
{
	return (HW_I2C_WriteReg1Byte(MPU6050_ADDR_7BIT, reg, value) == 0U);
}

static uint8_t MPU6050_ReadReg(uint8_t reg, uint8_t *value)
{
	return (HW_I2C_ReadReg1Byte(MPU6050_ADDR_7BIT, reg, value) == 0U);
}

uint8_t MPU6050_Init(void)
{
	uint8_t id; // device id
    g_MPU6050_ready = 0U; // reset ready flag

	if ((HW_I2C_HasInitialized() == 0U) &&
        (HW_I2C_Init() == 1U))
	{// I2C can't be initialized
		return 1U;
	}

    //magical config that make MPU6050 work
	if ( (MPU6050_WriteReg(MPU6050_PWR_MGMT_1,   0x01U) == 0U) ||
		 (MPU6050_WriteReg(MPU6050_PWR_MGMT_2,   0x00U) == 0U) ||
		 (MPU6050_WriteReg(MPU6050_SMPLRT_DIV,   0x09U) == 0U) ||
		 (MPU6050_WriteReg(MPU6050_CONFIG,       0x06U) == 0U) ||
		 (MPU6050_WriteReg(MPU6050_GYRO_CONFIG,  0x18U) == 0U) ||
		 (MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x18U) == 0U) )
	{// some config doesn't go well
		return 1U;
	}

    if (MPU6050_ReadReg(MPU6050_WHO_AM_I, &id) == 0U)
	{// device ID mismatched
		return 1U;
	}

	PidParams_t initPID;
	PID_ClearParams(&initPID);
	initPID.Ki = MPU6050_PID_INITIAL_KI;
	initPID.Kp = MPU6050_PID_INITIAL_KP;
	initPID.iTermClamp = MPU6050_PID_ITERM_CLAMP_RPM;
	initPID.pTermClamp = MPU6050_PID_PTERM_CLAMP_PRM;
	initPID.rampStep   = MPU6050_PID_RAMP_SIZE_DPS;
	PID_CopyParams(&g_offsetPID, &initPID);

	g_MPU6050_gyroOffsetDps = MPU6050_GYRO_INIT_OFFSET_DPS;
    g_MPU6050_ready = 1U; // set ready flag
    return (uint8_t)(!(id == MPU6050_ID));
}

/// @brief Read the Yaw data from the MPU6050
/// @param data Pointer to store the Yaw data
/// @return 0 if successful, 1 if failed
uint8_t MPU6050_ReadYawData(int16_t *data)
{
	uint8_t high;
	uint8_t low;
    g_MPU6050_ready = 0U; // reset ready flag

    //scq
	if (data == 0)
	{// invalid pointer
		g_MPU6050_msgReady = -1;
		return 1U;
	}

	if ( (MPU6050_ReadReg(MPU6050_GYRO_ZOUT_H, &high) == 0U) ||
		 (MPU6050_ReadReg(MPU6050_GYRO_ZOUT_L, &low)  == 0U) )
	{// can't retrieve some part(s) of gyro Yaw-axis data
		g_MPU6050_msgReady = -1;
		return 1U;
	}

	*data = (int16_t)(((uint16_t)high << 8) | low);

    g_MPU6050_ready = 1U; // set ready flag
	g_MPU6050_msgReady = 1U;
	return 0U; // data secured
}

void MPU6050_YawRawToDps(int16_t rawIn)
{
	static float s_filteredYawDps = 0.0f;

	float currYawDps = (float)((float)rawIn / MPU6050_GYRO_RAW_PER_DPS) + g_MPU6050_gyroOffsetDps;

	if(s_filteredYawDps == 0.0f)
		s_filteredYawDps = currYawDps;
	else
		s_filteredYawDps = MPU6050_YAW_FILTER_COEF * (currYawDps - s_filteredYawDps);

    g_offsetPID.actual = s_filteredYawDps;
	tg_yawDps = currYawDps;
}

/// @return 1 if MPU6050 module is ready to be used; 0 otherwise
uint8_t MPU6050_IsReady(void)
{
    return g_MPU6050_ready;
}

/// @return 1 if sample time has reached; 0 otherwise
uint8_t MPU6050_SampleTimeReached(uint32_t nowMs)
{
    if ((nowMs - g_MPU6050_lastSampleMs) >= MPU6050_SAMPLE_INTVL_MS)
    {
        g_MPU6050_lastSampleMs = nowMs;
		if(g_MPU6050_msgReady == 0)
		{
			g_MPU6050_msgReady = 1U;
		}
        return 1U; // sample interval has elapsed
    }
    else
    {
        return 0U; // sample interval has not elapsed
    }
}

uint8_t MPU6050_IsMsgReady(void)
{
	if(g_MPU6050_msgReady == 1U)
	{
		g_MPU6050_msgReady = 0U; // reset the flag after reading
		return 1U; // message is ready
	}
	else
	{
		return 0U; // message is not ready: hasn't been a new sample or the last sample is broken
	}
}

void MPU6050_GetOffsetPidParams(PidParams_t **paramsOut)
{
	*paramsOut = &g_offsetPID;
}

void MPU6050_WriteOffset(float offset)
{
	g_MPU6050_gyroOffsetDps = offset;
}