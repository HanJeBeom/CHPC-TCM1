/******************************************************************************
 * @file d_eeprom.c
 * @author jylee1 (jylee1@gst-in.com)
 * @brief 
 * @version 0.1
 * @date 2020-08-05
 * 
 * @copyright Copyright (c) 2020 Global Standard Technology Co., Ltd.
 * 
 * All rights reserved.
 * This file is part of closed source software.
 * Unauthorized reproduction and redistribution prohibited.
 *****************************************************************************/

/*****************************************************************************/
/** INCLUDES *****************************************************************/
/*****************************************************************************/

#include "UserDrivers.h"

/*****************************************************************************/
/** MACRO DEFINITIONS ********************************************************/
/*****************************************************************************/

#define EEPR_DEVICE_ID 0x51 // 50 <= 24xx02, 24xx04

#define EEPR_I2C_TIMEOUT		100
#define EEPR_READY_CHECK_PERIOD	1.0F

/*****************************************************************************/
/** DATATYPES ****************************************************************/
/*****************************************************************************/

/*****************************************************************************/
/** FUNCTION DECLARATIONS ****************************************************/
/*****************************************************************************/

static uint8_t eeprom_write( uint32_t dstAdd, void * wbuf, uint32_t len );
static uint8_t eeprom_read( uint32_t srcAdd, void * rbuf, uint32_t len );
static uint8_t eeprom_ensure_ready( void );

/*****************************************************************************/
/** LOCAL VARIABLES **********************************************************/
/*****************************************************************************/

static EEPR_Status_ut EEPR_Status;
static AppTimerData_ut timerEepromReadyCheck = { .All = 0 };

/*****************************************************************************/
/** GLOBAL VARIABLES *********************************************************/
/*****************************************************************************/

/* TCM1 configuration data structure */
CCMRAM EEPR_CONFIG_st CFG;

/* EEPROM Driver */
const EEPR_st EEPR =
{
	.Write = eeprom_write,
	.Read = eeprom_read,
	.Status = &EEPR_Status,
};

/*****************************************************************************/
/** FUNCTION DEFINITIONS *****************************************************/
/*****************************************************************************/

static uint8_t eeprom_ensure_ready( void )
{
	if( EEPR.Status->initiated )
	{
		return HAL_OK;
	}

	if( HAL_OK == HAL_I2C_IsDeviceReady( I2C.PORT[ I2C_EEPR ]->Handle, EEPR_DEVICE_ID << 1, 3, 100 ) )
	{
		EEPR.Status->initiated = 1;
		return HAL_OK;
	}

	EEPR.Status->initiated = 0;
	return HAL_ERROR;
}

/******************************************************************************
 * @brief 
 * 
 * @param dstAdd 
 * @param wbuf 
 * @param len 
 * @return HAL status
 * 
 * @retval HAL_OK
 * @retval HAL_ERROR
 *****************************************************************************/
static uint8_t eeprom_write( uint32_t dstAdd, void * wbuf, uint32_t len )
{
	uint8_t ret_code = HAL_ERROR;

	uint8_t len_to_write = MIN( len, 32 - ( dstAdd % 32 ) );
	uint16_t write_index = 0;

	if( dstAdd + len >= EEPR_MAX_SZ ) return ret_code;

	if( HAL_OK != eeprom_ensure_ready() ) return ret_code;

	uint32_t tickstart = HAL_GetTick();

	while( len )
	{
		if( ( HAL_GetTick() - tickstart ) > 300 )
		{
			ret_code = HAL_TIMEOUT;
			break;
		}

		ret_code = HAL_I2C_Mem_Write( I2C.PORT[ I2C_EEPR ]->Handle, EEPR_DEVICE_ID << 1, dstAdd % 65536, I2C_MEMADD_SIZE_16BIT, &((uint8_t*)wbuf)[ write_index ], len_to_write, EEPR_I2C_TIMEOUT );

		AppTimer.Delay( 0.005F );

		if( ret_code != HAL_OK ) break;

		len -= len_to_write;
		dstAdd += len_to_write;
		write_index += len_to_write;

		len_to_write = MIN( len, 32 );
	}

	return ret_code;
}

/******************************************************************************
 * @brief 
 * 
 * @param srcAdd 
 * @param rbuf 
 * @param len s
 * @return uint8_t 
 *****************************************************************************/
static uint8_t eeprom_read( uint32_t srcAdd, void * rbuf, uint32_t len )
{
	uint8_t ret_code = HAL_ERROR;

	if( HAL_OK == eeprom_ensure_ready() )
	{
		ret_code = HAL_I2C_Mem_Read( I2C.PORT[ I2C_EEPR ]->Handle, EEPR_DEVICE_ID << 1, srcAdd, I2C_MEMADD_SIZE_16BIT, rbuf, len, EEPR_I2C_TIMEOUT );
	}

	return ret_code;
}

/******************************************************************************
 * @brief 
 * 
 *****************************************************************************/
void EepromInit( void )
{
	EEPR.Status->initiated = 0;
	(void)eeprom_ensure_ready();
	AppTimer.Start( &timerEepromReadyCheck, EEPR_READY_CHECK_PERIOD );
}

/******************************************************************************
 * @brief Retry EEPROM ready check every 1 second when boot-time init failed.
 *****************************************************************************/
void EepromTask( void )
{
	if( EEPR.Status->initiated )
	{
		return;
	}

	if( !AppTimer.IsExpired( &timerEepromReadyCheck ) )
	{
		return;
	}

	AppTimer.Start( &timerEepromReadyCheck, EEPR_READY_CHECK_PERIOD );
	(void)eeprom_ensure_ready();
}
