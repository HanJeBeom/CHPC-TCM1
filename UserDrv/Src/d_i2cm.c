/******************************************************************************
 * @file d_i2cm.c
 * @author jylee1 (jylee1@gst-in.com)
 * @brief 
 * @version 0.1
 * @date 2020-08-06
 * 
 * @copyright Copyright (c) 2023 Global Standard Technology Co., Ltd.
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

#define I2CM_TIMEOUT 100

/*****************************************************************************/
/** DATATYPES ****************************************************************/
/*****************************************************************************/

/*****************************************************************************/
/** FUNCTION DECLARATIONS ****************************************************/
/*****************************************************************************/
static void i2c_master_xfer( I2C_Port_et port, uint8_t slave_addr, uint8_t *pckt_to_xfer, uint8_t send_bytes, uint8_t recv_bytes );
static uint8_t i2c_wait_xfer_complete( I2C_Port_et port, uint32_t Timeout );

/*****************************************************************************/
/** EXTERNAL VARIABLES *******************************************************/
/*****************************************************************************/
extern I2C_HandleTypeDef hi2c1;

/*****************************************************************************/
/** LOCAL VARIABLES **********************************************************/
/*****************************************************************************/
static I2C_PORT_st I2C_Port[ MAX_I2C ] =
{
	{
		.Handle = &hi2c1,
		.xferBuf = { 0, },
	}
};

/*****************************************************************************/
/** GLOBAL VARIABLES *********************************************************/
/*****************************************************************************/
const I2C_st I2C =
{
	.PORT[ 0 ] = &I2C_Port[ 0 ],
	.StartXfer = i2c_master_xfer,
	.WaitForXferComplete = i2c_wait_xfer_complete,
};

/*****************************************************************************/
/** FUNCTION DEFINITIONS *****************************************************/
/*****************************************************************************/

/******************************************************************************
 * @brief 
 * 
 *****************************************************************************/
void I2cInit( void )
{
}


/******************************************************************************
 * @brief 
 * 
 * @param port 
 * @param slave_addr 
 * @param pckt_to_xfer 
 * @param send_bytes 
 * @param recv_bytes 
 *****************************************************************************/
static void i2c_master_xfer( I2C_Port_et port, uint8_t slave_addr, uint8_t *pckt_to_xfer, uint8_t send_bytes, uint8_t recv_bytes )
{
	I2C_PORT_st * i2c = I2C.PORT[ port ];

	if( HAL_OK != HAL_I2C_IsDeviceReady( i2c->Handle, slave_addr << 1, 3, I2CM_TIMEOUT ) ) return;

	if( send_bytes )
	{
		i2c->xferBuf[ 0 ] = ( slave_addr << 1 );
	}

	if( recv_bytes )
	{
		i2c->xferBuf[ send_bytes ? send_bytes + 1 : 0 ] = ( slave_addr << 1 ) | 1;
	}

	for( int i = 0; i < send_bytes; ++i )
	{
		i2c->xferBuf[ i + 1 ] = pckt_to_xfer[ i ];
	}

	i2c->status.xfer = I2C_XFER_PENDING;

	if( send_bytes )
	{
		HAL_I2C_Master_Transmit( i2c->Handle, slave_addr << 1, &i2c->xferBuf[ 1 ], send_bytes, I2CM_TIMEOUT );
	}

	if( recv_bytes )
	{
		HAL_I2C_Master_Receive( i2c->Handle, slave_addr << 1 | 1, &i2c->xferBuf[ send_bytes + 2 ], recv_bytes, I2CM_TIMEOUT );
	}

	i2c->status.xfer = I2C_XFER_DONE;

	i2c->status.error = 0;
}

/******************************************************************************
 * @brief 
 * 
 * @param port 
 *****************************************************************************/
static uint8_t i2c_wait_xfer_complete( I2C_Port_et port, uint32_t Timeout )
{
	I2C_PORT_st *i2c = I2C.PORT[ port ];

	uint32_t Tickstart = HAL_GetTick();

	I2C_HandleTypeDef * hi2c = i2c->Handle;

	while( i2c->status.xfer == I2C_XFER_PENDING )
	{
		if (((HAL_GetTick() - Tickstart) > Timeout) || (Timeout == 0U))
		{
			hi2c->PreviousState     = HAL_I2C_MODE_NONE;
			hi2c->State             = HAL_I2C_STATE_READY;
			hi2c->Mode              = HAL_I2C_MODE_NONE;
			hi2c->ErrorCode         |= HAL_I2C_ERROR_TIMEOUT;

			/* Process Unlocked */
			__HAL_UNLOCK(hi2c);

			return HAL_TIMEOUT;
		}
	}

	return HAL_OK;
}
