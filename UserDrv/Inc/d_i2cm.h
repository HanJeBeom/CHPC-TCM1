/******************************************************************************
 * @file d_i2cm.h
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

#ifndef _D_I2CM_H_
#define _D_I2CM_H_

#ifdef __cplusplus
extern "C" {
#endif

/*****************************************************************************/
/** INCLUDES *****************************************************************/
/*****************************************************************************/

#include "UserDrivers.h"

/*****************************************************************************/
/** MACRO DEFINITIONS ********************************************************/
/*****************************************************************************/

#define I2C_POLLING_DELAY 100

/*****************************************************************************/
/** DATATYPES ****************************************************************/
/*****************************************************************************/

typedef enum I2C_Port_enum_Tag {
	I2C_EEPR = 0,
	MAX_I2C
} I2C_Port_et;

typedef enum I2C_Status_enum_Tag {
	I2C_XFER_PENDING = 1,
	I2C_XFER_DONE = 2,
} I2C_Status_et;

typedef struct I2C_PORT_struct_Tag
{
	I2C_HandleTypeDef *Handle;
	uint8_t xferBuf[ 256 ];

	union {
		struct {
			uint32_t error :30;
			uint32_t xfer :2;
		};
		uint32_t all;
	} status;
} I2C_PORT_st;

typedef struct I2C_struct_Tag
{
	I2C_PORT_st * PORT[ MAX_I2C ];
	void (*StartXfer)( I2C_Port_et port, uint8_t slave_addr, uint8_t *pckt_to_xfer, uint8_t send_bytes, uint8_t recv_bytes );
	uint8_t (*WaitForXferComplete)( I2C_Port_et port, uint32_t Timeout );
} I2C_st;

/*****************************************************************************/
/** FUNCTION DECLARATIONS ****************************************************/
/*****************************************************************************/

void I2cInit( void );

/*****************************************************************************/
/** LOCAL VARIABLES **********************************************************/
/*****************************************************************************/

/*****************************************************************************/
/** GLOBAL VARIABLES *********************************************************/
/*****************************************************************************/

extern const I2C_st I2C;

/*****************************************************************************/
/** FUNCTION DEFINITIONS *****************************************************/
/*****************************************************************************/



#ifdef __cpluscplus
}
#endif

#endif /* _D_I2CM_H_ */
