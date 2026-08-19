/******************************************************************************
 * @file d_tcn75a.c
 * @author jylee1
 * @brief 
 * @version 0.1
 * @date 2023-06-21
 * 
 * @copyright Copyright (c) 2023
 * 
 *****************************************************************************/

/*****************************************************************************/
/** INCLUDES *****************************************************************/
/*****************************************************************************/
#include "UserDrivers.h"

/*****************************************************************************/
/** MACRO DEFINITIONS ********************************************************/
/*****************************************************************************/

#define TCN75A_DEVICE_ID 0x48 // 0b01001xxx

#define TCN75A_REG_TEMPERATURE 0x00
#define TCN75A_REG_CONFIG 0x01
#define TCN75A_REG_HYSTERESIS 0x02
#define TCN75A_REG_LIMIT_SET 0x03

#define TCN75A_CFG_NORMAL			0x00
#define TCN75A_CFG_SHUTDOWN			0x01
#define TCN75A_CFG_COMPARATOR		0x00
#define TCN75A_CFG_INTERRUPT		0x02
#define TCN75A_CFG_ALERT_POL_LOW	0x00
#define TCN75A_CFG_ALERT_POL_HIGH	0x04
#define TCN75A_CFG_FAULT_QUEUE_1	0x00
#define TCN75A_CFG_FAULT_QUEUE_2	0x08
#define TCN75A_CFG_FAULT_QUEUE_4	0x10
#define TCN75A_CFG_FAULT_QUEUE_6	0x18
#define TCN75A_CFG_RESOULTION_9		0x00
#define TCN75A_CFG_RESOULTION_10	0x20
#define TCN75A_CFG_RESOULTION_11	0x40
#define TCN75A_CFG_RESOULTION_12	0x60
#define TCN75A_CFG_BIT_ONESHOT		0x80

#define TCN75A_I2C_TIMEOUT		100

/*****************************************************************************/
/** DATATYPES ****************************************************************/
/*****************************************************************************/

/*****************************************************************************/
/** FUNCTION DECLARATIONS ****************************************************/
/*****************************************************************************/

void tcn75a_set_resolution( tcn75a_resolution_et resolution );
float tcn75a_get_temperature( void );

/*****************************************************************************/
/** EXTERNAL VARIABLES *******************************************************/
/*****************************************************************************/

/*****************************************************************************/
/** LOCAL VARIABLES **********************************************************/
/*****************************************************************************/

static tcn75a_status_ut status = { 0 };

/*****************************************************************************/
/** GLOBAL VARIABLES *********************************************************/
/*****************************************************************************/

const TCN75A_st TCN75A = {
	.SetResolution = tcn75a_set_resolution,
	.GetTemperature = tcn75a_get_temperature,
	.Status = &status,
};

/*****************************************************************************/
/** FUNCTION DEFINITIONS *****************************************************/
/*****************************************************************************/

/******************************************************************************
 * @brief 
 * 
 * @param resolution 
 *****************************************************************************/
void tcn75a_set_resolution( tcn75a_resolution_et resolution )
{
	uint8_t config[ 2 ] = { TCN75A_REG_CONFIG, 0x00 };

	config[ 1 ] = resolution;

	I2C.StartXfer( I2C_EEPR, TCN75A_DEVICE_ID, config, 2, 0 );

	status.resolution = ( resolution >> 5 ) & 0x3;

	I2C.WaitForXferComplete( I2C_EEPR, TCN75A_I2C_TIMEOUT );
}

/******************************************************************************
 * @brief 
 * 
 * @return float 
 *****************************************************************************/
float tcn75a_get_temperature( void )
{
	float cTemp = 0.0f;
	uint8_t pkt[ 2 ] = { TCN75A_REG_TEMPERATURE, 0x00 };

	if( status. initiated )
	{
		I2C.StartXfer( I2C_EEPR, TCN75A_DEVICE_ID, pkt, 1, 2 );
		I2C.WaitForXferComplete( I2C_EEPR, TCN75A_I2C_TIMEOUT );

		uint8_t temp_hibyte = I2C.PORT[ 0 ]->xferBuf[ 3 ];
		uint8_t temp_lobyte = I2C.PORT[ 0 ]->xferBuf[ 4 ];

		// Convert the data
		int temp = ( temp_hibyte * 256 + ( temp_lobyte & 0xF0 ) ) / 16;
		if( temp > 2047 )
		{
			temp -= 4096;
		}

		cTemp = temp / ( float )( 2 << status.resolution );
	}

	return cTemp;
}

/******************************************************************************
 * @brief 
 * 
 *****************************************************************************/
void TCN75AInit( void )
{
	if( HAL_OK == HAL_I2C_IsDeviceReady( I2C.PORT[ I2C_EEPR ]->Handle, TCN75A_DEVICE_ID << 1, 3, 1000 ) )
	{
		TCN75A.Status->initiated = 1;

		tcn75a_set_resolution( TCN75A_CFG_RESOULTION_12 );
	}
}
