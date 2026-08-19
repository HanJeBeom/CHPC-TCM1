/******************************************************************************
 * @file d_dac.c
 * @author jylee1 (jylee1@gst-in.com)
 * @brief 
 * @version 0.1
 * @date 2023-05-22
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

#define AD5422_1DEV_COMMAND_SIZE	(24 / 8)	// 24bit / 8bit

#define AD5422_CMD_NOP		0x00
#define AD5422_CMD_DATA	 	0x01
#define AD5422_CMD_READBACK	0x02
#define AD5422_CMD_CTRL	 	0x55
#define AD5422_CMD_RESET	0x56

#define AD5422_CTRL_RANGE_mask	(0x07)
#define AD5422_CTRL_DCEN_mask	(0x01 << 3)
#define AD5422_CTRL_SREN_mask	(0x01 << 4)
#define AD5422_CTRL_SRSTEP_mask	(0x07 << 5)		// 128LSB
#define AD5422_CTRL_SRCLK_mask	(0x01 << 8)		// 198,410Hz
#define AD5422_CTRL_OUTEN_mask	(0x01 << 12)
#define AD5422_CTRL_REXT_mask	(0x01 << 13)
#define AD5422_CTRL_OVRRNG_mask	(0x00 << 14)	//disable
#define AD5422_CTRL_CLRSEL_mask	(0x00 << 15)	//disable

#define AD5422_RESET_EN		 (0x01)

#define AD5422_SPI_TIMEOUT		100

/*****************************************************************************/
/** DATATYPES ****************************************************************/
/*****************************************************************************/

typedef struct AD5422_command_struct_Tag {
	uint16_t cmd;							// D23 to D16
	uint16_t data;							// D15 to D0
} ad5422_command_st;

typedef struct AD5422_status_reg_struct_Tag {
	uint16_t over_temp : 1;
	uint16_t slew_active : 1;
	uint16_t Iout_fault : 1;
	uint16_t : 13;
} ad5422_status_reg_st;

/*****************************************************************************/
/** FUNCTION DECLARATIONS ****************************************************/
/*****************************************************************************/

static uint32_t ad5422_write_register( ad5422_command_st* command, uint8_t num );
static int8_t dac_range_setup( uint8_t ch, dac_vi_set_et set_range );
static void dac_output( uint8_t ch, uint16_t value );
static void dac_reset( void );
static void init_dac_daisychain( void );

/*****************************************************************************/
/** EXTERNAL VARIABLES *******************************************************/
/*****************************************************************************/

extern SPI_HandleTypeDef hspi1;

/*****************************************************************************/
/** LOCAL VARIABLES **********************************************************/
/*****************************************************************************/

static uint16_t dac_data_buff[ MAX_DAC_CH ] = { 0 };

/*****************************************************************************/
/** GLOBAL VARIABLES *********************************************************/
/*****************************************************************************/

const DAC_st AD5422 = {
	.hspi = &hspi1,
	.port = SPI_DAC,
	.pData = dac_data_buff,
	.cs = {
		.port = DAC_NSS_PORT,
		.pin = DAC_NSS_PIN,
	},
	.SetupRange = dac_range_setup,
	.Output = dac_output,
};

/*****************************************************************************/
/** FUNCTION DEFINITIONS *****************************************************/
/*****************************************************************************/

/***************************************************************************
 * @brief 
 * 
 * @param ch 
 * @param set_range 
 * @return int8_t 
 **************************************************************************/
static int8_t dac_range_setup( uint8_t ch, dac_vi_set_et set_range )
{
	if( set_range == DAC_SET_ERR )
	{
		return DAC_SET_ERR;
	}
	
	ad5422_command_st command[ MAX_DAC_CH ];

	for( uint8_t i = 0; i < MAX_DAC_CH; i++ )
	{
		if( i == ch )
		{
			command[ i ].cmd = AD5422_CMD_CTRL;
			command[ i ].data = AD5422_CTRL_CLRSEL_mask | AD5422_CTRL_OVRRNG_mask | AD5422_CTRL_REXT_mask | AD5422_CTRL_OUTEN_mask
									| AD5422_CTRL_SRCLK_mask | AD5422_CTRL_SRSTEP_mask | AD5422_CTRL_SREN_mask | AD5422_CTRL_DCEN_mask | set_range;
		}
		else
		{
			command[ i ].cmd = AD5422_CMD_NOP;
			command[ i ].data = 0;
		}
	}

	return ad5422_write_register( command, MAX_DAC_CH );
}

/***************************************************************************
 * @brief 
 * 
 **************************************************************************/
static void dac_reset( void )
{
	ad5422_command_st   command[ MAX_DAC_CH ];

	for( int i = 0; i < MAX_DAC_CH; i++ )
	{
		command[ i ].cmd = AD5422_CMD_RESET;
		command[ i ].data = AD5422_RESET_EN;
	}

	ad5422_write_register( command, MAX_DAC_CH );
}

/***************************************************************************
 * @brief 
 * 
 **************************************************************************/
static void init_dac_daisychain( void )
{
	ad5422_command_st command[ MAX_DAC_CH ];

	for( uint8_t i = 0; i < MAX_DAC_CH; i++)
	{
		command[ i ].cmd = AD5422_CMD_CTRL;
		command[ i ].data = AD5422_CTRL_CLRSEL_mask | AD5422_CTRL_OVRRNG_mask | AD5422_CTRL_REXT_mask | AD5422_CTRL_OUTEN_mask
							| AD5422_CTRL_SRCLK_mask | AD5422_CTRL_SRSTEP_mask | AD5422_CTRL_SREN_mask | AD5422_CTRL_DCEN_mask;
	}
	ad5422_write_register( command, 1 );
	ad5422_write_register( command, 2 );
}

/***************************************************************************
 * @brief 
 * 
 * @param command 
 * @param num 
 * @return uint32_t 
 * @note H/W 순서가 반대이므로 command 순서를 반대로 출력함.
 **************************************************************************/
static uint32_t ad5422_write_register( ad5422_command_st* command, uint8_t num )
{
	uint8_t txdata_buf[ AD5422_1DEV_COMMAND_SIZE * MAX_DAC_CH ] = { 0, };

	if( num == MAX_DAC_CH )
	{
		txdata_buf[ 0 ]	= ( uint8_t ) command[ 1 ].cmd;
		txdata_buf[ 1 ] = ( uint8_t )( command[ 1 ].data >> 8 );
		txdata_buf[ 2 ] = ( uint8_t )( command[ 1 ].data & 0xff );
		txdata_buf[ 3 ]	= ( uint8_t ) command[ 0 ].cmd;
		txdata_buf[ 4 ] = ( uint8_t )( command[ 0 ].data >> 8 );
		txdata_buf[ 5 ] = ( uint8_t )( command[ 0 ].data & 0xff );
	}
	else
	{
		txdata_buf[ 0 ] = ( uint8_t ) command[ 0 ].cmd;
		txdata_buf[ 1 ] = ( uint8_t )( command[ 0 ].data >> 8 );
		txdata_buf[ 2 ] = ( uint8_t )( command[ 0 ].data & 0xff );
	}

	DIO.Output( AD5422.cs.port, AD5422.cs.pin, GPIO_LOW );
	SPI.Transmit( AD5422.port, txdata_buf, num * AD5422_1DEV_COMMAND_SIZE, AD5422_SPI_TIMEOUT );
	SPI.WaitXferComplete( AD5422.port, AD5422_SPI_TIMEOUT );
	DIO.Output( AD5422.cs.port, AD5422.cs.pin, GPIO_HIGH );

	return 0;
}

/***************************************************************************
 * @brief 
 * 
 * @param ch 
 * @param value 
 **************************************************************************/
static void dac_output( uint8_t ch, uint16_t value )
{
	if( ch >= MAX_DAC_CH ) return;
	ad5422_command_st command[ MAX_DAC_CH ];

	if( 0 <= value )//&& AD5422.pData[ ch ] != value )
	{
		AD5422.pData[ ch ] = value;
	
		for( uint8_t i = 0; i < MAX_DAC_CH; i++ )
		{
			command[ i ].cmd = AD5422_CMD_DATA;
			command[ i ].data = AD5422.pData[ i ];
		}

		ad5422_write_register( command, MAX_DAC_CH );
	}
}
/***************************************************************************
 * @brief 
 * 
 **************************************************************************/
void AD5422Init( void )
{
	init_dac_daisychain();
	dac_reset();
	init_dac_daisychain();
}
