/******************************************************************************
 * @file d_dio.c
 * @author jylee1 (jylee1@gst-in.com)
 * @brief 
 * @version 0.1
 * @date 2023-05-03
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

#define ID_CHATTERING_PREVENT_CNT 5

/*****************************************************************************/
/** DATATYPES ****************************************************************/
/*****************************************************************************/

/*****************************************************************************/
/** LOCAL VARIABLES **********************************************************/
/*****************************************************************************/

static void digital_output( int port, int pin, bool high );
static bool digital_input( int port, int pin );
static uint8_t get_board_id( void );

/*****************************************************************************/
/** GLOBAL VARIABLES *********************************************************/
/*****************************************************************************/

DIO_t DIO = {
	.Output = digital_output,
	.Input = digital_input,
	.GetBoardID = get_board_id,
};

/*****************************************************************************/
/** FUNCTION DEFINITIONS *****************************************************/
/*****************************************************************************/

/******************************************************************************
 * @brief
 *
 * @param port
 * 
 * @return GPIO_TypeDef *
 *****************************************************************************/
static GPIO_TypeDef * find_gpio( int port )
{
	switch( port )
	{
		case 'A': case 'a': return GPIOA;
		case 'B': case 'b': return GPIOB;
		case 'C': case 'c': return GPIOC;
		case 'D': case 'd': return GPIOD;
		case 'E': case 'e': return GPIOE;
		case 'F': case 'f': return GPIOF;
		default: return NULL;
	}
}

/******************************************************************************
 * @brief
 *
 * @param port
 * @param pin
 * @param high
 *****************************************************************************/
static void digital_output( int port, int pin, bool high )
{
	GPIO_TypeDef *gpio = find_gpio( port );

	if( gpio )
	{
		HAL_GPIO_WritePin( gpio, 1 << pin, high );
	}
}

/******************************************************************************
 * @brief
 *
 * @param port
 * @param pin
 * 
 * @retval true
 * @retval false
 *****************************************************************************/
static bool digital_input( int port, int pin )
{
	GPIO_TypeDef *gpio = find_gpio( port );

	if( gpio )
	{
		return HAL_GPIO_ReadPin( gpio, 1 << pin );
	}
	return false;
}

/******************************************************************************
 * @brief Get Board ID. To prevent chattering cause by noise, such as electric
 * interference, a value must be read 5 times to be recognized as a valid value.
 *
 * @return uint8_t
 * @retval 0x00 ~ 0x0f
 *****************************************************************************/
static uint8_t get_board_id( void )
{
	static uint8_t bdid = 0xff;
	
	if( bdid > 0x0F )
	{
		uint8_t cnt = ID_CHATTERING_PREVENT_CNT;
		do
		{
			uint8_t temp_id = 0;
			temp_id = digital_input( BDID3 );
			temp_id <<= 1;
			temp_id |= digital_input( BDID2 );
			temp_id <<= 1;
			temp_id |= digital_input( BDID1 );
			temp_id <<= 1;
			temp_id |= digital_input( BDID0 );

			if( temp_id != bdid )
			{
				cnt = ID_CHATTERING_PREVENT_CNT;
				bdid = temp_id;
			}
			else
			{
				cnt--;
			}
		} while( cnt );
	}

	return bdid;
}
