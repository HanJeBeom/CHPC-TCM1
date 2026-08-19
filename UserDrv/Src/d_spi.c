/******************************************************************************
 * @file d_spi.c
 * @author jylee1 (jylee1@gst-in.com)
 * @brief 
 * @version 0.1
 * @date 2023-05-08
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

/*****************************************************************************/
/** DATATYPES ****************************************************************/
/*****************************************************************************/

/*****************************************************************************/
/** FUNCTION DECLARATIONS ****************************************************/
/*****************************************************************************/

static bool spi_transmit( spi_port_et port, uint8_t *data, uint16_t sz, uint32_t Timeout );
static bool spi_receive( spi_port_et port, uint8_t *data, uint16_t sz, uint32_t Timeout );
static bool spi_transceive( spi_port_et port, uint8_t *txData, uint8_t *rxData, uint16_t sz, uint32_t Timeout );
static uint8_t spi_wait_xfer_complete( spi_port_et port, uint32_t Timeout );

/*****************************************************************************/
/** EXTERNAL VARIABLES *******************************************************/
/*****************************************************************************/

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;

/*****************************************************************************/
/** LOCAL VARIABLES **********************************************************/
/*****************************************************************************/

/*****************************************************************************/
/** GLOBAL VARIABLES *********************************************************/
/*****************************************************************************/

const SPI_st SPI = {
	.Transmit = spi_transmit,
	.Receive = spi_receive,
	.Transceive = spi_transceive,
	.WaitXferComplete = spi_wait_xfer_complete,
	.handle = { &hspi1, &hspi2, },
};

/*****************************************************************************/
/** FUNCTION DEFINITIONS *****************************************************/
/*****************************************************************************/

/***************************************************************************
 * @brief
 *
 * @param[in] port SPI port number.
 * @param[in] data
 * @param[in] sz
 * @param[in] Timeout
 *
 * @retval true
 * @retval false
 **************************************************************************/
static bool spi_transmit( spi_port_et port, uint8_t *data, uint16_t sz, uint32_t Timeout )
{
	SPI_HandleTypeDef *hspi = ( port == SPI_DAC ) ? &hspi1 : &hspi2;

	return HAL_SPI_Transmit( hspi, data, sz, Timeout );
	//return HAL_SPI_Transmit_IT( hspi, data, sz );
}

/*************************************************************************
 * @brief  Tx Transfer completed callback.
 * 
 * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
 *			   the configuration information for SPI module.
 ************************************************************************/
void HAL_SPI_TxCpltCallback( SPI_HandleTypeDef *hspi )
{
	if( hspi == &hspi1 )
	{
		DIO.Output( DAC_NSS, GPIO_HIGH );
	}

	if( hspi == &hspi2 )
	{
		DIO.Output( ADC12_NSS, GPIO_HIGH );
		DIO.Output( ADC34_NSS, GPIO_HIGH );
	}
}

/*************************************************************************
 * @brief
 *
 * @param[in] port SPI port number.
 * @param[out] data
 * @param[in] sz
 * @param[in] Timeout

 * @retval true
 * @retval false
 ************************************************************************/
static bool spi_receive( spi_port_et port, uint8_t *data, uint16_t sz, uint32_t Timeout )
{
	SPI_HandleTypeDef *hspi = ( port == SPI_DAC ) ? &hspi1 : &hspi2;

	return HAL_SPI_Receive( hspi, data, sz, Timeout );
//	return HAL_SPI_Receive_IT( hspi, data, sz );
}

/*************************************************************************
 * @brief  Rx Transfer completed callback.
 * 
 * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
 *			   the configuration information for SPI module.
 ************************************************************************/
void HAL_SPI_RxCpltCallback( SPI_HandleTypeDef *hspi )
{
	if( hspi == &hspi1 )
	{
		DIO.Output( DAC_NSS, GPIO_HIGH );
	}

	if( hspi == &hspi2 )
	{
		DIO.Output( ADC12_NSS, GPIO_HIGH );
		DIO.Output( ADC34_NSS, GPIO_HIGH );
	}
}

/*************************************************************************
 * @brief
 *
 * @param port SPI port number.
 * @param[in] txData
 * @param[out] rxData
 * @param[in] sz
 * @param[in] Timeout
 * 
 * @return HAL status
 * 
 * @retval HAL_OK
 * @retval HAL_BUSY
 * @retval HAL_ERROR
 ************************************************************************/
static bool spi_transceive( spi_port_et port, uint8_t *txData, uint8_t *rxData, uint16_t sz, uint32_t Timeout )
{
	SPI_HandleTypeDef *hspi = ( port == SPI_DAC ) ? &hspi1 : &hspi2;

	return HAL_SPI_TransmitReceive( hspi, txData, rxData, sz, Timeout );
//	return HAL_SPI_TransmitReceive_IT( hspi, txData, rxData, sz );
}

/*************************************************************************
 * @brief
 *
 * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
 *			   the configuration information for SPI module.
 ************************************************************************/
void HAL_SPI_TxRxCpltCallback( SPI_HandleTypeDef *hspi )
{
	if( hspi == &hspi1 )
	{
		DIO.Output( DAC_NSS, GPIO_HIGH );
	}

	if( hspi == &hspi2 )
	{
		DIO.Output( ADC12_NSS, GPIO_HIGH );
		DIO.Output( ADC34_NSS, GPIO_HIGH );
	}
}

/******************************************************************************
 * @brief 
 * 
 * @param[in] port SPI port number.
 * @param[in] Timeout SPI timeout. unit: ms
 *****************************************************************************/
static uint8_t spi_wait_xfer_complete( spi_port_et port, uint32_t Timeout )
{
	SPI_HandleTypeDef *hspi = ( port == SPI_DAC ) ? &hspi1 : &hspi2;

	uint32_t tickstart = HAL_GetTick();

	while( HAL_SPI_GetState( hspi ) != HAL_SPI_STATE_READY )
	{
		if( ( HAL_GetTick() - tickstart ) >= Timeout )
		{
			hspi->State = HAL_I2C_STATE_READY;
			__HAL_UNLOCK(hspi);

			return HAL_TIMEOUT;
		}
	}

	return HAL_OK;
}
