/******************************************************************************
 * @file a_usbcdc.c
 * @author jylee1 (jylee1@gst-in.com)
 * @brief 
 * @version 0.1
 * @date 2023-06-27
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
#include <UserApp.h>

/*****************************************************************************/
/** MACRO DEFINITIONS ********************************************************/
/*****************************************************************************/

#define USB_CDC_TX_CYCLE_TIME 0.001f
#define USB_TX_BUF_SIZE 512
#define USB_RX_BUF_SIZE 512
#define USB_TX_QUEUE_SIZE 4096
#define USE_CDC_TX_QUEUE

#define USB_STATE_CHECK_TIMEOUT 100

/*****************************************************************************/
/** DATATYPES ****************************************************************/
/*****************************************************************************/

/*****************************************************************************/
/** FUNCTION DECLARATIONS ****************************************************/
/*****************************************************************************/

static uint32_t usb_write( const uint8_t * const buf, uint32_t size );
static void uprintf( const char * fmt, ... );
static uint32_t usb_read( uint8_t * rbuf, uint32_t size );
static uint8_t usb_is_connected( void );
static void usb_purge( void );

/*****************************************************************************/
/** EXTERNAL VARIABLES *******************************************************/
/*****************************************************************************/

#ifdef USE_CDC_TX_QUEUE
extern uint8_t UserTxBufferFS[];
#endif

/*****************************************************************************/
/** LOCAL VARIABLES **********************************************************/
/*****************************************************************************/

GstRingBufHandle_t rbUsbRx;
static CCMRAM uint8_t usb_rx_ring_buf[ USB_RX_BUF_SIZE ];
static CCMRAM uint8_t usb_print_buf[ USB_TX_BUF_SIZE ];
#ifndef USE_CDC_TX_QUEUE
static CCMRAM uint8_t usb_write_buf[ USB_TX_BUF_SIZE ];
#endif
static CCMRAM uint8_t usb_putchar_buf;
#ifdef USE_CDC_TX_QUEUE
static CCMRAM uint8_t usb_tx_ring_buf[ USB_TX_QUEUE_SIZE ];
GstRingBufHandle_t rbUsbTx;
static AppTimerData_ut timerUsbCdcTxCycle;
#endif

/*****************************************************************************/
/** GLOBAL VARIABLES *********************************************************/
/*****************************************************************************/

const CDC_st CDC =
{
	.Write = usb_write,
	.Printf = uprintf,
	.Read = usb_read,
	.Purge = usb_purge,
	.IsConnected = usb_is_connected,
};

/*****************************************************************************/
/** FUNCTION DEFINITIONS *****************************************************/
/*****************************************************************************/

#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE * f)
#endif

#include <usb_device.h>
#include <usbd_core.h>
#include <usbd_desc.h>
#include <usbd_cdc.h>
#include <usbd_cdc_if.h>
#include "usbd_cdc_if.h"

/******************************************************************************
 * @brief standard output (file desc 1) implementation for use printf
 * 
 *****************************************************************************/
PUTCHAR_PROTOTYPE
{
	if( CDC_PortOpen() )
	{
		extern USBD_HandleTypeDef hUsbDeviceFS;
		USBD_CDC_HandleTypeDef *hcdc = ( USBD_CDC_HandleTypeDef* )hUsbDeviceFS.pClassData;

		uint32_t tickstart = HAL_GetTick();
		while( hcdc->TxState != 0 )
		{
			if( HAL_GetTick() - tickstart > USB_STATE_CHECK_TIMEOUT )
			{
				return 0;
			}
			AppTimer.Delay( 0.0001f );		// hold 100us for TxState check.
		}
		usb_putchar_buf = ( uint8_t )ch;
		if( USBD_OK != CDC_Transmit_FS( &usb_putchar_buf, 1 ) )
		{
			return 0;
		}
		tickstart = HAL_GetTick();
		while( hcdc->TxState != 0 )
		{
			if( HAL_GetTick() - tickstart > USB_STATE_CHECK_TIMEOUT )
			{
				return 0;
			}
			AppTimer.Delay( 0.0001f );
		}
	}
	return ch;
}

/******************************************************************************
 * @brief 
 * 
 * @param fmt 
 * @param ... 
 *****************************************************************************/
static void uprintf( const char * fmt, ... )
{
	va_list args;

	if( CDC_PortOpen() )
	{
		int32_t sz = 0;
		va_start( args, fmt );
		sz = vsnprintf( ( char * )usb_print_buf, USB_TX_BUF_SIZE, fmt, args );
		va_end( args );

		if( sz > 0 )
		{
			usb_print_buf[ sz ] = '\0';
			usb_write( usb_print_buf, sz );
		}
	}
}

/******************************************************************************
 * @brief 
 * 
 * @param rbuf 
 * @param size 
 * @return uint32_t 
 *****************************************************************************/
static uint32_t usb_read( uint8_t * rbuf, uint32_t size )
{
	return Ring.Get( &rbUsbRx, rbuf, size );
}

/******************************************************************************
 * @brief 
 * 
 * @return uint8_t 
 *****************************************************************************/
static uint8_t usb_is_connected( void )
{
	return CDC_PortOpen();
}

/******************************************************************************
 * @brief 
 * 
 *****************************************************************************/
void UsbCdcInit( void )
{
#ifdef USE_CDC_TX_QUEUE
	Ring.Init( &rbUsbTx, usb_tx_ring_buf, USB_TX_QUEUE_SIZE );
#endif
	Ring.Init( &rbUsbRx, usb_rx_ring_buf, USB_RX_BUF_SIZE );
}

/******************************************************************************
 * @brief 
 * 
 * @param[in] buf 
 * @param size 
 * @return uint32_t 
 *****************************************************************************/
static uint32_t usb_write( const uint8_t * const buf, uint32_t size )
{
#ifdef USE_CDC_TX_QUEUE
	if( ( NULL == buf ) || ( 0 == size ) )
	{
		return 0;
	}

	/* Queue a complete message or none of it.  Partial CSV rows cannot be
	 * distinguished from transport corruption by the host logger. */
	uint32_t used = Ring.Length( &rbUsbTx );
	uint32_t free = ( USB_TX_QUEUE_SIZE - 1U ) - used;
	if( size > free )
	{
		return 0;
	}
	return Ring.Put( &rbUsbTx, buf, size );
#else
	extern USBD_HandleTypeDef hUsbDeviceFS;
	USBD_CDC_HandleTypeDef *hcdc = ( USBD_CDC_HandleTypeDef* )hUsbDeviceFS.pClassData;
	uint32_t written = 0;

	if( ( NULL == hcdc ) || ( NULL == buf ) )
	{
		return 0;
	}

	while( written < size )
	{
		uint32_t chunk = MIN( size - written, USB_TX_BUF_SIZE );
		uint32_t tickstart = HAL_GetTick();

		if( ( chunk > 1 ) && ( 0 == ( chunk % CDC_DATA_FS_MAX_PACKET_SIZE ) ) )
		{
			chunk--;
		}

		while( hcdc->TxState != 0 )
		{
			if( HAL_GetTick() - tickstart > USB_STATE_CHECK_TIMEOUT )
			{
				return written;
			}
			AppTimer.Delay( 0.0001f );		// hold 100us for TxState check.
		}

		memcpy( usb_write_buf, &buf[ written ], chunk );

		if( USBD_OK != CDC_Transmit_FS( usb_write_buf, ( uint16_t )chunk ) )
		{
			return written;
		}

		tickstart = HAL_GetTick();
		while( hcdc->TxState != 0 )
		{
			if( HAL_GetTick() - tickstart > USB_STATE_CHECK_TIMEOUT )
			{
				return written;
			}
			AppTimer.Delay( 0.0001f );
		}

		written += chunk;
	}
	return written;
#endif
}

/******************************************************************************
 * @brief 
 * 
 *****************************************************************************/
static void usb_purge( void )
{
	Ring.Purge( &rbUsbRx );
#ifdef USE_CDC_TX_QUEUE
	Ring.Purge( &rbUsbTx );
#endif
}

/******************************************************************************
 * @brief 
 * 
 *****************************************************************************/
void UsbCdcTask( void )
{
	/* rx
		* refer CDC_Receive_FS function in USBDEVICE/App/usbd_cdc_if.c */

#ifdef USE_CDC_TX_QUEUE
	/* tx
		* transmit data from tx queue. but code below make no sense currently
		* because tx data transmitted directly without queue.
		* If you don't use the queue in the future, it won't be matter if you delete the code below. */
	if( AppTimer.IsExpired( &timerUsbCdcTxCycle ) )
	{
		extern USBD_HandleTypeDef hUsbDeviceFS;

		AppTimer.Start( &timerUsbCdcTxCycle, USB_CDC_TX_CYCLE_TIME );

		USBD_CDC_HandleTypeDef *hcdc = ( USBD_CDC_HandleTypeDef* )hUsbDeviceFS.pClassData;
		if( ( NULL != hcdc ) && ( 0 == hcdc->TxState ) )
		{
			uint16_t count = Ring.Length( &rbUsbTx );
			if( count )
			{
				count = MIN( count, APP_TX_DATA_SIZE );

				if( ( count % CDC_DATA_FS_MAX_PACKET_SIZE ) == 0 ) count--;

				if( CDC_PortOpen() )
				{
					Ring.Peek( &rbUsbTx, UserTxBufferFS, count );
					if( USBD_OK == CDC_Transmit_FS( UserTxBufferFS, count ) )
					{
						/* Discard only after USB accepted the complete chunk. */
						Ring.Get( &rbUsbTx, UserTxBufferFS, count );
					}
				}
			}
		}
	}
#endif
}
