/******************************************************************************
 * @file d_uart.c
 * @author jylee1 (jylee1@gst-in.com)
 * @brief 
 * @version 0.1
 * @date 2020-07-20
 * 
 * @copyright Copyright (c) 2020 Global Standard Technology Co., Ltd.
 * 
 * All rights reserved.
 * This file is part of closed source software.
 * Unauthorized reproduction and redistribution prohibited.
 *****************************************************************************/

#include "UserDrivers.h"

/*****************************************************************************/
/** INCLUDES *****************************************************************/
/*****************************************************************************/

/*****************************************************************************/
/** MACRO DEFINITIONS ********************************************************/
/*****************************************************************************/

#define UART_DMA  1
#define UART_IT   2
#define UART_POLL 3

#define UART_TX_MODE UART_DMA
#define UART_RX_MODE UART_DMA

#define RS485_DE_HW_CTRL 1


#define UART_OUT_BUFFER_SZ 256
#define UART_IN_BUFFER_SZ 256


/*****************************************************************************/
/** DATATYPES ****************************************************************/
/*****************************************************************************/

/*****************************************************************************/
/** FUNCTION DECLARATIONS ****************************************************/
/*****************************************************************************/

static uint32_t get_wordlength( uint8_t databits, uint8_t parity );
static uint32_t get_stopbits( uint8_t sbits );
static uint32_t get_parity( uint8_t parity );

void UART_Open( UART_PORT_et port, uint32_t ulBaudrate, uint16_t uDatalen, uint16_t uStopbit, uint16_t uParity );
void UART_Close( UART_PORT_et port );
static int32_t UART_Read( UART_PORT_et port, uint8_t *buf, int32_t length );
static int32_t UART_Write( UART_PORT_et port, uint8_t *buf, int32_t length );
static void UartError_Handler(void);

/*****************************************************************************/
/** EXTERNAL VARIABLES *******************************************************/
/*****************************************************************************/

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

/*****************************************************************************/
/** LOCAL VARIABLES **********************************************************/
/*****************************************************************************/

//static AppTimer_Instance_st timeoutUart[ MAX_UART ];

static CCMRAM GstRingBufHandle_t uart_rxq[ MAX_UART ];
static CCMRAM uint8_t uart_rx_ringbuf[ MAX_UART ][ UART_RX_RING_SZ ];
static uint8_t uartrxbuf[ MAX_UART ][ UART_IN_BUFFER_SZ ];
static uint8_t uarttxbuf[ MAX_UART ][ UART_OUT_BUFFER_SZ ];

static UART_Instance_st Uart_PLC_Inst = {0};
static UART_Instance_st Uart_SMPS_Inst = {0};

/*****************************************************************************/
/** GLOBAL VARIABLES *********************************************************/
/*****************************************************************************/

const UART_st UART = {
	.PORT = { &Uart_PLC_Inst, &Uart_SMPS_Inst },
	.Open = UART_Open,
	.Close = UART_Close,
	.Read = UART_Read,
	.Write = UART_Write
};

/*****************************************************************************/
/** FUNCTION DEFINITIONS *****************************************************/
/*****************************************************************************/

/******************************************************************************
 * @brief Get the word length
 * 
 * @param databits 
 * @param parity 
 * @return uint32_t 
 *****************************************************************************/
static uint32_t get_wordlength( uint8_t databits, uint8_t parity )
{
	uint32_t hal_databits;
	uint8_t wordlength = databits;

	if( UART_PARITY_NONE != get_parity( parity ) )
	{
		wordlength += 1;
	}

	switch( wordlength )
	{
		case 8:		hal_databits = UART_WORDLENGTH_8B;	break;
		case 9:		hal_databits = UART_WORDLENGTH_9B;	break;
		default:	hal_databits = UART_WORDLENGTH_8B;	break;
	}

	return hal_databits;
}

/******************************************************************************
 * @brief Get the stopbits
 * 
 * @param sbits 
 * @return uint32_t 
 *****************************************************************************/
static uint32_t get_stopbits( uint8_t sbits )
{
	uint32_t hal_stopbits;

	switch( sbits )
	{
		case 1:		hal_stopbits = UART_STOPBITS_1;		break;
		case 2:		hal_stopbits = UART_STOPBITS_2;		break;
		default:	hal_stopbits = UART_STOPBITS_1;		break;
	}
	return hal_stopbits;
}

/******************************************************************************
 * @brief Get the parity
 * 
 * @param parity 
 * @return uint32_t 
 *****************************************************************************/
static uint32_t get_parity( uint8_t parity )
{
	uint32_t hal_parity;

	switch( parity )
	{
		case 0:		hal_parity = UART_PARITY_NONE;	break;
		case 1:		hal_parity = UART_PARITY_ODD;	break;
		case 2:		hal_parity = UART_PARITY_EVEN;	break;
		default:	hal_parity = UART_PARITY_NONE;	break;
	}

	return hal_parity;
}

/******************************************************************************
 * @brief 
 * 
 * @param port 
 * @param ulBaudrate 
 * @param uDatalen 
 * @param uStopbit 
 * @param uParity 
 *****************************************************************************/
void UART_Open( UART_PORT_et port, uint32_t ulBaudrate, uint16_t uDatalen, uint16_t uStopbit, uint16_t uParity )
{
	UART_HandleTypeDef * huart = NULL;
	UART_Instance_st * uart = NULL;

	switch( port )
	{
		case UART_PLC:
			huart = &huart2;
			break;
		case UART_SMPS:
			huart = &huart3;
			break;
		default: return;
	}

	uart = UART.PORT[ port ];

	uart->handle = huart;
	uart->init_req = 0;
	uart->func = NULL;

	HAL_UART_DeInit( UART.PORT[ port ]->handle );

	uart->handle->Init.BaudRate = ulBaudrate;
	uart->handle->Init.WordLength = get_wordlength( uDatalen, uParity );
	uart->handle->Init.StopBits = get_stopbits( uStopbit );
	uart->handle->Init.Parity = get_parity( uParity );
	if( HAL_UART_Init( uart->handle ) != HAL_OK )
	{
		UartError_Handler();
	}

	uart->rxq = &uart_rxq[ port ];
	uart->handle = huart;
	uart->init_req = 0;
	uart->func = NULL;
	uart->txbuf = uarttxbuf[ port ];
	uart->txbufsz = UART_OUT_BUFFER_SZ;
	uart->rxbuf = uartrxbuf[ port ];
	uart->rxbufsz = UART_IN_BUFFER_SZ;
	uart->stat.all = 0;

	Ring.Init( uart->rxq, uart_rx_ringbuf[ port ], sizeof( uart_rx_ringbuf[ port ] ) );

	uart->stat.inited = 1;

#if UART_RX_MODE == UART_DMA
	HAL_UART_Receive_DMA( uart->handle, uart->rxbuf, uart->rxbufsz );
#elif UART_RX_MODE == UART_IT
	HAL_UART_Receive_IT( uart->handle, uart->rxbuf, 1 );
#endif

}

/******************************************************************************
 * @brief 
 * 
 * @param port 
 * @param enable 
 *****************************************************************************/
static void uart_rx_enable( UART_PORT_et port, uint8_t enable )
{
	if( port == UART_PLC )
	{
		DIO.Output( UART2_RE, enable ? GPIO_HIGH : GPIO_LOW );
	}
	else if( port == UART_SMPS )
	{
		DIO.Output( UART3_RE, enable ? GPIO_HIGH : GPIO_LOW );
	}
}

/******************************************************************************
 * @brief 
 * 
 * @param port 
 * @param tx_enable 
 *****************************************************************************/
static void uart_tx_enable( UART_PORT_et port, uint8_t enable )
{
	if( port == UART_PLC )
	{
		DIO.Output( UART2_DE, enable ? GPIO_HIGH : GPIO_LOW );
	}
	else if( port == UART_SMPS )
	{
		DIO.Output( UART3_DE, enable ? GPIO_HIGH : GPIO_LOW );
	}
	DIO.Output( LED_COM, enable );
}

#if UART_RX_MODE == UART_DMA
/******************************************************************************
 * @brief This function will be called from UART task and DMA Half/Full Completion ISR. 
 *        The uart RX queue will be corrupted by reentrant problem.
 *        So it must be locked before do to access NDTR of DMA register and to put data to queue.
 * 
 * @note The reentrant issue is occurred when called from ISR during executed from UART Rx task.
 *       The reason of called from ISR is to prevent the loss of received data
 *       when the UART task is entered "Ready state" and suddenly a lot of data is received.
 *       UART RX DMA half/full completion interrupt will be occurred even in this situation.
 *
 *       If so, data loss can be prevented sufficiently by calling this function from the ISR.
 *       But calling this function from the ISR is occurred reentrant problem.
 *       So it use lock feature for solve the reentrant problem.
 * 
 * @param uart 
 *****************************************************************************/
void check_rxdma_buffer( UART_Instance_st * uart )
{
    if( uart->stat.dma_lock == 0 )
    {
		uart->stat.dma_lock = 1;

		/* Calculate current position in buffer */
		volatile uint32_t ndtr = uart->handle->hdmarx->Instance->NDTR;
		volatile size_t dmabufsz = uart->handle->RxXferSize;
		volatile size_t pos = dmabufsz - ndtr;
		volatile size_t old_pos = uart->lastRcvdPos;

		if( pos != old_pos )
		{
			DIO.Output( LED_COM, true );
			uart->stat.received = 1;
			if( pos > old_pos ) /* Current position is over previous one */
			{
				/* We are in "linear" mode */
				/* Process data directly by subtracting "pointers" */
				Ring.Put( uart->rxq, &uart->rxbuf[ old_pos ], pos - old_pos );
			}
			else
			{
				/* We are in "overflow" mode */
				/* First process data to the end of buffer */
				Ring.Put( uart->rxq, &uart->rxbuf[ old_pos ], dmabufsz - old_pos );
				if( pos )
				{
					/* Check and continue with beginning of buffer */
					Ring.Put( uart->rxq, &uart->rxbuf[ 0 ], pos );
				}
			}
			uart->lastRcvdTimestamp = AppTimer.GetCurrentSecs();
			uart->lastRcvdPos = pos;
			DIO.Output( LED_COM, false );
		}
		uart->stat.dma_lock = 0;
    }
}
#endif

/******************************************************************************
 * @brief 
 * 
 * @param port 
 *****************************************************************************/
void UART_Close( UART_PORT_et port )
{
	const UART_Instance_st * uart = UART.PORT[ port ];
#if UART_TX_MODE == UART_DMA
	HAL_UART_AbortTransmit( uart->handle );
#elif UART_TX_MODE == IT
	HAL_UART_AbortTransmit_IT( uart->handle );
#endif
	HAL_UART_DeInit( uart->handle );
}

/******************************************************************************
 * @brief 
 * 
 *****************************************************************************/
void UartInit( void )
{
	UART_Open( UART_PLC, PLC_UART_BAUDRATE, 8, 1, 0 );
	UART_Open( UART_SMPS, SMPS_UART_BAUDRATE, 8, 1, 0 );

	uart_rx_enable( UART_PLC, 1 );
	uart_rx_enable( UART_SMPS, 1 );
	uart_tx_enable( UART_PLC, 0 );
	uart_tx_enable( UART_SMPS, 0 );

	UART_Write( UART_PLC, ( uint8_t * )"\xAA\xAA\xAA\xAA\xAA\xAA\xAA\xAA\xAA\xAA", 10 );
	UART_Write( UART_SMPS, ( uint8_t * )"\xAA\xAA\xAA\xAA\xAA\xAA\xAA\xAA\xAA\xAA", 10 );
}

/******************************************************************************
 * @brief 
 * 
 *****************************************************************************/
void UartRxTask( void )
{
	UART_Instance_st * uart = NULL;
	static UART_PORT_et port = MAX_UART;

	TICK_UP( port, MAX_UART );
	uart = UART.PORT[ port ];

	if( NULL == uart->handle ) return;
#if 0 // Does it need?
	if( uart->init_req )
	{
		if( UART_PLC == port )
		{
			UART.Open( UART_PLC, CFG.uart.baudrate * 100UL, 8, CFG.uart.stopbit, CFG.uart.parity );
		}
		else
		{
			UART.Open( UART_SMPS, 115200UL, 8, 1, 0 );
		}
	}
#endif
#if UART_RX_MODE == UART_DMA
	check_rxdma_buffer( uart );
#elif UART_RX_MODE == UART_IT
	if( uart->handle->RxISR == NULL )
	{
		HAL_UART_Receive_IT( uart->handle, uart->rxbuf, 1 );
	}
#endif
}

/******************************************************************************
 * @brief 
 * 
 * @param port 
 * @param buf 
 * @param length 
 * @return uint32_t 
 *****************************************************************************/
static int32_t UART_Write( UART_PORT_et port, uint8_t *buf, int32_t length )
{
	const UART_Instance_st * uart = UART.PORT[ port ];
	uint32_t tx_busy_cnt = 0;

	uart_tx_enable( port, 1 );

#if UART_TX_MODE == UART_DMA
	while( ( HAL_DMA_STATE_BUSY == uart->handle->hdmatx->State )
			|| ( ( HAL_UART_STATE_BUSY_TX & uart->handle->gState ) == HAL_UART_STATE_BUSY_TX ) )
	{
		tx_busy_cnt++;
		if( tx_busy_cnt > 100 )
		{
			HAL_UART_AbortTransmit( uart->handle );
		}
	}
	length = MIN(length, uart->txbufsz );
	memcpy( uart->txbuf, buf, length );
	HAL_StatusTypeDef ret = HAL_UART_Transmit_DMA( uart->handle, uart->txbuf, length );
	if( HAL_OK != ret ) length = -1;
#elif UART_TX_MODE == UART_IT
	static uint32_t tx_busy_cnt = 0;
	while( HAL_STATE_READY == uart->handle->gState )
	{
		tx_busy_cnt++;
		if( tx_busy_cnt > 100 )
		{
			HAL_UART_ITStop( uart->handle );
			HAL_UART_Receive_IT( uart->handle, uart->rxbuf, uart->rxbufsz );
		}
	}
	length = MIN(length, DMA_TX_BUFFER_SIZE );
	memcpy( uart->txbuffer, data, length );
	HAL_StatusTypeDef ret = HAL_UART_Transmit_IT( uart->handle, ( uint8_t * )uart->txbuffer, length );
	if( HAL_OK != ret ) length = -1;
#elif UART_TX_MODE == UART_POLL
	uint32_t timeout = fmaxf( length * ( 1000.0f / uart->handle->Init.BaudRate ) * 10.0f, 1.0f ) * 3;
	while( uart->handle->gState != HAL_UART_STATE_READY )
	{
		tx_busy_cnt++;
		if( tx_busy_cnt > 100 )
		{
			break;
		}
	}

	HAL_UART_Transmit( uart->handle, buf, length, timeout );

	length = uart->handle->TxXferSize - uart->handle->TxXferCount;
	uart_tx_enable( port, 0 );
#endif

	return length;
}

#if UART_TX_MODE == UART_IT
/******************************************************************************
 * @brief 
 * 
 * @param port 
 * @param timeout 
 * @return uint32_t 
 *****************************************************************************/
uint32_t UartTimedOut( UART_PORT_et port, uint32_t timeout )
{
	const UART_Instance_st * uart = UART.PORT[ port ];

	return timeout <= ( AppTimer.GetCurrentSecs() - uart->lastRcvdTimestamp );
}
#endif
/******************************************************************************
 * @brief 
 * 
 * @param port 
 * @param buf 
 * @param length 
 * @return uint32_t 
 *****************************************************************************/
int32_t UART_Read( UART_PORT_et port, uint8_t *buf, int32_t length )
{
	const UART_Instance_st * uart = UART.PORT[ port ];

	uint8_t word_length = 0;
	uint16_t read_cnt = 0;

	read_cnt = Ring.Get( uart->rxq, buf, length );

	switch( uart->handle->Init.WordLength )
	{
		case UART_WORDLENGTH_8B: word_length = 8; break;
		case UART_WORDLENGTH_9B: word_length = 9; break;
		default: return 0;
	}

	if( uart->handle->Init.Parity != UART_PARITY_NONE )
	{
		word_length--;
	}

	if( word_length == 7 )
	{
		for( int idx = 0; idx < read_cnt; ++idx )
		{
			buf[ idx ] = buf[ idx ] & 0x7F;
		}
	}

	return read_cnt;
}

/******************************************************************************
 * @brief 
 * 
 *****************************************************************************/
static void UartError_Handler(void)
{
  /* User can add his own implementation to report the HAL error return state */
}

/******************************************************************************
 * @brief 
 * 
 * @param port 
 * @return float 
 *****************************************************************************/
float UartLastReceivedTimestamp( UART_PORT_et port )
{
	return UART.PORT[ port ]->lastRcvdTimestamp;
}

/******************************************************************************
 * @brief 
 * 
 * @param huart 
 *****************************************************************************/
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if( ( huart->Instance->SR & USART_SR_TC ) == USART_SR_TC )
	{
		UART_PORT_et port;
		for( port = UART_PLC; ( port < MAX_UART ) && ( huart != UART.PORT[ port ]->handle ); ++port )
			__NOP();

		if( port < MAX_UART )
		{
			uart_tx_enable( port, 0 );
		}
	}
}

/******************************************************************************
 * @brief 
 * 
 * @param huart 
 *****************************************************************************/
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
#if UART_RX_MODE == UART_IT
	UART_Instance_st * uart = NULL;
	UART_PORT_et port;

	for( port = 0; port < MAX_UART; port++ )
	{
		if( huart == UART.PORT[ port ]->handle )
		{
			uart = UART.PORT[ port ];
			break;
		}
	}

	if( uart == NULL ) return;

	if( uart )
	{
		DIO.Output( LED_COM, true );
		uart->stat.received = 1;
		uart->lastRcvdTimestamp = AppTimer.GetCurrentSecs();
		Ring.Put( uart->rxq, uart->rxbuf, 1 );
	}
#endif
}

/******************************************************************************
 * @brief 
 * 
 * @param huart 
 *****************************************************************************/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
#if UART_RX_MODE == UART_IT
	UART_Instance_st * uart = NULL;
	UART_PORT_et port;

	for( port = 0; port < MAX_UART; port++ )
	{
		if( huart == UART.PORT[ port ]->handle )
		{
			uart = UART.PORT[ port ];
			break;
		}
	}

	if( uart == NULL ) return;

	if( uart )
	{
		DIO.Output( LED_COM, true );
		uart->stat.received = 1;
		uart->lastRcvdTimestamp = AppTimer.GetCurrentSecs();
		Ring.Put( uart->rxq, uart->rxbuf, 1 );
	}

	HAL_UART_Receive_IT( huart, huart->pRxBuffPtr - huart->RxXferSize, 1 );
#endif
}

/******************************************************************************
 * @brief  UART error callbacks.
 * @param  huart  Pointer to a UART_HandleTypeDef structure that contains
 *                the configuration information for the specified UART module.
 * @retval None
 *****************************************************************************/
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	volatile uint32_t e = huart->ErrorCode;
	// HAL_UART_ERROR_NONE /*!< No error                */
	// HAL_UART_ERROR_PE   /*!< Parity error            */
	// HAL_UART_ERROR_NE   /*!< Noise error             */
	// HAL_UART_ERROR_FE   /*!< Frame error             */
	// HAL_UART_ERROR_ORE  /*!< Overrun error           */
	// HAL_UART_ERROR_DMA  /*!< DMA transfer error      */
	// HAL_UART_ERROR_RTO  /*!< Receiver Timeout error  */

	__HAL_UART_CLEAR_PEFLAG(huart);
	__HAL_UART_CLEAR_NEFLAG(huart);
	__HAL_UART_CLEAR_FEFLAG(huart);
	__HAL_UART_CLEAR_OREFLAG(huart);

	UART_Instance_st * uart = NULL;
	UART_PORT_et port;

	for( port = 0; port < MAX_UART; port++ )
	{
		if( huart == UART.PORT[ port ]->handle )
		{
			uart = UART.PORT[ port ];
			break;
		}
	}

	if( uart == NULL ) return;

	/*!< Overrun error           */
	/*!< Parity error           */
	/*!< Frame error           */
	/*!< Noise error           */
	/*!< DMA transfer error      */
	if( e & ( HAL_UART_ERROR_ORE | HAL_UART_ERROR_PE | HAL_UART_ERROR_FE | HAL_UART_ERROR_NE | HAL_UART_ERROR_DMA ) )
	{
#if UART_RX_MODE == UART_DMA
		HAL_UART_AbortReceive( huart );
		uart->lastRcvdPos = 0;
		Ring.Init( uart->rxq, uart_rx_ringbuf[ port ], sizeof( uart_rx_ringbuf[ port ] ) );
		HAL_UART_Receive_DMA( huart, uart->rxbuf, uart->rxbufsz );
#elif UART_RX_MODE == UART_IT
		HAL_UART_AbortReceive_IT( huart );
		HAL_UART_Receive_IT( huart, uart->rxbuf, 1 );
#endif
	}

	uart->stat.all &= ~(0x3F << 2);
	uart->stat.all |= e << 2;
}
