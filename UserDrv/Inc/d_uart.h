/******************************************************************************
 * @file d_uart.h
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

#ifndef INC_D_UART_H_
#define INC_D_UART_H_

#ifdef __cplusplus
extern "C" {
#endif

/*****************************************************************************/
/** INCLUDES *****************************************************************/
/*****************************************************************************/

#include <UserDrivers.h>

/*****************************************************************************/
/** MACRO DEFINITIONS ********************************************************/
/*****************************************************************************/

#define PLC_UART_BAUDRATE 57600UL
#define SMPS_UART_BAUDRATE 115200UL

#define UART_TX_RING_SZ 512
#define UART_RX_RING_SZ 512

/*****************************************************************************/
/** DATATYPES ****************************************************************/
/*****************************************************************************/
typedef enum UART_PORT_enum_Tag {
	UART_PLC = 0,
	UART_SMPS,

	MAX_UART
} UART_PORT_et;

typedef struct UART_Instance_struct UART_Instance_st;
struct UART_Instance_struct {
	UART_HandleTypeDef * handle;
	uint8_t * rxbuf;
	uint16_t rxbufsz;
	uint8_t * txbuf;
	uint16_t txbufsz;
	GstRingBufHandle_t * rxq;
	void (*func)( UART_Instance_st * uart );
	uint8_t init_req;
	float lastRcvdTimestamp;
	uint32_t lastRcvdPos;
	union {
		struct {
			uint32_t received : 1;
			uint32_t dma_lock : 1;

			uint32_t err_parity : 1;
			uint32_t err_noise : 1;
			uint32_t err_frame : 1;
			uint32_t err_overrun : 1;
			uint32_t err_dma_transfer : 1;
			uint32_t err_receiver_timeout : 1;

			uint32_t : 8;

			uint32_t : 8;

			uint32_t : 7;
			uint32_t inited : 1;
		};
		uint32_t all;
	} stat;
};

typedef struct UART_struct_Tag {
	UART_Instance_st * PORT[ MAX_UART ];
	void (*Open)( UART_PORT_et port, uint32_t baudrate, uint16_t databit, uint16_t stopbit, uint16_t parity );
	void (*Close)( UART_PORT_et port );
	int32_t (*Read)( UART_PORT_et port, uint8_t * buf, int32_t length );
	int32_t (*Write)( UART_PORT_et port, uint8_t * buf, int32_t length );
} UART_st;


/*****************************************************************************/
/** FUNCTION DECLARATIONS ****************************************************/
/*****************************************************************************/

void UartInit( void );
void UartRxTask( void );

/*****************************************************************************/
/** EXTERNAL VARIABLES *******************************************************/
/*****************************************************************************/

extern const UART_st UART;

/*****************************************************************************/
/** LOCAL VARIABLES **********************************************************/
/*****************************************************************************/

/*****************************************************************************/
/** GLOBAL VARIABLES *********************************************************/
/*****************************************************************************/

/*****************************************************************************/
/** FUNCTION DEFINITIONS *****************************************************/
/*****************************************************************************/




#ifdef __cplusplus
}
#endif

#endif /* _D_UART_H_ */
