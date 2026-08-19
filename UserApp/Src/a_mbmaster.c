/******************************************************************************
 * @file a_mbmaster.c
 * @author Seo Yujeong (yjseo@gst-in.com)
 * @brief 
 * @version 0.1
 * @date 2024-04-03
 * 
 * @copyright Copyright (c) 2024 Global Standard Technology Co., Ltd.
 * 
 * All rights reserved.
 * This file is part of closed source software.
 * Unauthorized reproduction and redistribution prohibited.
 *****************************************************************************/

/*****************************************************************************/
/** INCLUDES *****************************************************************/
/*****************************************************************************/

#include "UserApp.h"

/*****************************************************************************/
/** MACRO DEFINITIONS ********************************************************/
/*****************************************************************************/

#define TX_WAIT_DELAY 0.05f
//#define RECEIVE_TIMEOUT 2.0f  // 장석주D 요청 (정성민B) 2025-09-09
#define RECEIVE_TIMEOUT 0.5f

#define COMM_FAIL_TIME_DEFAULT 10.0f
#define COMM_FAIL_TIME_MIN 10.0f
#define COMM_FAIL_TIME_MAX 1000.0f

/*****************************************************************************/
/** DATATYPES ****************************************************************/
/*****************************************************************************/

typedef enum { MBM_SUCCESS, MBM_NOT_COMPLETED, MBM_TIMEOUT } mbm_status_et;

/*****************************************************************************/
/** FUNCTION DECLARATIONS ****************************************************/
/*****************************************************************************/

static void modbus_master_read_holding_reg_parser( uint8_t protocol, Modbus_Rx_st *modbus_rx, const DataBlock_st *db );
static void modbus_master_resp_preset_multiple_reg( uint8_t protocol, Modbus_Rx_st *modbus_rx, const DataBlock_st *db );
static mbm_status_et modbus_master( smps_cmd_st* SmpsCmd );

/*****************************************************************************/
/** LOCAL VARIABLES **********************************************************/
/*****************************************************************************/

static const Modbus_Master_st MBM = {
	.RxFrameCheck = ModbusMasterResponseFrameParse,
	.Query = ModbusMasterQueryBuild,
	.RxFrameParser =
	{
		[ Modbus_FunCode_Read_Holding_Reg ] = modbus_master_read_holding_reg_parser,
		[ Modbus_FunCode_Preset_Multiple_Reg ] = modbus_master_resp_preset_multiple_reg,
	},
};

static uint8_t rcvd_frame[ 256 ];
static uint8_t query_frame[ 256 ];
static uint32_t smps_read_sequence[ MAX_SMPS_ID ];
static uint32_t smps_last_read_tick_ms[ MAX_SMPS_ID ];
static uint8_t smps_has_read[ MAX_SMPS_ID ];

/*****************************************************************************/
/** GLOBAL VARIABLES *********************************************************/
/*****************************************************************************/

/*****************************************************************************/
/** FUNCTION DEFINITIONS *****************************************************/
/*****************************************************************************/

/******************************************************************************
 * @brief modbus_master_read_holding_reg_parser
 *****************************************************************************/
static void modbus_master_read_holding_reg_parser( uint8_t protocol, Modbus_Rx_st *modbus_rx, const DataBlock_st *db )
{
	uint16_t index = modbus_rx->starting_address_read - db->HoldingsStart;
	uint16_t count = modbus_rx->byte_count / 2;

	if( index + count > db->HoldingsCnt ) return;

	for( uint16_t i = 0; i < count; ++ i )
	{
		db->Holdings[ index + i ] = modbus_rx->data_write[ i ];
	}
}

/******************************************************************************
 * @brief modbus_master_resp_preset_multiple_reg
 *****************************************************************************/
static void modbus_master_resp_preset_multiple_reg( uint8_t protocol, Modbus_Rx_st *modbus_rx, const DataBlock_st *db )
{
	( void )protocol;
	( void )modbus_rx;
	( void )db;
}

/******************************************************************************
 * @brief Set smps comm timeout bit
 * 
 * @param SmpsCmd 
 *****************************************************************************/
static void set_smps_comm_fail( smps_cmd_st* SmpsCmd )
{
	switch( SmpsCmd->smps_type )
	{
		case OUT_SMPS_CHPP_5521:
			MBM_READ_DB[ SmpsCmd->smps_id ].Holdings[ MBM_HR_CHPP_5521_FAULT_STATUS_OF_SMPS ] |= MBM_HR_CHPP_5521_COMM_FAIL_BIT;
			break;
		case OUT_SMPS_CHPP_8021:
			MBM_READ_DB[ SmpsCmd->smps_id ].Holdings[ MBM_HR_CHPP_8021_FAULT_STATUS_OF_SMPS ] |= MBM_HR_CHPP_8021_COMM_FAIL_BIT;
			break;
		default:
			break;
	}
}

/******************************************************************************
 * @brief modbus_master
 *****************************************************************************/
static mbm_status_et modbus_master( smps_cmd_st* SmpsCmd )
{
	AppTimerData_ut timerModbusFrame = SmpsCmd->timerModbusFrame;
	AppTimerData_ut timerTxWait = SmpsCmd->timerTxWait;
	AppTimerData_ut timerRecvTimeout = SmpsCmd->timerRecvTimeout;
	AppTimerData_ut timerCommTransactionTimeout = SmpsCmd->timerCommTransactionTimeout;
	uint16_t rcvd_length = SmpsCmd->rcvd_length;
	uint16_t old_rcvd_length = SmpsCmd->old_rcvd_length;
	Modbus_Rx_st modbus_rx = SmpsCmd->modbus_rx;
	mbm_status_et mbm_status = MBM_NOT_COMPLETED;
	enum { MBM_INIT, QUERY_TO_SLAVE, RX_WAIT, RX_PARSER, TX_WAIT } stage = SmpsCmd->mbm_stage;

	float comm_fail_timeout = 60.0f;

#if 0
	// TODO: read from holding register for comm transaction error time
	comm_fail_timeout = MBS.Holdings[ MBM_HR_COMM_TRANSACTION_FAIL_TIMER ];
#endif
	if( comm_fail_timeout < COMM_FAIL_TIME_MIN )
	{
		comm_fail_timeout = COMM_FAIL_TIME_MIN;
	}
	else if( comm_fail_timeout > COMM_FAIL_TIME_MAX )
	{
		comm_fail_timeout = COMM_FAIL_TIME_MAX;
	}

	if( AppTimer.IsRun( &timerCommTransactionTimeout ) && AppTimer.IsExpired( &timerCommTransactionTimeout ) )
	{
		AppTimer.Stop( &timerCommTransactionTimeout );
		set_smps_comm_fail( SmpsCmd );
	}

	switch( stage )
	{
		default:
		case MBM_INIT:
			AppTimer.Stop( &timerModbusFrame );
			AppTimer.Stop( &timerTxWait );
			AppTimer.Stop( &timerRecvTimeout );
			AppTimer.Stop( &timerCommTransactionTimeout );
			stage = QUERY_TO_SLAVE;
			break;

		case QUERY_TO_SLAVE:
			rcvd_length = 0;
			old_rcvd_length = 0;
			rcvd_length = MBM.Query( 'R', SmpsCmd->smps_id, SmpsCmd->mbm_fcode, SmpsCmd->mbm_addr, SmpsCmd->mbm_qty, 0, 0, SmpsCmd->DBlock, query_frame );
			UART.Write( UART_SMPS, query_frame, rcvd_length );
			if( !AppTimer.IsRun( &timerCommTransactionTimeout ) )
			{
				AppTimer.Start( &timerCommTransactionTimeout, comm_fail_timeout );
			}

			rcvd_length = 0;

			if( SmpsCmd->smps_id == 0 )
			{
				AppTimer.Start( &timerTxWait, TX_WAIT_DELAY );
				stage = TX_WAIT;
			}
			else
			{
				AppTimer.Start( &timerRecvTimeout, RECEIVE_TIMEOUT );
				stage = RX_WAIT;
			}
			break;

		case RX_WAIT:
			rcvd_length += UART.Read( UART_SMPS, &rcvd_frame[ rcvd_length ], sizeof( rcvd_frame ) - rcvd_length );

			if( rcvd_length && ( old_rcvd_length != rcvd_length ) )
			{
				float rtu_frame_time = RTU_FRAME_IDLE_MIN;

				AppTimer.Stop( &timerRecvTimeout );
				if( UART.PORT[ UART_SMPS ]->handle->Init.BaudRate <= 19200 )
				{
					rtu_frame_time = MAX_BITS_PER_CHAR / UART.PORT[ UART_SMPS ]->handle->Init.BaudRate * RTU_FRAME_IDLE_CHAR * SAFETY_FACTOR;
				}
				AppTimer.Start( &timerModbusFrame, rtu_frame_time );
				old_rcvd_length = rcvd_length;
			}

			if( AppTimer.IsRun( &timerModbusFrame ) && AppTimer.IsExpired( &timerModbusFrame ) )
			{
				AppTimer.Stop( &timerModbusFrame );
				stage = RX_PARSER;
			}

			if( AppTimer.IsRun( &timerRecvTimeout ) && AppTimer.IsExpired( &timerRecvTimeout ) )
			{
				AppTimer.Stop( &timerRecvTimeout );

				mbm_status = MBM_TIMEOUT;
				stage = MBM_INIT;
			}
			break;

		case RX_PARSER:
			memset( &modbus_rx, 0, sizeof( modbus_rx ) );
			if( Modbus_ExCode_Normal == MBM.RxFrameCheck( 'R', rcvd_frame, rcvd_length, &modbus_rx ) )
			{
				// Restart Comm. Warning timer when normal response frame received
				AppTimer.Start( &timerCommTransactionTimeout, comm_fail_timeout );
				modbus_rx.starting_address_read = SmpsCmd->mbm_addr;
				if( MBM.RxFrameParser[ modbus_rx.function_code ] )
				{
					MBM.RxFrameParser[ modbus_rx.function_code ]( 'R', &modbus_rx, SmpsCmd->DBlock );
				}
				if( ( Modbus_FunCode_Read_Holding_Reg == modbus_rx.function_code )
					&& ( SmpsCmd->smps_id < MAX_SMPS_ID ) )
				{
					smps_read_sequence[ SmpsCmd->smps_id ]++;
					smps_last_read_tick_ms[ SmpsCmd->smps_id ] = HAL_GetTick();
					smps_has_read[ SmpsCmd->smps_id ] = 1;
				}
			}

			AppTimer.Start( &timerTxWait, TX_WAIT_DELAY );
			stage = TX_WAIT;
			break;

		case TX_WAIT:
			if( AppTimer.IsExpired( &timerTxWait ) )
			{
				AppTimer.Stop( &timerTxWait );
				memset( rcvd_frame, 0, sizeof( rcvd_frame ) );
				memset( query_frame, 0, sizeof( query_frame ) );
				stage = MBM_INIT;
				mbm_status = SUCCESS;
			}
			break;
	}

	SmpsCmd->timerModbusFrame = timerModbusFrame;
	SmpsCmd->timerTxWait = timerTxWait;
	SmpsCmd->timerRecvTimeout = timerRecvTimeout;
	SmpsCmd->timerCommTransactionTimeout = timerCommTransactionTimeout;
	SmpsCmd->rcvd_length = rcvd_length;
	SmpsCmd->old_rcvd_length = old_rcvd_length;
	SmpsCmd->modbus_rx = modbus_rx;

	SmpsCmd->mbm_stage = stage;

	return mbm_status;
}

/******************************************************************************
 * @brief Return the number of valid Read Holding Register responses.
 *****************************************************************************/
uint32_t ModbusMasterGetReadSequence( uint8_t smps_id )
{
	if( smps_id >= MAX_SMPS_ID ) return 0;
	return smps_read_sequence[ smps_id ];
}

/******************************************************************************
 * @brief Return milliseconds since the last valid SMPS read response.
 *****************************************************************************/
uint32_t ModbusMasterGetDataAgeMs( uint8_t smps_id )
{
	if( ( smps_id >= MAX_SMPS_ID ) || !smps_has_read[ smps_id ] )
	{
		return UINT32_MAX;
	}
	return HAL_GetTick() - smps_last_read_tick_ms[ smps_id ];
}

/******************************************************************************
 * @brief ModbusMasterTask
 *****************************************************************************/
void ModbusMasterTask( void )
{
	static uint8_t cmd_idx = 0;

	if( SMPSCMD[ cmd_idx ].smps_func != SMPS_CMD_NONE )
	{
		mbm_status_et status = MBM_NOT_COMPLETED;
		status = modbus_master( &SMPSCMD[ cmd_idx ] );
		switch (status)
		{
		case SUCCESS:
			SMPSCMD[ cmd_idx ].smps_func = SMPS_CMD_NONE;
			TICK_UP( cmd_idx, MAX_SMPS_CMD );
			break;
		case MBM_TIMEOUT:
			SMPSCMD[ cmd_idx ].smps_func = SMPS_CMD_NONE;
			break;
		case MBM_NOT_COMPLETED:
			break;
		default:
			break;
		}
	}
	else
	{
		TICK_UP( cmd_idx, MAX_SMPS_CMD );
	}
}
