/******************************************************************************
 * @file a_can.c
 * @author jylee1 (jylee1@gst-in.com)
 * @brief 
 * @version 0.1
 * @date 2023-06-12
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
#include "UserApp.h"

/*****************************************************************************/
/** MACRO DEFINITIONS ********************************************************/
/*****************************************************************************/

#define MAX_INTERNAL_TE 4
#define MAX_INTERNAL_AO 2
#define MAX_INTERNAL_DO 2

#define MAX_TMP2_BOARDS 4
#define MAX_TMP2_CHANNEL 4
#define MAX_SSR_BOARDS 4
#define MAX_SSR_CHANNEL 4
#define MAX_DO_BOARDS 4
#define MAX_DO_CHANNEL 4
#define MAX_AO_BOARDS 4
#define MAX_AO_CHANNEL 4

#define CAN_ID_AO_RANGE 0xB0
#define CAN_ID_AO_DATA 0xB8
#define CAN_ID_SSR_FREQ 0x100
#define CAN_ID_SSR_DUTY 0x108
#define CAN_ID_TMP2_CFG 0x0F0
#define CAN_ID_TMP2_DATA 0x0F8

#define CAN_ID_KEEP_ALIVE 0x00F

#define CAN_TX_KEEP_ALIVE_MSG_TIME 0.1f

/*****************************************************************************/
/** DATATYPES ****************************************************************/
/*****************************************************************************/

/*****************************************************************************/
/** FUNCTION DECLARATIONS ****************************************************/
/*****************************************************************************/

static __RAM_FUNC int32_t can_get_pv_data( int8_t ch );
static __RAM_FUNC int32_t can_get_amp_data( int8_t ch );
static __RAM_FUNC void can_set_temp_cfg( int8_t ch, uint16_t type, uint16_t sps );
static __RAM_FUNC void can_set_do_freq( int8_t ch, uint16_t freq );
static __RAM_FUNC void can_set_do_data( int8_t ch, uint16_t do_data);
static __RAM_FUNC void can_set_ao_data( int8_t ch, uint16_t ao_data );
static __RAM_FUNC void can_set_ao_range( int8_t ch, uint16_t range );

/******************** *********************************************************/
/** GLOBAL VARIABLES *********************************************************/
/*****************************************************************************/

const remote_io_st RemoteIO =
{
	.GetPvData = can_get_pv_data,
	.GetAmpData = can_get_amp_data,
	.SetTempCfg = can_set_temp_cfg,
	.SetDoFreq = can_set_do_freq,
	.SetDoData = can_set_do_data,
	.SetAoData = can_set_ao_data,
	.SetAoRange = can_set_ao_range,
};

/*****************************************************************************/
/** LOCAL VARIABLES **********************************************************/
/*****************************************************************************/

static CCMRAM uint8_t CAN_TMP_CFG[ MAX_TMP2_BOARDS ][ 2 ][ MAX_TMP2_CHANNEL ] = { 0 };
static CCMRAM uint32_t CAN_TMP_DATA[ MAX_TMP2_BOARDS ][ MAX_TMP2_CHANNEL ] = { 0 };
static CCMRAM uint32_t CAN_SSR_AMP[ MAX_SSR_BOARDS ][ MAX_SSR_CHANNEL ] = { 0 };
static CCMRAM uint16_t CAN_DO_FREQ[ MAX_SSR_BOARDS ][ MAX_SSR_CHANNEL ] = { 0 };
static CCMRAM uint16_t CAN_DO_DATA[ MAX_DO_BOARDS ][ MAX_DO_CHANNEL ] = { 0 };
static CCMRAM uint16_t CAN_AO_DATA[ MAX_AO_BOARDS ][ MAX_AO_CHANNEL ] = { 0 };
static CCMRAM uint8_t CAN_AO_RANGE[ MAX_AO_BOARDS ][ MAX_AO_CHANNEL ] = { 0 };

static uint8_t keep_alive_data[ MAX_CAN_MSG_LEN ] = { 0 };

static AppTimerData_ut timerCanKeepAlive = { 0 };

/*****************************************************************************/
/** FUNCTION DEFINITIONS *****************************************************/
/*****************************************************************************/

/*************************************************************************
 * @brief Initialize CAN Application
 *
 ************************************************************************/
void CanInit( void )
{
	// filter for RTR message(configuration request) receiving 
	CAN.SetFilter( CAN_ID_TMP2_CFG, 0x3F0 );
	CAN.SetFilter( CAN_ID_SSR_FREQ, 0x3F0 );

	// RTR 응답할 설정값 셋팅 (TEP2)
	for( int loop_idx = 0; loop_idx < MAX_CONTROL_LOOP; ++loop_idx )
	{
		const control_loop_config_st * const pCfg = Controller.GetConfig( loop_idx );
		if( ( SEN_COMM != pCfg->InputType ) && ( MAX_INTERNAL_TE <= pCfg->InputChannel ) )
		{
			can_set_temp_cfg( pCfg->InputChannel, pCfg->InputType, pCfg->SamplePeriod );
		}
	}

#if 0
	// RTR 응답할 설정값 셋팅 (SSR)
	for( int ssr_bd = 0; ssr_bd < MAX_SSR_BOARDS; ++ssr_bd )
	{
		uint8_t can_data[ 6 ] = { 0 };
		for( int ch = 0; ch < MAX_SSR_CHANNEL; ++ch )
		{
			for( int loop_idx = 0; loop_idx < MAX_CONTROL_LOOP; ++loop_idx )
			{
				const control_loop_config_st * const pCfg = Controller.GetConfig( loop_idx );
				if( OUT_PWM == pCfg->OutputType )
				{
//					if( ( ssr_bd * MAX_SSR_CHANNEL + ch ) == ( pCfg->OutputChannel - MAX_PWM_CHANNEL ) )
//					{
//						can_data[ 0 + ch * 2 ] = pCfg->PWMFreq >> 8;
//						can_data[ 1 + ch * 2 ] = pCfg->PWMFreq & 0xff;
//					}
				}
			}
		}
		CAN.SetTxMsg( CAN_ID_SSR + ssr_bd, false, 6, can_data );
	}
#endif
}

/*************************************************************************
 * @brief CAN Receive Task
 *
 ************************************************************************/
void CanRxTask( void )
{
	int dlc = 0;
	uint8_t can_data[ MAX_CAN_MSG_LEN ] = { 0 };

	for( uint32_t tmp2_bdid = 0; tmp2_bdid < MAX_TMP2_BOARDS; ++tmp2_bdid )
	{
		memset( can_data, 0, sizeof( can_data ) );
		dlc = CAN.GetMessage( tmp2_bdid + CAN_ID_TMP2_DATA, can_data );
		if( 0 < dlc )
		{
			uint8_t ch = can_data[ 0 ] % 4;
			memcpy( &CAN_TMP_DATA[ tmp2_bdid ][ ch ], &can_data[ 1 ], 4 );
		}
	}

	for( uint32_t ssr_bdid = 0; ssr_bdid < MAX_SSR_BOARDS; ++ssr_bdid )
	{
		memset( can_data, 0, sizeof( can_data ) );
		dlc = CAN.GetMessage( ssr_bdid + CAN_ID_SSR_DUTY, can_data );
		if( 0 < dlc )
		{
			memcpy( &CAN_SSR_AMP[ ssr_bdid ], can_data, dlc );
		}
	}
}

/******************************************************************************
 * @brief CAN Transmit Task (Periodically)
 * 
 *****************************************************************************/
void CanTxTask( void )
{
	if( AppTimer.IsExpired( &timerCanKeepAlive ) )
	{
		AppTimer.Start( &timerCanKeepAlive, CAN_TX_KEEP_ALIVE_MSG_TIME );

		CAN.SendMessage( CAN_ID_KEEP_ALIVE, false, 1, keep_alive_data );

		keep_alive_data[ 0 ]++;
	}
}

/******************************************************************************
 * @brief 
 * 
 * @param type 
 * @return uint16_t 
 *****************************************************************************/
static uint16_t conv_remote_type( sensor_et type )
{
	switch( type )
	{
		case SEN_RTD2X:	return 4;
		case SEN_RTD:	return 3;
		case SEN_TC_K:	return 1;
		case SEN_TC_J:	return 2;
		case SEN_TC_E:	return 0;
		case SEN_TC_S:	return 0;
		case SEN_TC_T:	return 0;
		case SEN_TC_R:	return 0;
		default:		return 0;
	}
}

/*************************************************************************
 * @brief Get process value from received CAN message
 * 
 * @param ch 
 * @return Temperature(Process Value) / unit 0.001ºC
 ************************************************************************/
int32_t can_get_pv_data( int8_t ch )
{
	int32_t pv = MINIMUM_PV;

	if( ( MAX_INTERNAL_TE <= ch ) && ( ch < MAX_CONTROL_LOOP ) )
	{
		uint8_t bdid = ch / MAX_TMP2_CHANNEL - 1;
		uint8_t bdch = ch % MAX_TMP2_CHANNEL;
		pv = CAN_TMP_DATA[ bdid ][ bdch ];
	}

	return pv;
}

/*************************************************************************
 * @brief Get current consumption at heater
 * 
 * @param ch 
 * @return Ampere / unit 1 mA
 ************************************************************************/
int32_t can_get_amp_data( int8_t ch )
{
	int32_t amp = 0;
	if( ( MAX_INTERNAL_AO <= ch ) && ( ch < MAX_CONTROL_LOOP ) )
	{
		uint8_t bdid = ( ch - 2 ) / MAX_AO_CHANNEL;
		uint8_t bdch = ( ch - 2 ) % MAX_AO_CHANNEL;

		amp = CAN_SSR_AMP[ bdid ][ bdch ];
	}

	return amp;
}

/******************************************************************************
 * @brief 
 * 
 * @param ch 
 * @param type 
 * @param sps 
 *****************************************************************************/
void can_set_temp_cfg( int8_t ch, uint16_t type, uint16_t sps )
{
	if( ( MAX_INTERNAL_TE <= ch ) && ( ch < MAX_CONTROL_LOOP ) )
	{
		uint8_t bdid = ch / MAX_TMP2_CHANNEL - 1;
		uint8_t bdch = ch % MAX_TMP2_CHANNEL;
		CAN_TMP_CFG[ bdid ][ 0 ][ bdch ] = conv_remote_type( type );
		CAN_TMP_CFG[ bdid ][ 1 ][ bdch ] = sps;

		CAN.SetTxMsg( CAN_ID_TMP2_CFG + bdid, false, 8, ( uint8_t * )CAN_TMP_CFG[ bdid ] );
	}
}

/*************************************************************************
 * @brief 
 ************************************************************************/
void can_set_do_freq( int8_t ch, uint16_t freq )
{
	if( ( MAX_INTERNAL_DO <= ch ) && ( ch < MAX_CONTROL_LOOP ) )
	{
		uint8_t bdid = ( ch - MAX_INTERNAL_DO ) / MAX_DO_CHANNEL;
		uint8_t bdch = ( ch - MAX_INTERNAL_DO ) % MAX_DO_CHANNEL;

		CAN_DO_FREQ[ bdid ][ bdch ] = freq;

		CAN.SetTxMsg( CAN_ID_SSR_FREQ + bdid, false, 8, ( uint8_t * )CAN_DO_FREQ[ bdid ] );
	}
}

/*************************************************************************
 * @brief 
 ************************************************************************/
void can_set_do_data( int8_t ch, uint16_t do_data )
{
	if( ( MAX_INTERNAL_DO <= ch ) && ( ch < MAX_CONTROL_LOOP ) )
	{
		uint8_t bdid = ( ch - MAX_INTERNAL_DO ) / MAX_DO_CHANNEL;
		uint8_t bdch = ( ch - MAX_INTERNAL_DO ) % MAX_DO_CHANNEL;

		CAN_DO_DATA[ bdid ][ bdch ] = do_data;
		CAN.SendMessage( CAN_ID_SSR_DUTY + bdid, false, 8, ( uint8_t * )CAN_DO_DATA[ bdid ] );
	}
}

/*************************************************************************
 * @brief 
 ************************************************************************/
void can_set_ao_data( int8_t ch, uint16_t ao_data )
{
	if( ( MAX_INTERNAL_AO <= ch ) && ( ch < MAX_CONTROL_LOOP ) )
	{
		uint8_t bdid = ( ch - MAX_INTERNAL_AO ) / MAX_AO_CHANNEL;
		uint8_t bdch = ( ch - MAX_INTERNAL_AO ) % MAX_AO_CHANNEL;

		CAN_AO_DATA[ bdid ][ bdch ] = ao_data;

		CAN.SendMessage( CAN_ID_AO_DATA + bdid, false, 8, ( uint8_t * )CAN_AO_DATA[ bdid ] );
	}
}

/*************************************************************************
 * @brief 
 ************************************************************************/
void can_set_ao_range( int8_t ch, uint16_t range )
{
	if( ( MAX_INTERNAL_AO <= ch ) && ( ch < MAX_CONTROL_LOOP ) )
	{
		uint8_t bdid = ( ch - 2 ) / MAX_AO_CHANNEL;
		uint8_t bdch = ( ch - 2 ) % MAX_AO_CHANNEL;

		CAN_AO_RANGE[ bdid ][ bdch ] = range;

		CAN.SetTxMsg( CAN_ID_AO_RANGE + bdid, false, 4, ( uint8_t * )CAN_AO_RANGE[ bdid ] );
	}
}
