/******************************************************************************
 * @file a_fltchk.c
 * @author Lee Jinyoung (jylee1@gst-in.com)
 * @brief 
 * @version 0.1
 * @date 2023-06-20
 * 
 * @copyright Copyright (c) 2023 Global Standard Technology Co., Ltd.
 * 
 * All rights reserved.
 * This file is part of closed source software.
 * Unauthorized reproduction and redistribution prohibited.
 * 
 *****************************************************************************/

/*****************************************************************************/
/** INCLUDES *****************************************************************/
/*****************************************************************************/
#include "UserApp.h"

/*****************************************************************************/
/** MACRO DEFINITIONS ********************************************************/
/*****************************************************************************/

#define BANDOVER_CHECK_TIME 3.0f
#define OVER_CURRENT_CHECK_TIME 5.0f

#define HIGH_AMP_ALERT 2.75f
#define HIGH_AMP_WARNING 2.6f

#define MCU_HIGH_TEMP 82.0f
#define MCU_HIGH_TEMP_WARN_CNT 30

/*****************************************************************************/
/** DATATYPES ****************************************************************/
/*****************************************************************************/

/*****************************************************************************/
/** FUNCTION DECLARATIONS ****************************************************/
/*****************************************************************************/

static bool pv_over_high_temp( int PV, int high_temp );
static bool pv_under_low_temp( int PV, int low_temp );
static bool is_pv_in_band( int PV, int high_temp, int low_temp );
int fault_check_pv_band_over( int loop_idx, int32_t PV, int32_t high_temp, int32_t low_temp );
static bool fault_check_control_time_over( int loop_idx, int32_t PV, const control_loop_config_st *const pCfg );
static bool find_config_error( const control_loop_config_st *const pCfg );
static void fault_output( uint8_t flt_status );

/*****************************************************************************/
/** LOCAL VARIABLES **********************************************************/
/*****************************************************************************/

static int32_t prevSV[ MAX_CONTROL_LOOP ] = { 0 };									// previous PV for check change rate

static AppTimerData_ut timerPvBandOver[ MAX_CONTROL_LOOP ];
static AppTimerData_ut timerControlTimeOver[ MAX_CONTROL_LOOP ];
static AppTimerData_ut timerOverCurrent[ MAX_CONTROL_LOOP ];

static uint32_t mcu_high_temp_count = 0;

/*****************************************************************************/
/** GLOBAL VARIABLES *********************************************************/
/*****************************************************************************/

/*****************************************************************************/
/** FUNCTION DEFINITIONS *****************************************************/
/*****************************************************************************/

/******************************************************************************
 * @brief Check fault status of each control loop
 * 
 *****************************************************************************/
void FaultCheckTask( void )
{
	static int loop_idx = 0;

	if( !CFG.system.Run )
	{
		fault_output( 0 );
		return;
	}

	const control_loop_config_st *const pCfg = Controller.GetConfig( loop_idx );

	if( pCfg && pCfg->Enable )
	{
		loop_fault_status_ut cflt = { 0 };
		int32_t PV = 0;
		int32_t SV = 0;
		int32_t high_temp = 0;
		int32_t low_temp = 0;
		int16_t amps = 0;

		cflt = Controller.GetFault( loop_idx );
		PV = Controller.GetPV( loop_idx );
		SV = pCfg->SV;
		high_temp = SV + ( int32_t )pCfg->HighOverAlarm * 10;
		low_temp = SV - ( int32_t )pCfg->LowUnderAlarm * 10;
		amps = RemoteIO.GetAmpData( loop_idx );

		if( pCfg->ControlType == CTRL_FULL_TEST || pCfg->ControlType == CTRL_HALF_TEST )
		{
			Controller.ClearAlarm( 0xFFFF, 0xFFFF );

			// Stop all fault check timers
			AppTimer.Stop( &timerPvBandOver[ loop_idx ] );
			AppTimer.Stop( &timerOverCurrent[ loop_idx ] );
			AppTimer.Stop( &timerControlTimeOver[ loop_idx ] );
		}
		else
		{
			int smps_id = Controller.GetConfig( loop_idx )->OutputChannel;
			if( Controller.GetConfig( loop_idx )->OutputType == OUT_SMPS_CHPP_8021 )
			{
				if( MBM_READ_DB[ smps_id ].Holdings[ MBM_HR_CHPP_8021_FAULT_STATUS_OF_SMPS ] )
				{
					cflt.SmpsCommError = 1;
				}
			}
			else if( Controller.GetConfig( loop_idx )->OutputType == OUT_SMPS_CHPP_5521 )
			{
				if( MBM_READ_DB[ smps_id ].Holdings[ MBM_HR_CHPP_5521_FAULT_STATUS_OF_SMPS ] )
				{
					cflt.SmpsCommError = 1;
				}
			}

			// PV Band Check
			int band_check = fault_check_pv_band_over( loop_idx, PV, high_temp, low_temp );
			if( 1 == band_check )
			{
				cflt.HighTemp = 1;
			}
			else if( -1 == band_check )
			{
				cflt.LowTemp = 1;
			}

			// Control Time over
			if( fault_check_control_time_over( loop_idx, PV, pCfg ) )
			{
				cflt.ControlTimeOver = 1;
			}

			// FaultCheckSVError
			if( MINIMUM_PV > SV || SV > MAXIMUM_PV )
			{
				cflt.SVError = 1;
			}

			// ConfigError
			if( find_config_error( pCfg ) )
			{
				cflt.ConfigError = 1;
			}

			// OverCurrAlarm
			if( amps > HIGH_AMP_ALERT )
			{
				cflt.OverCurrAlarm = 1;
			}
			else if( amps > HIGH_AMP_WARNING )
			{
				if( AppTimer.IsRun( &timerOverCurrent[ loop_idx ] ) )
				{
					if( AppTimer.IsExpired( &timerOverCurrent[ loop_idx ] ) )
					{
						cflt.OverCurrAlarm = 1;
					}
				}
				else
				{
					AppTimer.Start( &timerOverCurrent[ loop_idx ], OVER_CURRENT_CHECK_TIME );
				}
			}
			else
			{
				AppTimer.Stop( &timerOverCurrent[ loop_idx ] );
			}
		}

		Controller.SetFault( loop_idx, cflt );
	}

	global_fault_status_ut gflt = { .All = 0 };


	if( MCU_HIGH_TEMP < ADC_MCU.GetTemp( 0 ) )        // MCU Operating Temperature : 85°C
	{
		mcu_high_temp_count++;
		if( mcu_high_temp_count > MCU_HIGH_TEMP_WARN_CNT )
		{
			gflt.McuTempHigh = 1 ;
		}
	}
	else
	{
		mcu_high_temp_count = 0;
	}

#if 0
	if( gflt.All )
	{
		flt_output = CFG.system.FaultRelayNc ? FLT_RLY_OFF : FLT_RLY_ON;
		led_output = LED_ON;

	}
#endif

	TICK_UP( loop_idx, MAX_CONTROL_LOOP );

	if( 0 == loop_idx )
	{
		int loop = 0;
		for( ; loop < MAX_CONTROL_LOOP; loop++ )
		{
			if( Controller.GetFault( loop ).All )
			{
				fault_output( 1 );
				break;
			}
		}

		if( loop == MAX_CONTROL_LOOP )
		{
			fault_output( 0 );
		}
	}
}

/******************************************************************************
 * @brief 
 * 
 * @param PV Process value
 * @param high_temp high limitation of temperature band
 * @retval true [pv is over high_temp]
 * @retval false [pv is not over high_temp]
 *****************************************************************************/
static bool pv_over_high_temp( int PV, int high_temp )
{
	return PV > high_temp;
}

/******************************************************************************
 * @brief 
 * 
 * @param PV Process value
 * @param low_temp low limitation of temperature band
 * @retval true [pv is under low_temp]
 * @retval false [pv is not under low_temp]
 *****************************************************************************/
static bool pv_under_low_temp( int PV, int low_temp )
{
	return PV < low_temp;
}

/******************************************************************************
 * @brief 
 * 
 * @param PV Process value
 * @param high_temp high limitation of temperature band
 * @param low_temp low limitation of temperature band
 * @retval true [pv is in the band between high_temp and low_temp]
 * @retval false [pv is out of band]
 *****************************************************************************/
static bool is_pv_in_band( int PV, int high_temp, int low_temp )
{
	if( !pv_over_high_temp( PV, high_temp ) && !pv_under_low_temp( PV, low_temp ) )
	{
		return true;
	}

	return false;
}

/******************************************************************************
 * @brief 
 * 
 * @param loop_idx index of control loop to check fault
 * @param PV Process value
 * @param high_temp max limitation of temperature band
 * @param low_temp min limitation of temperature band
 * 
 * @retval 1 [high over]
 * @retval -1 [low under]
 * @retval 0 [fault not occurs]
 *****************************************************************************/
int fault_check_pv_band_over( int loop_idx, int32_t PV, int32_t high_temp, int32_t low_temp )
{
	static bool pv_in_band[ MAX_CONTROL_LOOP ] = { 0 };
	static int32_t old_temp[ MAX_CONTROL_LOOP ][ 2 ] = { { 0 }, };
	AppTimerData_ut *pTimer = &timerPvBandOver[ loop_idx ];

	if( ( old_temp[ loop_idx ][ 0 ] != high_temp ) || ( old_temp[ loop_idx ][ 1 ] != low_temp ) )
	{
		AppTimer.Stop( pTimer );
		AppTimer.Stop( &timerControlTimeOver[ loop_idx ] );
		old_temp[ loop_idx ][ 0 ] = high_temp;
		old_temp[ loop_idx ][ 1 ] = low_temp;
		pv_in_band[ loop_idx ] = false;
	}

	if( is_pv_in_band( PV, high_temp, low_temp ) )
	{
		AppTimer.Stop( pTimer );
		AppTimer.Stop( &timerControlTimeOver[ loop_idx ] );
		pv_in_band[ loop_idx ] = true;
	}
	else
	{
		if( AppTimer.IsRun( pTimer ) )
		{
			if( AppTimer.IsExpired( pTimer ) )
			{
				if( pv_over_high_temp( PV, high_temp ) )
				{
					return 1;
				}
				if( pv_under_low_temp( PV, low_temp ) )
				{
					return -1;
				}
			}
		}
		else
		{
			if( pv_in_band[ loop_idx ] )
			{
				AppTimer.Start( pTimer, BANDOVER_CHECK_TIME );
			}
		}
	}

	return 0;
}

/******************************************************************************
 * @brief 
 * 
 * @param[in] loop_idx index of control loop to check fault
 * @param[in] SV Set value
 * @param[in] pCfg configuration structure of control loop
 * 
 * @retval true [control time is over. eg. PV change too slow]
 * @retval false [fault not occurs]
 *****************************************************************************/
static bool fault_check_control_time_over( int loop_idx, int32_t PV, const control_loop_config_st *const pCfg )
{
	if( pCfg->ControlTimeOver )
	{
		AppTimerData_ut * pTimer = &timerControlTimeOver[ loop_idx ];
		if( prevSV[ loop_idx ] != pCfg->SV )
		{
			prevSV[ loop_idx ] = pCfg->SV;
			AppTimer.Start( pTimer, pCfg->ControlTimeOver );
		}
		else
		{
			if( AppTimer.IsRun( pTimer ) && AppTimer.IsExpired( pTimer ) )
			{
				return true;
			}

			int high_temp = pCfg->SV + pCfg->HighOverAlarm * 10;
			int low_temp = pCfg->SV - pCfg->LowUnderAlarm * 10;
			if( is_pv_in_band( PV, high_temp, low_temp ) )
			{
				AppTimer.Stop( pTimer );
			}
		}
	}
	return false;
}

/******************************************************************************
 * @brief 
 * 
 * @param[in] pCfg 
 * 
 * @retval true [found error in configuration]
 * @retval false [not found error]
 *****************************************************************************/
static bool find_config_error( const control_loop_config_st *const pCfg )
{
	if( ( pCfg->OutputMax < pCfg->OutputMin )
		|| ( pCfg->ControlType > CTRL_BYPASS )
		|| ( pCfg->ControlType <= CTRL_PID && !pCfg->Pb && !pCfg->Ti && !pCfg->Td )
		|| ( pCfg->ControlPeriod < MIN_CONTROL_PERIOD )
	)
	{
		return true;
	}
	return false;
}

/******************************************************************************
 * @brief 
 * 
 *****************************************************************************/
static void fault_output( uint8_t flt_status )
{
	if( flt_status )
	{
		DIO.Output( FLT_RLY, CFG.system.FaultRelayNc ? FLT_RLY_ON : FLT_RLY_OFF );
		DIO.Output( LED_ERR, LED_ON );
	}
	else
	{
		DIO.Output( FLT_RLY, CFG.system.FaultRelayNc ? FLT_RLY_OFF : FLT_RLY_ON );
		DIO.Output( LED_ERR, LED_OFF );
	}
}
