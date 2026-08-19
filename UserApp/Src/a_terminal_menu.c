/******************************************************************************
 * @file a_terminal_menu.c
 * @author Lee Jinyoung (jylee1@gst-in.com)
 * @brief 
 * @version 0.1
 * @date 2023-07-12
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

#include <stddef.h>

#include <UserDrivers.h>
#include <UserApp.h>

/* The legacy interactive VT100 terminal menu is replaced by the machine-
 * readable test command interface (a_test_func.c). The whole menu tree is
 * compiled out unless USE_LEGACY_TERMINAL is defined in UserApp.h.
 * See docs/harness/test_func.md.
 */
#if defined( USE_LEGACY_TERMINAL )

/*****************************************************************************/
/** MACRO DEFINITIONS ********************************************************/
/*****************************************************************************/

/*****************************************************************************/
/** DATATYPES ****************************************************************/
/*****************************************************************************/

/*****************************************************************************/
/** FUNCTION DECLARATIONS ****************************************************/
/*****************************************************************************/

/*****************************************************************************/
/** EXTERNAL VARIABLES *******************************************************/
/*****************************************************************************/

/*****************************************************************************/
/** LOCAL VARIABLES **********************************************************/
/*****************************************************************************/

static int loop_cfg_ch = 0;

/*****************************************************************************/
/** FUNCTION DEFINITIONS *****************************************************/
/*****************************************************************************/

void terminal_menu_splash_screen( void )
{
	tprintf( VT100_CURSOR_TO_HOME );
	
	tprintf( VT100_ERASE_LN_ALL "[System %s]", MBSDB.Holdings[ MBS_HR_RUN ] ? "Running" : "Stopped" );
	tprintf( "  [VER %04X %04X %6X]", CFG.system.MCU, CFG.system.REVISION, CFG.system.BUILD_DATE );
	tprintf( "  [%s]  [%d]  [%s]\r\n", Sys.GetResetCauseName(), Sys.GetResetCount(), Sys.GetLastExceptionName() );
	tprintf( VT100_ERASE_LN_ALL "+----+---------+----------+----------+--------+--------+--------+--------+\r\n");
	tprintf( VT100_ERASE_LN_ALL "| CH | Status  |    PV    |    SV    |   MV   |   Pb   |   Ti   |   Td   |\r\n");
	tprintf( VT100_ERASE_LN_ALL "+----+---------+----------+----------+--------+--------+--------+--------+\r\n");
	tprintf( VT100_ERASE_LN_ALL "| CJ | ------- | %8.3f | -------- | ------ | ------ | ------ | ------ |\r\n", Temp.GetCjTemp());

	for( int idx = 0; idx < MAX_CONTROL_LOOP; ++idx )
	{
		int PV = Controller.GetPV( idx );
		float MV = Controller.GetMV( idx ) * MULTIPLY_FLOAT_TO_PERCENT;
		const control_loop_config_st * const pCfg = Controller.GetConfig( idx );

		tprintf( VT100_ERASE_LN_ALL "| %2d | %-7s | %8.3f | %8.2f | %6.1f | %6.1f | %6.1f | %6.2f |\r\n",
					idx, pCfg->Enable ? "Run" : "Stopped", PV * 0.001f, pCfg->SV * 0.001f, MV,
					pCfg->Pb * 0.1f, pCfg->Ti * 0.1f, pCfg->Td * 0.01f );
	}
	tprintf( VT100_ERASE_LN_ALL "+----+---------+----------+----------+--------+--------+--------+--------+\r\n");
}

static void tfunc_print_out_bd_exist( void * param )
{
	tprintf( "\r\nOUT Board [%s]\r\n" VT100_CURSOR_POS_RESTORE, DIO.Input( BD_CHECK ) ? "Not Exist" : "OK" );
}

static void tfunc_dac_test( void * param )
{
	float now = AppTimer.GetCurrentSecs();
	srand( *((uint32_t*)&now) );
	int ch1 = rand() % 65535;
	int ch2 = rand() % 65535;

	if( 'V' == (int)param )
	{
		AD5422.SetupRange( 0, DAC_V_M10_to_10 );
		AD5422.SetupRange( 1, DAC_V_M10_to_10 );

		AD5422.Output( 0, ch1 );
		AD5422.Output( 1, ch2 );
		tprintf( "\r\nDAC 1CH %.2fV, 2CH %.2fV" VT100_ERASE_LN_TO_END VT100_CURSOR_POS_RESTORE, (ch1 - 32768) / 3276.8f, (ch2 - 32768) / 3276.8f );
	}
	else if( 'C' == (int)param )
	{
		AD5422.SetupRange( 0, DAC_I_0_to_24 );
		AD5422.SetupRange( 1, DAC_I_0_to_24 );

		AD5422.Output( 0, ch1 );
		AD5422.Output( 1, ch2 );
		tprintf( "\r\nDAC 1CH %.2fmA, 2CH %.2fmA" VT100_ERASE_LN_TO_END VT100_CURSOR_POS_RESTORE, 24 * ch1 / 65535.0f, 24 * ch2 / 65535.0f );
	}
}

static void tfunc_pwm_test( void * param )
{
	PWM.Stop( 0 );
	PWM.SetFreq( 0, 1UL );
	PWM.SetDuty( 0, 0.90f );
	PWM.Start( 0 );

	PWM.Stop( 1 );
	PWM.SetFreq( 1, 0.33f );
	PWM.SetDuty( 1, 0.90f );
	PWM.Start( 1 );

	tprintf( "\r\nPWM 1CH 1Hz 90%% duty, 2CH 0.33Hz 90%% duty" VT100_CURSOR_POS_RESTORE );
}

static void tfunc_smps_pola_test( void * param )
{
	if( DIO.Input( SMPS_POLA1 ) == SMPS_POLA_HEAT )
	{
		DIO.Output( SMPS_POLA1, SMPS_POLA_COOL );
		DIO.Output( SMPS_POLA2, SMPS_POLA_HEAT );
		tprintf( "\r\nPOLA 1CH COOL, 2CH HEAT" VT100_CURSOR_POS_RESTORE );
	}
	else
	{
		DIO.Output( SMPS_POLA1, SMPS_POLA_HEAT );
		DIO.Output( SMPS_POLA2, SMPS_POLA_COOL );
		tprintf( "\r\nPOLA 1CH HEAT, 2CH COOL" VT100_CURSOR_POS_RESTORE );
	}

}

static void tfunc_led_test( void * param )
{
	bool level = DIO.Input( LED_COM );
	DIO.Output( LED_COM, !level );
	DIO.Output( LED_ERR, !level );

	tprintf( "\r\nLED COM/ERR [%s]" VT100_CURSOR_POS_RESTORE, !level ? "ON" : "OFF" );
}
static uint8_t adc_test( void * param )
{
	static AppTimerData_ut timer_adc = {0};

	if( AppTimer.IsExpired( &timer_adc ))
	{
		AppTimer.Start( &timer_adc, 1.0f );
		tprintf( "\r\nADC TEST [%d] [%d] [%d] [%d]" VT100_CURSOR_POS_RESTORE,
				AD7124.GetValue( 0 ), AD7124.GetValue( 1 ), AD7124.GetValue( 2 ), AD7124.GetValue( 3 ) );
	}

	return 1;
}

static void tfunc_adc_test( void * param )
{
	tprintf( "\r\n\r\nADC TEST ON\r\nPress ESC to stop" );
	for( int i = 0; i < MAX_TEMP_CHANNEL; i++ )
	{
		Temp.SetType( i, SEN_RTD2X, DEFAULT_SAMPLING_PERIOD_MS );
	}
	AddTerminalFunction( adc_test, NULL );
}

uint8_t uart_tx_test( void * p )
{
	static AppTimerData_ut timer_tx = {0};

	if( AppTimer.IsExpired( &timer_tx ))
	{
		AppTimer.Start( &timer_tx, 1.0f );
		UART.Write( UART_PLC, ( uint8_t * )"UART PLC\r\n", 10 );
		UART.Write( UART_SMPS, ( uint8_t * )"UART SMPS\r\n", 11 );

		tprintf( "." );
	}

	return 1;
}

static void tfunc_uart_test( void * param )
{
	tprintf( "\r\n\r\nUART TEST ON\r\nPress ESC to stop" );
	AddTerminalFunction( uart_tx_test, NULL );
}

static void tfunc_can_test( void * param )
{
	uint8_t data[ 8 ] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
	CAN.SendMessage(0, 0, 8, data);

	tprintf( "\r\nCAN SEND TEST [ ID 0 ][ ff ff ff ff ff ff ff ff ]" VT100_CURSOR_POS_RESTORE );
}

static void tfunc_eepr_test( void * param )
{
	uint8_t ebuf[ 10 ] = { 0 };
	EEPR.Read( 0, ebuf, 10 );

	tprintf( "\r\n\r\nEEPR Read\r\n");
	for( int i = 0; i < 10; ++i )
	{
		tprintf( " %x", ebuf[ i ] );
	}
	tprintf( VT100_ERASE_LN_TO_END VT100_CURSOR_POS_RESTORE );
}

static void tfunc_fault_test( void * param )
{
	MBSDB.Holdings[ MBS_HR_FAULT_NO_NC ] = !MBSDB.Holdings[ MBS_HR_FAULT_NO_NC ];

	if( MBSDB.Holdings[ MBS_HR_FAULT_NO_NC ] )
	{
		//NC
		tprintf( "\r\nFAULT RELAY NC " );
	}
	else
	{
		//NO
		tprintf( "\r\nFAULT RELAY NO " );
	}

	tprintf( DIO.Input( FLT_RLY ) ? "[ON]" : "[OFF]" );
	tprintf( VT100_ERASE_LN_TO_END VT100_CURSOR_POS_RESTORE );
}

static void tfunc_loop_system_enable( void * param )
{
	uint16_t current = MBSDB.Holdings[ MBS_HR_RUN ];

	MBSDB.Holdings[ MBS_HR_RUN ] = !current;

	tprintf("\r\nSystem [%s]" VT100_ERASE_LN_TO_END VT100_CURSOR_POS_RESTORE, !current?"Run":"Stopped");
}

static void tfunc_loop_cfg_save( void * param )
{
	MBSDB.Holdings[ MBS_HR_SAVE_PARAMETER ] = 1;

	tprintf( "\r\nConfig Save" VT100_ERASE_LN_TO_END VT100_CURSOR_POS_RESTORE );
}

static void tfunc_loop_config_show( void * param )
{
	const control_loop_config_st * pCfg = Controller.GetConfig( loop_cfg_ch );

	if( !pCfg ) return;

	tprintf("\r\n\r\n");
	tprintf("SV                 : %.3f\r\n", pCfg->SV * 0.001f             );    // default: 0         / Range: -273150 ~ 2000000 (unit 0.001ºC)
	tprintf("Enable             : %d\r\n", pCfg->Enable                    );    // default: 0         / Range: 0, 1
	tprintf("SamplePeriod       : %.3f\r\n", pCfg->SamplePeriod * 0.001f   );    // default: 50        / Range: 0 ~ 60000 ( ms )
	tprintf("ControlPeriod      : %.3f\r\n", pCfg->ControlPeriod * 0.001f  );    // default: 50        / Range: 0 ~ 60000 ( ms )
	tprintf("InputType          : %d\r\n", pCfg->InputType                 );    // default: RTD2X     / Range: 0 ~ 8
	tprintf("InputChannel       : %d\r\n", pCfg->InputChannel              );    // default: 1         / Range: 0 ~ 20
	tprintf("OutputType         : %d\r\n", pCfg->OutputType                );    // default: N.A       / Range: 0 ~ 9
	tprintf("OutputChannel      : %d\r\n", pCfg->OutputChannel             );    // default: 0         / Range: 0 ~ 20
	tprintf("HighOverAlarm      : %.2f\r\n", pCfg->HighOverAlarm * 0.01f   );    // default: 2000      / Range: 0 ~ 20000 ( * 0.01K )
	tprintf("LowUnderAlarm      : %.2f\r\n", pCfg->LowUnderAlarm * 0.01f   );    // default: 2000      / Range: 0 ~ 20000 ( * -0.01K )
	tprintf("OverCurrAlarm      : %.3f\r\n", pCfg->OverCurrAlarm * 0.001f  );    // default: 10000     / Range: 1 ~ 10000 ( 0.001A )
	tprintf("ControlTimeOver    : %d\r\n", pCfg->ControlTimeOver           );    // default: 3600      / Range: 0 ~ 3600 (sec)
	tprintf("TempOffset         : %.2f\r\n", pCfg->TempOffset * 0.001f     );    // default: 0         / Range: -30000 ~ 30000 ( * 0.001K )
	tprintf("Pb                 : %.2f\r\n", pCfg->Pb * 0.01f              );    // default: 0         / Range: 0 ~ 60000 ( 0.01K )
	tprintf("Ti                 : %.1f\r\n", pCfg->Ti * 0.1f               );    // default: 0         / Range: 0 ~ 60000 ( 0.1s )
	tprintf("Td                 : %.1f\r\n", pCfg->Td * 0.01f              );    // default: 0         / Range: 0 ~ 60000 ( 0.01s )
	tprintf("Saturated_I        : %.3f\r\n", pCfg->Saturated_I * 0.001f    );    // default: 5000      / Range: 0 ~ 60000 ( * 0.001 )
	tprintf("InputFilerCoef     : %.2f\r\n", pCfg->InputFilterCoeff * 0.1f );    // default: 0         / Range: 0 ~ 30000 ( * 0.1s )
	tprintf("Reserved           : %.2f\r\n", pCfg->Reserved                );    // default: 0         / Range: 
	tprintf("AutoTuneEnabled    : %d\r\n", pCfg->AutoTuneEnabled           );    // default: 0         / Range: 0, 1
	tprintf("OutputMax          : %.1f\r\n", pCfg->OutputMax * 0.1f        );    // default: 1000      / Range: -1000 ~ 1000 ( * 0.1% )
	tprintf("OutputMin          : %.1f\r\n", pCfg->OutputMin * 0.1f        );    // default: -1000     / Range: -1000 ~ 1000 ( * 0.1% )
	tprintf("PWMFreq            : %.1f\r\n", pCfg->PWMFreq * 0.1f          );    // default: 10        / Range: 0 ~ 60000 ( 0.1Hz )
	tprintf("OutputDelay        : %d\r\n", pCfg->OutputDelay               );    // default: 0         / Range: 0 ~ 999 (sec)
	tprintf("ControlType        : %d\r\n", pCfg->ControlType               );    // default: 2DOF PID  / Range: 2DOF PID, PID, ONOFF
	tprintf("StartDelay         : %d\r\n", pCfg->StartDelay                );    // default: 10        / Range: 0 ~ 999 (sec)
	tprintf("SmpsLeakageCurrent : %d\r\n", pCfg->SmpsLeakageCurrent        );    // default: 100       / Range: 0 ~ 65535 (mA)
	tprintf("SmpsConfig     	: %d\r\n", pCfg->SmpsConfig.all            );    // default: 0         / Range: 
	tprintf( VT100_CURSOR_POS_RESTORE );
}

static void tfunc_loop_select( void * param )
{
	loop_cfg_ch = (int)param;
}

static uint8_t loop_channel_log( void * param )
{
	static AppTimerData_ut timer_channel_log = { .All = 0 };
	if( AppTimer.IsExpired( &timer_channel_log ) )
	{
		AppTimer.Start( &timer_channel_log, 0.19275f );
		const control_loop_config_st * const pCfg = Controller.GetConfig( loop_cfg_ch );

		if( !pCfg ) return 0;

		int PV = Controller.GetPV( loop_cfg_ch );
		float MV = Controller.GetMV( loop_cfg_ch ) * MULTIPLY_FLOAT_TO_PERCENT;

		tprintf( "%.2f, %2d, %d, %.3f, %.2f, %.1f, %.2f, %.1f, %.1f,\r\n",
					AppTimer.GetCurrentSecs(), loop_cfg_ch, pCfg->Enable, PV / 1000.0f, pCfg->SV / 1000.0f, MV,
					pCfg->Pb / 100.0f, pCfg->Ti / 10.0f, pCfg->Td / 10.0f );

		return 1;
	}

	return 0;
}

static void tfunc_loop_ch_log( void * param )
{
	tprintf( "\r\n\r\nLoop Controller CSV format log\r\nPress ESC to stop\r\n" );
	tprintf( "elapsed time, CH, Status, PV, SV, MV, Pb, Ti, Td,\r\n");
	AddTerminalFunction( loop_channel_log, NULL );
}

static void tfunc_loop_toggle_enable( void * param )
{
	uint16_t current = MBSDB.Holdings[ MBS_HR_CH1_ENABLE + loop_cfg_ch * MBS_HR_CH_SPAN ];

	MBSDB.Holdings[ MBS_HR_CH1_ENABLE + loop_cfg_ch * MBS_HR_CH_SPAN ] = !current;

	tprintf( "\r\nCH %d [%s]" VT100_ERASE_LN_TO_END VT100_CURSOR_POS_RESTORE, loop_cfg_ch + 1, !current?"Enabled":"Stopped" );
}

static void tfunc_loop_input_type( void * param )
{
	int p = (int)param;

	MBSDB.Holdings[ MBS_HR_CH1_INPUT_TYPE + loop_cfg_ch * MBS_HR_CH_SPAN ] = p;

	tprintf( "\r\nCH %d INPUT TYPE %d" VT100_ERASE_LN_TO_END VT100_CURSOR_POS_RESTORE, loop_cfg_ch + 1, p );
}

static void tfunc_loop_input_ch( void * param )
{
	int p = (int)param;

	MBSDB.Holdings[ MBS_HR_CH1_INPUT_CHANNEL + loop_cfg_ch * MBS_HR_CH_SPAN ] = p;

	tprintf( "\r\nCH %d INPUT CH [%d]" VT100_ERASE_LN_TO_END VT100_CURSOR_POS_RESTORE, loop_cfg_ch + 1, p + 1 );
}

static void tfunc_loop_output_type( void * param )
{
	int p = (int)param;

	MBSDB.Holdings[ MBS_HR_CH1_OUTPUT_TYPE + loop_cfg_ch * MBS_HR_CH_SPAN ] = p;

	tprintf( "\r\nCH %d OUTPUT TYPE [%d]" VT100_ERASE_LN_TO_END VT100_CURSOR_POS_RESTORE, loop_cfg_ch + 1, p );
}

static void tfunc_loop_output_ch( void * param )
{
	int p = (int)param;

	MBSDB.Holdings[ MBS_HR_CH1_OUTPUT_CHANNEL + loop_cfg_ch * MBS_HR_CH_SPAN ] = p;

	tprintf( "\r\nCH %d OUTPUT CH %d" VT100_ERASE_LN_TO_END VT100_CURSOR_POS_RESTORE, loop_cfg_ch + 1, p + 1 );
}

static void tfunc_loop_sv_change( void * param )
{
	int32_t sv = MBSDB.Holdings[ MBS_HR_CH1_SV_L + loop_cfg_ch * MBS_HR_CH_SPAN ];
	sv += ( uint32_t )MBSDB.Holdings[ MBS_HR_CH1_SV_H + loop_cfg_ch * MBS_HR_CH_SPAN ] << 16;

	sv += ( int )param;

	MBSDB.Holdings[ MBS_HR_CH1_SV_H + loop_cfg_ch * MBS_HR_CH_SPAN ] = ( sv >> 16 ) & 0xffff;
	MBSDB.Holdings[ MBS_HR_CH1_SV_L + loop_cfg_ch * MBS_HR_CH_SPAN ] = sv & 0xffff;
	tfunc_loop_config_show( NULL );
}

static void tfunc_loop_pb_change( void * param )
{
	MBSDB.Holdings[ MBS_HR_CH1_PB + loop_cfg_ch * MBS_HR_CH_SPAN ] += ( int )param;
	tfunc_loop_config_show( NULL );
}

static void tfunc_loop_ti_change( void * param )
{
	MBSDB.Holdings[ MBS_HR_CH1_TI + loop_cfg_ch * MBS_HR_CH_SPAN ] += ( int )param;
	tfunc_loop_config_show( NULL );
}

static void tfunc_loop_td_change( void * param )
{
	MBSDB.Holdings[ MBS_HR_CH1_TD + loop_cfg_ch * MBS_HR_CH_SPAN ] += ( int )param;
	tfunc_loop_config_show( NULL );
}

static void tfunc_config_reset( void * param )
{
	extern IWDG_HandleTypeDef hiwdg;
	static const uint8_t reset_data[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, };

	switch( (int)param )
	{
		case 0:
			tprintf( "\r\nConfig RESET"  );
			for( int addr = EEPR_CONFIG_ADDR; addr < EEPR_CONFIG_SZ; addr += 16 )
			{
				EEPR.Write( addr, ( uint8_t * )reset_data, 16 );
				HAL_IWDG_Refresh( &hiwdg );
				if( ( addr % 256 ) == 0 ) tprintf( "." );
			}
			tprintf( " OK!" VT100_ERASE_LN_TO_END VT100_CURSOR_POS_RESTORE );
			break;
		case 1:
			tprintf( "\r\nCalibration RESET" );
			for( int addr = EEPR_CALIIB_START_ADDRESS; addr < EEPR_CALIIB_END_ADDRESS; addr += 16 )
			{
				EEPR.Write( addr, ( uint8_t * )reset_data, 16 );
				HAL_IWDG_Refresh( &hiwdg );
				if( ( addr % 256 ) == 0 ) tprintf( "." );
			}
			tprintf( " OK!" VT100_ERASE_LN_TO_END VT100_CURSOR_POS_RESTORE );
			break;
		default:
			break;
	}
}

/******************************************************************************
 * @brief cal_Calibration
 *****************************************************************************/
static uint8_t tfunc_rtd_ch_calib( void * param )
{
	if( Calib.Run( SEN_RTD, Calib.status.ch, (int)param ) )
	{
		Calib.status.On = 0;
		AddTerminalFunction( NULL, param );
	}

	return 1;
}

static uint8_t tfunc_tc_ch_calib( void * param )
{
	if( Calib.Run( SEN_TC_K, Calib.status.ch, (int)param ) )
	{
		Calib.status.On = 0;
		AddTerminalFunction( NULL, param );
	}

	return 1;
}

static void tfunc_rtd( void * param )
{
	Calib.status.ch = (int)param;
	Calib.status.On = 1;

	tprintf( VT100_ERASE_LN_ALL "\r\nRTD %dCH CALIBTRAION" VT100_ERASE_SCR_TO_END, Calib.status.ch + 1 );
	AddTerminalFunction( tfunc_rtd_ch_calib, 0 );
}

static void tfunc_tc( void * param )
{
	Calib.status.ch = (int)param;
	Calib.status.On = 1;
	
	tprintf( VT100_ERASE_LN_ALL "\r\nTC %dCH CALIBTRAION" VT100_ERASE_SCR_TO_END, Calib.status.ch + 1 );
	AddTerminalFunction( tfunc_tc_ch_calib, 0 );
}

static uint8_t tfunc_show_rtd( void * param )
{
	static AppTimerData_ut timer_rtd_show;

	if( AppTimer.IsExpired( &timer_rtd_show ) )
	{
		AppTimer.Start( &timer_rtd_show, 0.5F );
		tprintf( VT100_ERASE_LN_ALL"\r\nRTD TEMPERATURE [%.3f] [%.3f] [%.3f] [%.3f]" VT100_ERASE_LN_TO_END VT100_CURSOR_POS_RESTORE,
				Temp.GetTemp( 0 ), Temp.GetTemp( 1 ), Temp.GetTemp( 2 ), Temp.GetTemp( 3 ) );
	}

	return 1;
}

static void tfunc_rtd_temp_test( void * param )
{
	for( int i = 0; i < MAX_TEMP_CHANNEL; i++ )
	{
		Temp.SetType( i, SEN_RTD2X, DEFAULT_SAMPLING_PERIOD_MS );
	}

	AddTerminalFunction( tfunc_show_rtd, 0 );
}

static uint8_t tfunc_show_tc( void * param )
{
	static AppTimerData_ut timer_tc_show;

	if( AppTimer.IsExpired( &timer_tc_show ) )
	{
		AppTimer.Start( &timer_tc_show, 0.5F );

		tprintf( VT100_ERASE_LN_ALL"\r\nTC TEMPERATURE [%.3f] [%.3f] [%.3f] [%.3f]" VT100_ERASE_LN_TO_END VT100_CURSOR_POS_RESTORE,
				Temp.GetTCTemp( SEN_TC_K, 0, 0 ), Temp.GetTCTemp( SEN_TC_K, 1, 0 ), Temp.GetTCTemp( SEN_TC_K, 2, 0 ), Temp.GetTCTemp( SEN_TC_K, 3, 0 ) );
	}

	return 1;
}

static void tfunc_tc_temp_test( void * param )
{
	for( int i = 0; i < MAX_TEMP_CHANNEL; i++ )
	{
		Temp.SetType( i, SEN_TC_K, DEFAULT_SAMPLING_PERIOD_MS );
	}

	AddTerminalFunction( tfunc_show_tc, 0 );
}

/*****************************************************************************/
/** MENU LISTS ***************************************************************/
/*****************************************************************************/

static const menu_st tmenu_bd_test[] =
{
	{ 'E', "BD EXIST?", tfunc_print_out_bd_exist, NULL, NULL },
	{ 'V', "DAC TEST -10 ~ 10V", tfunc_dac_test, ( void * )'V', NULL },
	{ 'C', "DAC TEST  0 ~ 24mA", tfunc_dac_test, ( void * )'C', NULL },
	{ 'P', "PWM TEST", tfunc_pwm_test, NULL, NULL },
	{ 'S', "SMPS POLA TEST", tfunc_smps_pola_test, NULL, NULL },
	{ '\0', NULL, NULL, NULL, NULL },
};

static const menu_st tmenu_temperature_test[] = 
{
	{ 'R', "RTD", tfunc_rtd_temp_test, NULL, NULL },
	{ 'T', "TC", tfunc_tc_temp_test, NULL, NULL },
	{ '\0', NULL, NULL, NULL, NULL }
};

static const menu_st tmenu_dist_module_test[] =
{
	{ 'I', "Analog Input", NULL, NULL, NULL },
	{ 'T', "Temp. Input", NULL, NULL, NULL },
	{ 'O', "Analog Output", NULL, NULL, NULL },
	{ 'D', "Digital Output", NULL, NULL, NULL },
	{ '\0', NULL, NULL, NULL, NULL },
};

static const menu_st tmenu_hw_test[] = 
{
	{ 'X', "OUTPUT BOARD", NULL, NULL, tmenu_bd_test },
	{ 'L', "LED TEST", tfunc_led_test, NULL, NULL,  },
	{ 'A', "ADC TEST", tfunc_adc_test, NULL, NULL },
	{ 'U', "UART TEST", tfunc_uart_test, NULL, NULL },
	{ 'C', "CAN TEST", tfunc_can_test, NULL, NULL },
	{ 'E', "EEPR TEST", tfunc_eepr_test, NULL, NULL },
	{ 'F', "FAULT TEST", tfunc_fault_test, NULL, NULL },
	{ 'T', "TEMPERATURE TEST", NULL, NULL, tmenu_temperature_test },
	{ 'D', "Distributed Module", NULL, NULL, tmenu_dist_module_test },

	{ '\0', NULL, NULL, NULL, NULL },
};

static const menu_st tmenu_rtd_calib[] =
{
	{ '1', "RTD 1CH CALIB.", tfunc_rtd, (void *)0, NULL },
	{ '2', "RTD 2CH CALIB.", tfunc_rtd, (void *)1, NULL },
	{ '3', "RTD 3CH CALIB.", tfunc_rtd, (void *)2, NULL },
	{ '4', "RTD 4CH CALIB.", tfunc_rtd, (void *)3, NULL },
	{ '\0', NULL, NULL, NULL, NULL }
};

static const menu_st tmenu_tc_calib[] =
{
	{ '1', "TC 1CH CALIB.", tfunc_tc, (void *)0, NULL },
	{ '2', "TC 2CH CALIB.", tfunc_tc, (void *)1, NULL },
	{ '3', "TC 3CH CALIB.", tfunc_tc, (void *)2, NULL },
	{ '4', "TC 4CH CALIB.", tfunc_tc, (void *)3, NULL },
	{ '\0', NULL, NULL, NULL, NULL }
};

static const menu_st tmenu_calibration[] =
{
	{ 'R', "RTD", NULL, NULL, tmenu_rtd_calib },
	{ 'T', "TC", NULL, NULL, tmenu_tc_calib },
	{ '\0', NULL, NULL, NULL, NULL }
};

static const menu_st tmenu_loop_input_type[] =
{
	{ 'X', "NONE", tfunc_loop_input_type, (void *)SEN_NONE, NULL },
	{ 'D', "RTD 2X", tfunc_loop_input_type, (void *)SEN_RTD2X, NULL },
	{ 'R', "RTD", tfunc_loop_input_type, (void *)SEN_RTD, NULL },
	{ 'K', "TC K", tfunc_loop_input_type, (void *)SEN_TC_K, NULL },
	{ 'J', "TC J", tfunc_loop_input_type, (void *)SEN_TC_J, NULL },
	{ 'E', "TC E", tfunc_loop_input_type, (void *)SEN_TC_E, NULL },
	{ 'S', "TC S", tfunc_loop_input_type, (void *)SEN_TC_S, NULL },
	{ 'T', "TC T", tfunc_loop_input_type, (void *)SEN_TC_T, NULL },
	{ 'C', "COMM", tfunc_loop_input_type, (void *)SEN_COMM, NULL },

	{ '\0', NULL, NULL, NULL, NULL }
};

static const menu_st tmenu_loop_input_ch[] =
{
	{ '0', "NONE", tfunc_loop_input_ch, (void *)-1, NULL },
	{ '1', "ADC 1CH", tfunc_loop_input_ch, (void *)0, NULL },
	{ '2', "ADC 2CH", tfunc_loop_input_ch, (void *)1, NULL },
	{ '3', "ADC 3CH", tfunc_loop_input_ch, (void *)2, NULL },
	{ '4', "ADC 4CH", tfunc_loop_input_ch, (void *)3, NULL },
	{ '5', "ADC 5CH (EX 1CH)", tfunc_loop_input_ch, (void *)4, NULL },
	{ '\0', NULL, NULL, NULL, NULL }
};

static const menu_st tmenu_loop_output_type[] =
{
	{ '1', "0~5V", tfunc_loop_output_type, (void *)OUT_V_P0_P5, NULL },
	{ '2', "0~10V", tfunc_loop_output_type, (void *)OUT_V_P0_P10, NULL },
	{ '3', "-5~5V", tfunc_loop_output_type, (void *)OUT_V_M5_P5, NULL },
	{ '4', "-10~10V", tfunc_loop_output_type, (void *)OUT_V_M10_P10, NULL },
	{ '5', "4~20mA", tfunc_loop_output_type, (void *)OUT_A_4M_20M, NULL },
	{ '6', "0~20mA", tfunc_loop_output_type, (void *)OUT_A_0M_20M, NULL },
	{ '7', "0~24mA", tfunc_loop_output_type, (void *)OUT_A_0M_24M, NULL },
	{ '8', "PWM", tfunc_loop_output_type, (void *)OUT_PWM, NULL },
	{ '9', "SMPS 6921", tfunc_loop_output_type, (void *)OUT_SMPS_NHPP_6921, NULL },
	{ 'A', "SMPS 7321", tfunc_loop_output_type, (void *)OUT_SMPS_NHPP_7325, NULL },
	{ 'B', "SMPS 6021", tfunc_loop_output_type, (void *)OUT_SMPS_CHPP_5521, NULL },
	{ 'C', "SMPS 8021", tfunc_loop_output_type, (void *)OUT_SMPS_CHPP_8021, NULL },
	{ 'D', "SMPS 2032", tfunc_loop_output_type, (void *)OUT_SMPS_NHPP_2032, NULL },
	{ 'E', "SMPS 1531", tfunc_loop_output_type, (void *)OUT_SMPS_NHPP_1531, NULL },
	{ '\0', NULL, NULL, NULL, NULL }
};

static const menu_st tmenu_loop_output_ch[] =
{
	{ '1', "DAC 1CH", tfunc_loop_output_ch, (void *)0, NULL },
	{ '2', "DAC 2CH", tfunc_loop_output_ch, (void *)1, NULL },
	{ '5', "DAC 5CH (EX 0BD 1CH)", tfunc_loop_output_ch, (void *)4, NULL },
	{ '6', "DAC 6CH (EX 0BD 2CH)", tfunc_loop_output_ch, (void *)5, NULL },
	{ '7', "DAC 5CH (EX 0BD 3CH)", tfunc_loop_output_ch, (void *)6, NULL },
	{ '8', "DAC 6CH (EX 0BD 4CH)", tfunc_loop_output_ch, (void *)7, NULL },
	{ '9', "DAC 5CH (EX 1BD 1CH)", tfunc_loop_output_ch, (void *)8, NULL },
	{ 'A', "DAC 6CH (EX 1BD 2CH)", tfunc_loop_output_ch, (void *)9, NULL },
	{ '\0', NULL, NULL, NULL, NULL }
};

static const menu_st tmenu_loop_cfg[]=
{
	{ 'S', "SHOW CONFIGs", tfunc_loop_config_show, NULL, NULL },
	{ 'L', "CHANNEL LOG", tfunc_loop_ch_log, NULL, NULL },
	{ '\0', "----", NULL, NULL, NULL },
	{ 'E', "ENABLE", tfunc_loop_toggle_enable, NULL, NULL },
	{ 'T', "INPUT TYPE", NULL, NULL, tmenu_loop_input_type },
	{ 'C', "INPUT CH", NULL, NULL, tmenu_loop_input_ch },
	{ 'O', "OUTPUT TYPE", NULL, NULL, tmenu_loop_output_type },
	{ 'X', "OUTPUT CH", NULL, NULL, tmenu_loop_output_ch },
	{ '1', "SV UP", tfunc_loop_sv_change, (void *)100, NULL },
	{ '2', "SV DN", tfunc_loop_sv_change, (void *)-100, NULL },
	{ '3', "PB UP", tfunc_loop_pb_change, (void *)1, NULL },
	{ '4', "PB DN", tfunc_loop_pb_change, (void *)-1, NULL },
	{ '5', "TI UP", tfunc_loop_ti_change, (void *)1, NULL },
	{ '6', "TI DN", tfunc_loop_ti_change, (void *)-1, NULL },
	{ '7', "TD UP", tfunc_loop_td_change, (void *)1, NULL },
	{ '8', "TD DN", tfunc_loop_td_change, (void *)-1, NULL },
	{ '\0', NULL, NULL, NULL, NULL }
};

static const menu_st tmenu_loop_sel[] =
{
	{ 'E', "Run/Stop", tfunc_loop_system_enable, NULL, NULL },
	{ 'S', "SAVE CFG", tfunc_loop_cfg_save, NULL, NULL },
	{ '\0', "----------", NULL, NULL, NULL },
	{ '1', "LOOP 1CH", tfunc_loop_select, (void *)0, tmenu_loop_cfg },
	{ '2', "LOOP 2CH", tfunc_loop_select, (void *)1, tmenu_loop_cfg },
	{ '3', "LOOP 3CH", tfunc_loop_select, (void *)2, tmenu_loop_cfg },
	{ '4', "LOOP 4CH", tfunc_loop_select, (void *)3, tmenu_loop_cfg },
	{ '5', "LOOP 5CH", tfunc_loop_select, (void *)4, tmenu_loop_cfg },
	{ '\0', NULL, NULL, NULL, NULL }
};

static const menu_st tmenu_config_reset[] =
{
	{ '\0', "WARNING", NULL, NULL, NULL },
	{ '\0', "DO NOT reset while on process line.", NULL, NULL, NULL },
	{ '\0', "----------", NULL, NULL, NULL },
	{ 'R', "Config RESET", tfunc_config_reset, (void*)0, NULL },
	{ 'C', "Calibration RESET", tfunc_config_reset, (void*)1, NULL },
	{ '\0', NULL, NULL, NULL, NULL }
};

/******************************************************************************
 * @brief 
 * 
 *****************************************************************************/
const menu_st MenuHome[] =
{
	{ 'T', "HW Test", NULL, NULL, tmenu_hw_test },
	{ 'C', "Temp. Calib.", NULL, NULL, tmenu_calibration },
	{ 'L', "Loop Config Test", NULL, NULL, tmenu_loop_sel },
	{ '\0', "----------", NULL, NULL, NULL },
	{ 'R', "Config Reset", NULL, NULL, tmenu_config_reset },
	{ '\0', NULL, NULL, NULL, NULL }
};

#endif /* USE_LEGACY_TERMINAL */
