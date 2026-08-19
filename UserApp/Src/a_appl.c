/******************************************************************************
 * @file a_appl.c
 * @author jylee1 (jylee1@gst-in.com)
 * @brief 
 * @version 0.1
 * @date 2023-04-27
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

/*****************************************************************************/
/** DATATYPES ****************************************************************/
/*****************************************************************************/

typedef enum
{
	BOOT_LED_IDLE  = 0,
	BOOT_LED_RUN_1,
	BOOT_LED_COM_1,
	BOOT_LED_ERR_1,
	BOOT_LED_RUN_2,
	BOOT_LED_COM_2,
	BOOT_LED_ERR_2,
} boot_led_step_et;

/*****************************************************************************/
/** FUNCTION DECLARATIONS ****************************************************/
/*****************************************************************************/

static void boot_led_start( void );
static void boot_led_task( void );

/*****************************************************************************/
/** GLOBAL VARIABLES *********************************************************/
/*****************************************************************************/

/*****************************************************************************/
/** LOCAL VARIABLES **********************************************************/
/*****************************************************************************/

static AppTimerData_ut can_start_delay_timer = { .All = 0 };
static AppTimerData_ut run_led_toggle_timer  = { .All = 0 };
static AppTimerData_ut boot_led_timer        = { .All = 0 };
static bool            run_led_status        = true;
static boot_led_step_et boot_led_step        = BOOT_LED_IDLE;

/*****************************************************************************/
/** FUNCTION DEFINITIONS *****************************************************/
/*****************************************************************************/

/******************************************************************************
 * @brief 
 * 
 *****************************************************************************/
void UserApplication_Init( void )
{
	__HAL_DBGMCU_FREEZE_IWDG();
	__HAL_DBGMCU_FREEZE_WWDG();

	DIO.GetBoardID();

	AppTimer_Init();

	I2cInit();
	EepromInit();
	TCN75AInit();
	AD7124Init();
	AD5422Init();

	ControllerTaskInit();
	CanInit();
	UartInit();
	UsbCdcInit();

	CalibrationInit();
	if( Calib.IsDone() )
	{
		boot_led_start();
	}
	TempInit();

#if !defined( USE_LEGACY_TERMINAL )
	TestFuncInit();
#endif

	ADC_MCU.Start();

	if( Sys.GetResetCause() != RESET_CAUSE_POWER_ON_POWER_DOWN_RESET )
	{
		int reset_cnt = 0;
		EEPR.Read( 0x079C, &reset_cnt, 4 );
		reset_cnt++;
		EEPR.Write( 0x079C, &reset_cnt, 4 );
	}

	AppTimer.Scan_Time.Update( false );

	AppTimer.Start( &can_start_delay_timer, 5.0F );
}

/******************************************************************************
 * @brief 
 * 
 *****************************************************************************/
/******************************************************************************
 * @brief Start the boot LED sequence (called once if calibration is done).
 *****************************************************************************/
static void boot_led_start( void )
{
	boot_led_step = BOOT_LED_RUN_1;
	DIO.Output( LED_RUN, LED_ON );
	AppTimer.Start( &boot_led_timer, 0.5f );
}

/******************************************************************************
 * @brief Non-blocking state machine: cycles LED_RUN → LED_COM → LED_ERR × 2.
 *****************************************************************************/
static void boot_led_task( void )
{
	if( BOOT_LED_IDLE == boot_led_step )           return;
	if( !AppTimer.IsExpired( &boot_led_timer ) )   return;

	switch( boot_led_step )
	{
		case BOOT_LED_RUN_1:
		case BOOT_LED_RUN_2:   DIO.Output( LED_RUN, LED_OFF );  break;
		case BOOT_LED_COM_1:
		case BOOT_LED_COM_2:   DIO.Output( LED_COM, LED_OFF );  break;
		case BOOT_LED_ERR_1:
		case BOOT_LED_ERR_2:   DIO.Output( LED_ERR, LED_OFF );  break;
		default:               break;
	}

	boot_led_step++;

	switch( boot_led_step )
	{
		case BOOT_LED_RUN_1:
		case BOOT_LED_RUN_2:   DIO.Output( LED_RUN, LED_ON );  AppTimer.Start( &boot_led_timer, 0.5f );  break;
		case BOOT_LED_COM_1:
		case BOOT_LED_COM_2:   DIO.Output( LED_COM, LED_ON );  AppTimer.Start( &boot_led_timer, 0.5f );  break;
		case BOOT_LED_ERR_1:
		case BOOT_LED_ERR_2:   DIO.Output( LED_ERR, LED_ON );  AppTimer.Start( &boot_led_timer, 0.5f );  break;
		default:               boot_led_step = BOOT_LED_IDLE;                                             break;
	}
}

void UserApplication( void )
{
	if( AppTimer.IsExpired( &run_led_toggle_timer ) )
	{
		AppTimer.Start( &run_led_toggle_timer, CFG.system.Run ? 0.1f : 0.5f );

		if( BOOT_LED_IDLE == boot_led_step )
		{
			DIO.Output( LED_RUN, run_led_status );
			run_led_status = !run_led_status;
		}
	}

	boot_led_task();

	if( AppTimer.IsExpired( &can_start_delay_timer ) )
	{
		AppTimer.Stop( &can_start_delay_timer );
		CAN.Start();
	}

	CanRxTask();
	AD7124Task();
	TemperatureTask();
	ControllerTask();
	OutputTask();

	CanTxTask();

	UartRxTask();
	UsbCdcTask();
	EepromTask();

	//TestSmpsTask();
	ModbusMasterTask();
	ModbusSlaveTask();

	FaultCheckTask();

#if defined( USE_LEGACY_TERMINAL )
	TerminalTask();
#else
	TestFuncTask();
#endif

	LogicTask();

	AppTimer.Scan_Time.Update( true );
}
