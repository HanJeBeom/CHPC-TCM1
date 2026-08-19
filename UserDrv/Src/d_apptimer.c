/******************************************************************************
 * @file d_apptimer.c
 * @author jylee1 (jylee1@gst-in.com)
 * @brief 
 * @version 0.1
 * @date 2023-05-03
 * 
 * @copyright Copyright (c) 2023
 * 
 This file is part of closed source software.
 Unauthorized reproduction and redistribution prohibited.
 *****************************************************************************/

/*****************************************************************************/
/** INCLUDES *****************************************************************/
/*****************************************************************************/
#include "d_apptimer.h"

#include "stm32f4xx.h"

/*****************************************************************************/
/** MACRO DEFINITIONS ********************************************************/
/*****************************************************************************/
#define SysTick_Ext_gp								24												// 확장비트 위치
#define SysTick_Ext_Inc_Val							( 1 << SysTick_Ext_gp )							// "SysTick"인터럽트 발생시 마다 증가하는 값

#define SysTick_CVR_Max								( SysTick_Ext_Inc_Val - 1 )						// CVR : Current Value Register

#define App_TickTocks_Max							0x3FFFFFFFFFFFFFFFULL							// 62bits
#define App_TickTocks_Mask( ticktock )				( ticktock & App_TickTocks_Max )

#define App_Timer_Status_bit_RunStop_bp				30
#define App_Timer_Status_bit_Over_bp				31
#define App_Timer_Status_bit_RunStop_bm				( 1 << App_Timer_Status_bit_RunStop_bp )
#define App_Timer_Status_bit_Over_bm				( 1 << App_Timer_Status_bit_Over_bp )

#define App_TickTocks_Update_OnDemand               // 앱타이머 동작 개시시, 설정시간 초과 확인시등 필요할 때 마다 "App_Timer.Clock.TickTocks"값을 최신 값으로 갱신.

/*****************************************************************************/
/** DATATYPES ****************************************************************/
/*****************************************************************************/

/*****************************************************************************/
/** LOCAL VARIABLES **********************************************************/
/*****************************************************************************/
static __RAM_FUNC uint32_t App_Timer_Control( AppTimerData_ut *Timer, int Command, float Set_Time );
static __RAM_FUNC void app_timer_start( AppTimerData_ut *Timer, float Set_Time );
static __RAM_FUNC void app_timer_stop( AppTimerData_ut *Timer );
static __RAM_FUNC uint32_t App_Timer_Query_Run( AppTimerData_ut *Timer );
static __RAM_FUNC uint32_t App_Timer_Query_Over( AppTimerData_ut *Timer );
static __RAM_FUNC void App_Timer_Delay( float Delay_Time );
static __RAM_FUNC float App_Timer_GetCurrentSecs( void );
static __RAM_FUNC void App_Timer_Update_Clock_Service( void );
static __RAM_FUNC void Update_Current_TickTocks( void );
static __RAM_FUNC void App_Timer_Update_Scan_Time( bool update );

/*****************************************************************************/
/** GLOBAL VARIABLES *********************************************************/
/*****************************************************************************/

/**
 * @brief Application Timer
 * 
 */
AppTimer_t AppTimer =
{
	.Control = App_Timer_Control,
	.Start = app_timer_start,
	.Stop = app_timer_stop,
	.IsRun = App_Timer_Query_Run,
	.IsExpired = App_Timer_Query_Over,
	.Delay = App_Timer_Delay,
	.GetCurrentSecs = App_Timer_GetCurrentSecs,

	.Clock.Update_Flags = App_Timer_Update_Clock_Service,
	.Clock.Update_TickTocks = Update_Current_TickTocks,

	.Scan_Time.Update = App_Timer_Update_Scan_Time,
};

/*****************************************************************************/
/** FUNCTION DEFINITIONS *****************************************************/
/*****************************************************************************/

/******************************************************************************
 * @brief    Set configuration for Systick
 *
 * @note     Reload Value Register 값을 최대값(0xFFFFFF)으로 설정
 * @note                 - ( 0xFFFFFF + 1 ) / 168MHz 주기로 인터럽트 발생 -> 99.86438ms
 * @note            ※ 인터럽트 마다 [AppTimer.Clock.TickTocks]를 [1 << 24]값 만큼 증가
 *****************************************************************************/
static void Set_Application_Timer( void )
{
	SysTick_Config( SysTick_LOAD_RELOAD_Msk + 1 );
}

/******************************************************************************
 * @brief       Update_Current_TickTocks
 * @details     1. "AppTimer.Clock.TickTocks" + SysTick_CVR
 *                  - SysTick, that counts down from the reload value to zero
 *
 * @param
 * @return
 *****************************************************************************/
static void Update_Current_TickTocks( void )
{
	uint32_t prim = __get_PRIMASK();

	__disable_irq();
	__DMB();

	uint64_t ticktocks = AppTimer.Clock.TickTocks;
	ticktocks &= ~SysTick_CVR_Max;
	ticktocks |= ~SysTick->VAL & SysTick_CVR_Max;
	/*
	 이 시점에서 SysTick->VAL 값이 '0' 이라면!
	 즉  SysTick->CTRL 레지스터의 "COUNTFLAG" 비트가 '1'로 SysTick 인터럽트가 펜딩 상태가 되면!
	 "Enable_global_interrupt()" 실행 후 "AppTimer.Clock.TickTocks" 값이 바로 업데이트 됨.
	 */
	AppTimer.Clock.TickTocks = ticktocks;

	if( !prim )
	{
		__DMB();
		__enable_irq();
	}
}

/******************************************************************************
 * @brief       App_Timer_Run_Stop
 * @details     RUN & STOP APPLICATION TIMER
 *
 * @param       Timer : Timer Instance
 *              Command :'R'->Run, 'S'->Stop)
 *              Set_Time : 20.83ns (@ 48MHz) ~ 9607679205 secs (304.6575 years) max.
 * @return
 *****************************************************************************/
static void App_Timer_Run_Stop( AppTimerData_ut *Timer, int Command, float Set_Time )
{
	if( Command == 'R' )  // Run
	{
#if defined( App_TickTocks_Update_OnDemand )
		Update_Current_TickTocks();
#endif
		//uint64_t systicks = ( uint64_t )( Set_Time *  sysclk_get_cpu_hz());  //(float)sysclk_get_cpu_hz());
		uint64_t systicks = ( uint64_t )( Set_Time * HAL_RCC_GetSysClockFreq() );
		Timer->bit.Ticks = AppTimer.Clock.TickTocks + systicks;
		Timer->bit.Over_Flag = 0;
		Timer->bit.Run_Flag = 1;
	}
	else if( Command == 'S' )  // Stop
	{
		Timer->bit.Over_Flag = 0;
		Timer->bit.Run_Flag = 0;
	}
}

/******************************************************************************
 * @brief       App_Timer_Query_Run
 * @details
 *
 * @param       Timer : Timer Instance
 * @return      0 : Stopped
 *              1 : Run
 *****************************************************************************/
static uint32_t App_Timer_Query_Run( AppTimerData_ut *Timer )
{
	return Timer && Timer->bit.Run_Flag;
}

/******************************************************************************
 * @brief       App_Timer_Query_Over
 * @details
 *
 * @param       Timer: Timer Instance
 * @return      0 : Timer is not expired
 *              1 : Timer is expired(When bit  is set to "1")
 *****************************************************************************/
static uint32_t App_Timer_Query_Over( AppTimerData_ut *Timer )
{
	if( Timer ) //&& !Timer->bit.Over_Flag )
	{
#if defined( App_TickTocks_Update_OnDemand )
		Update_Current_TickTocks();
#endif

		if( AppTimer.Clock.TickTocks >= App_TickTocks_Mask( Timer->All ) )
		{
			Timer->bit.Over_Flag = 1;
		}

		return Timer->bit.Over_Flag;
	}

	return 0;
}

/******************************************************************************
 * @brief       App_Timer_Control
 * @details
 *
 * @param       Timer_No: Timer No.
 *              Command:'R'->Run, 'r'->force restart, 'S'->Stop
 *              Set_Time: ...
 * @return      bit30 : Timer Status(0:Stop, 1:Running)
 *              bit31 : Time Over(When bit  is set to "1")
 *****************************************************************************/
static uint32_t App_Timer_Control( AppTimerData_ut *Timer, int Command, float Set_Time )
{
	uint32_t Return_Value = 0;

	if( Command == 'R' )  // Run
	{
		if( !Timer->bit.Run_Flag )
		{
			App_Timer_Run_Stop( Timer, 'R', Set_Time ); // Update Elapsed ticktocks & .Over_Flag = 0 & .Run_Flag = 1
		}
	}
	else if( Command == 'S' )  // Stop
	{
		App_Timer_Run_Stop( Timer, 'S', Set_Time );     // .Over_Flag & .Run_Flag = 0
	}
	else if( Command == 'r' )  // Restart          /*  2020-01-30: by Kang Min Soo */
	{
		App_Timer_Run_Stop( Timer, 'R', Set_Time );
	}
	// Set Return bit
	if( Timer->bit.Run_Flag )
	{
		Return_Value |= App_Timer_Status_bit_RunStop_bm;
		if( App_Timer_Query_Over( Timer ) )
		{
			Return_Value |= App_Timer_Status_bit_Over_bm;
			App_Timer_Run_Stop( Timer, 'R', Set_Time ); // Restart : Update Elapsed ticktocks & .Over_Flag = 0 & .Run_Flag = 1
		}
		else
		{
			Return_Value &= ~App_Timer_Status_bit_Over_bm;
		}
	}

	return Return_Value;
}

/******************************************************************************
 * @brief       Start Timer
 * @details
 *
 * @param       Timer pointer to Timer instance
 *****************************************************************************/
static void app_timer_start( AppTimerData_ut *Timer, float Set_Time )
{
	if( Timer )
	{
#if defined( App_TickTocks_Update_OnDemand )
		Update_Current_TickTocks();
#endif
		uint64_t systicks = ( uint64_t )( Set_Time * HAL_RCC_GetSysClockFreq() );
		Timer->bit.Ticks = AppTimer.Clock.TickTocks + systicks;
		Timer->bit.Over_Flag = 0;
		Timer->bit.Run_Flag = 1;
	}
}

/******************************************************************************
 * @brief       Stop Timer
 * @details
 *
 * @param       Timer: pointer to Timer instance
 *****************************************************************************/
static void app_timer_stop( AppTimerData_ut *Timer )
{
	if( Timer )
	{
		Timer->bit.Over_Flag = 0;
		Timer->bit.Run_Flag = 0;
	}
}

/******************************************************************************
 * @brief       App_Timer_Delay
 * @details
 *
 * @param       Delay_Time
 * @return
 *****************************************************************************/
static void App_Timer_Delay( float Delay_Time )
{
	static AppTimerData_ut Timer_Delay;
	volatile uint32_t Timer_Return;

	Update_Current_TickTocks();
	App_Timer_Control( &Timer_Delay, 'R', Delay_Time );
	do
	{
		Update_Current_TickTocks();
		Timer_Return = App_Timer_Query_Over( &Timer_Delay );
	} while( !Timer_Return );
	App_Timer_Control( &Timer_Delay, 'S', 0.0F );
}

/******************************************************************************
 * @brief
 *
 * @return
 *****************************************************************************/
static float App_Timer_GetCurrentSecs( void )
{
	Update_Current_TickTocks();
	return AppTimer.Clock.TickTocks / ( float )HAL_RCC_GetSysClockFreq();
}

/******************************************************************************
 * @brief       App_Timer_Update_Clock_Service
 * @details
 *
 * @param
 * @return
 *****************************************************************************/
static void App_Timer_Update_Clock_Service( void )
{
	static AppTimerData_ut Timer_Clock_Service;

	if( AppTimer.Control( &Timer_Clock_Service, 'R', 0.1F ) & App_Timer_Status_bit_Over_bm )
	{
		AppTimer.Clock.Flag.bit.Clock_200ms ^= 1;  // 0N -> 100ms & OFF -> 100ms
		++AppTimer.Clock.Flag.Group.Tick_Count;
		if( AppTimer.Clock.Flag.Group.Tick_Count >= 10 )
		    AppTimer.Clock.Flag.Group.Tick_Count = 0;
		if( AppTimer.Clock.Flag.Group.Tick_Count < 5 )
		AppTimer.Clock.Flag.bit.Clock_1s = 0;
		else
		AppTimer.Clock.Flag.bit.Clock_1s = 1;
	}
	// 매 100ms, 500ms마다 1스캔 동안만 ON이 되도록...
	AppTimer.Clock.Flag.Group.Tick = AppTimer.Clock.Flag.Group.Clock ^ AppTimer.Clock.Flag.Group.oldClock;
	AppTimer.Clock.Flag.Group.oldClock = AppTimer.Clock.Flag.Group.Clock;
}

/******************************************************************************
 * @brief       App_Timer_Update_Scan_Time
 * @details
 *
 * @param       update: false => re-init scan time / true => update scan time
 * @return
 *****************************************************************************/
static void App_Timer_Update_Scan_Time( bool update )
{
	uint64_t ticktocks;

	Update_Current_TickTocks();
	ticktocks = AppTimer.Clock.TickTocks;
	if( update )
	{
		// 스캔시간 계산
		AppTimer.Scan_Time.Now = ( float )( ticktocks - AppTimer.Scan_Time.old_TickTock ) / HAL_RCC_GetSysClockFreq();
		if( AppTimer.Scan_Time.Now > AppTimer.Scan_Time.Max )
		    AppTimer.Scan_Time.Max = AppTimer.Scan_Time.Now;
#if 0
            static uint32_t errcount;
            if ( AppTimer.Scan_Time.old_TickTock >= ticktocks )
                ++errcount;
            if ( AppTimer.Scan_Time.Now > 1.0F )
                ++errcount;
#endif
	}
	else
	{
		AppTimer.Scan_Time.Now = 0;
		AppTimer.Scan_Time.Max = 0;
	}
	AppTimer.Scan_Time.old_TickTock = ticktocks;
}

/******************************************************************************
 * @brief       AppTimer_Tick_Handler
 * @details     Tick handler
 *              It must be called by SysTick Handler
 *
 * @param
 * @return
 *****************************************************************************/
void AppTimer_Tick_Handler( void )
{
	uint64_t ticktocks = AppTimer.Clock.TickTocks;

	ticktocks &= ~SysTick_CVR_Max;
	ticktocks |= ~SysTick->VAL & SysTick_CVR_Max;
	ticktocks += SysTick_Ext_Inc_Val;
	AppTimer.Clock.TickTocks = ticktocks;

	SysTick->CTRL;  // Clear "COUNTFLAG"
}

/******************************************************************************
 * @brief      Initialize AppTimer
 * @details
 *
 *****************************************************************************/
void AppTimer_Init( void )
{
	Set_Application_Timer();
}
