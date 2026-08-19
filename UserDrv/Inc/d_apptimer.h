/******************************************************************************
 * @file d_apptimer.h
 * @author jylee1 (jylee1@gst-in.com)
 * @brief 
 * @version 0.1
 * @date 2023-05-03
 * 
 * @copyright Copyright (c) 2023
 * 
 *****************************************************************************/

#ifndef INC_D_APPTIMER_H_
#define INC_D_APPTIMER_H_

#ifdef __cplusplus
extern "C" {
#endif

/*****************************************************************************/
/** INCLUDES *****************************************************************/
/*****************************************************************************/

#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

/*****************************************************************************/
/** MACRO DEFINITIONS ********************************************************/
/*****************************************************************************/

/*****************************************************************************/
/** DATATYPES ****************************************************************/
/*****************************************************************************/

typedef union {
	struct
	{
		uint64_t Ticks : 62;
		uint64_t Run_Flag : 1;		// bit-62
		uint64_t Over_Flag : 1;	 	// bit-63
	}bit;
	uint64_t All;
} AppTimerData_ut;

typedef union App_Timer_Clock_union
{
	struct {
		uint32_t Tick_Counter_100ms_bit0 : 1;
		uint32_t Tick_Counter_100ms_bit1 : 1;
		uint32_t Tick_Counter_100ms_bit2 : 1;
		uint32_t Tick_Counter_100ms_bit3 : 1;
		uint32_t Tick_Counter_100ms_bit4 : 1;
		uint32_t Tick_Counter_100ms_bit5 : 1;
		uint32_t Tick_Counter_100ms_bit6 : 1;
		uint32_t Tick_Counter_100ms_bit7 : 1;
		uint32_t Clock_200ms : 1;				// ON -> 100ms & OFF -> 100ms
		uint32_t Clock_1s : 1;					// ON -> 500ms & OFF -> 500ms
		uint32_t Clock_bit11_10 : 2;
		uint32_t Tick_100ms : 1;				// ON for 1-scan time every 100ms
		uint32_t Tick_500ms : 1;				// ON for 1-scan time every 500ms
		uint32_t Tick_bit15_14 : 2;
		uint32_t oldClock_bit0 : 1;
		uint32_t oldClock_bit1 : 1;
		uint32_t oldClock_bit2 : 1;
		uint32_t oldClock_bit3 : 1;
		uint32_t bit31_20 : 12;
	} bit;
	struct {
		uint32_t Tick_Count : 8;
		uint32_t Clock : 4;
		uint32_t Tick : 4;
		uint32_t oldClock : 4;
		uint32_t bit31_20 : 12;
	} Group;
}AppTimerClock_ut;

typedef struct App_Timer_Scan_Time_struct
{
	float Now;
	float Max;
	uint64_t old_TickTock;
	void ( *Update )( bool update );
}AppTimerScanTime_st;

typedef struct {
	uint32_t ( *Control )( AppTimerData_ut *Timer, int Command, float Set_Time );
	void ( *Start )( AppTimerData_ut *Timer, float Set_Time );
	void ( *Stop )( AppTimerData_ut *Timer );
	uint32_t ( *IsRun )( AppTimerData_ut *Timer );
	uint32_t ( *IsExpired )( AppTimerData_ut *Timer );
	void ( *Delay )( float Delay_Time );
	float ( *GetCurrentSecs )( void );
	struct{
		AppTimerClock_ut Flag;
		void ( *Update_Flags )( void );
		uint64_t TickTocks;
		void ( *Update_TickTocks )( void );
	}Clock;
	AppTimerScanTime_st Scan_Time;
} AppTimer_t;

/*****************************************************************************/
/** FUNCTION DECLARATIONS ****************************************************/
/*****************************************************************************/

void AppTimer_Tick_Handler( void );
void AppTimer_Init( void );

/*****************************************************************************/
/** EXTERNAL VARIABLES *******************************************************/
/*****************************************************************************/

extern AppTimer_t AppTimer;

/*****************************************************************************/
/** GLOBAL VARIABLES *********************************************************/
/*****************************************************************************/

/*****************************************************************************/
/** FUNCTION DEFINITIONS *****************************************************/
/*****************************************************************************/


#ifdef __cplusplus
}
#endif

#endif /* INC_D_APPTIMER_H_ */
