/******************************************************************************
 * @file d_system.h
 * @author Lee Jinyoung (jylee1@gst-in.com)
 * @brief 
 * @version 0.1
 * @date 2023-10-05
 * 
 * @copyright Copyright (c) 2023 Global Standard Technology Co., Ltd.
 * 
 * All rights reserved.
 * This file is part of closed source software.
 * Unauthorized reproduction and redistribution prohibited.
 *****************************************************************************/

#ifndef _D_SYSTEM_H_
#define _D_SYSTEM_H_

/*****************************************************************************/
/** INCLUDES *****************************************************************/
/*****************************************************************************/

/*****************************************************************************/
/** MACRO DEFINITIONS ********************************************************/
/*****************************************************************************/

#define EXCPT_ILLEGAL_INSTRUCTION 0x0050
#define EXCPT_ACCESS_VIOLATION 0x0051
#define EXCPT_MISALIGNMENT 0x0100
#define EXCPT_DIVIDEBYZERO 0x0102
#define EXCPT_FPU_ERROR 0x0150
#define EXCPT_FPU_UNDERFLOW 0x0157
#define EXCPT_UNKNOWN 0x7fff
#define EXCPT_NONCONTINUABLE 0x0104

/*****************************************************************************/
/** DATATYPES ****************************************************************/
/*****************************************************************************/

/******************************************************************************
 * @brief Possible STM32 system reset causes
 *****************************************************************************/

typedef enum reset_cause_e
{
	RESET_CAUSE_UNKNOWN = 0,
	RESET_CAUSE_LOW_POWER_RESET,
	RESET_CAUSE_WINDOW_WATCHDOG_RESET,
	RESET_CAUSE_INDEPENDENT_WATCHDOG_RESET,
	RESET_CAUSE_SOFTWARE_RESET,
	RESET_CAUSE_POWER_ON_POWER_DOWN_RESET,
	RESET_CAUSE_EXTERNAL_RESET_PIN_RESET,
	RESET_CAUSE_BROWNOUT_RESET,
} reset_cause_et;

typedef struct System_struct_Tag {
	reset_cause_et (*GetResetCause)( void );
	char const * const (*GetResetCauseName)( void );
	int (*GetResetCount)( void );
	int (*GetLastException)( void );
	char const * const (*GetLastExceptionName)( void );
} system_st;

/*****************************************************************************/
/** FUNCTION DECLARATIONS ****************************************************/
/*****************************************************************************/

/*****************************************************************************/
/** EXTERNAL VARIABLES *******************************************************/
/*****************************************************************************/

extern const system_st Sys;

/*****************************************************************************/
/** LOCAL VARIABLES **********************************************************/
/*****************************************************************************/

/*****************************************************************************/
/** GLOBAL VARIABLES *********************************************************/
/*****************************************************************************/

/*****************************************************************************/
/** FUNCTION DEFINITIONS *****************************************************/
/*****************************************************************************/




#endif /* _D_SYSTEM_H_ */
