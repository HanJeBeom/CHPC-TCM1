/******************************************************************************
 * @file d_system.c
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

/*****************************************************************************/
/** INCLUDES *****************************************************************/
/*****************************************************************************/

#include "UserDrivers.h"

/*****************************************************************************/
/** MACRO DEFINITIONS ********************************************************/
/*****************************************************************************/

/*****************************************************************************/
/** DATATYPES ****************************************************************/
/*****************************************************************************/

typedef struct
{
	uint32_t code;
	char *name;
} CODE_STRING_TABLE;

/*****************************************************************************/
/** FUNCTION DECLARATIONS ****************************************************/
/*****************************************************************************/

static reset_cause_et get_reset_cause( void );
static char const * const get_reset_cause_name( void );
static int get_reset_count( void );
static int get_last_exception( void );
static char const * const get_last_exception_name( void );

/*****************************************************************************/
/** EXTERNAL VARIABLES *******************************************************/
/*****************************************************************************/

/*****************************************************************************/
/** LOCAL VARIABLES **********************************************************/
/*****************************************************************************/

static reset_cause_et reset_cause;
static int reset_count = 0;

static const CODE_STRING_TABLE reset_name_tbl[] =
{
	{ RESET_CAUSE_UNKNOWN, "UNKNOWN" },
	{ RESET_CAUSE_LOW_POWER_RESET, "LOW POWER RESET" },
	{ RESET_CAUSE_WINDOW_WATCHDOG_RESET, "W-WDG RESET" },
	{ RESET_CAUSE_INDEPENDENT_WATCHDOG_RESET, "I-WDG RESET" },
	{ RESET_CAUSE_SOFTWARE_RESET, "SOFTWARE RESET" },
	{ RESET_CAUSE_POWER_ON_POWER_DOWN_RESET, "POWER-ON RESET (POR) / POWER-DOWN RESET (PDR)" },
	{ RESET_CAUSE_EXTERNAL_RESET_PIN_RESET, "EXTERNAL RESET" },
	{ RESET_CAUSE_BROWNOUT_RESET, "BROWNOUT RESET (BOR)" },
	{ 0, NULL }
};

static int last_exception = 0;
static const CODE_STRING_TABLE exception_name_tbl[] =
{
	{ EXCPT_ILLEGAL_INSTRUCTION, "ILLEGAL INSTRUCTION" },
	{ EXCPT_ACCESS_VIOLATION, "ACCESS VIOLATION" },
	{ EXCPT_MISALIGNMENT, "MISALIGNMENT" },
	{ EXCPT_DIVIDEBYZERO, "DIV BY 0" },
	{ EXCPT_FPU_ERROR, "FPU ERROR" },
	{ EXCPT_FPU_UNDERFLOW, "FPU UNDERFLOW" },
	{ EXCPT_UNKNOWN, "UNKNOWN" },
	{ EXCPT_NONCONTINUABLE, "NONCONTINUABLE" },
	{ 0, NULL }
};

/*****************************************************************************/
/** GLOBAL VARIABLES *********************************************************/
/*****************************************************************************/

const system_st Sys = {
	.GetResetCause = get_reset_cause,
	.GetResetCauseName = get_reset_cause_name,
	.GetResetCount = get_reset_count,
	.GetLastException = get_last_exception,
	.GetLastExceptionName = get_last_exception_name,
};

/*****************************************************************************/
/** FUNCTION DEFINITIONS *****************************************************/
/*****************************************************************************/

/******************************************************************************
 * @brief Obtain the STM32 system reset cause
 * 
 * @return The system reset cause
 * 
 * @note any of the STM32 Hardware Abstraction Layer (HAL) Reset and Clock
 *       Controller (RCC) header files, such as
 *       "STM32Cube_FW_F7_V1.12.0/Drivers/STM32F7xx_HAL_Driver/Inc/stm32f7xx_hal_rcc.h",
 *       "STM32Cube_FW_F2_V1.7.0/Drivers/STM32F2xx_HAL_Driver/Inc/stm32f2xx_hal_rcc.h",
 *       etc., indicate that the brownout flag, `RCC_FLAG_BORRST`, will be set in
 *       the event of a "POR/PDR or BOR reset". This means that a Power-On Reset
 *       (POR), Power-Down Reset (PDR), OR Brownout Reset (BOR) will trip this flag.
 *       See the doxygen just above their definition for the
 *       `__HAL_RCC_GET_FLAG()` macro to see this: 
 *       "@arg RCC_FLAG_BORRST: POR/PDR or BOR reset." <== indicates the Brownout
 *        Reset flag will *also* be set in the event of a POR/PDR.
 *        Therefore, you must check the Brownout Reset flag, `RCC_FLAG_BORRST`, *after*
 *        first checking the `RCC_FLAG_PORRST` flag in order to ensure first that the
 *        reset cause is NOT a POR/PDR reset.

 *****************************************************************************/
static reset_cause_et get_reset_cause( void )
{
	if( reset_cause == RESET_CAUSE_UNKNOWN )
	{
		if( __HAL_RCC_GET_FLAG( RCC_FLAG_LPWRRST ) )
		{
			reset_cause = RESET_CAUSE_LOW_POWER_RESET;
		}
		else if( __HAL_RCC_GET_FLAG( RCC_FLAG_WWDGRST ) )
		{
			reset_cause = RESET_CAUSE_WINDOW_WATCHDOG_RESET;
		}
		else if( __HAL_RCC_GET_FLAG( RCC_FLAG_IWDGRST ) )
		{
			reset_cause = RESET_CAUSE_INDEPENDENT_WATCHDOG_RESET;
		}
		else if( __HAL_RCC_GET_FLAG( RCC_FLAG_SFTRST ) )
		{
			// This reset is induced by calling the ARM CMSIS
			// `NVIC_SystemReset()` function!
			reset_cause = RESET_CAUSE_SOFTWARE_RESET;
		}
		else if( __HAL_RCC_GET_FLAG( RCC_FLAG_PORRST ) )
		{
			reset_cause = RESET_CAUSE_POWER_ON_POWER_DOWN_RESET;
		}
		else if( __HAL_RCC_GET_FLAG( RCC_FLAG_PINRST ) )
		{
			reset_cause = RESET_CAUSE_EXTERNAL_RESET_PIN_RESET;
		}
		// Needs to come *after* checking the `RCC_FLAG_PORRST` flag in order to
		// ensure first that the reset cause is NOT a POR/PDR reset. See note
		// above.
		else if( __HAL_RCC_GET_FLAG( RCC_FLAG_BORRST ) )
		{
			reset_cause = RESET_CAUSE_BROWNOUT_RESET;
		}
		else
		{
			reset_cause = RESET_CAUSE_UNKNOWN;
		}

		// Clear all the reset flags or else they will remain set during future
		// resets until system power is fully removed.
		__HAL_RCC_CLEAR_RESET_FLAGS( );
	}

	return reset_cause;
}

/******************************************************************************
 * @brief Obtain the system reset cause as an ASCII-printable name string 
 *        from a reset cause type
 * 
 * @param[in] reset_cause      The previously-obtained system reset cause
 * @return const char const* const 
 *****************************************************************************/
static char const * const get_reset_cause_name( void )
{
	int i;
	for( i = 0; reset_name_tbl[ i ].name != NULL && reset_name_tbl[ i ].name[0] != '\0'; i++ )
	{
		if( reset_name_tbl[ i ].code == reset_cause )
		{
			return reset_name_tbl[ i ].name;
		}
	}

	return "RESET CAUSE NOT MATCHED";
}

/******************************************************************************
 * @brief Get the reset count
 * 
 * @return int 
 *****************************************************************************/
static int get_reset_count( void )
{
	if( !reset_count )
	{
		EEPR.Read( 0x79C, &reset_count, 4 );
	}
	return reset_count;
}

/******************************************************************************
 * @brief Get the last exception
 * 
 * @return int 
 *****************************************************************************/
static int get_last_exception( void )
{
	if( !last_exception )
	{
		EEPR.Read( 0x798, &last_exception, 4 );
	}
	return last_exception;
}

/******************************************************************************
 * @brief Get the last exception name
 * 
 * @return char const* const 
 *****************************************************************************/
static char const * const get_last_exception_name( void )
{
	get_last_exception();

		int i;
	for( i = 0; exception_name_tbl[ i ].name != NULL && exception_name_tbl[ i ].name[0] != '\0'; i++ )
	{
		if( exception_name_tbl[ i ].code == last_exception )
		{
			return exception_name_tbl[ i ].name;
		}
	}

	return "NO EXCEPTION";
}
