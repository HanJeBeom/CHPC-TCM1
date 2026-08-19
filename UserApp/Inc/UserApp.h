/******************************************************************************
 * @file UserApp.h
 * @author jylee1 (jylee1@gst-in.com)
 * @brief 
 * @version 0.1
 * @date 2023-06-22
 * 
 * @copyright Copyright (c) 2023 Global Standard Technology Co., Ltd.
 * 
 * All rights reserved.
 * This file is part of closed source software.
 * Unauthorized reproduction and redistribution prohibited.
 *****************************************************************************/

#ifndef INC_USERAPP_H_
#define INC_USERAPP_H_

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

#include <stm32f4xx.h>
#include <stm32f4xx_hal.h>

#include <UserDrivers.h>

#include <a_appl.h>
#include <a_calib.h>
#include <a_can.h>
#include <a_control.h>
#include <a_fltchk.h>
#include <a_logic.h>
#include <a_mbmaster.h>
#include <a_mbslave.h>
#include <a_output.h>
#include <a_temp.h>
#include <a_terminal.h>
#include <a_test_func.h>
#include <a_test_smps.h>
#include <a_usbcdc.h>

/*****************************************************************************/
/** MACRO DEFINITIONS ********************************************************/
/*****************************************************************************/

/* Production test feature switch (see docs/harness/test_func.md).
 * Defined : use the legacy interactive VT100 terminal (TerminalTask).
 * Undefined (default) : use the machine-readable test command interface
 *                       (TestFuncTask, a_test_func.c).
 */
// #define USE_LEGACY_TERMINAL

#define MINIMUM_PV -273150      // -273.150ºC a.k.a. 0K (Zero Kelvin)
#define MAXIMUM_PV 2000000      // 2000.000ºC

#define MULTIPLY_FLOAT_TO_PERMYRIAD 10000.0f
#define MULTIPLY_PERMYRIAD_TO_FLOAT 0.0001f
#define MULTIPLY_FLOAT_TO_PERMIL 1000.0f
#define MULTIPLY_PERMIL_TO_FLOAT 0.001f
#define MULTIPLY_FLOAT_TO_PERCENT 100.0f
#define MULTIPLY_PERCENT_TO_FLOAT 0.01f

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

/*****************************************************************************/
/** GLOBAL VARIABLES *********************************************************/
/*****************************************************************************/

/*****************************************************************************/
/** FUNCTION DEFINITIONS *****************************************************/
/*****************************************************************************/



#ifdef __cplusplus
}
#endif

#endif /* INC_USERAPP_H_ */
