/******************************************************************************
 * @file common.h
 * @author jylee1 (jylee1@gst-in.com)
 * @brief 
 * @version 0.1
 * @date 2023-05-10
 * 
 * @copyright Copyright (c) 2023 Global Standard Technology Co., Ltd.
 * 
 * All rights reserved.
 * This file is part of closed source software.
 * Unauthorized reproduction and redistribution prohibited.
 *****************************************************************************/

#ifndef INC_COMMON_H_
#define INC_COMMON_H_

#ifdef __cplusplus
extern "C" {
#endif

/*****************************************************************************/
/** INCLUDES *****************************************************************/
/*****************************************************************************/

#include "version.h"

/*****************************************************************************/
/** MACRO DEFINITIONS ********************************************************/
/*****************************************************************************/

#define MAX( a, b ) (((a) > (b)) ? (a) : (b))
#define MIN( a, b ) (((a) < (b)) ? (a) : (b))

#define TICK_UP( i, max ) do { i++; if( i >= max ){ i = 0; } } while( 0 )
#define TICK_DN( i, max ) do { if( i == 0 ){ i = max; } i--; } while( 0 )

#define TCM1_PLM_CODE 0x33100059
#define TCM1_PLM_REV  0x0000


/* CCM RAM (Core Coupled Memory)
   ------------
  64-Kbyte CCM data RAM (0x10000000 - 0x1000ffff)

  CCM RAM is connected to the CPU only through the D-Bus. DMA is not connected to CCM RAM.
  Therefore, it is not possible to execute instructions from CCM RAM, which is not connected
  to the I-Bus, but the latency can be reduced because it is not interrupted by DMA and other
  interrupts.

   "__attribute__((section(".ccmram")))".
*/
#define CCMRAM __attribute__((section(".ccmram")))

/*****************************************************************************/
/** EXTERNAL VARIABLES *******************************************************/
/*****************************************************************************/

/*****************************************************************************/
/** LOCAL VARIABLES **********************************************************/
/*****************************************************************************/

/*****************************************************************************/
/** GLOBAL VARIABLES *********************************************************/
/*****************************************************************************/


#ifdef __cplusplus
}
#endif

#endif /* INC_COMMON_H_ */
