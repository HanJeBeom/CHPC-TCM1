/******************************************************************************
 * @file version.h
 * @author jylee1 (jylee1@gst-in.com)
 * @brief 
 * @version 0.1
 * @date 2020-07-23
 * 
 * @copyright Copyright (c) 2023 Global Standard Technology Co., Ltd.
 * 
 * All rights reserved.
 * This file is part of closed source software.
 * Unauthorized reproduction and redistribution prohibited.
 *****************************************************************************/

#ifndef _VERSION_H_
#define _VERSION_H_

#ifdef __cplusplus
extern "C" {
#endif

/*****************************************************************************/
/** INCLUDES *****************************************************************/
/*****************************************************************************/

/*****************************************************************************/
/** MACRO DEFINITIONS ********************************************************/
/*****************************************************************************/
#define BUILD_Y_0 (__DATE__[ 7])
#define BUILD_Y_1 (__DATE__[ 8])
#define BUILD_Y_2 (__DATE__[ 9])
#define BUILD_Y_3 (__DATE__[10])

#define BUILD_Y \
	( \
		(BUILD_Y_2 == '1') ? BUILD_Y_3 : \
		(BUILD_Y_2 == '2') ? BUILD_Y_3 - '0' + 'A' : \
		(BUILD_Y_2 == '3') ? BUILD_Y_3 - '0' + 'K' : \
		' ' \
	)


#define BUILD_MONTH_IS_JAN (__DATE__[1] == 'a' && __DATE__[2] == 'n')
#define BUILD_MONTH_IS_FEB (__DATE__[0] == 'F')
#define BUILD_MONTH_IS_MAR (__DATE__[1] == 'a' && __DATE__[2] == 'r')
#define BUILD_MONTH_IS_APR (__DATE__[0] == 'A' && __DATE__[1] == 'p')
#define BUILD_MONTH_IS_MAY (__DATE__[1] == 'a' && __DATE__[2] == 'y')
#define BUILD_MONTH_IS_JUN (__DATE__[1] == 'u' && __DATE__[2] == 'n')
#define BUILD_MONTH_IS_JUL (__DATE__[1] == 'u' && __DATE__[2] == 'l')
#define BUILD_MONTH_IS_AUG (__DATE__[0] == 'A' && __DATE__[1] == 'u')
#define BUILD_MONTH_IS_SEP (__DATE__[0] == 'S')
#define BUILD_MONTH_IS_OCT (__DATE__[0] == 'O')
#define BUILD_MONTH_IS_NOV (__DATE__[0] == 'N')
#define BUILD_MONTH_IS_DEC (__DATE__[0] == 'D')


#define BUILD_M_0 \
	((BUILD_MONTH_IS_OCT || BUILD_MONTH_IS_NOV || BUILD_MONTH_IS_DEC) ? '1' : '0')

#define BUILD_M_1 \
	( \
		(BUILD_MONTH_IS_JAN) ? '1' : \
		(BUILD_MONTH_IS_FEB) ? '2' : \
		(BUILD_MONTH_IS_MAR) ? '3' : \
		(BUILD_MONTH_IS_APR) ? '4' : \
		(BUILD_MONTH_IS_MAY) ? '5' : \
		(BUILD_MONTH_IS_JUN) ? '6' : \
		(BUILD_MONTH_IS_JUL) ? '7' : \
		(BUILD_MONTH_IS_AUG) ? '8' : \
		(BUILD_MONTH_IS_SEP) ? '9' : \
		(BUILD_MONTH_IS_OCT) ? '0' : \
		(BUILD_MONTH_IS_NOV) ? '1' : \
		(BUILD_MONTH_IS_DEC) ? '2' : \
		/* error default */	'?' \
	)

#define BUILD_M \
	( \
		(BUILD_MONTH_IS_JAN) ? '1' : \
		(BUILD_MONTH_IS_FEB) ? '2' : \
		(BUILD_MONTH_IS_MAR) ? '3' : \
		(BUILD_MONTH_IS_APR) ? '4' : \
		(BUILD_MONTH_IS_MAY) ? '5' : \
		(BUILD_MONTH_IS_JUN) ? '6' : \
		(BUILD_MONTH_IS_JUL) ? '7' : \
		(BUILD_MONTH_IS_AUG) ? '8' : \
		(BUILD_MONTH_IS_SEP) ? '9' : \
		(BUILD_MONTH_IS_OCT) ? 'A' : \
		(BUILD_MONTH_IS_NOV) ? 'B' : \
		(BUILD_MONTH_IS_DEC) ? 'C' : \
		/* error default */	' ' \
	)

#define BUILD_D_0 ((__DATE__[4] >= '0') ? (__DATE__[4]) : '0')
#define BUILD_D_1 (__DATE__[ 5])

#define BUILD_D \
	( \
		(BUILD_D_0 == '0') ? BUILD_D_1 : \
		(BUILD_D_0 == '1') ? BUILD_D_1 - '0' + 'A' : \
		(BUILD_D_0 == '2') ? BUILD_D_1 - '0' + 'K' : \
		(BUILD_D_0 == '3') ? BUILD_D_1 - '0' + 'U' : \
		' ' \
	)


#define BUILD_HO_0 (__TIME__[0])
#define BUILD_HO_1 (__TIME__[1])

#define BUILD_MI_0 (__TIME__[3])
#define BUILD_MI_1 (__TIME__[4])

#define BUILD_SC_0 (__TIME__[6])
#define BUILD_SC_1 (__TIME__[7])


#define BUILD_Y_BCD ( ( BUILD_Y_2 - '0' ) << 4 | ( BUILD_Y_3 - '0' ) )
#define BUILD_M_BCD ( ( BUILD_M_0 - '0' ) << 4 | ( BUILD_M_1 - '0' ) )
#define BUILD_D_BCD ( ( BUILD_D_0 - '0' ) << 4 | ( BUILD_D_1 - '0' ) )

#define BUILD_VERSION_BCD ( ( BUILD_Y_BCD << 16) | ( BUILD_M_BCD << 8 ) | BUILD_D_BCD )

#define BUILD_Y_NUM ( ( BUILD_Y_2 - '0' ) * 10 + ( BUILD_Y_3 - '0' ) )
#define BUILD_M_NUM ( ( BUILD_M_0 - '0' ) * 10 + ( BUILD_M_1 - '0' ) )
#define BUILD_D_NUM ( ( BUILD_D_0 - '0' ) * 10 + ( BUILD_D_1 - '0' ) )

/*****************************************************************************/
/** DATATYPES ****************************************************************/
/*****************************************************************************/

/*****************************************************************************/
/** FUNCTION DECLARATIONS ****************************************************/
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

#endif /* _VERSION_H_ */
