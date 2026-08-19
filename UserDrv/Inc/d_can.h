/******************************************************************************
 * @file d_can.h
 * @author jylee1 (jylee1@gst-in.com)
 * @brief 
 * @version 0.1
 * @date 2023-05-24
 * 
 * @copyright Copyright (c) 2023 Global Standard Technology Co., Ltd.
 * 
 * All rights reserved.
 * This file is part of closed source software.
 * Unauthorized reproduction and redistribution prohibited.
 *****************************************************************************/

#ifndef INC_D_CAN_H_
#define INC_D_CAN_H_

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

/*****************************************************************************/
/** MACRO DEFINITIONS ********************************************************/
/*****************************************************************************/

#define MAX_CAN_MSG_LEN 8

/*****************************************************************************/
/** DATATYPES ****************************************************************/
/*****************************************************************************/

typedef struct CAN_struct_Tag {
	CAN_HandleTypeDef *inst;
	/**********************************************************************
	 * @brief Set filter for receiving message
	 * 
	 * @param id filter id corresponding each bit
	 * @param mask filter mask corresponding each bit // 0: mask bit not applied, 1: mask bit applied
	 *********************************************************************/
	void (*SetFilter)( uint32_t id, uint32_t mask );
	bool (*Start)( void );

	/**********************************************************************
	 * @brief Prepare CAN Tx Message for responding about remote frame 
	 * 
	 * @param id CAN ID
	 * @param remote remote frame
	 * @param dlc data length of content
	 * @param data buffer to send
	 *********************************************************************/
	void (*SetTxMsg)( uint32_t id, bool remote, uint8_t dlc, uint8_t data[ 8 ] );
	/**********************************************************************
	 * @brief Send CAN Message
	 * 
	 * @param id CAN ID
	 * @param remote remote frame
	 * @param dlc data length of content
	 * @param data buffer to send
	 *********************************************************************/
	bool (*SendMessage)( uint32_t id, bool remote, uint8_t dlc, uint8_t data[ 8 ] );
	int32_t (*GetMessage)( uint32_t id, uint8_t *data );
} CAN_st;

/*****************************************************************************/
/** FUNCTION DECLARATIONS ****************************************************/
/*****************************************************************************/

/*****************************************************************************/
/** EXTERNAL VARIABLES *******************************************************/
/*****************************************************************************/

extern const CAN_st CAN;

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

#endif /* INC_D_CAN_H_ */
