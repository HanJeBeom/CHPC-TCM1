/******************************************************************************
 * @file m_ringbuf.h
 * @author jylee1 (jylee1@gst-in.com)
 * @brief 
 * @version 0.1
 * @date 2019-11-12
 * 
 * @copyright Copyright (c) 2019 Global Standard Technology Co., Ltd.
 * 
 * All rights reserved.
 * This file is part of closed source software.
 * Unauthorized reproduction and redistribution prohibited.
 *****************************************************************************/

#ifndef _L_QUEUE_H_
#define _L_QUEUE_H_

#ifdef __cplusplus
extern "C" {
#endif

/*****************************************************************************/
/** INCLUDES *****************************************************************/
/*****************************************************************************/

#include <stdint.h>

/*****************************************************************************/
/** MACRO DEFINITIONS ********************************************************/
/*****************************************************************************/

/*****************************************************************************/
/** DATATYPES ****************************************************************/
/*****************************************************************************/

typedef struct
{
	uint8_t * data;
	uint16_t head;
	uint16_t tail;
	uint16_t size;
	struct {
		uint16_t init : 1;
		uint16_t lock : 1;
	} stat;
}GstRingBufHandle_t;

typedef struct {
	uint8_t (*Init)( GstRingBufHandle_t * pR, uint8_t * Buf, uint16_t queueSize );
	uint8_t (*IsFull)( GstRingBufHandle_t * pR );
	uint8_t (*IsEmpty)( GstRingBufHandle_t * pR );
	uint32_t (*Length)( GstRingBufHandle_t * pR );
	uint32_t (*Get)( GstRingBufHandle_t * pR, uint8_t * data, uint32_t count );
	uint32_t (*Peek)( GstRingBufHandle_t * pR, uint8_t * data, uint32_t count );
	uint32_t (*Put)( GstRingBufHandle_t * pR, const uint8_t * const src, uint32_t count );
	void (*Purge)( GstRingBufHandle_t * pR );
}GstRingBuf_t;

/*****************************************************************************/
/** FUNCTION DECLARATIONS ****************************************************/
/*****************************************************************************/

/*****************************************************************************/
/** GLOBAL VARIABLES *********************************************************/
/*****************************************************************************/

extern GstRingBuf_t const Ring;

/*****************************************************************************/
/** LOCAL VARIABLES **********************************************************/
/*****************************************************************************/

/*****************************************************************************/
/** FUNCTION DEFINITIONS *****************************************************/
/*****************************************************************************/


#ifdef __cplusplus
}
#endif

#endif /* _L_QUEUE_H_ */
