/******************************************************************************
 * @file m_ringbuf.c
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

/*****************************************************************************/
/** INCLUDES *****************************************************************/
/*****************************************************************************/
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include "m_ringbuf.h"

#include "common.h"

/*****************************************************************************/
/** MACRO DEFINITIONS ********************************************************/
/*****************************************************************************/

/*****************************************************************************/
/** DATATYPES ****************************************************************/
/*****************************************************************************/

/*****************************************************************************/
/** FUNCTION DECLARATIONS ****************************************************/
/*****************************************************************************/

static uint8_t init_q( GstRingBufHandle_t * pQ, uint8_t * Buf, uint16_t queueSize );
static uint8_t is_full( GstRingBufHandle_t * pQ );
static uint8_t is_empty( GstRingBufHandle_t * pQ );
static uint8_t lock( GstRingBufHandle_t * pQ );
static uint8_t unlock( GstRingBufHandle_t * pQ );
static uint32_t length( GstRingBufHandle_t * pQ );
static uint32_t get( GstRingBufHandle_t * pQ, uint8_t * data, uint32_t count );
static uint32_t peek( GstRingBufHandle_t * pQ, uint8_t * data, uint32_t count );
static uint32_t put( GstRingBufHandle_t * pQ, const uint8_t * const src, uint32_t count );
static void purge( GstRingBufHandle_t * pQ );

/*****************************************************************************/
/** GLOBAL VARIABLES *********************************************************/
/*****************************************************************************/

GstRingBuf_t const Ring = { .Init = init_q, .IsFull = is_full, .IsEmpty = is_empty, .Length = length, .Get = get, .Peek = peek, .Put = put, .Purge = purge };

/*****************************************************************************/
/** LOCAL VARIABLES **********************************************************/
/*****************************************************************************/

/*****************************************************************************/
/** FUNCTION DEFINITIONS *****************************************************/
/*****************************************************************************/

/******************************************************************************
 * @brief 
 * 
 * @param pQ 
 * @param Buf 
 * @param queueSize 
 * @return uint8_t 
 *****************************************************************************/
static uint8_t init_q( GstRingBufHandle_t * pQ, uint8_t * Buf, uint16_t queueSize )
{
	if( pQ && ( pQ->stat.init != 1 ) )
	{
		pQ->head = 0;
		pQ->tail = 0;
		if( Buf && queueSize )
		{
			pQ->size = queueSize;
			pQ->data = Buf;
			memset( pQ->data, 0, pQ->size );
			pQ->stat.lock = 0;
			pQ->stat.init = 1;
			return 1;
		}
		else
		{
			pQ->size = 0;
			pQ->data = NULL;
			return 0;
		}
	}

	return 0;
}

/******************************************************************************
 * @brief 
 * 
 * @param pQ 
 * @return uint8_t 
 *****************************************************************************/
static uint8_t is_full( GstRingBufHandle_t * pQ )
{
	if( !pQ || ( pQ->stat.init != 1 ) ) return 1;

	if( pQ->tail == ( ( pQ->head + 1 ) % pQ->size ) ) return 1;
	else return 0;
}

/******************************************************************************
 * @brief 
 * 
 * @param pQ 
 * @return uint8_t 
 *****************************************************************************/
static uint8_t is_empty( GstRingBufHandle_t * pQ )
{
	if( !pQ || ( pQ->stat.init != 1 ) ) return 1;

	return ( pQ->head == pQ->tail );
}

/******************************************************************************
 * @brief 
 * 
 * @param pQ 
 * @return uint32_t 
 *****************************************************************************/
static uint32_t length( GstRingBufHandle_t * pQ )
{
	if( !pQ || ( pQ->stat.init != 1 ) ) return 0;

	if( pQ->head < pQ->tail ) return ( pQ->size + pQ->head ) - pQ->tail;
	else return pQ->head - pQ->tail;
}

/******************************************************************************
 * @brief 
 * 
 * @param pQ 
 * @return uint8_t 
 * @return uint8_t success(1)/fail(0) for lock
 *****************************************************************************/
static uint8_t lock( GstRingBufHandle_t * pQ )
{
	if( !pQ->stat.lock )
	{
		pQ->stat.lock = 1;
		return 1;
	}
	else
	{
		return 0;
	}
}

/******************************************************************************
 * @brief 
 * 
 * @param pQ 
 * @return uint8_t success(1)/fail(0) for unlock
 *****************************************************************************/
static uint8_t unlock( GstRingBufHandle_t * pQ )
{
	if( pQ->stat.lock )
	{
		pQ->stat.lock = 0;
		return 1;
	}
	else
	{
		return 0;
	}
}

/******************************************************************************
 * @brief 
 * 
 * @param[in] pQ 
 * @param[out] dst 
 * @param count 
 * @return uint32_t 
 *****************************************************************************/
static uint32_t get( GstRingBufHandle_t * pQ, uint8_t * dst, uint32_t count )
{
	uint32_t getCnt = 0;
	uint32_t loopCount = 0;

	if( !pQ || ( pQ->stat.init != 1 ) || !dst ) return 0;

	uint16_t qlen = length( pQ );
	loopCount = MIN( count, qlen );
	if( loopCount && pQ->data )
	{
#if 0
		for( getCnt = 0; getCnt < loopCount; ++getCnt )
		{
			if( is_empty( pQ ) ) break;

			*dst++ = pQ->data[ pQ->tail++ ];
			if( pQ->tail == pQ->size ) pQ->tail = 0;
		}
#else
		if( is_empty( pQ ) ) return 0;
		if( lock( pQ ) )
		{
			if( pQ->tail + loopCount < pQ->size )
			{
				memcpy( dst, &pQ->data[ pQ->tail ], loopCount );
				pQ->tail += loopCount;
			}
			else
			{
				memcpy( dst, &pQ->data[ pQ->tail ], pQ->size - pQ->tail );
				memcpy( &dst[ pQ->size - pQ->tail ], pQ->data, loopCount - ( pQ->size - pQ->tail ) );
				pQ->tail = loopCount - ( pQ->size - pQ->tail );
			}
			getCnt = loopCount;

			unlock( pQ );
		}
#endif
	}

	return getCnt;
}

/******************************************************************************
 * @brief 
 * 
 * @param[in] pQ 
 * @param[out] dst 
 * @param count 
 * @return uint32_t 
 *****************************************************************************/
static uint32_t peek( GstRingBufHandle_t * pQ, uint8_t * dst, uint32_t count )
{
	uint32_t peekCnt = 0;
	uint16_t loopCount = 0;

	if( !pQ || ( pQ->stat.init != 1 ) || !dst ) return 0;

	uint16_t qlen = length( pQ );
	loopCount = MIN( count, qlen );
	if( loopCount && pQ->data )
	{
#if 0
		uint16_t tail = pQ->tail;
		for( peekCnt = 0; peekCnt < loopCount; ++peekCnt )
		{
			if( is_empty( pQ ) ) break;

			*dst++ = pQ->data[ tail++ ];
			if( tail == pQ->size ) tail = 0;
		}
#else
		if( is_empty( pQ ) ) return 0;
		if( lock( pQ ) )
		{
			if( pQ->tail + loopCount <= pQ->size )
			{
				memcpy( dst, &pQ->data[ pQ->tail ], loopCount );
			}
			else
			{
				memcpy( dst, &pQ->data[ pQ->tail ], pQ->size - pQ->tail );
				memcpy( &dst[ pQ->size - pQ->tail ], pQ->data, loopCount - ( pQ->size - pQ->tail ) );
			}
			peekCnt = loopCount;

			unlock( pQ );
		}
#endif
	}

	return peekCnt;
}

/******************************************************************************
 * @brief 
 * 
 * @param[in] pQ 
 * @param[in] src 
 * @param count 
 * @return uint32_t 
 *****************************************************************************/
static uint32_t put( GstRingBufHandle_t * pQ, const uint8_t * const src, uint32_t count )
{
	uint32_t putCnt = 0;

	if( !pQ || ( pQ->stat.init != 1 ) || !src || !count || pQ->stat.lock ) return 0;

	if( pQ->data )
	{
#if 0
		for( putCnt = 0; putCnt < count; ++putCnt )
		{
			if( is_full( pQ ) )
			{
				pQ->tail++;
				if( pQ->tail == pQ->size ) pQ->tail = 0;
			}

			pQ->data[ pQ->head++ ] = *src++;
			if( pQ->head == pQ->size ) pQ->head = 0;
		}
#else
		if( lock( pQ ) )
		{
			count = MIN( count, pQ->size - 1 );

			if( pQ->head + count < pQ->size )
			{
				memcpy( &pQ->data[ pQ->head ], src, count );
				pQ->head += count;
				putCnt = count;
			}
			else
			{
				memcpy( &pQ->data[ pQ->head ], src, pQ->size - pQ->head );
				memcpy( pQ->data, &src[ pQ->size - pQ->head ], count - ( pQ->size - pQ->head ) );
				pQ->head = count - ( pQ->size - pQ->head );
				putCnt = count;
			}
			unlock( pQ );
		}
#endif
	}

	return putCnt;
}

/******************************************************************************
 * @brief set the head and tail to zero, and clear the data buffer
 * 
 * @param pQ 
 *****************************************************************************/
static void purge( GstRingBufHandle_t * pQ )
{
	if( !pQ || ( pQ->stat.init != 1 ) ) return;

	if( pQ->data )
	{
		pQ->head = 0;
		pQ->tail = 0;
		memset( pQ->data, 0, pQ->size );
		pQ->stat.lock = 0;
	}
}
