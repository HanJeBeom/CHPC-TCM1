/******************************************************************************
 * @file d_can.c
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

/*****************************************************************************/
/** INCLUDES *****************************************************************/
/*****************************************************************************/

#include "UserDrivers.h"

/*****************************************************************************/
/** MACRO DEFINITIONS ********************************************************/
/*****************************************************************************/

#define MAX_CAN_TX_MSG 49
#define MAX_CAN_RX_MSG 0x1FF

//#define FEATURE_CAN_RX_WITH_DYNAMIC_LIST

/*****************************************************************************/
/** DATATYPES ****************************************************************/
/*****************************************************************************/

#ifdef FEATURE_CAN_RX_WITH_DYNAMIC_LIST
typedef struct CAN_Message_list_Tag {
	uint32_t id;
	uint8_t dlc;
	uint8_t data[8];
	uint32_t timestamp;
	struct CAN_Message_list_Tag *next;
	struct CAN_Message_list_Tag *prev;
} can_message_list_st;
#endif

/*****************************************************************************/
/** FUNCTION DECLARATIONS ****************************************************/
/*****************************************************************************/

static void can_set_filter( uint32_t id, uint32_t mask );
static bool can_start( void );
static __RAM_FUNC int16_t find_can_tx_block( uint32_t id );
static __RAM_FUNC void can_set_tx_msg( uint32_t id, bool remote, uint8_t dlc, uint8_t data[ 8 ] );
static __RAM_FUNC bool can_send_msg( uint32_t id, bool remote, uint8_t dlc, uint8_t data[ 8 ] );
static __RAM_FUNC int32_t can_get_msg( uint32_t id, uint8_t *data );
static __RAM_FUNC void can_rxbuff_to_rxmsg( uint32_t id, uint8_t dlc, uint8_t *data, uint32_t ts );

/*****************************************************************************/
/** EXTERNAL VARIABLES *******************************************************/
/*****************************************************************************/

extern CAN_HandleTypeDef hcan1;

/*****************************************************************************/
/** LOCAL VARIABLES **********************************************************/
/*****************************************************************************/

static uint8_t can_started = 0;
static CAN_RxHeaderTypeDef canRxHdr = { 0 };
static int filterBank = 0;
static uint8_t canRxData[ MAX_CAN_MSG_LEN ] = { 0 };
static CCMRAM struct can_tx_message_struct_Tag {
	uint32_t StdId : 24;
	uint16_t DLC : 4;
	uint8_t RTR : 1;
	uint8_t Enabled : 1;
	uint8_t :2;
	uint8_t Data[ MAX_CAN_MSG_LEN ];
} canTxMsg[ MAX_CAN_TX_MSG ] = { 0 };

#ifdef FEATURE_CAN_RX_WITH_DYNAMIC_LIST
can_message_list_st *can_rx_list = NULL;
#else
static CCMRAM struct can_rx_message_struct_Tag {
	uint32_t StdId : 24;
	uint32_t DLC : 4;
	uint32_t Received : 1;
	uint32_t : 3;
	uint8_t Data[ MAX_CAN_MSG_LEN ];
} canRxMsg[ MAX_CAN_RX_MSG ] = { 0 };
#endif /* FEATURE_CAN_RX_WITH_DYNAMIC_LIST */
static uint32_t can_last_error;

/*****************************************************************************/
/** GLOBAL VARIABLES *********************************************************/
/*****************************************************************************/

const CAN_st CAN = {
	.inst = &hcan1,
	.SetFilter = can_set_filter,
	.Start = can_start,
	.SetTxMsg = can_set_tx_msg,
	.SendMessage = can_send_msg,
	.GetMessage = can_get_msg,
};

/*****************************************************************************/
/** FUNCTION DEFINITIONS *****************************************************/
/*****************************************************************************/

/******************************************************************************
 * @brief
 *
 * @param id
 * @param mask
 *****************************************************************************/
static void can_set_filter( uint32_t id, uint32_t mask )
{
	CAN_FilterTypeDef filterConfig = { 0 };

	if( 13 < filterBank ) return;		/*!< between 0 and 13. */

	filterConfig.FilterIdHigh = id & 0xFFFF ;
	filterConfig.FilterMaskIdHigh = mask & 0xFFFF;
//	filterConfig.FilterIdLow = 0x0ff;
//	filterConfig.FilterMaskIdLow = 0x700;

	filterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	filterConfig.FilterBank = filterBank;			/*!< between 0 and 13. */
	filterBank++;
	filterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	filterConfig.FilterScale = CAN_FILTERSCALE_16BIT;
	filterConfig.FilterActivation = CAN_FILTER_ENABLE;

	HAL_CAN_ConfigFilter( CAN.inst, &filterConfig );
	HAL_CAN_ActivateNotification( CAN.inst, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_BUSOFF );
}

/******************************************************************************
 * @brief
 *
 * @retval false => started successfully
 * @retval true => can't started
 *****************************************************************************/
static bool can_start( void )
{
	if( HAL_OK == HAL_CAN_Start( CAN.inst ) )
	{
		can_started = 1;
	}
	return can_started ? false : true;
}

/******************************************************************************
 * @brief 
 * 
 * @param id 
 * @return int16_t 
 * @retval -1 : Availabe message block is Not found
 * @retval other : Available block is Found
 *****************************************************************************/
static int16_t find_can_tx_block( uint32_t id )
{
	for(int i = 0; i < MAX_CAN_TX_MSG; ++i )
	{
		if( canTxMsg[ i ].StdId == id )
		{
			return i;
		}

		if( canTxMsg[ i ].Enabled == 0 )
		{
			return i;
		}
	}

	return -1;
}

/******************************************************************************
 * @brief 
 * 
 * @param id 
 * @param remote 
 * @param dlc 
 * @param data 
 *****************************************************************************/
static void can_set_tx_msg( uint32_t id, bool remote, uint8_t dlc, uint8_t data[ 8 ] )
{
	int16_t idx = find_can_tx_block( id );

	if( 0 <= idx && idx < MAX_CAN_TX_MSG )
	{
		uint8_t length = MIN( dlc , MAX_CAN_MSG_LEN );
		canTxMsg[ idx ].StdId = id;
		canTxMsg[ idx ].RTR = remote ? CAN_RTR_REMOTE : CAN_RTR_DATA;
		canTxMsg[ idx ].DLC = length;
		memcpy( canTxMsg[ idx ].Data, data, length );
		canTxMsg[ idx ].Enabled = 1;
	}
}

/******************************************************************************
 * @brief
 *
 * @param id
 * @param remote
 * @param dlc
 * @param data
 * 
 * @retval true
 * @retval false
 *****************************************************************************/
static bool can_send_msg( uint32_t id, bool remote, uint8_t dlc, uint8_t data[ 8 ] )
{
	bool ret_code = true;
	//if( can_started )
	{
		uint32_t TxMailBox = HAL_CAN_GetTxMailboxesFreeLevel( CAN.inst );
		CAN_TxHeaderTypeDef canTxHdr = { 0 };
		canTxHdr.StdId = id;
		canTxHdr.RTR = remote ? CAN_RTR_REMOTE : CAN_RTR_DATA;
		canTxHdr.IDE = CAN_ID_STD;
		canTxHdr.DLC = MIN( dlc , MAX_CAN_MSG_LEN );

		ret_code = HAL_CAN_AddTxMessage( CAN.inst, &canTxHdr, data, &TxMailBox );
	}

	return ret_code;
}

/******************************************************************************
 * @brief
 *
 * @param id
 * @param data
 * @return
 *****************************************************************************/
static int32_t can_get_msg( uint32_t id, uint8_t *data )
{
#ifdef FEATURE_CAN_RX_WITH_DYNAMIC_LIST
	for( can_message_list_st *get_pos = can_rx_list; get_pos != can_rx_list; get_pos = get_pos->next )
	{
		if( get_pos->id == id )
		{
			int32_t dlc = get_pos->dlc;
			memcpy( data, get_pos->data, dlc );

			if( get_pos->next == get_pos )
			{
				can_rx_list = NULL;
			}
			else
			{
				get_pos->prev->next = get_pos->next;
				get_pos->next->prev = get_pos->prev;
			}

			free( get_pos );

			return dlc;
		}
		else if( get_pos->id < id )
		{
			return -2;
		}
	}

	return -1;
#else
	if( id == canRxMsg[ id ].StdId )
	{
		if( canRxMsg[ id ].Received )
		{
			int dlc = canRxMsg[ id ].DLC;
			memcpy( data, canRxMsg[ id ].Data, dlc );
			//memset( &canRxMsg[ id ], 0, sizeof( canRxMsg[ 0 ] ) );
			canRxMsg[ id ].Received = 0;
			return dlc;
		}

		return -2;
	}

	return -1;
#endif /* FEATURE_CAN_RX_WITH_DYNAMIC_LIST */
}

#ifdef FEATURE_CAN_RX_WITH_DYNAMIC_LIST
/******************************************************************************
 * @brief
 *
 * @param pMsg
 *****************************************************************************/
void insert_to_list( can_message_list_st *pMsg )
{
	if( NULL == can_rx_list )
	{
		can_message_list_st *new_item = malloc( sizeof(can_message_list_st) );
		if( new_item )
		{
			memcpy( new_item, pMsg, sizeof(can_message_list_st) );

			can_rx_list = new_item;
			new_item->next = new_item;
			new_item->prev = new_item;
		}
	}
	else
	{
		for( can_message_list_st *insert_pos = can_rx_list; insert_pos != can_rx_list; insert_pos = insert_pos->next )
		{
			if( pMsg->id == insert_pos->id )
			{
				insert_pos->dlc = pMsg->dlc;
				memcpy( insert_pos->data, pMsg->data, pMsg->dlc );
			}
			else if( pMsg->id > insert_pos->id )
			{
				can_message_list_st *new_item = malloc( sizeof(can_message_list_st) );
				memcpy( new_item, pMsg, sizeof(can_message_list_st) );

				if( new_item )
				{
					insert_pos->prev->next = new_item;
					insert_pos->prev = new_item;
					new_item->prev = insert_pos->prev;
					new_item->next = insert_pos;
				}
			}
		}
	}
}

/******************************************************************************
 * @brief
 *
 * @param pMsg
 *****************************************************************************/
void remove_from_list( can_message_list_st *pMsg )
{
	if( pMsg->next == pMsg )
	{
		can_rx_list = NULL;
	}
	else
	{
		pMsg->prev->next = pMsg->next;
		pMsg->next->prev = pMsg->prev;
	}

	free( pMsg );
}
#endif /* FEATURE_CAN_RX_WITH_DYNAMIC_LIST */

/******************************************************************************
 * @brief
 *
 * @param id
 * @param dlc
 * @param data
 * @param ts
 *****************************************************************************/
static void can_rxbuff_to_rxmsg( uint32_t id, uint8_t dlc, uint8_t *data, uint32_t ts )
{
	canRxMsg[ id ].StdId = id;
	canRxMsg[ id ].DLC = dlc;
	memcpy( canRxMsg[ id ].Data, data, dlc );
	canRxMsg[ id ].Received = 1;
#ifdef FEATURE_CAN_RX_WITH_DYNAMIC_LIST
	can_message_list_st msg;

	msg.id = id;
	msg.dlc = dlc;
	msg.timestamp = ts;
	memcpy( msg.data, data, dlc );
	msg.next = NULL;
	msg.prev = NULL;

	insert_to_list( &msg );
#endif /* FEATURE_CAN_RX_WITH_DYNAMIC_LIST */
}

/******************************************************************************
 * @brief
 *
 * @param pRxHdr
 *****************************************************************************/
static void can_rtr_response( CAN_RxHeaderTypeDef *pRxHdr )
{
	for( int idx = 0; idx < MAX_CAN_TX_MSG; ++idx )
	{
		if( canTxMsg[ idx ].StdId == pRxHdr->StdId )
		{
			CAN.SendMessage( canTxMsg[ idx ].StdId, 0, canTxMsg[ idx ].DLC, canTxMsg[ idx ].Data );
		}
	}
}

/******************************************************************************
 * @brief
 *
 * @param hcan
 *****************************************************************************/
void HAL_CAN_RxFifo0MsgPendingCallback( CAN_HandleTypeDef *hcan )
{
	if( can_started )
	{
		if( HAL_OK == HAL_CAN_GetRxMessage( CAN.inst, CAN_RX_FIFO0, &canRxHdr, canRxData ) )
		{
			if( CAN_ID_STD == canRxHdr.IDE )
			{
				if( CAN_RTR_REMOTE == canRxHdr.RTR )
				{
					can_rtr_response( &canRxHdr );
				}
				else
				{
					can_rxbuff_to_rxmsg( canRxHdr.StdId, canRxHdr.DLC, canRxData, canRxHdr.Timestamp );
				}
			}
		}
	}
}

/******************************************************************************
 * @brief
 *
 * @param hcan
 *****************************************************************************/
void HAL_CAN_ErrorCallback( CAN_HandleTypeDef *hcan )
{
	// it occurred can bus-off
	can_last_error = HAL_CAN_GetError( hcan );
	if( HAL_CAN_ERROR_BOF & can_last_error )
	{
		HAL_CAN_ResetError( hcan );
	}
}
