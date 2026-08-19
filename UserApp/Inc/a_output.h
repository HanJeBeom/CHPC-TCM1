/******************************************************************************
 * @file a_output.h
 * @author Seo Yujeong (yjseo@gst-in.com)
 * @brief 
 * @version 0.1
 * @date 2024-04-03
 * 
 * @copyright Copyright (c) 2024 Global Standard Technology Co., Ltd.
 * 
 * All rights reserved.
 * This file is part of closed source software.
 * Unauthorized reproduction and redistribution prohibited.
 *****************************************************************************/

#ifndef INC_A_OUTPUT_H_
#define INC_A_OUTPUT_H_
/*****************************************************************************/
/** INCLUDES *****************************************************************/
/*****************************************************************************/

/*****************************************************************************/
/** MACRO DEFINITIONS ********************************************************/
/*****************************************************************************/

/*****************************************************************************/
/** DATATYPES ****************************************************************/
/*****************************************************************************/

typedef enum smps_func_enum
{
	SMPS_CMD_NONE = 0,
	SMPS_CMD_RUN,
	SMPS_CMD_STOP,
	SMPS_CMD_RESET,
	SMPS_CMD_READ,
	SMPS_CMD_WRITE,
} smps_func_et;

typedef struct smps_cmd_struct
{
	uint8_t smps_type;
	uint8_t smps_id;
	smps_func_et smps_func;
	uint8_t mbm_fcode;
	uint16_t mbm_addr;
	uint8_t mbm_qty;
	const DataBlock_st *DBlock;
	uint8_t mbm_stage;
	AppTimerData_ut timerModbusFrame;
	AppTimerData_ut timerTxWait;
	AppTimerData_ut timerRecvTimeout;
	AppTimerData_ut timerCommTransactionTimeout;
	uint16_t rcvd_length;
	uint16_t old_rcvd_length;
	Modbus_Rx_st modbus_rx;

} smps_cmd_st;

/*****************************************************************************/
/** GLOBAL VARIABLES *********************************************************/
/*****************************************************************************/

extern smps_cmd_st SMPSCMD[ MAX_SMPS_CMD ];

/*****************************************************************************/
/** LOCAL VARIABLES **********************************************************/
/*****************************************************************************/

/*****************************************************************************/
/** FUNCTION DEFINITIONS *****************************************************/
/*****************************************************************************/
void OutputTask( void );
void Reset_smps_comm( output_et out_type, uint8_t smps_id );
uint8_t OutputRequestSmpsRead( output_et out_type, uint8_t smps_id );
#endif /* INC_A_OUTPUT_H_ */
