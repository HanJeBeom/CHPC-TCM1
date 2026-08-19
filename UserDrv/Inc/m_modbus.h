/******************************************************************************
 * @file m_modbus.h
 * @author jylee1 (jylee1@gst-in.com)
 * @brief Modbus module
 * @version 0.1
 * @date 2023-06-22
 * 
 * @copyright Copyright (c) 2023 Global Standard Technology Co., Ltd.
 * 
 * All rights reserved.
 * This file is part of closed source software.
 * Unauthorized reproduction and redistribution prohibited.
 *****************************************************************************/

#ifndef _M_MODBUS_H_
#define _M_MODBUS_H_

#ifdef __cplusplus
extern "C" {
#endif

/*****************************************************************************/
/** INCLUDES *****************************************************************/
/*****************************************************************************/

#include <m_datablock.h>
#include <stdint.h>

/*****************************************************************************/
/** MACRO DEFINITIONS ********************************************************/
/*****************************************************************************/

#ifndef NULL
#define NULL            ((void *) 0)
#endif

#define RTU_FRAME_IDLE_CHAR 3.5f
#define MAX_BITS_PER_CHAR 11.0f		// START 1 + DATA 8 + PARITY 1 + STOP 1
#define SAFETY_FACTOR 1.25f

/* The implementation of RTU reception driver may imply the management of
   a lot of interruptions due to the t1.5 and t3.5 timers. With high
   communication baud rates, this leads to a heavy CPU load. Consequently
   these two timers must be strictly respected when the baud rate is equal
   or lower than 19200 Bps. For baud rates greater than 19200 Bps, fixed
   values for the 2 timers should be used: it is recommended to use a value
   of 750μs for the inter-character time-out (t1.5) and a value of 1.750ms
   for inter-frame delay (t3.5).
*/
#define RTU_FRAME_IDLE_MIN ( 0.00175f * SAFETY_FACTOR )

/*****************************************************************************/
/** DATATYPES ****************************************************************/
/*****************************************************************************/

typedef enum Modbus_ExCode_enum
{
	Modbus_ExCode_Normal                = 0U,
	Modbus_ExCode_Illegal_FunCode       = 1U,           /* Not supported Function Code */
	Modbus_ExCode_Illegal_Address       = 2U,
	Modbus_ExCode_Illegal_DataValue     = 3U,
	Modbus_ExCode_Slave_Failure         = 4U,
	Modbus_ExCode_Acknowledge           = 5U,
	Modbus_ExCode_Slave_Busy            = 6U,
	Modbus_ExCode_Negative_Ack          = 7U,
	Modbus_ExCode_Mem_Parity_Error      = 8U,           /* Memory Parity Error */
	Modbus_ExCode_GTW_Path_Unavailable  = 10U,          /* Gateway Path Unavailable */
	Modbus_ExCode_GTW_TDEV_Failure      = 11U,          /* Gateway Target Device Failed to respond */
}Modbus_ExCode_et;

typedef enum Modbus_FunCode_enum
{
	Modbus_FunCode_Read_Coil_Status       = 0x01U,    /* 0X reference(0XXXX) */
	Modbus_FunCode_Read_Input_Status      = 0x02U,    /* 1X reference(1XXXX) */
	Modbus_FunCode_Read_Holding_Reg       = 0x03U,    /* 4X reference(4XXXX) */
	Modbus_FunCode_Read_Input_Reg         = 0x04U,    /* 3X reference(3XXXX) */
	Modbus_FunCode_Force_Single_Coil      = 0x05U,    /* 0X reference(0XXXX) */
	Modbus_FunCode_Force_Holding_Reg      = 0x06U,
#if 0
	Modbus_FunCode_Read_Exception_Status  = 0x07U,    /*                     */
	Modbus_FunCode_Diagnostic             = 0x08U,    /*                     */
	Modbus_FunCode_Get_Com_Event_Counter  = 0x0BU,
	Modbus_FunCode_Get_Com_Event_Log      = 0x0CU,
#endif
	Modbus_FunCode_Force_Multiple_Coils   = 0x0FU,    /* 0X reference(0XXXX) */
	Modbus_FunCode_Preset_Multiple_Reg    = 0x10U,    /* 4X reference(4XXXX) */
#if 0
	Modbus_FunCode_Report_Slave_ID        = 0x11U,
	Modbus_FunCode_Read_File_Record       = 0x14U,    /*                     */
	Modbus_FunCode_Write_File_Record      = 0x15U,    /*                     */
	Modbus_FunCode_Mask_Write_Register    = 0x16U,    /*                     */
#endif
	Modbus_FunCode_Read_Write_4X_Reg      = 0x17U,    /* 4X reference(4XXXX) */
#if 0
	Modbus_FunCode_Read_FIFO_Queue        = 0x18U,    /* 4X reference(4XXXX) */
	Modbus_FunCode_Read_Device_Info       = 0x2BU,
#endif

	Modbus_FunCode_Max
}Modbus_FunCode_et;

typedef struct Modbus_MBAP_Header_struct
{
	uint16_t Transaction_ID;					/* For Modbus_TCP/IP */
	uint16_t Protocol_ID;						/* For Modbus_TCP/IP */
	uint16_t Length;							/* For Modbus_TCP/IP */
	uint8_t Unit_ID;							/* Slave Address     */
}Modbus_MBAP_Header_st;

typedef struct Mdobus_Rx_Struct
{
	Modbus_MBAP_Header_st MBAP_Header;
	int8_t start_code;							/* For Modbus-ASCII */
	uint8_t function_code;
	uint16_t starting_address_read;
	uint16_t no_of_registers_read;
	uint16_t starting_address_write;
	uint16_t no_of_registers_write;
	uint8_t exception_status;
	uint8_t byte_count;
	uint16_t data_write[ 128 ];
	uint16_t checksum;							/*	LRC or CRC	*/
	uint16_t end_code;							/*	For Modbuf-ASCII	*/
}Modbus_Rx_st;

typedef struct Modbus_Server_struct
{
	uint8_t *Id;
	Modbus_ExCode_et (*RxFrameCheck)( int8_t frame_format, uint8_t const *rx_buf, uint16_t rx_cnt, DataBlock_st const *db,
			Modbus_Rx_st *modbus_rx );
	uint16_t (*Function[ Modbus_FunCode_Max ])( uint8_t protocol, Modbus_Rx_st *modbus_rx, DataBlock_st const *db,
		uint8_t *tx_buf );
	uint16_t (*Run)( uint8_t * rcvd_frame, uint16_t length, uint8_t *tx_buf );
	void (*UpdateFromHoldings)( void );
	void (*UpdateInputs)( void );
} Modbus_Server_st;

typedef struct Modbus_Master_struct
{
	Modbus_ExCode_et (*RxFrameCheck)( int8_t frame_format, uint8_t *rx_frame, uint32_t rcvd_bytes,
			volatile Modbus_Rx_st *parsing_result );
	uint32_t (*Query)( int8_t Frame_Format, uint8_t Slave_Address, uint8_t Function_Code, uint16_t Start_Address,
			uint16_t No_Of_Items, uint16_t aStart_Address, uint16_t aNo_Of_Items, DataBlock_st const *db, uint8_t *Query_Frame );
	void (*RxFrameParser[ Modbus_FunCode_Max ])( uint8_t protocol, Modbus_Rx_st *modbus_rx, const DataBlock_st *db );
} Modbus_Master_st;

typedef struct Modbus_struct
{
	Modbus_Server_st Slave;
	Modbus_Master_st Master;
} Modbus_st;

/*****************************************************************************/
/** FUNCTION DECLARATIONS ****************************************************/
/*****************************************************************************/

Modbus_ExCode_et MbSlaveCheckRequestFrame( int8_t frame_format, uint8_t const *rx_buf, uint16_t rx_cnt, DataBlock_st const *db,
		Modbus_Rx_st *modbus_rx );
uint16_t MbSlaveBuildResponseFrame( uint8_t frame_format, uint16_t txn_id, uint16_t protocol_id, uint8_t slave_address,
		uint8_t function_code, uint16_t starting_address, uint16_t no_of_response_bytes, uint16_t *response_data,
		uint8_t *response_frame );
uint32_t ModbusMasterQueryBuild( int8_t Frame_Format, uint8_t Slave_Address, uint8_t Function_Code, uint16_t Start_Address,
		uint16_t No_Of_Items, uint16_t aStart_Address, uint16_t aNo_Of_Items, DataBlock_st const *db, uint8_t *Query_Frame );
Modbus_ExCode_et ModbusMasterResponseFrameParse( int8_t frame_format, uint8_t *rx_frame, uint32_t rcvd_bytes,
		volatile Modbus_Rx_st *parsing_result );
uint16_t modbus_slave_resp_read_coil_status( uint8_t protocol, Modbus_Rx_st *modbus_rx, DataBlock_st const *db,
		uint8_t *tx_buf );
uint16_t modbus_slave_resp_read_input_status( uint8_t protocol, Modbus_Rx_st *modbus_rx, DataBlock_st const *db,
		uint8_t *tx_buf );
uint16_t modbus_slave_resp_read_holding_reg( uint8_t protocol, Modbus_Rx_st *modbus_rx, DataBlock_st const *db,
		uint8_t *tx_buf );
uint16_t modbus_slave_resp_read_input_reg( uint8_t protocol, Modbus_Rx_st *modbus_rx, DataBlock_st const *db,
		uint8_t *tx_buf );
uint16_t modbus_slave_resp_force_single_coil( uint8_t protocol, Modbus_Rx_st *modbus_rx, DataBlock_st const *db,
		uint8_t *tx_buf );
uint16_t modbus_slave_resp_preset_holding_reg( uint8_t protocol, Modbus_Rx_st *modbus_rx, DataBlock_st const *db,
		uint8_t *tx_buf );
uint16_t modbus_slave_resp_force_multiple_coils( uint8_t protocol, Modbus_Rx_st *modbus_rx, DataBlock_st const *db,
		uint8_t *tx_buf );
uint16_t modbus_slave_resp_preset_multiple_reg( uint8_t protocol, Modbus_Rx_st *modbus_rx, DataBlock_st const *db,
		uint8_t *tx_buf );
uint16_t modbus_slave_resp_read_write_4x_reg( uint8_t protocol, Modbus_Rx_st *modbus_rx, DataBlock_st const *db,
		uint8_t *tx_buf );
uint16_t modbus_exception_response( uint8_t protocol, Modbus_Rx_st * modbus_rx, uint8_t * tx_buf, Modbus_ExCode_et ex_code );


/*****************************************************************************/
/** GLOBAL VARIABLES *********************************************************/
/*****************************************************************************/

/*****************************************************************************/
/** LOCAL VARIABLES **********************************************************/
/*****************************************************************************/

/*****************************************************************************/
/** FUNCTION DEFINITIONS *****************************************************/
/*****************************************************************************/


#ifdef __cplusplus
}
#endif

#endif /* _M_MODBUS_H_ */
