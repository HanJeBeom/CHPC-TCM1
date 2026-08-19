/******************************************************************************
 * @file m_modbus.c
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

/*****************************************************************************/
/** INCLUDES *****************************************************************/
/*****************************************************************************/

#include <stdlib.h>
#include <string.h>

#include <m_modbus.h>

/*****************************************************************************/
/** MACRO DEFINITIONS ********************************************************/
/*****************************************************************************/

#define Modbus_ASCII_Start_Code ':'
#define Modbus_ASCII_End_Code 0x0D0A

#define MODBUS_TCP_MINIMUM_BYTES 7
#define MODBUS_TCP_MAXIMUM_BYTES 260

#define MODBUS_ASCII_MINIMUM_BYTES 9
#define MODBUS_ASCII_MAXIMUM_BYTES 513

#define MODBUS_RTU_MINIMUM_BYTES 4
#define MODBUS_RTU_MAXIMUM_BYTES 256

#define Modbus_FunCode_Exception_Responses		0x80	/* It response that MSB of function code is '1' by Slave when exception like a invalid data address occured. */

#define MAX_QUANTITY_OF_READ_BITS		2000U
#define MAX_QUANTITY_OF_WRITE_BITS		1968U
#define MAX_QUANTITY_OF_READ_REGS		125U
#define MAX_QUANTITY_OF_WRITE_REGS		123U
#define MAX_QUANTITY_OF_WRITE_4X_REGS	121U

/*****************************************************************************/
/** DATATYPES ****************************************************************/
/*****************************************************************************/

/*****************************************************************************/
/** FUNCTION DECLARATIONS ****************************************************/
/*****************************************************************************/

static uint8_t generate_lrc( uint8_t *Source_Data, uint16_t Source_Data_Length );
static uint16_t ascii_to_hex( uint8_t const *ascii, uint16_t NoOfChars, uint8_t *hex );
static void hex_to_ascii( uint8_t const *hex, uint16_t Bytes, uint8_t *ascii );
static uint32_t bit_move_u8( uint8_t *src, int32_t src_start_bit_no, uint8_t *dest, int32_t dest_start_bit_no, int32_t bits );
uint16_t CRC16( uint8_t const *fstData, uint16_t Bytes ) __attribute__ ((weak, alias ("CRC16_u16")));

/*****************************************************************************/
/** GLOBAL VARIABLES *********************************************************/
/*****************************************************************************/

/*****************************************************************************/
/** LOCAL VARIABLES **********************************************************/
/*****************************************************************************/

static uint16_t const wCRCTable[ ] =
{
	0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
	0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
	0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
	0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
	0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
	0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
	0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
	0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
	0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
	0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
	0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
	0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
	0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
	0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
	0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
	0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
	0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
	0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
	0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
	0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
	0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
	0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
	0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
	0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
	0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
	0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
	0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
	0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
	0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
	0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
	0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
	0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
};

static uint8_t RxHex[ 260 ] = { 0 };
static uint8_t FrameBin[ 260 ] = { 0, };

/*****************************************************************************/
/** FUNCTION DEFINITIONS *****************************************************/
/*****************************************************************************/

/******************************************************************************
 * @brief 
 * 
 * @param protocol 
 * @param modbus_rx 
 * @param db 
 * @param tx_buf 
 * @return uint16_t 
 *****************************************************************************/
uint16_t modbus_slave_resp_read_coil_status( uint8_t protocol, Modbus_Rx_st *modbus_rx, DataBlock_st const *db, uint8_t *tx_buf )
{
	uint16_t tx_cnt = 0;
	uint16_t coil_status = 0;
	uint16_t bytes;
	int32_t src_starting_address = ( int32_t )modbus_rx->starting_address_read - db->CoilsStart;

	bytes = bit_move_u8( ( uint8_t* )db->Coils, src_starting_address, ( uint8_t* ) &coil_status,
						 0, ( int32_t )modbus_rx->no_of_registers_read );

	tx_cnt = MbSlaveBuildResponseFrame( protocol,
										modbus_rx->MBAP_Header.Transaction_ID,
										modbus_rx->MBAP_Header.Protocol_ID,
										modbus_rx->MBAP_Header.Unit_ID,
										Modbus_FunCode_Read_Coil_Status,
										modbus_rx->starting_address_read,
										bytes,
										&coil_status,
										tx_buf );

	return ( tx_cnt );
}

/******************************************************************************
 * @brief 
 * 
 * @param protocol 
 * @param modbus_rx 
 * @param db 
 * @param tx_buf 
 * @return uint16_t 
 *****************************************************************************/
uint16_t modbus_slave_resp_read_input_status( uint8_t protocol, Modbus_Rx_st *modbus_rx, DataBlock_st const *db, uint8_t *tx_buf )
{
	uint16_t tx_cnt = 0;
	uint16_t input_status = 0;
	uint16_t bytes;
	int32_t src_starting_address = ( int32_t )modbus_rx->starting_address_read - db->DInputsStart;

	bytes = bit_move_u8( ( uint8_t* )db->DInputs, src_starting_address, ( uint8_t* ) &input_status,
						 0,
						 ( int32_t )modbus_rx->no_of_registers_read );

	tx_cnt = MbSlaveBuildResponseFrame( protocol,
										modbus_rx->MBAP_Header.Transaction_ID,
										modbus_rx->MBAP_Header.Protocol_ID,
										modbus_rx->MBAP_Header.Unit_ID,
										Modbus_FunCode_Read_Input_Status,
										modbus_rx->starting_address_read,
										bytes,
										&input_status,
										tx_buf );

	return ( tx_cnt );
}

/******************************************************************************
 * @brief 
 * 
 * @param protocol 
 * @param modbus_rx 
 * @param db 
 * @param tx_buf 
 * @return uint16_t 
 *****************************************************************************/
uint16_t modbus_slave_resp_read_holding_reg( uint8_t protocol, Modbus_Rx_st *modbus_rx, DataBlock_st const *db, uint8_t *tx_buf )
{
	uint16_t tx_cnt = 0;
	uint16_t *data_buf = db->Holdings;

	data_buf += ( modbus_rx->starting_address_read - db->HoldingsStart );
	tx_cnt = MbSlaveBuildResponseFrame( protocol,
										modbus_rx->MBAP_Header.Transaction_ID,
										modbus_rx->MBAP_Header.Protocol_ID,
										modbus_rx->MBAP_Header.Unit_ID,
										Modbus_FunCode_Read_Holding_Reg,
										modbus_rx->starting_address_read,
										modbus_rx->no_of_registers_read << 1,
										data_buf,
										tx_buf );

	return ( tx_cnt );
}

/******************************************************************************
 * @brief 
 * 
 * @param protocol 
 * @param modbus_rx 
 * @param db 
 * @param tx_buf 
 * @return uint16_t 
 *****************************************************************************/
uint16_t modbus_slave_resp_read_input_reg( uint8_t protocol, Modbus_Rx_st *modbus_rx, DataBlock_st const *db, uint8_t *tx_buf )
{
	uint16_t tx_cnt = 0;
	uint16_t *data_buf = db->Inputs;

	data_buf += ( modbus_rx->starting_address_read - db->InputsStart );
	tx_cnt = MbSlaveBuildResponseFrame( protocol,
										modbus_rx->MBAP_Header.Transaction_ID,
										modbus_rx->MBAP_Header.Protocol_ID,
										modbus_rx->MBAP_Header.Unit_ID,
										Modbus_FunCode_Read_Input_Reg,
										modbus_rx->starting_address_read,
										modbus_rx->no_of_registers_read << 1,
										data_buf,
										tx_buf );

	return ( tx_cnt );
}

/******************************************************************************
 * @brief 
 * 
 * @param protocol 
 * @param modbus_rx 
 * @param db 
 * @param tx_buf 
 * @return uint16_t 
 *****************************************************************************/
uint16_t modbus_slave_resp_force_single_coil( uint8_t protocol, Modbus_Rx_st *modbus_rx, DataBlock_st const *db, uint8_t *tx_buf )
{
	uint16_t tx_cnt = 0;
	uint16_t *data_buf = NULL;
	uint8_t bit = 0;

	uint16_t tmpData = 0;
	uint16_t D1 = 0;
	uint16_t D2 = 0;

	D1 = modbus_rx->data_write[ 0 ];
	D2 = modbus_rx->data_write[ 0 ];
	tmpData = D1 << 8;
	tmpData |= ( D2 >> 8 ) & 0x00FF;

	data_buf = db->Coils;
	data_buf += ( modbus_rx->starting_address_write >> 4 ) - db->CoilsStart;
	bit = modbus_rx->starting_address_write & 0x0F;

	if( tmpData == 0xFF00 )
	{
		( *data_buf ) |= ( 1 << bit );
	}
	else if( tmpData == 0x0000 )
	{
		( *data_buf ) &= ~( 1 << bit );
	}

	tx_cnt = MbSlaveBuildResponseFrame( protocol,
										modbus_rx->MBAP_Header.Transaction_ID,
										modbus_rx->MBAP_Header.Protocol_ID,
										modbus_rx->MBAP_Header.Unit_ID,
										Modbus_FunCode_Force_Single_Coil,
										modbus_rx->starting_address_write,
										modbus_rx->no_of_registers_write,
										modbus_rx->data_write,
										tx_buf );

	return ( tx_cnt );
}

/******************************************************************************
 * @brief 
 * 
 * @param protocol 
 * @param modbus_rx 
 * @param db 
 * @param tx_buf 
 * @return uint16_t 
 *****************************************************************************/
uint16_t modbus_slave_resp_preset_holding_reg( uint8_t protocol, Modbus_Rx_st *modbus_rx, DataBlock_st const *db,
											   uint8_t *tx_buf )
{
	uint16_t tx_cnt = 0;
	uint16_t *data_buf = db->Holdings;

	data_buf[ modbus_rx->starting_address_write - db->HoldingsStart ] = modbus_rx->data_write[ 0 ];

	tx_cnt = MbSlaveBuildResponseFrame( protocol,
										modbus_rx->MBAP_Header.Transaction_ID,
										modbus_rx->MBAP_Header.Protocol_ID,
										modbus_rx->MBAP_Header.Unit_ID,
										Modbus_FunCode_Force_Holding_Reg,
										modbus_rx->starting_address_write,
										modbus_rx->no_of_registers_write,
										&data_buf[ modbus_rx->starting_address_write - db->HoldingsStart ],
										tx_buf );

	return ( tx_cnt );
}

/******************************************************************************
 * @brief 
 * 
 * @param protocol 
 * @param modbus_rx 
 * @param db 
 * @param tx_buf 
 * @return uint16_t 
 *****************************************************************************/
uint16_t modbus_slave_resp_force_multiple_coils( uint8_t protocol, Modbus_Rx_st *modbus_rx, DataBlock_st const *db,
												 uint8_t *tx_buf )
{
	uint16_t tx_cnt = 0;

	int32_t dest_starting_address = ( int32_t )modbus_rx->starting_address_write - db->CoilsStart;
	uint16_t *data_buf = db->Coils;

	bit_move_u8( ( uint8_t* )modbus_rx->data_write, 0, ( uint8_t* )data_buf,
				 dest_starting_address,
				 ( int32_t )modbus_rx->no_of_registers_write );

	tx_cnt = MbSlaveBuildResponseFrame( protocol,
										modbus_rx->MBAP_Header.Transaction_ID,
										modbus_rx->MBAP_Header.Protocol_ID,
										modbus_rx->MBAP_Header.Unit_ID,
										Modbus_FunCode_Force_Multiple_Coils,
										modbus_rx->starting_address_write,
										modbus_rx->no_of_registers_write,
										NULL,
										tx_buf );

	return ( tx_cnt );
}

/******************************************************************************
 * @brief 
 * 
 * @param protocol 
 * @param modbus_rx 
 * @param db 
 * @param tx_buf 
 * @return uint16_t 
 *****************************************************************************/
uint16_t modbus_slave_resp_preset_multiple_reg( uint8_t protocol, Modbus_Rx_st *modbus_rx, DataBlock_st const *db,
												uint8_t *tx_buf )
{
	uint16_t tx_cnt = 0;
	uint16_t *data_buf = &db->Holdings[ modbus_rx->starting_address_write - db->HoldingsStart ];

	for( uint16_t written = 0; written < modbus_rx->no_of_registers_write; ++written )
	{
		data_buf[ written ] = modbus_rx->data_write[ written ];
	}

	tx_cnt = MbSlaveBuildResponseFrame( protocol,
										modbus_rx->MBAP_Header.Transaction_ID,
										modbus_rx->MBAP_Header.Protocol_ID,
										modbus_rx->MBAP_Header.Unit_ID,
										Modbus_FunCode_Preset_Multiple_Reg,
										modbus_rx->starting_address_write,
										modbus_rx->no_of_registers_write << 1,
										NULL,
										tx_buf );

	return ( tx_cnt );
}

/******************************************************************************
 * @brief 
 * 
 * @param protocol 
 * @param modbus_rx 
 * @param db 
 * @param tx_buf 
 * @return uint16_t 
 *****************************************************************************/
uint16_t modbus_slave_resp_read_write_4x_reg( uint8_t protocol, Modbus_Rx_st *modbus_rx, DataBlock_st const *db, uint8_t *tx_buf )
{
	uint16_t tx_cnt = 0;
	uint16_t *data_buf = &db->Holdings[ modbus_rx->starting_address_write - db->HoldingsStart ];

	/* This function code performs a combination of one read operation and one write operation in a
	single MODBUS transaction. The write operation is performed before the read.  */
	for( uint16_t written = 0; written < modbus_rx->no_of_registers_write; ++written )
	{
		data_buf[ written ] = modbus_rx->data_write[ written ];
	}

	data_buf = &db->Holdings[ modbus_rx->starting_address_read - db->HoldingsStart ];

	tx_cnt = MbSlaveBuildResponseFrame( protocol,
										modbus_rx->MBAP_Header.Transaction_ID,
										modbus_rx->MBAP_Header.Protocol_ID,
										modbus_rx->MBAP_Header.Unit_ID,
										Modbus_FunCode_Read_Write_4X_Reg,
										modbus_rx->starting_address_read,
										modbus_rx->no_of_registers_read << 1,
										data_buf,
										tx_buf );

	return ( tx_cnt );
}

/******************************************************************************
 * @brief 
 * 
 * @param protocol 
 * @param modbus_rx 
 * @param tx_buf 
 * @param ex_code 
 * @return uint16_t 
 *****************************************************************************/
uint16_t modbus_exception_response( uint8_t protocol, Modbus_Rx_st *modbus_rx, uint8_t *tx_buf, Modbus_ExCode_et ex_code )
{
	uint16_t tx_cnt = 0;
	uint16_t ex_code_word = ex_code;	// for suppress warning unaligned memory

	tx_cnt = MbSlaveBuildResponseFrame( protocol,
										modbus_rx->MBAP_Header.Transaction_ID,
										modbus_rx->MBAP_Header.Protocol_ID,
										modbus_rx->MBAP_Header.Unit_ID,
										( modbus_rx->function_code | 0x80 ),
										modbus_rx->starting_address_write,
										modbus_rx->no_of_registers_write << 1,
										&ex_code_word,
										tx_buf );

	return ( tx_cnt );
}

/******************************************************************************
 * @brief 
 * 
 * @param frame_format 
 * @param rx_buf 
 * @param rx_cnt 
 * @param db 
 * @param modbus_rx 
 * @return Modbus_ExCode_et 
 *****************************************************************************/
Modbus_ExCode_et MbSlaveCheckRequestFrame( int8_t frame_format, uint8_t const *rx_buf, uint16_t rx_cnt, DataBlock_st const *db,
										   Modbus_Rx_st *modbus_rx )
{
	volatile uint8_t const *RxPacket = NULL;
	volatile uint16_t ADU_Index = 0;
	Modbus_ExCode_et Return_Value = Modbus_ExCode_Slave_Failure;
	uint16_t Rxed_CRC16 = 0x0;
	uint16_t LoopCount = 0;
	uint16_t pdu_len = 0;

	switch( frame_format )
	{
		case 'T': /* TCP */
			if( ( MODBUS_TCP_MINIMUM_BYTES <= rx_cnt ) && ( MODBUS_TCP_MAXIMUM_BYTES >= rx_cnt ) )
			{
				RxPacket = rx_buf;
				modbus_rx->MBAP_Header.Length = ( ( uint16_t )RxPacket[ 4 ] << 8 ) | ( uint16_t )RxPacket[ 5 ];
				if( modbus_rx->MBAP_Header.Length == ( rx_cnt - 6 ) )
				{
					modbus_rx->MBAP_Header.Transaction_ID = ( ( uint16_t )RxPacket[ 0 ] << 8 ) | ( uint16_t )RxPacket[ 1 ];
					modbus_rx->MBAP_Header.Protocol_ID = ( ( uint16_t )RxPacket[ 2 ] << 8 ) | ( uint16_t )RxPacket[ 3 ];
					modbus_rx->MBAP_Header.Unit_ID = RxPacket[ 6 ];

					if( ( 0U == modbus_rx->MBAP_Header.Protocol_ID ) && ( 2U <= modbus_rx->MBAP_Header.Length ) )
					{
						ADU_Index = 7;
						pdu_len = modbus_rx->MBAP_Header.Length - 1U;
						Return_Value = Modbus_ExCode_Normal;
					}
				}
			}
			break;
		case 'A': /* ASCII */
			if( ( MODBUS_ASCII_MINIMUM_BYTES <= rx_cnt ) && ( MODBUS_ASCII_MAXIMUM_BYTES >= rx_cnt ) )
			{
				uint16_t rx_hex_bytes = 0;
				modbus_rx->start_code = rx_buf[ 0 ];
				modbus_rx->end_code = ( ( uint16_t )rx_buf[ rx_cnt - 2 ] << 8 ) | ( uint16_t )rx_buf[ rx_cnt - 1 ];
				if( ( modbus_rx->start_code == Modbus_ASCII_Start_Code ) && ( modbus_rx->end_code == Modbus_ASCII_End_Code ) )
				{
					rx_hex_bytes = ascii_to_hex( rx_buf + 1, rx_cnt - 3, RxHex ); /* Slave Address ~ LRC */
					modbus_rx->checksum = RxHex[ ( ( rx_cnt - 3 ) >> 1 ) - 1 ]; /* LRC Check */
					if( ( 3U <= rx_hex_bytes ) &&
						( modbus_rx->checksum == generate_lrc( RxHex, ( ( rx_cnt - 5 ) >> 1 ) ) ) )
					{
						modbus_rx->MBAP_Header.Unit_ID = RxHex[ 0 ]; /* Slave Address */
						RxPacket = RxHex;
						ADU_Index = 1;
						pdu_len = rx_hex_bytes - 2U;
						Return_Value = Modbus_ExCode_Normal;
					}
				}
			}
			break;
		case 'R': /* RTU */
			if( ( MODBUS_RTU_MINIMUM_BYTES <= rx_cnt ) && ( MODBUS_RTU_MAXIMUM_BYTES >= rx_cnt ) )
			{
				Rxed_CRC16 = ( ( uint16_t )rx_buf[ rx_cnt - 1 ] << 8 ) | ( uint16_t )rx_buf[ rx_cnt - 2 ];
				if( Rxed_CRC16 == CRC16( rx_buf, rx_cnt - 2 ) )
				{
					RxPacket = rx_buf;
					ADU_Index = 0;
					modbus_rx->MBAP_Header.Unit_ID = RxPacket[ ADU_Index++ ]; /*	Slave ID	*/
					pdu_len = rx_cnt - 3U;
					Return_Value = Modbus_ExCode_Normal;
				}
			}
			break;
		default:
			Return_Value = Modbus_ExCode_Slave_Failure;
			break;
	}

	if( Modbus_ExCode_Normal == Return_Value )
	{
		modbus_rx->function_code = RxPacket[ ADU_Index++ ];
		if( ( modbus_rx->function_code == Modbus_FunCode_Read_Coil_Status ) /*	0x01	*/
		    || ( modbus_rx->function_code == Modbus_FunCode_Read_Input_Status ) /*	0x02	*/
		    || ( modbus_rx->function_code == Modbus_FunCode_Read_Holding_Reg ) /*	0x03	*/
		    || ( modbus_rx->function_code == Modbus_FunCode_Read_Input_Reg ) /*	0x04	*/
		    || ( modbus_rx->function_code == Modbus_FunCode_Force_Single_Coil ) /*	0x05	*/
		    || ( modbus_rx->function_code == Modbus_FunCode_Force_Holding_Reg ) /*	0x06	*/
		    || ( modbus_rx->function_code == Modbus_FunCode_Force_Multiple_Coils ) /*	0x0F	*/
		    || ( modbus_rx->function_code == Modbus_FunCode_Preset_Multiple_Reg ) /*	0x10	*/
		    || ( modbus_rx->function_code == Modbus_FunCode_Read_Write_4X_Reg ) /*	0x17	*/
		    )
		{
			if( ( modbus_rx->function_code == Modbus_FunCode_Read_Coil_Status )
				|| ( modbus_rx->function_code == Modbus_FunCode_Read_Input_Status )
				|| ( modbus_rx->function_code == Modbus_FunCode_Read_Holding_Reg )
				|| ( modbus_rx->function_code == Modbus_FunCode_Read_Input_Reg )
				|| ( modbus_rx->function_code == Modbus_FunCode_Read_Write_4X_Reg )
				)
			{
				if( ( modbus_rx->function_code != Modbus_FunCode_Read_Write_4X_Reg ) && ( 5U != pdu_len ) )
				{
					Return_Value = Modbus_ExCode_Illegal_DataValue;
				}
				else if( ( modbus_rx->function_code == Modbus_FunCode_Read_Write_4X_Reg ) && ( pdu_len < 10U ) )
				{
					Return_Value = Modbus_ExCode_Illegal_DataValue;
				}

				if( Modbus_ExCode_Normal != Return_Value )
				{
					return ( Return_Value );
				}

				modbus_rx->starting_address_read = ( uint16_t )RxPacket[ ADU_Index++ ] << 8;
				modbus_rx->starting_address_read |= ( uint16_t )RxPacket[ ADU_Index++ ];
				modbus_rx->no_of_registers_read = ( uint16_t )RxPacket[ ADU_Index++ ] << 8;
				modbus_rx->no_of_registers_read |= ( uint16_t )RxPacket[ ADU_Index++ ];

				if( !modbus_rx->no_of_registers_read )
				{
					Return_Value = Modbus_ExCode_Illegal_DataValue;
				}
				else
				{
					uint32_t max_quantity = 0U;
					uint32_t max_request_quantity = 0U;
					uint16_t start_addr = 0U;
					uint32_t last_addr = 0U;
					uint32_t requested_last_addr = 0U;
					switch( modbus_rx->function_code )
					{
						case Modbus_FunCode_Read_Coil_Status:
							max_quantity = db->CoilsCnt;
							max_request_quantity = MAX_QUANTITY_OF_READ_BITS;
							start_addr = db->CoilsStart;
							break;
						case Modbus_FunCode_Read_Input_Status:
							max_quantity = db->DInputsCnt;
							max_request_quantity = MAX_QUANTITY_OF_READ_BITS;
							start_addr = db->DInputsStart;
							break;
						case Modbus_FunCode_Read_Input_Reg:
							max_quantity = db->InputsCnt;
							max_request_quantity = MAX_QUANTITY_OF_READ_REGS;
							start_addr = db->InputsStart;
							break;
						case Modbus_FunCode_Read_Holding_Reg:
						case Modbus_FunCode_Read_Write_4X_Reg:
							max_quantity = db->HoldingsCnt;
							max_request_quantity = MAX_QUANTITY_OF_READ_REGS;
							start_addr = db->HoldingsStart;
							break;
					}

					last_addr = ( uint32_t )start_addr + max_quantity - 1U;
					requested_last_addr = ( uint32_t )modbus_rx->starting_address_read + modbus_rx->no_of_registers_read - 1U;

					if( ( max_quantity < modbus_rx->no_of_registers_read )
						|| ( max_request_quantity < modbus_rx->no_of_registers_read ) )
					{
						Return_Value = Modbus_ExCode_Illegal_DataValue;
					}
					else if( ( modbus_rx->starting_address_read < start_addr ) ||
							 ( requested_last_addr > last_addr )
							 )
					{
						Return_Value = Modbus_ExCode_Illegal_Address;
					}
				}
			}
			if( ( modbus_rx->function_code == Modbus_FunCode_Force_Single_Coil )
				|| ( modbus_rx->function_code == Modbus_FunCode_Force_Holding_Reg )
				|| ( modbus_rx->function_code == Modbus_FunCode_Force_Multiple_Coils )
				|| ( modbus_rx->function_code == Modbus_FunCode_Preset_Multiple_Reg )
				|| ( modbus_rx->function_code == Modbus_FunCode_Read_Write_4X_Reg )
				)
			{
				if( ( ( modbus_rx->function_code == Modbus_FunCode_Force_Single_Coil ) ||
					  ( modbus_rx->function_code == Modbus_FunCode_Force_Holding_Reg ) ) && ( 5U != pdu_len ) )
				{
					Return_Value = Modbus_ExCode_Illegal_DataValue;
				}
				else if( ( ( modbus_rx->function_code == Modbus_FunCode_Force_Multiple_Coils ) ||
						   ( modbus_rx->function_code == Modbus_FunCode_Preset_Multiple_Reg ) ) && ( pdu_len < 6U ) )
				{
					Return_Value = Modbus_ExCode_Illegal_DataValue;
				}
				else if( ( modbus_rx->function_code == Modbus_FunCode_Read_Write_4X_Reg ) && ( pdu_len < 10U ) )
				{
					Return_Value = Modbus_ExCode_Illegal_DataValue;
				}

				if( Modbus_ExCode_Normal != Return_Value )
				{
					return ( Return_Value );
				}

				modbus_rx->starting_address_write = ( uint16_t )RxPacket[ ADU_Index++ ] << 8;
				modbus_rx->starting_address_write |= ( uint16_t )RxPacket[ ADU_Index++ ];

				if( modbus_rx->function_code != Modbus_FunCode_Force_Single_Coil && modbus_rx->function_code
								!= Modbus_FunCode_Force_Holding_Reg )
				{
					modbus_rx->no_of_registers_write = ( uint16_t )RxPacket[ ADU_Index++ ] << 8;
					modbus_rx->no_of_registers_write |= ( uint16_t )RxPacket[ ADU_Index++ ];
					modbus_rx->byte_count = RxPacket[ ADU_Index++ ];

					if( modbus_rx->function_code == Modbus_FunCode_Force_Multiple_Coils )
					{
						uint16_t expected_byte_count = ( modbus_rx->no_of_registers_write + 7U ) >> 3;
						if( ( modbus_rx->byte_count != expected_byte_count ) ||
							( pdu_len != ( 6U + modbus_rx->byte_count ) ) )
						{
							Return_Value = Modbus_ExCode_Illegal_DataValue;
						}
					}
					else if( ( modbus_rx->function_code == Modbus_FunCode_Preset_Multiple_Reg ) ||
							 ( modbus_rx->function_code == Modbus_FunCode_Read_Write_4X_Reg ) )
					{
						uint16_t expected_byte_count = modbus_rx->no_of_registers_write << 1;
						uint16_t expected_pdu_len = 6U + modbus_rx->byte_count;

						if( modbus_rx->function_code == Modbus_FunCode_Read_Write_4X_Reg )
						{
							expected_pdu_len = 10U + modbus_rx->byte_count;
						}

						if( ( modbus_rx->byte_count != expected_byte_count ) ||
							( pdu_len != expected_pdu_len ) )
						{
							Return_Value = Modbus_ExCode_Illegal_DataValue;
						}
					}
				}

				if( Modbus_ExCode_Normal != Return_Value )
				{
					return ( Return_Value );
				}

				if( ( !modbus_rx->no_of_registers_write ) && ( ( modbus_rx->function_code != Modbus_FunCode_Force_Single_Coil )
								&& ( modbus_rx->function_code != Modbus_FunCode_Force_Holding_Reg ) ) )
				{
					Return_Value = Modbus_ExCode_Illegal_DataValue;
				}
				else
				{
					uint16_t start_addr = 0U;
					uint32_t last_addr = 0U;
					uint32_t requested_last_addr = 0U;

					if( modbus_rx->function_code == Modbus_FunCode_Force_Multiple_Coils )
					{
						if( ( modbus_rx->no_of_registers_write > db->CoilsCnt )
							|| ( modbus_rx->no_of_registers_write > MAX_QUANTITY_OF_WRITE_BITS ) )
						{
							Return_Value = Modbus_ExCode_Illegal_DataValue;
						}
						else
						{
							start_addr = db->CoilsStart;
							last_addr = ( uint32_t )start_addr + db->CoilsCnt - 1U;
							requested_last_addr = ( uint32_t )modbus_rx->starting_address_write + modbus_rx->no_of_registers_write - 1U;
							if( ( modbus_rx->starting_address_write < start_addr ) ||
								( requested_last_addr > last_addr ) )
							{
								Return_Value = Modbus_ExCode_Illegal_Address;
							}
							else
							{
								for( LoopCount = 0; LoopCount < modbus_rx->byte_count; ++LoopCount ) /* byte_count can be odd number */
								{
									if( LoopCount & 1 ) /*  Odd Number */
									{
										modbus_rx->data_write[ LoopCount >> 1 ] |= ( uint16_t )RxPacket[ ADU_Index++ ] << 8;
									}
									else /* Even Number */
									{
										modbus_rx->data_write[ LoopCount >> 1 ] = ( uint16_t )RxPacket[ ADU_Index++ ]; /* lower address is transmitted first. */
									}
								}
							}
						}
					}
					else if( ( modbus_rx->function_code == Modbus_FunCode_Preset_Multiple_Reg )
							 || ( modbus_rx->function_code == Modbus_FunCode_Read_Write_4X_Reg ) )
					{
						if( ( modbus_rx->no_of_registers_write > db->HoldingsCnt )
							|| ( ( modbus_rx->function_code == Modbus_FunCode_Preset_Multiple_Reg ) &&
								 ( modbus_rx->no_of_registers_write > MAX_QUANTITY_OF_WRITE_REGS ) )
							|| ( ( modbus_rx->function_code == Modbus_FunCode_Read_Write_4X_Reg ) &&
								 ( modbus_rx->no_of_registers_write > MAX_QUANTITY_OF_WRITE_4X_REGS ) ) )
						{
							Return_Value = Modbus_ExCode_Illegal_DataValue;
						}
						else
						{
							start_addr = db->HoldingsStart;
							last_addr = ( uint32_t )start_addr + db->HoldingsCnt - 1U;
							requested_last_addr = ( uint32_t )modbus_rx->starting_address_write + modbus_rx->no_of_registers_write - 1U;
							if( ( modbus_rx->starting_address_write < start_addr ) || ( requested_last_addr > last_addr ) )
							{
								Return_Value = Modbus_ExCode_Illegal_Address;
							}
							else
							{
								for( LoopCount = 0; LoopCount < modbus_rx->byte_count; ++LoopCount ) /* byte_count is always odd number */
								{
									if( LoopCount & 1 ) /* Odd Number */
										modbus_rx->data_write[ LoopCount >> 1 ] |= ( uint16_t )RxPacket[ ADU_Index++ ];
									else
										/* Even Number */
										modbus_rx->data_write[ LoopCount >> 1 ] = ( uint16_t )RxPacket[ ADU_Index++ ] << 8; /* The upper byte of 16 bits is transmitted first. */
								}
							}
						}
					}
					else if( modbus_rx->function_code == Modbus_FunCode_Force_Single_Coil )
					{
						last_addr = ( uint32_t )db->CoilsStart + db->CoilsCnt - 1U;
						if( ( modbus_rx->starting_address_write < db->CoilsStart ) ||
							( modbus_rx->starting_address_write > last_addr ) )
						{
							Return_Value = Modbus_ExCode_Illegal_Address;
						}
						else
						{
							uint16_t coil_value = ( uint16_t )RxPacket[ ADU_Index ] << 8;
							coil_value |= ( uint16_t )RxPacket[ ADU_Index + 1U ];
							if( ( coil_value != 0xFF00U ) && ( coil_value != 0x0000U ) )
							{
								Return_Value = Modbus_ExCode_Illegal_DataValue;
								return ( Return_Value );
							}

							modbus_rx->no_of_registers_write = 0; /* Force set */
							modbus_rx->byte_count = 2; /* Force set */

							for( LoopCount = 0; LoopCount < modbus_rx->byte_count; ++LoopCount ) /* byte_count can be odd number */
							{
								if( LoopCount & 1 ) /* Odd Number */
									modbus_rx->data_write[ LoopCount >> 1 ] |= ( uint16_t )RxPacket[ ADU_Index++ ] << 8;
								else
									/* Even Number */
									modbus_rx->data_write[ LoopCount >> 1 ] = ( uint16_t )RxPacket[ ADU_Index++ ]; /* lower address is transmitted first. */
							}
						}
					}
					else if( modbus_rx->function_code == Modbus_FunCode_Force_Holding_Reg )
					{
						last_addr = ( uint32_t )db->HoldingsStart + db->HoldingsCnt - 1U;
						if( ( modbus_rx->starting_address_write < db->HoldingsStart ) ||
							( modbus_rx->starting_address_write > last_addr ) )
						{
							Return_Value = Modbus_ExCode_Illegal_Address;
						}
						else
						{
							modbus_rx->no_of_registers_write = 1; /* Force set */
							modbus_rx->byte_count = 2;

							for( LoopCount = 0; LoopCount < modbus_rx->byte_count; ++LoopCount ) /* byte_count can be odd number */
							{
								if( LoopCount & 1 ) /* Odd Number */
									modbus_rx->data_write[ LoopCount >> 1 ] |= ( uint16_t )RxPacket[ ADU_Index++ ]; /* The lower byte of 16 bits is transmitted first. */
								else
									/* Even Number */
									modbus_rx->data_write[ LoopCount >> 1 ] = ( uint16_t )RxPacket[ ADU_Index++ ] << 8;
							}
						}
					}
				}
			}
		}
		else
		{
			Return_Value = Modbus_ExCode_Illegal_FunCode;
		}
	}

	return ( Return_Value );
}

/******************************************************************************
 * @brief 
 * 
 * @param frame_format 
 * @param txn_id 
 * @param protocol_id 
 * @param slave_address 
 * @param function_code 
 * @param starting_address 
 * @param no_of_response_bytes 
 * @param response_data 
 * @param response_frame 
 * @return uint16_t 
 *****************************************************************************/
uint16_t MbSlaveBuildResponseFrame( uint8_t frame_format, uint16_t txn_id, uint16_t protocol_id,
									uint8_t slave_address,
									uint8_t function_code, uint16_t starting_address,
									uint16_t no_of_response_bytes,
									uint16_t *response_data, uint8_t *response_frame )
{
	uint16_t LoopCount;
	uint16_t ADU_Index = 0;
	uint16_t Calc_CRC16;
	uint8_t Calc_LRC;

	/* It doesn't response when broadcast message received. */
	if( slave_address == 0 )
	{
		return 0;
	}

	if( frame_format == 'T' )
	{
		/* Transaction Identifier */
		FrameBin[ ADU_Index++ ] = ( uint8_t )( txn_id >> 8 );
		FrameBin[ ADU_Index++ ] = ( uint8_t )( txn_id & 0xFF );
		/* Protocol Identifier(Modbus Protocol = 0) */
		FrameBin[ ADU_Index++ ] = ( uint8_t )( protocol_id >> 8 );
		FrameBin[ ADU_Index++ ] = ( uint8_t )( protocol_id & 0xFF );
		/* Length(Number of following bytes, 2bytes) */
		/*
		 Frame[ 4 ], Frame[ 5 ]
		 */
		ADU_Index++;
		ADU_Index++;
	}

	FrameBin[ ADU_Index++ ] = slave_address;
	FrameBin[ ADU_Index++ ] = function_code;

	if( function_code & 0x80 )
	{
		/* EXCEPTION RESPONSE */
		uint8_t *BytePointer;
		BytePointer = ( uint8_t* )response_data;
		FrameBin[ ADU_Index++ ] = ( *BytePointer );
	}
	else if( ( function_code == Modbus_FunCode_Read_Coil_Status ) || ( function_code == Modbus_FunCode_Read_Input_Status ) )
	{
		if( NULL == response_data ) return 0;

		/* Discrete Read( First, Lower Address ) */
		uint8_t *BytePointer;
		FrameBin[ ADU_Index++ ] = ( uint8_t )no_of_response_bytes;
		BytePointer = ( uint8_t* )response_data;
		for( LoopCount = 0; LoopCount < no_of_response_bytes; ++LoopCount )
			FrameBin[ ADU_Index++ ] = BytePointer[ LoopCount ];
	}
	else if( ( function_code == Modbus_FunCode_Read_Holding_Reg ) || ( function_code == Modbus_FunCode_Read_Input_Reg )
			 || ( function_code == Modbus_FunCode_Read_Write_4X_Reg ) )
	{
		if( NULL == response_data ) return 0;

		/* Word Read( First, Upper Byte ) */
		FrameBin[ ADU_Index++ ] = ( uint8_t )no_of_response_bytes;
		for( LoopCount = 0; LoopCount < no_of_response_bytes >> 1; ++LoopCount )
		{
			FrameBin[ ADU_Index++ ] = ( uint8_t )( response_data[ LoopCount ] >> 8 );
			FrameBin[ ADU_Index++ ] = ( uint8_t )( response_data[ LoopCount ] & 0xFF );
		}
	}
	else if( ( function_code == Modbus_FunCode_Force_Multiple_Coils ) || ( function_code == Modbus_FunCode_Preset_Multiple_Reg ) )
	{
		/* Discrete & Word Write */
		FrameBin[ ADU_Index++ ] = ( uint8_t )( starting_address >> 8 );
		FrameBin[ ADU_Index++ ] = ( uint8_t )( starting_address & 0xFF );
		if( function_code == Modbus_FunCode_Force_Multiple_Coils )
		{
			/* Unlike other commands, this command only transfers the number of response bytes in bytes. */
			/* So the value of no_of_response_bytes is not divided by two. */
			FrameBin[ ADU_Index++ ] = ( uint8_t )( no_of_response_bytes >> 8 );
			FrameBin[ ADU_Index++ ] = ( uint8_t )( no_of_response_bytes & 0xFF );
		}
		else
		{
			FrameBin[ ADU_Index++ ] = ( uint8_t )( ( no_of_response_bytes >> 1 ) >> 8 );
			FrameBin[ ADU_Index++ ] = ( uint8_t )( ( no_of_response_bytes >> 1 ) & 0xFF );
		}
	}
	else if( function_code == Modbus_FunCode_Force_Holding_Reg )
	{
		if( NULL == response_data ) return 0;

		FrameBin[ ADU_Index++ ] = ( uint8_t )( starting_address >> 8 );
		FrameBin[ ADU_Index++ ] = ( uint8_t )( starting_address & 0xFF );
		FrameBin[ ADU_Index++ ] = ( uint8_t )( *response_data >> 8 );
		FrameBin[ ADU_Index++ ] = ( uint8_t )( *response_data & 0xFF );
	}
	else if( function_code == Modbus_FunCode_Force_Single_Coil )
	{
		if( NULL == response_data ) return 0;

		FrameBin[ ADU_Index++ ] =  ( uint8_t )( starting_address >> 8 );
		FrameBin[ ADU_Index++ ] =  ( uint8_t )( starting_address & 0xFF );

		FrameBin[ ADU_Index++ ] =  ( uint8_t )( response_data[ 0 ] & 0xFF );
		FrameBin[ ADU_Index++ ] =  ( uint8_t )( response_data[ 0 ] >> 8 );
	}

	if( frame_format == 'T' )
	{
		FrameBin[ 4 ] = ( uint8_t )( ( ADU_Index - 6 ) >> 8 );
		FrameBin[ 5 ] = ( uint8_t )( ADU_Index - 6 );
		/* Copy Data */
		for( LoopCount = 0; LoopCount < ADU_Index; ++LoopCount )
			response_frame[ LoopCount ] = FrameBin[ LoopCount ];
	}
	else if( frame_format == 'A' )
	{
		Calc_LRC = generate_lrc( FrameBin, ADU_Index );
		FrameBin[ ADU_Index++ ] = Calc_LRC;
		/* ASCII Framing */
		response_frame[ 0 ] = Modbus_ASCII_Start_Code;
		hex_to_ascii( FrameBin, ADU_Index, response_frame + 1 );
		ADU_Index = ( ADU_Index << 1 ) + 1;
		response_frame[ ADU_Index++ ] = ( uint8_t )( Modbus_ASCII_End_Code >> 8 ); /* CR */
		response_frame[ ADU_Index++ ] = ( uint8_t )( Modbus_ASCII_End_Code & 0xff ); /* LF */
	}
	else if( frame_format == 'R' )
	{
		/* Copy Data */
		for( LoopCount = 0; LoopCount < ADU_Index; ++LoopCount )
			response_frame[ LoopCount ] = FrameBin[ LoopCount ];

		Calc_CRC16 = CRC16( FrameBin, ADU_Index );
		/* In the CRC code, the lower byte is transmitted first. */
		response_frame[ ADU_Index++ ] = ( uint8_t )Calc_CRC16;
		response_frame[ ADU_Index++ ] = ( uint8_t )( Calc_CRC16 >> 8 );
	}

	return ( ADU_Index );
}

/******************************************************************************
 * @brief 
 * @note ADU : Application Data Unit
 * @note MBAP : MODBUS Application Protocol
 * @note		 - Transaction Identifier : 2bytes
 * @note		 - Protocol Identifier : 2bytes
 * @note		 - Length : 2bytes(Number of following bytes, including the Unit Identifier and data fields)
 * @note		 - Unit Identifier : 1byte
 * @note -------------------------------------------------------------------------------
 * @note Frame_Format : 'A'(Modbus/ASCII), 'R'(Modbus/RTU), 'T'(Modbus/TCP)
 * @note -------------------------------------------------------------------------------
 * @note MODBUS RTU ADU = Server(Slave) address(1 byte) + Function Code(1 byte) + Data(252 bytes) + CRC(2 bytes) = 256 bytes
 * @note MODBUS ASCII ADU = Start(1 char) + Server(Slave) address(2 chars) + Function Code(2 chars) + Data( 252 chars) + LRC(2 char) + End(CR,LF, 2 chars) = 513 chars
 * @note TCP MODBUS ADU = MBAP (7 bytes) + Data(253 bytes) = 260 bytes
 * 
 * @param frame_format 
 * @param rx_frame 
 * @param rcvd_bytes 
 * @param parsing_result 
 * @return Modbus_ExCode_et 
 * @retval Exception Code (like a 0x80)
 *****************************************************************************/
Modbus_ExCode_et ModbusMasterResponseFrameParse( int8_t frame_format, uint8_t *rx_frame, uint32_t rcvd_bytes,
												 volatile Modbus_Rx_st *parsing_result )
{
	uint8_t *rx_packet = 0;
	uint16_t crc16_result;
	uint32_t loop_count;
	uint32_t pdu_index;
	Modbus_ExCode_et return_value = Modbus_ExCode_Slave_Failure;

	/* ======================
	 *		MODBUS - TCP
	 * ====================== */
	if( frame_format == 'T' )
	{
		if( ( MODBUS_TCP_MINIMUM_BYTES <= rcvd_bytes ) && ( rcvd_bytes <= MODBUS_TCP_MAXIMUM_BYTES ) )
		{
			rx_packet = rx_frame;
			parsing_result->MBAP_Header.Length = ( ( uint16_t )rx_packet[ 4 ] << 8 ) | ( uint16_t )rx_packet[ 5 ];
			if( parsing_result->MBAP_Header.Length == ( rcvd_bytes - 6 ) )
			{
				parsing_result->MBAP_Header.Transaction_ID = ( ( uint16_t )rx_packet[ 0 ] << 8 ) | ( uint16_t )rx_packet[ 1 ];
				parsing_result->MBAP_Header.Protocol_ID = ( ( uint16_t )rx_packet[ 2 ] << 8 ) | ( uint16_t )rx_packet[ 3 ];
				parsing_result->MBAP_Header.Unit_ID = rx_packet[ 6 ];

				pdu_index = 7;

				return_value = Modbus_ExCode_Normal;
			}
		}
	}
	/* ======================
	 *		MODBUS - ASCII
	 * ====================== */
	else if( frame_format == 'A' )
	{
		if( ( MODBUS_ASCII_MINIMUM_BYTES <= rcvd_bytes ) && ( rcvd_bytes <= MODBUS_ASCII_MAXIMUM_BYTES ) )
		{
			parsing_result->start_code = rx_frame[ 0 ];
			parsing_result->end_code = ( ( uint16_t )rx_frame[ rcvd_bytes - 2 ] << 8 ) | ( uint16_t )rx_frame[ rcvd_bytes - 1 ];
			if( ( parsing_result->start_code == Modbus_ASCII_Start_Code ) && ( parsing_result->end_code == Modbus_ASCII_End_Code ) )
			{
				ascii_to_hex( rx_frame + 1, rcvd_bytes - 3, RxHex ); /* Slave Address ~ LRC */
				parsing_result->checksum = RxHex[ ( ( rcvd_bytes - 3 ) >> 1 ) - 1 ]; /* LRC Check */
				if( parsing_result->checksum == generate_lrc( RxHex, ( ( rcvd_bytes - 5 ) >> 1 ) ) )
				{
					parsing_result->MBAP_Header.Unit_ID = RxHex[ 0 ]; /* Slave Address */
					rx_packet = RxHex;
					pdu_index = 1;
					return_value = Modbus_ExCode_Normal;
				}
			}
		}
	}
	/* ======================
	 *		MODBUS - RTU
	 * ====================== */
	else if( frame_format == 'R' )
	{
		if( ( MODBUS_RTU_MINIMUM_BYTES <= rcvd_bytes ) && ( rcvd_bytes <= MODBUS_RTU_MAXIMUM_BYTES ) )
		{
			parsing_result->checksum = ( ( uint16_t )rx_frame[ rcvd_bytes - 1 ] << 8 )
							| ( uint16_t )rx_frame[ rcvd_bytes - 2 ];
			crc16_result = CRC16( rx_frame, rcvd_bytes - 2 );

			if( parsing_result->checksum == crc16_result )
			{
				rx_packet = rx_frame;
				pdu_index = 0;
				parsing_result->MBAP_Header.Unit_ID = rx_packet[ pdu_index++ ]; /*	Slave address	*/
				return_value = Modbus_ExCode_Normal;
			}
		}
	}

	if( return_value == Modbus_ExCode_Normal ) /*	by KangMinSoo	2017-05-25	*/
	{
		parsing_result->function_code = rx_packet[ pdu_index++ ];
		if( ( parsing_result->function_code == Modbus_FunCode_Force_Multiple_Coils ) ||
			( parsing_result->function_code == Modbus_FunCode_Preset_Multiple_Reg ) )
		{
			parsing_result->starting_address_write = ( uint16_t )rx_packet[ pdu_index++ ] << 8;
			parsing_result->starting_address_write |= ( uint16_t )rx_packet[ pdu_index++ ];
			parsing_result->no_of_registers_write = ( uint16_t )rx_packet[ pdu_index++ ] << 8;
			parsing_result->no_of_registers_write |= ( uint16_t )rx_packet[ pdu_index++ ];
		}
		else if( ( parsing_result->function_code == Modbus_FunCode_Read_Coil_Status )
				 || ( parsing_result->function_code == Modbus_FunCode_Read_Input_Status )
				 || ( parsing_result->function_code == Modbus_FunCode_Read_Holding_Reg )
				 || ( parsing_result->function_code == Modbus_FunCode_Read_Input_Reg )
				 || ( parsing_result->function_code == Modbus_FunCode_Read_Write_4X_Reg ) )
		{
			uint32_t byte_count = ( uint32_t )rx_packet[ pdu_index++ ];
			parsing_result->byte_count = ( uint8_t )byte_count;
			if( ( parsing_result->function_code == Modbus_FunCode_Read_Coil_Status ) ||
				( parsing_result->function_code == Modbus_FunCode_Read_Input_Status ) )
			{ /* LSB 1st */
				for( loop_count = 0; loop_count < byte_count; ++loop_count )
				{
					if( loop_count & 1 ) /* odd-numbered */
						parsing_result->data_write[ loop_count >> 1 ] |= ( uint16_t )rx_packet[ pdu_index++ ] << 8;
					else
						/* even-numbered */
						parsing_result->data_write[ loop_count >> 1 ] = ( uint16_t )rx_packet[ pdu_index++ ];
				}
			}
			else
			{ /* MSB 1st */
				for( loop_count = 0; loop_count < byte_count; ++loop_count )
				{
					if( loop_count & 1 ) /* odd-numbered */
						parsing_result->data_write[ loop_count >> 1 ] |= ( uint16_t )rx_packet[ pdu_index++ ];
					else
						/* even-numbered */
						parsing_result->data_write[ loop_count >> 1 ] = ( uint16_t )rx_packet[ pdu_index++ ] << 8;
				}
			}
		}
#if 0
		else if ( parsing_result->function_code == Modbus_FunCode_Read_Exception_Status )
			parsing_result->exception_status = rx_packet[ pdu_index++ ];
#endif
		else if( parsing_result->function_code & Modbus_FunCode_Exception_Responses )
		{
			/* Exception Response : MSB of function code is '1' */
			uint8_t exception_code;
			exception_code = ( uint16_t )rx_packet[ pdu_index++ ];
			parsing_result->data_write[ 0 ] = ( uint16_t )exception_code;
			return_value = ( Modbus_ExCode_et )exception_code;
		}
		else
		{
			/* Unsupported Function Code */
			return_value = Modbus_ExCode_Illegal_FunCode;
		}
	}

	return return_value;
}

/******************************************************************************
 * @brief 
 * 
 * @param Frame_Format 
 * @param Slave_Address 
 * @param Function_Code 
 * @param Start_Address 
 * @param No_Of_Items 
 * @param aStart_Address 
 * @param aNo_Of_Items 
 * @param db 
 * @param Query_Frame 
 * @return uint32_t 
 *****************************************************************************/
uint32_t ModbusMasterQueryBuild( int8_t Frame_Format,
								 uint8_t Slave_Address,
								 uint8_t Function_Code,
								 uint16_t Start_Address,
								 uint16_t No_Of_Items,
								 uint16_t aStart_Address,
								 uint16_t aNo_Of_Items,
								 DataBlock_st const *db,
								 uint8_t *Query_Frame )
{
	uint16_t CRC16_Result;
	uint32_t ADU_Index = 0;
	uint32_t Loop_Count;

	uint16_t *Writing_Datas = NULL;

	switch( Function_Code )
	{
		case Modbus_FunCode_Force_Multiple_Coils:
			Writing_Datas = &db->Coils[ Start_Address - db->CoilsStart ];
			break;
		case Modbus_FunCode_Preset_Multiple_Reg:
			Writing_Datas = &db->Holdings[ Start_Address - db->HoldingsStart ];
			break;
		case Modbus_FunCode_Read_Write_4X_Reg:
			Writing_Datas = &db->Holdings[ aStart_Address - db->HoldingsStart ];
			break;
	}

	memset( FrameBin, 0, sizeof( FrameBin ) );

	if( Frame_Format == 'T' )
	{
		static uint16_t Transaction_ID = 0; /* use sequence number or rand() function if supported. */
		/* Transaction Identifier */
		FrameBin[ ADU_Index++ ] = ( uint8_t )( Transaction_ID >> 8 );
		FrameBin[ ADU_Index++ ] = ( uint8_t )( Transaction_ID & 0xff );
		Transaction_ID++;
		/* Protocol Identifier(Modbus Protocol = 0) */
		FrameBin[ ADU_Index++ ] = ( uint8_t )0;
		FrameBin[ ADU_Index++ ] = ( uint8_t )0;
		/* Length(Number of following bytes, 2bytes) */
		/*
		 Frame_Bin[ 4 ], Frame_Bin[ 5 ]
		 */
		ADU_Index++;
		ADU_Index++;
	}

	FrameBin[ ADU_Index++ ] = Slave_Address;
	FrameBin[ ADU_Index++ ] = Function_Code;
	FrameBin[ ADU_Index++ ] = ( uint8_t )( Start_Address >> 8 );
	FrameBin[ ADU_Index++ ] = ( uint8_t )Start_Address;
	FrameBin[ ADU_Index++ ] = ( uint8_t )( No_Of_Items >> 8 );
	FrameBin[ ADU_Index++ ] = ( uint8_t )No_Of_Items;
	if( Function_Code == Modbus_FunCode_Read_Write_4X_Reg )
	{
		FrameBin[ ADU_Index++ ] = ( uint8_t )( aStart_Address >> 8 );
		FrameBin[ ADU_Index++ ] = ( uint8_t )aStart_Address;
		FrameBin[ ADU_Index++ ] = ( uint8_t )( aNo_Of_Items >> 8 );
		FrameBin[ ADU_Index++ ] = ( uint8_t )aNo_Of_Items;
		FrameBin[ ADU_Index++ ] = ( uint8_t )( aNo_Of_Items << 1 ); /* Byte Count */
	}
	else if( Function_Code == Modbus_FunCode_Force_Multiple_Coils )
	{
		uint16_t bits_2_bytes = No_Of_Items >> 3; /* divided by 8 */
		if( No_Of_Items & 0x07 ) /* plus 1 if remainder that divided by 8 is greater than 0. */
		bits_2_bytes += 1;
		FrameBin[ ADU_Index++ ] = ( uint8_t )bits_2_bytes; /* Byte Count */
	}
	else if( Function_Code == Modbus_FunCode_Preset_Multiple_Reg )
		FrameBin[ ADU_Index++ ] = ( uint8_t )( No_Of_Items << 1 ); /* Byte Count */

	if( ( Function_Code == Modbus_FunCode_Force_Multiple_Coils ) ||
		( Function_Code == Modbus_FunCode_Preset_Multiple_Reg )
		||
		( Function_Code == Modbus_FunCode_Read_Write_4X_Reg ) )
	{
		uint16_t No_Of_Bytes = ( uint16_t )FrameBin[ ADU_Index - 1 ];
		if( Function_Code == Modbus_FunCode_Force_Multiple_Coils )
		{ /* LSB 1st */
			for( Loop_Count = 0; Loop_Count < No_Of_Bytes; ++Loop_Count )
			{
				if( Loop_Count & 1 ) /* Odd-numbered */
					FrameBin[ ADU_Index++ ] = ( uint8_t )( Writing_Datas[ Loop_Count >> 1 ] >> 8 );
				else
					/* Even-numbered */
					FrameBin[ ADU_Index++ ] = ( uint8_t )Writing_Datas[ Loop_Count >> 1 ];
			}
		}
		else
		{ /* MSB 1st */
			for( Loop_Count = 0; Loop_Count < No_Of_Bytes; ++Loop_Count )
			{
				if( Loop_Count & 1 ) /* Odd-numbered */
					FrameBin[ ADU_Index++ ] = ( uint8_t )Writing_Datas[ Loop_Count >> 1 ];
				else
					/* Even-numbered */
					FrameBin[ ADU_Index++ ] = ( uint8_t )( Writing_Datas[ Loop_Count >> 1 ] >> 8 );
			}
		}
	}

	if( Frame_Format == 'T' )
	{
		FrameBin[ 4 ] = ( uint8_t )( ( ADU_Index - 6 ) >> 8 );
		FrameBin[ 5 ] = ( uint8_t )( ADU_Index - 6 );
		/* Copy Data */
		for( Loop_Count = 0; Loop_Count < ADU_Index; ++Loop_Count )
			Query_Frame[ Loop_Count ] = FrameBin[ Loop_Count ];
	}
	else if( Frame_Format == 'A' )
	{
		uint8_t Calc_LRC = generate_lrc( FrameBin, ADU_Index );
		FrameBin[ ADU_Index++ ] = Calc_LRC;
		/* ASCII Framing */
		Query_Frame[ 0 ] = Modbus_ASCII_Start_Code;
		hex_to_ascii( FrameBin, ADU_Index, Query_Frame + 1 );
		ADU_Index = ( ADU_Index << 1 ) + 1;
		Query_Frame[ ADU_Index++ ] = ( uint8_t )( Modbus_ASCII_End_Code >> 8 ); /* CR */
		Query_Frame[ ADU_Index++ ] = ( uint8_t )( Modbus_ASCII_End_Code & 0xff ); /* LF */
	}
	else if( Frame_Format == 'R' )
	{
		CRC16_Result = CRC16( FrameBin, ADU_Index );
		FrameBin[ ADU_Index++ ] = ( uint8_t )CRC16_Result;
		FrameBin[ ADU_Index++ ] = ( uint8_t )( CRC16_Result >> 8 );
		/* Copy Data */
		for( Loop_Count = 0; Loop_Count < ADU_Index; ++Loop_Count )
			Query_Frame[ Loop_Count ] = FrameBin[ Loop_Count ];
	}

	return ADU_Index;
}

/******************************************************************************
 * @brief convert 2 bytes of ascii data to 1 byte of hex.
 * 
 * @param ascii 
 * @param NoOfChars 
 * @param hex 
 * @return uint16_t 
 *****************************************************************************/
static uint16_t ascii_to_hex( uint8_t const *ascii, uint16_t NoOfChars, uint8_t *hex )
{
	uint16_t LoopCount;
	uint8_t Nibble_Value = 0;

	for( LoopCount = 0; LoopCount < NoOfChars; ++LoopCount )
	{
		/* 1 Char to Nibble */
		if( ( ascii[ LoopCount ] >= '0' ) && ( ascii[ LoopCount ] <= '9' ) )
			Nibble_Value = ascii[ LoopCount ] - '0';
		else if( ( ascii[ LoopCount ] >= 'A' ) && ( ascii[ LoopCount ] <= 'F' ) )
			Nibble_Value = ascii[ LoopCount ] - 'A' + 10;

		/* 2 Chars to Byte */
		if( LoopCount & 1 )
			hex[ LoopCount >> 1 ] |= Nibble_Value; /* Odd No. */
		else
			hex[ LoopCount >> 1 ] = Nibble_Value << 4; /* Even No. */
	}

	return ( ( LoopCount + 1 ) >> 1 );
}

/******************************************************************************
 * @brief convert 1 byte of hex to 2 bytes of ascii
 * 
 * @param hex 
 * @param Bytes 
 * @param ascii 
 *****************************************************************************/
void hex_to_ascii( uint8_t const *hex, uint16_t Bytes, uint8_t *ascii )
{
	uint16_t ascii_cnt;
	uint8_t nibble;

	for( ascii_cnt = 0; ascii_cnt < ( Bytes << 1 ); ++ascii_cnt )
	{
		if( ascii_cnt & 1 )
			nibble = hex[ ascii_cnt >> 1 ] & 0x0F; /* Odd */
		else
			nibble = ( hex[ ascii_cnt >> 1 ] >> 4 ) & 0x0F; /* Even */

		if( nibble <= 9 )
			ascii[ ascii_cnt ] = nibble + '0'; /* 0 ~ 9 --> '0' ~ '9' */
		else
			ascii[ ascii_cnt ] = nibble + 'A' - 10; /* 10 ~ 15 --> 'A' ~ 'F' */
	}
}

/******************************************************************************
 * @brief 
 * 
 * @note  src_addr+2   src_addr+1   src_addr
 * @note  (1101 1001)  (0110 1100)  (0101 1010) (binary)
 * @note                FEDC BA98    7654 3210  (bit position)
 * @note  src start bit 3
 * @note  dst start bit 0
 * @note  bits to move  9
 * @note ----------------------------------------------------
 * @note  dst_addr+1   dst_addr
 * @note  (0000 0001) (1000 1011)
 * 
 * @param[in] src : source address -> uint8*
 * @param[in] src_start_bit_no : start bit of source data to move
 * @param[out] dest : destination address -> uint8*
 * @param[in] dest_start_bit_no : start bit of destination data from move
 * @param[in] bits : Total number of bits to move
 * @return uint32_t 
 *****************************************************************************/
static uint32_t bit_move_u8( uint8_t *src, int32_t src_start_bit_no, uint8_t *dest, int32_t dest_start_bit_no, int32_t bits )
{
	int32_t src_bit_index;
	int32_t src_byte_index;
	int32_t dest_bit_index;
	int32_t dest_byte_index;
	int32_t bit_count;
	int32_t gap_bit_index;
	uint8_t src_temp;
	uint8_t dest_temp;

	for( bit_count = 0; bit_count < bits; ++bit_count )
	{
		src_bit_index = ( src_start_bit_no + bit_count ) & 7UL;
		src_byte_index = ( src_start_bit_no + bit_count ) >> 3;
		dest_bit_index = ( dest_start_bit_no + bit_count ) & 7UL;
		dest_byte_index = ( dest_start_bit_no + bit_count ) >> 3;

		src_temp = src[ src_byte_index ] & ( 1 << src_bit_index );
		dest_temp = dest[ dest_byte_index ] & ~( 1 << dest_bit_index );

		gap_bit_index = src_bit_index - dest_bit_index;
		if( gap_bit_index > 0 )
			dest[ dest_byte_index ] = dest_temp | ( src_temp >> abs( gap_bit_index ) );
		else if( gap_bit_index < 0 )
			dest[ dest_byte_index ] = dest_temp | ( src_temp << abs( gap_bit_index ) );
		else
			dest[ dest_byte_index ] = dest_temp | src_temp;
	}

	/* return bytes to move */
	return ( bit_count + 7 ) / 8;
}

/******************************************************************************
 * @brief Generate LRC for MODBUS ASCII
 * 
 * @note 1. Add all bytes in the message, excluding the starting "colon" and ending CRLF. Add them into an 8bit field, so that carries will be discarded.
 * @note 2. NOT operation the final field value to produce the one's complement.
 * @note 3. Add 1 to produce the two's complement.
 * 
 * @param Source_Data 
 * @param Source_Data_Length 
 * @return uint8_t 
 *****************************************************************************/
static uint8_t generate_lrc( uint8_t *Source_Data, uint16_t Source_Data_Length )
{
	uint8_t Source_Sum = 0;
	uint16_t LoopCount;

	for( LoopCount = 0; LoopCount < Source_Data_Length; ++LoopCount )
	{
		Source_Sum = Source_Sum + Source_Data[ LoopCount ];
	}
	return ( ~Source_Sum + 1 ); /* Twos's complement */
}

/******************************************************************************
 * @brief CRC16-CCITT using word
 * 
 * @param fstData 
 * @param Bytes 
 * @return uint16_t 
 *****************************************************************************/
uint16_t CRC16_u16( uint8_t const *fstData, uint16_t Bytes )
{
	/* FAST CRC16 using CRC16 TABLE */

	uint8_t nTemp;
	uint16_t wCRCWord = 0xFFFF;

	while( Bytes )
	{
		Bytes--;
		nTemp = *fstData++ ^ wCRCWord;
		wCRCWord >>= 8;
		wCRCWord ^= wCRCTable[ nTemp ];
	}
	return ( wCRCWord );
}

/******************************************************************************
 * @brief CRC16-CCITT using bit-move
 * 
 * @param fstData 
 * @param Bytes 
 * @return uint16_t 
 *****************************************************************************/
uint16_t CRC16_bit( uint8_t const *fstData, uint16_t Bytes )
{
	/* CRC16 implemented by calculating every bit (SLOW) */
	uint16_t CRC_Value = 0xFFFF;
	uint16_t BMsg; /* 8-bit byte of message */
	uint16_t oldLSB;
	uint8_t pNt;
	static const uint16_t Polynominal = 0xA001;

	while( Bytes )
	{
		Bytes--;
		BMsg = ( uint16_t ) *fstData;
		CRC_Value = CRC_Value ^ BMsg;
		for( pNt = 0; pNt < 8; pNt++ )
		{
			oldLSB = CRC_Value & 1;
			CRC_Value = CRC_Value >> 1;
			if( oldLSB )
			{
				CRC_Value = CRC_Value ^ Polynominal;
			}
		}
		fstData++;
	}
	return CRC_Value;
}
