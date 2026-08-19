/******************************************************************************
 * @file a_mbslave.c
 * @author jylee1 (jylee1@gst-in.com)
 * @brief 
 * @version 0.1
 * @date 2020-08-26
 * 
 * @copyright Copyright (c) 2020
 * 
 *****************************************************************************/

/*****************************************************************************/
/** INCLUDES *****************************************************************/
/*****************************************************************************/

#include <UserApp.h>

/*****************************************************************************/
/** MACRO DEFINITIONS ********************************************************/
/*****************************************************************************/

#define PLC_LOOP_WRITE_LAST_OFFSET		( MBS_HR_CH1_SMPS_CONFIG - MBS_HR_CH1_SV_L )
#define PLC_LOOP_OUTPUT_MAX_OFFSET		( MBS_HR_CH1_OUTPUT_MAX - MBS_HR_CH1_SV_L )
#define PLC_LOOP_OUTPUT_MIN_OFFSET		( MBS_HR_CH1_OUTPUT_MIN - MBS_HR_CH1_SV_L )
#define PLC_SMPS_CONFIG_WRITABLE_MASK	( 0x01F8U )
#define PLC_RTU_MIN_REQUEST_BYTES		( 8U )

/*****************************************************************************/
/** DATATYPES ****************************************************************/
/*****************************************************************************/

/*****************************************************************************/
/** FUNCTION DECLARATIONS ****************************************************/
/*****************************************************************************/
static void update_slave_input_regs( void );
static void update_setting_from_slave_holding_regs( void );

static uint16_t modbus_slave( uint8_t * rcvd_frame, uint16_t length, uint8_t *tx_buf );
static Modbus_ExCode_et plc_protocol_validate_request( Modbus_Rx_st const *modbus_rx, uint16_t length );
static Modbus_ExCode_et plc_protocol_validate_write_holding( Modbus_Rx_st const *modbus_rx, uint16_t length );
static bool plc_word_in_i16_range( uint16_t word, int16_t min, int16_t max );
static bool plc_holding_addr_is_readable( uint16_t addr );
static bool plc_holding_addr_is_writable( uint16_t addr );
static bool plc_holding_word_value_is_valid( uint16_t addr, uint16_t value );
static bool plc_holding_write_combined_values_are_valid( Modbus_Rx_st const *modbus_rx );
static Modbus_ExCode_et plc_holding_write_state_validate( Modbus_Rx_st const *modbus_rx );
static bool plc_write_touches( Modbus_Rx_st const *modbus_rx, uint16_t start, uint16_t count );
static uint16_t plc_holding_after_write( Modbus_Rx_st const *modbus_rx, uint16_t addr );
static int32_t plc_dint_after_write( Modbus_Rx_st const *modbus_rx, uint16_t addr_l );
static uint16_t modbus_slave_resp_plc_preset_multiple_reg( uint8_t protocol, Modbus_Rx_st *modbus_rx, DataBlock_st const *db,
														   uint8_t *tx_buf );

/*****************************************************************************/
/** LOCAL VARIABLES **********************************************************/
/*****************************************************************************/

static uint16_t rcvd_len = 0;
static uint8_t rcvd_frame[ 256 ];
//static AppTimer_Instance_st timerUART;

/*****************************************************************************/
/** GLOBAL VARIABLES *********************************************************/
/*****************************************************************************/

Modbus_Server_st MBS = {
	.Id = NULL,
	.Run = modbus_slave,
	.UpdateInputs = update_slave_input_regs,
	.UpdateFromHoldings = update_setting_from_slave_holding_regs,
	.RxFrameCheck = MbSlaveCheckRequestFrame,
	.Function =
	{
		[ Modbus_FunCode_Read_Coil_Status ] = modbus_slave_resp_read_coil_status,
		[ Modbus_FunCode_Read_Input_Status ] = modbus_slave_resp_read_input_status,
		[ Modbus_FunCode_Read_Holding_Reg ] = modbus_slave_resp_read_holding_reg,
		[ Modbus_FunCode_Read_Input_Reg ] = modbus_slave_resp_read_input_reg,
		[ Modbus_FunCode_Force_Single_Coil ] = modbus_slave_resp_force_single_coil,
		[ Modbus_FunCode_Force_Holding_Reg ] = modbus_slave_resp_preset_holding_reg,
		[ Modbus_FunCode_Force_Multiple_Coils ] = modbus_slave_resp_force_multiple_coils,
		[ Modbus_FunCode_Preset_Multiple_Reg ] = modbus_slave_resp_plc_preset_multiple_reg,
		[ Modbus_FunCode_Read_Write_4X_Reg ] = modbus_slave_resp_read_write_4x_reg,
	},
};

/*****************************************************************************/
/** FUNCTION DEFINITIONS *****************************************************/
/*****************************************************************************/

/******************************************************************************
 * @brief Check whether a signed 16-bit Modbus word is in range.
 *****************************************************************************/
static bool plc_word_in_i16_range( uint16_t word, int16_t min, int16_t max )
{
	int16_t value = ( int16_t )word;

	return ( min <= value ) && ( value <= max );
}

/******************************************************************************
 * @brief Return the holding register value after applying the pending write.
 *****************************************************************************/
static uint16_t plc_holding_after_write( Modbus_Rx_st const *modbus_rx, uint16_t addr )
{
	if( ( modbus_rx->starting_address_write <= addr ) &&
		( addr < ( modbus_rx->starting_address_write + modbus_rx->no_of_registers_write ) ) )
	{
		return modbus_rx->data_write[ addr - modbus_rx->starting_address_write ];
	}

	return MBSDB.Holdings[ addr ];
}

/******************************************************************************
 * @brief Return a signed DINT value after applying the pending write.
 *****************************************************************************/
static int32_t plc_dint_after_write( Modbus_Rx_st const *modbus_rx, uint16_t addr_l )
{
	uint16_t word_l = plc_holding_after_write( modbus_rx, addr_l );
	uint16_t word_h = plc_holding_after_write( modbus_rx, addr_l + 1U );

	return ( int32_t )( ( ( uint32_t )word_h << 16 ) | word_l );
}

/******************************************************************************
 * @brief Check if any register in [start, start + count) is being written.
 *****************************************************************************/
static bool plc_write_touches( Modbus_Rx_st const *modbus_rx, uint16_t start, uint16_t count )
{
	uint16_t write_start = modbus_rx->starting_address_write;
	uint16_t write_end = write_start + modbus_rx->no_of_registers_write;
	uint16_t check_end = start + count;

	return ( write_start < check_end ) && ( start < write_end );
}

/******************************************************************************
 * @brief Check whether a holding register is readable in the PLC protocol.
 *****************************************************************************/
static bool plc_holding_addr_is_readable( uint16_t addr )
{
	if( addr <= MBS_HR_REVISION )
	{
		return true;
	}

	if( ( MBS_HR_CH1_SV_L <= addr ) && ( addr <= MBS_HR_CH20_SMPS_CONFIG ) )
	{
		return true;
	}

	return false;
}

/******************************************************************************
 * @brief Check whether a holding register is writable in the PLC protocol.
 *****************************************************************************/
static bool plc_holding_addr_is_writable( uint16_t addr )
{
	if( addr <= MBS_HR_ENABLE_CH11_20 )
	{
		return true;
	}

	if( ( MBS_HR_ALARM_RESET_CH1_10 <= addr ) && ( addr <= MBS_HR_SAVE_PARAMETER ) )
	{
		return true;
	}

	if( ( MBS_HR_CH1_SV_L <= addr ) && ( addr <= MBS_HR_CH20_SMPS_CONFIG ) )
	{
		uint16_t offset = ( addr - MBS_HR_CH1_SV_L ) % MBS_HR_CH_SPAN;

		return offset <= PLC_LOOP_WRITE_LAST_OFFSET;
	}

	if( ( MBS_HR_CH1_BYPASS_MV <= addr ) && ( addr <= MBS_HR_CH20_BYPASS_MV ) )
	{
		return true;
	}

	if( ( MBS_HR_CH1_PV_L <= addr ) && ( addr <= MBS_HR_CH20_PV_H ) )
	{
		return true;
	}

	return false;
}

/******************************************************************************
 * @brief Check a single holding register value against PLC protocol limits.
 *****************************************************************************/
static bool plc_holding_word_value_is_valid( uint16_t addr, uint16_t value )
{
	if( addr <= MBS_HR_ENABLE_CH11_20 )
	{
		if( addr <= MBS_HR_FAULT_NO_NC )
		{
			return value <= 1U;
		}
		return value <= 0x03FFU;
	}

	if( ( MBS_HR_ALARM_RESET_CH1_10 <= addr ) && ( addr <= MBS_HR_ALARM_RESET_CH11_20 ) )
	{
		return value <= 0x03FFU;
	}

	if( addr == MBS_HR_SAVE_PARAMETER )
	{
		return ( value == 0U ) || ( value == MBS_PARAMETER_SAVE_COMMAND ) || ( value == MBS_PARAMETER_INIT_COMMAND );
	}

	if( ( MBS_HR_CH1_SV_L <= addr ) && ( addr <= MBS_HR_CH20_SMPS_CONFIG ) )
	{
		uint16_t offset = ( addr - MBS_HR_CH1_SV_L ) % MBS_HR_CH_SPAN;

		switch( offset )
		{
			case MBS_HR_CH1_SV_L - MBS_HR_CH1_SV_L:
			case MBS_HR_CH1_SV_H - MBS_HR_CH1_SV_L:
				return true;
			case MBS_HR_CH1_ENABLE - MBS_HR_CH1_SV_L:
			case MBS_HR_CH1_AUTOTUNE_ENABLED - MBS_HR_CH1_SV_L:
			case MBS_HR_CH1_COOL_HEAT_CTRL_MODE - MBS_HR_CH1_SV_L:
				return value <= 1U;
			case MBS_HR_CH1_SAMPLING_PERIOD - MBS_HR_CH1_SV_L:
				return value <= 60000U;
			case MBS_HR_CH1_CONTROL_PERIOD - MBS_HR_CH1_SV_L:
				return ( MIN_CONTROL_PERIOD <= value ) && ( value <= 60000U );
			case MBS_HR_CH1_INPUT_TYPE - MBS_HR_CH1_SV_L:
				return ( value == ( uint16_t )INVALID_CH ) || ( value <= SEN_COMM );
			case MBS_HR_CH1_INPUT_CHANNEL - MBS_HR_CH1_SV_L:
			case MBS_HR_CH1_OUTPUT_CHANNEL - MBS_HR_CH1_SV_L:
				return ( value == ( uint16_t )INVALID_CH ) || ( value <= MAX_CONTROL_LOOP );
			case MBS_HR_CH1_OUTPUT_TYPE - MBS_HR_CH1_SV_L:
				return value <= OUT_SMPS_NHPP_1531;
			case MBS_HR_CH1_HIGH_OVER_ALARM - MBS_HR_CH1_SV_L:
			case MBS_HR_CH1_LOW_UNDER_ALARM - MBS_HR_CH1_SV_L:
				return value <= 20000U;
			case MBS_HR_CH1_OVER_CURR_ALARM - MBS_HR_CH1_SV_L:
				return value <= 10000U;
			case MBS_HR_CH1_CONTROL_TIMEOVER_ALARM - MBS_HR_CH1_SV_L:
				return value <= 3600U;
			case MBS_HR_CH1_TEMP_OFFSET - MBS_HR_CH1_SV_L:
				return plc_word_in_i16_range( value, -10000, 10000 );
			case MBS_HR_CH1_PB - MBS_HR_CH1_SV_L:
			case MBS_HR_CH1_TI - MBS_HR_CH1_SV_L:
			case MBS_HR_CH1_TD - MBS_HR_CH1_SV_L:
			case MBS_HR_CH1_SATURATED_I - MBS_HR_CH1_SV_L:
			case MBS_HR_CH1_SECOND_PGAIN - MBS_HR_CH1_SV_L:
				return value <= 60000U;
			case MBS_HR_CH1_SECOND_DGAIN - MBS_HR_CH1_SV_L:
				return value == 0U;
			case PLC_LOOP_OUTPUT_MAX_OFFSET:
			case PLC_LOOP_OUTPUT_MIN_OFFSET:
				return plc_word_in_i16_range( value, -1000, 1000 );
			case MBS_HR_CH1_PWM_FREQ - MBS_HR_CH1_SV_L:
				return ( 1U <= value ) && ( value <= 60000U );
			case MBS_HR_CH1_OUTPUT_DELAY - MBS_HR_CH1_SV_L:
				return value <= 999U;
			case MBS_HR_CH1_CONTROL_TYPE - MBS_HR_CH1_SV_L:
				return value <= CTRL_BYPASS;
			case MBS_HR_CH1_START_DELAY - MBS_HR_CH1_SV_L:
				return value <= 999U;
			case MBS_HR_CH1_SMPS_LEAKAGE_CURRENT - MBS_HR_CH1_SV_L:
				return true;
			case MBS_HR_CH1_SMPS_CONFIG - MBS_HR_CH1_SV_L:
				return ( value & ~PLC_SMPS_CONFIG_WRITABLE_MASK ) == 0U;
			default:
				return false;
		}
	}

	if( ( MBS_HR_CH1_BYPASS_MV <= addr ) && ( addr <= MBS_HR_CH20_BYPASS_MV ) )
	{
		return plc_word_in_i16_range( value, -10000, 10000 );
	}

	if( ( MBS_HR_CH1_PV_L <= addr ) && ( addr <= MBS_HR_CH20_PV_H ) )
	{
		return true;
	}

	return false;
}

/******************************************************************************
 * @brief Check DINT fields and inter-register constraints after pending write.
 *****************************************************************************/
static bool plc_holding_write_combined_values_are_valid( Modbus_Rx_st const *modbus_rx )
{
	for( uint16_t loop = 0; loop < MAX_CONTROL_LOOP; ++loop )
	{
		uint16_t base = MBS_HR_CH1_SV_L + loop * MBS_HR_CH_SPAN;

		if( plc_write_touches( modbus_rx, base, 2U ) )
		{
			int32_t sv = plc_dint_after_write( modbus_rx, base );
			if( ( sv < MINIMUM_PV ) || ( MAXIMUM_PV < sv ) )
			{
				return false;
			}
		}

		if( plc_write_touches( modbus_rx, base + PLC_LOOP_OUTPUT_MAX_OFFSET, 2U ) )
		{
			int16_t output_max = ( int16_t )plc_holding_after_write( modbus_rx,
																	  base + PLC_LOOP_OUTPUT_MAX_OFFSET );
			int16_t output_min = ( int16_t )plc_holding_after_write( modbus_rx,
																	  base + PLC_LOOP_OUTPUT_MIN_OFFSET );

			if( output_min >= output_max )
			{
				return false;
			}
		}

		uint16_t pv_base = MBS_HR_CH1_PV_L + loop * MBS_PV_WORDS;
		if( plc_write_touches( modbus_rx, pv_base, MBS_PV_WORDS ) )
		{
			int32_t pv = plc_dint_after_write( modbus_rx, pv_base );
			if( ( pv < MINIMUM_PV ) || ( MAXIMUM_PV < pv ) )
			{
				return false;
			}
		}
	}

	return true;
}

/******************************************************************************
 * @brief Reject output route changes while the target loop is enabled.
 *****************************************************************************/
static Modbus_ExCode_et plc_holding_write_state_validate( Modbus_Rx_st const *modbus_rx )
{
	static const uint16_t output_route_offsets[] = {
		MBS_HR_CH1_OUTPUT_TYPE - MBS_HR_CH1_SV_L,
		MBS_HR_CH1_OUTPUT_CHANNEL - MBS_HR_CH1_SV_L,
		MBS_HR_CH1_PWM_FREQ - MBS_HR_CH1_SV_L,
		MBS_HR_CH1_CONTROL_TYPE - MBS_HR_CH1_SV_L,
	};
	for( uint16_t loop = 0; loop < MAX_CONTROL_LOOP; ++loop )
	{
		uint16_t base = MBS_HR_CH1_SV_L + loop * MBS_HR_CH_SPAN;
		bool output_route_changes = false;

		for( uint16_t idx = 0; idx < ( sizeof( output_route_offsets ) / sizeof( output_route_offsets[ 0 ] ) ); ++idx )
		{
			uint16_t addr = base + output_route_offsets[ idx ];

			if( plc_write_touches( modbus_rx, addr, 1U ) &&
				( plc_holding_after_write( modbus_rx, addr ) != MBSDB.Holdings[ addr ] ) )
			{
				output_route_changes = true;
				break;
			}
		}

		if( output_route_changes )
		{
			uint16_t enable_addr = MBS_HR_CH1_ENABLE + loop * MBS_HR_CH_SPAN;
			uint16_t enable_mask_addr = ( loop < 10U ) ? MBS_HR_ENABLE_CH1_10 : MBS_HR_ENABLE_CH11_20;
			uint16_t enable_mask_bit = loop % 10U;
			bool loop_enabled_after_write = 0U != plc_holding_after_write( modbus_rx, enable_addr );
			bool mask_enabled_after_write = 0U != ( plc_holding_after_write( modbus_rx, enable_mask_addr )
														& ( 1U << enable_mask_bit ) );

			if( loop_enabled_after_write || mask_enabled_after_write )
			{
				return Modbus_ExCode_Illegal_DataValue;
			}
		}
	}

	return Modbus_ExCode_Normal;
}

/******************************************************************************
 * @brief Validate FC10 write request against PLC protocol map and value limits.
 *****************************************************************************/
static Modbus_ExCode_et plc_protocol_validate_write_holding( Modbus_Rx_st const *modbus_rx, uint16_t length )
{
	if( ( modbus_rx->byte_count != ( modbus_rx->no_of_registers_write << 1 ) ) ||
		( length != ( uint16_t )( 9U + modbus_rx->byte_count ) ) )
	{
		return Modbus_ExCode_Illegal_DataValue;
	}

	for( uint16_t offset = 0; offset < modbus_rx->no_of_registers_write; ++offset )
	{
		uint16_t addr = modbus_rx->starting_address_write + offset;

		if( !plc_holding_addr_is_writable( addr ) )
		{
			return Modbus_ExCode_Illegal_Address;
		}

		if( !plc_holding_word_value_is_valid( addr, modbus_rx->data_write[ offset ] ) )
		{
			return Modbus_ExCode_Illegal_DataValue;
		}
	}

	if( !plc_holding_write_combined_values_are_valid( modbus_rx ) )
	{
		return Modbus_ExCode_Illegal_DataValue;
	}

	Modbus_ExCode_et state_ex_code = plc_holding_write_state_validate( modbus_rx );
	if( Modbus_ExCode_Normal != state_ex_code )
	{
		return state_ex_code;
	}

	return Modbus_ExCode_Normal;
}

/******************************************************************************
 * @brief Validate a parsed request against the PLC protocol specification.
 *****************************************************************************/
static Modbus_ExCode_et plc_protocol_validate_request( Modbus_Rx_st const *modbus_rx, uint16_t length )
{
	switch( modbus_rx->function_code )
	{
		case Modbus_FunCode_Read_Holding_Reg:
			if( length != 8U )
			{
				return Modbus_ExCode_Illegal_DataValue;
			}

			for( uint16_t offset = 0; offset < modbus_rx->no_of_registers_read; ++offset )
			{
				if( !plc_holding_addr_is_readable( modbus_rx->starting_address_read + offset ) )
				{
					return Modbus_ExCode_Illegal_Address;
				}
			}
			return Modbus_ExCode_Normal;

		case Modbus_FunCode_Read_Input_Reg:
			if( length != 8U )
			{
				return Modbus_ExCode_Illegal_DataValue;
			}
			if( ( modbus_rx->starting_address_read + modbus_rx->no_of_registers_read - 1U ) >= MBS_INPUT_REGS_CNT )
			{
				return Modbus_ExCode_Illegal_Address;
			}
			return Modbus_ExCode_Normal;

		case Modbus_FunCode_Force_Holding_Reg:
			if( length != 8U )
			{
				return Modbus_ExCode_Illegal_DataValue;
			}
			if( !plc_holding_addr_is_writable( modbus_rx->starting_address_write ) )
			{
				return Modbus_ExCode_Illegal_Address;
			}
			if( !plc_holding_word_value_is_valid( modbus_rx->starting_address_write,
				modbus_rx->data_write[ 0 ] ) )
			{
				return Modbus_ExCode_Illegal_DataValue;
			}
			if( !plc_holding_write_combined_values_are_valid( modbus_rx ) )
			{
				return Modbus_ExCode_Illegal_DataValue;
			}
			return plc_holding_write_state_validate( modbus_rx );

		case Modbus_FunCode_Preset_Multiple_Reg:
			return plc_protocol_validate_write_holding( modbus_rx, length );

		default:
			return Modbus_ExCode_Illegal_FunCode;
	}
}

/******************************************************************************
 * @brief Apply PLC FC10 writes only to writable holding registers.
 *****************************************************************************/
static uint16_t modbus_slave_resp_plc_preset_multiple_reg( uint8_t protocol, Modbus_Rx_st *modbus_rx, DataBlock_st const *db,
														   uint8_t *tx_buf )
{
	uint32_t holding_last_addr = ( uint32_t )db->HoldingsStart + db->HoldingsCnt - 1U;

	for( uint16_t offset = 0; offset < modbus_rx->no_of_registers_write; ++offset )
	{
		uint16_t addr = modbus_rx->starting_address_write + offset;

		if( ( addr < db->HoldingsStart ) || ( holding_last_addr < addr ) )
		{
			continue;
		}

		if( !plc_holding_addr_is_writable( addr ) )
		{
			continue;
		}

		db->Holdings[ addr - db->HoldingsStart ] = modbus_rx->data_write[ offset ];
	}

	return MbSlaveBuildResponseFrame( protocol,
									  modbus_rx->MBAP_Header.Transaction_ID,
									  modbus_rx->MBAP_Header.Protocol_ID,
									  modbus_rx->MBAP_Header.Unit_ID,
									  Modbus_FunCode_Preset_Multiple_Reg,
									  modbus_rx->starting_address_write,
									  modbus_rx->no_of_registers_write << 1,
									  NULL,
									  tx_buf );
}

/******************************************************************************
 * @brief MODBUS Slave
 * 
 * @param rcvd_frame 
 * @param length 
 * @param tx_buf 
 * @return uint16_t 
 *****************************************************************************/
static uint16_t modbus_slave( uint8_t * rcvd_frame, uint16_t length, uint8_t *tx_buf )
{
	uint16_t tx_len = 0;
	Modbus_Rx_st modbus_rx = { 0 };
	Modbus_ExCode_et ex_code = Modbus_ExCode_Normal;

	if( length < PLC_RTU_MIN_REQUEST_BYTES )
	{
		return 0;
	}

	ex_code = MBS.RxFrameCheck( 'R', rcvd_frame, length, &MBSDB, &modbus_rx );

	if( ( Modbus_ExCode_Normal != ex_code ) &&
		( 0 == modbus_rx.MBAP_Header.Unit_ID ) &&
		( 0 == modbus_rx.function_code ) )
	{
		return 0;
	}

	if( modbus_rx.MBAP_Header.Unit_ID == 0 || modbus_rx.MBAP_Header.Unit_ID == DIO.GetBoardID() )
	{
		if( modbus_rx.function_code &&
			( modbus_rx.function_code != Modbus_FunCode_Read_Holding_Reg ) &&
			( modbus_rx.function_code != Modbus_FunCode_Read_Input_Reg ) &&
			( modbus_rx.function_code != Modbus_FunCode_Force_Holding_Reg ) &&
			( modbus_rx.function_code != Modbus_FunCode_Preset_Multiple_Reg ) )
		{
			return modbus_exception_response( 'R', &modbus_rx, tx_buf, Modbus_ExCode_Illegal_FunCode );
		}

		if( Modbus_ExCode_Normal == ex_code )
		{
			ex_code = plc_protocol_validate_request( &modbus_rx, length );
			if( Modbus_ExCode_Normal != ex_code )
			{
				tx_len = modbus_exception_response( 'R', &modbus_rx, tx_buf, ex_code );
			}
			else if( MBS.Function[ modbus_rx.function_code ] )
			{
				tx_len = MBS.Function[ modbus_rx.function_code ]( 'R', &modbus_rx, &MBSDB, tx_buf );
			}
			else
			{
				tx_len = modbus_exception_response( 'R', &modbus_rx, tx_buf, Modbus_ExCode_Illegal_FunCode );
			}
		}
		else
		{
			if( Modbus_ExCode_Slave_Failure != ex_code )
			{
				tx_len = modbus_exception_response( 'R', &modbus_rx, tx_buf, ex_code );
			}
		}
	}

	return tx_len;
}

/******************************************************************************
 * @brief
 *
 *****************************************************************************/
static void update_slave_input_regs( void )
{
	for( int loop_idx = 0; loop_idx < MAX_CONTROL_LOOP; ++loop_idx )
	{
		// PV Update
		int32_t PV = Controller.GetPV( loop_idx );
		uint16_t pv_l_ir_idx = MBS_IR_CH1_PV_L + loop_idx * MBS_PV_WORDS;
		uint16_t pv_h_ir_idx = MBS_IR_CH1_PV_H + loop_idx * MBS_PV_WORDS;
		MBSDB.Inputs[ pv_h_ir_idx ] = ( PV >> 16 ) & 0xFFFF;
		MBSDB.Inputs[ pv_l_ir_idx ] = PV & 0xFFFF;

		// MV Update
		MBSDB.Inputs[ MBS_IR_CH1_MV + loop_idx ] = ( int16_t )( Controller.GetMV( loop_idx ) * MULTIPLY_FLOAT_TO_PERMIL );

		// Fault Status Update
		MBSDB.Inputs[ MBS_IR_CH1_FAULT + loop_idx ] = Controller.GetFault( loop_idx ).All;

		if( MBSDB.Inputs[ MBS_IR_CH1_FAULT + loop_idx ] )
		{
			MBSDB.Inputs[ MBS_IR_FAULT0 + loop_idx / 10 ] |= 1 << ( loop_idx % 10 );
		}
		else
		{
			MBSDB.Inputs[ MBS_IR_FAULT0 + loop_idx / 10 ] &= ~( 1 << ( loop_idx % 10 ) );
		}

		//SMPS Status Update
		uint8_t smps_id = Controller.GetConfig( loop_idx )->OutputChannel;
		if( 0 < smps_id && smps_id < MAX_SMPS_ID )
		{
			switch( Controller.GetConfig( loop_idx )->OutputType )
			{
				case OUT_SMPS_CHPP_8021:
					uint16_t * dst = &MBSDB.Inputs[ MBS_IR_CH1_SMPS_SET_OUTPUT_POWER + loop_idx * MBS_IR_SMPS_CH_SPAN ];
					uint16_t * src = &MBM_READ_DB[ smps_id ].Holdings[ MBM_HR_CHPP_8021_OUTPUT_POWER ];
					memcpy( dst, src, MBM_HR_CHPP_8021_REGS_CNT * sizeof( uint16_t ) );
					break;
				case OUT_SMPS_CHPP_5521:
					MBSDB.Inputs[ MBS_IR_CH1_SMPS_STATUS + loop_idx * MBS_IR_SMPS_CH_SPAN ] = MBM_READ_DB[ smps_id ].Holdings[ MBM_HR_CHPP_5521_STATUS_OF_SMPS ];
					MBSDB.Inputs[ MBS_IR_CH1_SMPS_MEASURED_OUTPUT_POWER + loop_idx * MBS_IR_SMPS_CH_SPAN ] = MBM_READ_DB[ smps_id ].Holdings[ MBM_HR_CHPP_5521_OUTPUT_CURRENT ];
					MBSDB.Inputs[ MBS_IR_CH1_SMPS_FAULT + loop_idx * MBS_IR_SMPS_CH_SPAN ] = MBM_READ_DB[ smps_id ].Holdings[ MBM_HR_CHPP_5521_FAULT_STATUS_OF_SMPS ];
					break;
				default:
					break;
			}
		}
	}
}

static uint16_t old_settings[ 2 ] = { 0 };

/******************************************************************************
 * @brief 
 * 
 *****************************************************************************/
static void propagate_enable_settings_to_each_loop( void )
{
	if( old_settings[ 0 ] != MBSDB.Holdings[ MBS_HR_ENABLE_CH1_10 ] )
	{
		old_settings[ 0 ] = MBSDB.Holdings[ MBS_HR_ENABLE_CH1_10 ];
		for( int idx = 0; idx < 10; ++idx )
		{
			MBSDB.Holdings[ MBS_HR_CH1_ENABLE + idx * MBS_HR_CH_SPAN ] = !!( old_settings[ 0 ] & ( 1 << idx ) );
		}
	}
	if( old_settings[ 1 ] != MBSDB.Holdings[ MBS_HR_ENABLE_CH11_20 ] )
	{
		old_settings[ 1 ] = MBSDB.Holdings[ MBS_HR_ENABLE_CH11_20 ];
		for( int idx = 0; idx < 10; ++idx )
		{
			MBSDB.Holdings[ MBS_HR_CH11_ENABLE + idx * MBS_HR_CH_SPAN ] = !!( old_settings[ 1 ] & ( 1 << idx ) );
		}
	}
}

/******************************************************************************
 * @brief 
 * 
 *****************************************************************************/
void merge_enable_settings_from_each_loop( void )
{
	uint16_t settings[ 2 ] = { 0 };
	for( int loop_idx = 0; loop_idx < 10; ++loop_idx )
	{
		settings[ 0 ] |= ( !!MBSDB.Holdings[ MBS_HR_CH1_ENABLE + loop_idx * MBS_HR_CH_SPAN ] ) << loop_idx;
		settings[ 1 ] |= ( !!MBSDB.Holdings[ MBS_HR_CH11_ENABLE + loop_idx * MBS_HR_CH_SPAN ] ) << loop_idx;
	}

	MBSDB.Holdings[ MBS_HR_ENABLE_CH1_10 ] = settings[ 0 ];
	MBSDB.Holdings[ MBS_HR_ENABLE_CH11_20 ] = settings[ 1 ];
	old_settings[ 0 ] = MBSDB.Holdings[ MBS_HR_ENABLE_CH1_10 ];
	old_settings[ 1 ] = MBSDB.Holdings[ MBS_HR_ENABLE_CH11_20 ];
}

/******************************************************************************
 * @brief
 *
 *****************************************************************************/
void save_parameter()
{
	for( int idx = 0; idx < MAX_CONTROL_LOOP; ++idx )
	{
		memcpy( &CFG.ch[ idx ], Controller.GetConfig( idx ), sizeof( CFG.ch[ idx ] ) );
	}

	CFG.system.Run = !!MBSDB.Holdings[ MBS_HR_RUN ];
	CFG.system.FaultRelayNc = !!MBSDB.Holdings[ MBS_HR_FAULT_NO_NC ];

	if( EEPR.Status->initiated )
	{
		EEPR.Write( 0, &CFG, sizeof( CFG ) );
	}
}

/******************************************************************************
 * @brief
 *
 *****************************************************************************/
void save_default_channels()
{
	for( int idx = 0; idx < MAX_CONTROL_LOOP; ++idx )
	{
		Controller.SetDefault( idx );
		memcpy( &CFG.ch[ idx ], Controller.GetConfig( idx ), sizeof( CFG.ch[ idx ] ) );
		memcpy( &MBSDB.Holdings[ MBS_HR_CH1_SV_L + idx * MBS_HR_CH_SPAN ], &CFG.ch[ idx ], sizeof( CFG.ch[ idx ] ) );
	}

	if( EEPR.Status->initiated )
	{
		EEPR.Write( 0, ( uint8_t* ) &CFG, sizeof( CFG ) );
	}
}

/******************************************************************************
 * @brief
 *
 *****************************************************************************/
static void update_setting_from_slave_holding_regs( void )
{
	static uint16_t last_save_command = 0;

	// ALARM Reset
	Controller.ClearAlarm( MBSDB.Holdings[ MBS_HR_ALARM_RESET_CH11_20 ], MBSDB.Holdings[ MBS_HR_ALARM_RESET_CH1_10 ] );
	MBSDB.Holdings[ MBS_HR_ALARM_RESET_CH1_10 ] = 0;
	MBSDB.Holdings[ MBS_HR_ALARM_RESET_CH11_20 ] = 0;

	propagate_enable_settings_to_each_loop();

	// Control Loop Configuration Update
	for( int loop_idx = 0; loop_idx < MAX_CONTROL_LOOP; ++loop_idx )
	{
		Controller.SetConfig( &MBSDB.Holdings[ MBS_HR_CH1_SV_L + loop_idx * MBS_HR_CH_SPAN ], loop_idx );

		uint16_t ch = MBSDB.Holdings[ MBS_HR_CH1_INPUT_CHANNEL + loop_idx * MBS_HR_CH_SPAN ];
		uint16_t type = MBSDB.Holdings[ MBS_HR_CH1_INPUT_TYPE + loop_idx * MBS_HR_CH_SPAN ];
		uint16_t sps = MBSDB.Holdings[ MBS_HR_CH1_SAMPLING_PERIOD + loop_idx * MBS_HR_CH_SPAN ];

		if( ( INVALID_CH != ch ) && ( type != SEN_COMM ) && ( Calib.status.On == 0 ) )
		{
			if( MBSDB.Holdings[ MBS_HR_CH1_INPUT_CHANNEL + loop_idx * MBS_HR_CH_SPAN ] < MAX_TEMP_CHANNEL )
			{
				bool test_override_active = TestFunc.IsSensorOverrideActive( ( uint8_t )ch )
					&& ( 0 == MBSDB.Holdings[ MBS_HR_RUN ] )
					&& ( 0 == CFG.system.Run );
				if( !test_override_active )
				{
					Temp.SetType( ch, type, sps );
				}
			}
			else
			{
				RemoteIO.SetTempCfg( ch, type, sps );
			}
		}
	}

	merge_enable_settings_from_each_loop();

	// EEPROM Save
	if( ( 0 != MBSDB.Holdings[ MBS_HR_RUN ] ) && ( 0 == CFG.system.Run ) )
	{
		save_parameter();
	}
	else
	{
		if( last_save_command != MBSDB.Holdings[ MBS_HR_SAVE_PARAMETER ] )
		{
			last_save_command = MBSDB.Holdings[ MBS_HR_SAVE_PARAMETER ];

			switch( MBSDB.Holdings[ MBS_HR_SAVE_PARAMETER ] )
			{
				case MBS_PARAMETER_SAVE_COMMAND:
					save_parameter();
					break;
				case MBS_PARAMETER_INIT_COMMAND:
					if( !!MBSDB.Holdings[ MBS_HR_RUN ] )
					{
						save_default_channels();
					}
					break;
				default:
					break;
			}
		}
	}

	CFG.system.Run = !!MBSDB.Holdings[ MBS_HR_RUN ];
	CFG.system.FaultRelayNc = !!MBSDB.Holdings[ MBS_HR_FAULT_NO_NC ];
}

/******************************************************************************
 * @brief Modbus Server for PLC Communication
 *
 *****************************************************************************/
void ModbusSlaveTask( void )
{
	static uint16_t old_len = 0;
	static AppTimerData_ut timerModbusFrame = { 0 };
	static uint32_t old_BaudRate = 0;
	static float rtu_frame_time = RTU_FRAME_IDLE_MIN;

	MBS.UpdateInputs();

	// parse completed frame before reading new data, so the next packet starts at rcvd_frame[0]
	if( AppTimer.IsRun( &timerModbusFrame ) && AppTimer.IsExpired( &timerModbusFrame ) )
	{
		uint8_t tx_buf[ 512 ];
		uint16_t tx_len = 0;

		AppTimer.Stop( &timerModbusFrame );

		tx_len = MBS.Run( rcvd_frame, rcvd_len, tx_buf );

		if( tx_len )
		{
			UART.Write( UART_PLC, tx_buf, tx_len );
		}

		memset( rcvd_frame, 0, sizeof( rcvd_frame ) );
		rcvd_len = 0;
		old_len = 0;
		UART.PORT[ UART_PLC ]->stat.received = 0;
	}

	// receive data for the current frame
	rcvd_len += UART.Read( UART_PLC, &rcvd_frame[ rcvd_len ], sizeof( rcvd_frame ) - rcvd_len );

	if( ( rcvd_len > 0 ) && ( rcvd_len != old_len ) )
	{
		if( old_BaudRate !=  UART.PORT[ UART_PLC ]->handle->Init.BaudRate )
		{
			old_BaudRate =  UART.PORT[ UART_PLC ]->handle->Init.BaudRate;
			rtu_frame_time = RTU_FRAME_IDLE_MIN;

			if( old_BaudRate <= 19200 )
			{
				rtu_frame_time = MAX_BITS_PER_CHAR / old_BaudRate * RTU_FRAME_IDLE_CHAR * SAFETY_FACTOR;
			}
		}

		AppTimer.Start( &timerModbusFrame, rtu_frame_time );
		old_len = rcvd_len;
	}

	MBS.UpdateFromHoldings();
}
