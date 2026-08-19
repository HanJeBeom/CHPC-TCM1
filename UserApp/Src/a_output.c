/******************************************************************************
 * @file a_output.c
 * @author Seo Yujeong (yjseo@gst-in.com)
 * @brief 
 * @version 0.1
 * @date 2023-07-26
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

#include "UserApp.h"

/*****************************************************************************/
/** MACRO DEFINITIONS ********************************************************/
/*****************************************************************************/

#define TIMER_CH_OUTPUT				( 0.02f )
#define NEXT_CHANNEL_GAP			( 30 )
#define MV_FULL_SCALE_RANGE			( 2.0000f )
#define MV_HALF_SCALE_RANGE			( 1.0000f )
#define REMOTE_FULL_SCALE_RANGE		( 8000UL )
#define OUT_POWER_RANGE_CHPP_8021	( 4000.0f )				// -2000 ~ 2000W
#define OUT_CURRENT_RANGE_CHPP_5521	( 24.0f )

#define INTERNAL_DAC				( 0 )
#define EXTERNAL_DAC				( 1 )

#define SMPS_CMD_STATUS_RUN			( 1 << 1 )
#define SMPS_CMD_STATUS_STOP		( 0 << 1 )
#define SMPS_CMD_STATUS_RESET		( 1 << 2 )
#define SMPS_DEFAULT_CH_ON			( 0xf << 3 )

#define ENABLE_TRANSFER_FUNCTION_FOR_SMPS_LOW_POWER_RANGE_AND_RESPONSE_DELAY	( 1 )

/*****************************************************************************/
/** DATATYPES ****************************************************************/
/*****************************************************************************/

typedef enum Remote_AO_Range_enum
{
	REMOTE_AO_RANGE_INVAILID = 0,
	REMOTE_AO_RANGE_0_5V = 1,
	REMOTE_AO_RANGE_0_10V = 2,
	REMOTE_AO_RANGE_0_20mA = 3,
	REMOTE_AO_RANGE_1_5V = 4,
	REMOTE_AO_RANGE_4_20mA = 7,
	REMOTE_AO_RANGE_0_24mA = 8,
	REMOTE_AO_RANGE_P_N_5V = 11,
	REMOTE_AO_RANGE_P_N_10V = 12,
	REMOTE_AO_RANGE_NOT_RECEIVED = 0xFF,
}Remote_AO_Range_et;

typedef struct output_struct
{
	uint8_t (*SetCfg)( output_et out_type, int8_t out_ch, ... );
	void (*Run)( output_et out_type, int8_t out_ch, float MV, ... );
	uint8_t (*Stop)( output_et out_type, int8_t out_ch, ... );
} output_st;

/*****************************************************************************/
/** FUNCTION DECLARATIONS ****************************************************/
/*****************************************************************************/
static Remote_AO_Range_et match_remote_ao_range( output_et range );
static dac_vi_set_et match_dac_range( output_et range );
static uint16_t MV_to_dacValue( output_et range, float MV, uint8_t external );
static float MV_to_PWMduty( float MV );
static int16_t Q_format( float num, uint8_t q );
static int16_t cal_smps_mv( output_et out_type, float MV );
void Reset_smps_comm( output_et out_type, uint8_t smps_id );
static uint8_t make_smps_mbm_query( uint8_t smps_id, output_et out_type, smps_func_et smps_func, float MV, uint16_t leakagecurrent, uint16_t smps_status );
static void funcSet_StartStop( output_et type, output_st* pOut );
static void funcSet_Run( output_et type, output_st* pOut );

static __RAM_FUNC uint8_t set_dac( output_et out_type, int8_t out_ch, ... );
static __RAM_FUNC void run_dac( output_et dac_range, int8_t out_ch, float mv, ... );
static __RAM_FUNC uint8_t stop_dac( output_et type, int8_t out_ch, ... );
static __RAM_FUNC uint8_t set_pwm( output_et out_type, int8_t out_ch, ... );
static __RAM_FUNC void run_pwm( output_et out_type, int8_t out_ch, float mv, ... );
static __RAM_FUNC uint8_t stop_pwm( output_et type, int8_t out_ch, ... );
static __RAM_FUNC uint8_t set_smps_comm( output_et out_type, int8_t out_ch, ... );
static __RAM_FUNC void run_smps_comm( output_et out_type, int8_t out_ch, float smps_mv, ... );
static __RAM_FUNC uint8_t stop_smps_comm( output_et out_type, int8_t out_ch, ... );
static __RAM_FUNC uint8_t set_smps_analog( output_et out_type, int8_t out_ch, ... );
static __RAM_FUNC void run_smps_analog( output_et out_type, int8_t out_ch, float smps_mv, ... );
static __RAM_FUNC uint8_t stop_smps_analog( output_et out_type, int8_t out_ch, ... );

/*****************************************************************************/
/** GLOBAL VARIABLES *********************************************************/
/*****************************************************************************/

/*****************************************************************************/
/** LOCAL VARIABLES **********************************************************/
/*****************************************************************************/
smps_cmd_st SMPSCMD[ MAX_SMPS_CMD ];			// read, write
static output_st OUTPUT[ MAX_CONTROL_LOOP ];

/*****************************************************************************/
/** FUNCTION DEFINITIONS *****************************************************/
/*****************************************************************************/

/******************************************************************************
 * @brief 
 * 
 * @param range 
 * @return Remote_AO_Range_et 
 *****************************************************************************/
static Remote_AO_Range_et match_remote_ao_range( output_et range )
{
	switch( range )
	{
		case OUT_V_P0_P5:
			return REMOTE_AO_RANGE_0_5V;
		case OUT_V_P0_P10:
			return REMOTE_AO_RANGE_0_10V;
		case OUT_V_M5_P5:
			return REMOTE_AO_RANGE_P_N_5V;
		case OUT_V_M10_P10:
			return REMOTE_AO_RANGE_P_N_10V;
		case OUT_A_4M_20M:
			return REMOTE_AO_RANGE_4_20mA;
		case OUT_A_0M_20M:
			return REMOTE_AO_RANGE_0_20mA;
		case OUT_A_0M_24M:
			return REMOTE_AO_RANGE_0_24mA;
		default:
			return REMOTE_AO_RANGE_INVAILID;
	}
}

/******************************************************************************
 * @brief 
 * 
 * @param range 
 * @return dac_vi_set_et 
 *****************************************************************************/
static dac_vi_set_et match_dac_range( output_et range )
{
	switch( range )
	{
		case OUT_V_P0_P5:
			return DAC_V_0_to_5;
		case OUT_V_P0_P10:
			return DAC_V_0_to_10;
		case OUT_V_M5_P5:
			return DAC_V_M5_to_5;
		case OUT_V_M10_P10:
			return DAC_V_M10_to_10;
		case OUT_A_4M_20M:
			return DAC_I_4_to_20;
		case OUT_A_0M_20M:
			return DAC_I_0_to_20;
		case OUT_A_0M_24M:
			return DAC_I_0_to_24;
		default:
			return DAC_SET_ERR;
	}
}

/******************************************************************************
 * @brief 
 * 
 * @param range 
 * @param MV 
 * @param external 
 * @return uint16_t 
 *****************************************************************************/
static uint16_t MV_to_dacValue( output_et range, float MV, uint8_t external )
{
	uint16_t value = 0;
	switch( range )
	{
		case OUT_V_P0_P5:
		case OUT_V_P0_P10:
		case OUT_A_4M_20M:
		case OUT_A_0M_20M:
		case OUT_A_0M_24M:
			if( !( 0 > MV ) )
			{
				if( external == EXTERNAL_DAC )
				{
					value = ( uint16_t )( MV * REMOTE_FULL_SCALE_RANGE / MV_HALF_SCALE_RANGE );
				}
				else
				{
					value = ( uint16_t )( MV * AD5422_DATA_MAX_VALUE / MV_HALF_SCALE_RANGE );
				}
			}
			break;

		case OUT_V_M5_P5:
		case OUT_V_M10_P10:
			if( external == EXTERNAL_DAC )
			{
				MV += MV_HALF_SCALE_RANGE;	//-1.0~1.0을 0.0~2.0으로 scale 변경
				value = ( uint16_t )( MV * REMOTE_FULL_SCALE_RANGE / MV_FULL_SCALE_RANGE );
			}
			else
			{
				MV += MV_HALF_SCALE_RANGE;
				value = ( uint16_t )( MV * AD5422_DATA_MAX_VALUE / MV_FULL_SCALE_RANGE );
			}
			break;

		default:
			break;
	}
	return value;
}

/******************************************************************************
 * @brief 
 * 
 * @param MV 
 * @return float 
 *****************************************************************************/
static float MV_to_PWMduty( float MV )
{
	float duty = 0.0f;

	if( !( 0 > MV ) )
	{
		duty = ( float )MV / MV_HALF_SCALE_RANGE;
	}

	return duty;
}

/******************************************************************************
 * @brief 
 * 
 * @param num 
 * @param q 
 * @return int16_t 
 *****************************************************************************/
static int16_t Q_format( float num, uint8_t q )
{
	return num * ( 1 << q );
}

/******************************************************************************
 * @brief 
 * 
 * @param out_type 
 * @param MV 
 * @return int16_t 
 *****************************************************************************/
static int16_t cal_smps_mv( output_et out_type, float MV )
{
	int16_t result = 0;
	switch( out_type )
	{
		case OUT_SMPS_CHPP_5521:
			result = Q_format( MV * OUT_CURRENT_RANGE_CHPP_5521 / MV_FULL_SCALE_RANGE, 7 );
			break;
		case OUT_SMPS_CHPP_8021:
			result = Q_format( MV * OUT_POWER_RANGE_CHPP_8021 / MV_FULL_SCALE_RANGE, 0 );
			break;
		case OUT_SMPS_NHPP_2032:
			break;
		case OUT_SMPS_NHPP_1531:
			break;
		default:
			break;
	}
	return result;
}

/******************************************************************************
 * @brief 
 * @param 
 * @return uint16_t 
 *****************************************************************************/
uint16_t set_smps_status( output_et out_type, smps_func_et smps_func, uint16_t smps_status )
{
	if( out_type == OUT_SMPS_CHPP_8021 )
	{
		switch( smps_func )
		{
			case SMPS_CMD_RUN:
			case SMPS_CMD_WRITE:
				smps_status |= SMPS_CMD_STATUS_RUN;
				break;
			case SMPS_CMD_STOP:
				smps_status &= ~SMPS_CMD_STATUS_RUN;
				break;
			case SMPS_CMD_RESET:
				smps_status &= ~SMPS_CMD_STATUS_RUN;
				smps_status |= SMPS_CMD_STATUS_RESET;
				break;
			default:
		}
	}
	return smps_status;
}

/******************************************************************************
 * @brief Reet the smps comm object
 * status가 stop일때만 reset 명령 전송
 * @param reset_flag
 * @return uint8_t 
 *****************************************************************************/
void Reset_smps_comm( output_et out_type, uint8_t smps_id )
{
	if( out_type == OUT_NONE ) return;

	uint8_t idx = smps_id * 2;
	SMPSCMD[ idx ].smps_type = out_type;
	SMPSCMD[ idx ].smps_id = smps_id;
	SMPSCMD[ idx ].smps_func = SMPS_CMD_RESET;
	SMPSCMD[ idx ].mbm_addr = MBS_HOLDING_START + 1;
	SMPSCMD[ idx ].mbm_qty = 1;
	SMPSCMD[ idx ].DBlock = &MBM_WRITE_DB[ smps_id ];
	SMPSCMD[ idx ].mbm_fcode = Modbus_FunCode_Preset_Multiple_Reg;
	SMPSCMD[ idx ].DBlock->Holdings[ SMPSCMD[ idx ].mbm_addr ] = set_smps_status( out_type, SMPS_CMD_RESET, 0 );
}

/******************************************************************************
 * @brief Set smps command
 * 
 * @param slaveid 
 * @param out_type 
 * @param focde
 * @return 1 : success, 0 : fail
 *****************************************************************************/
static uint8_t make_smps_mbm_query( uint8_t smps_id, output_et out_type, smps_func_et smps_func, float MV, uint16_t leakagecurrent, uint16_t smps_status )
{
	uint8_t idx = smps_id * 2;
	
	if( smps_func == SMPS_CMD_READ )
	{
		if ( smps_id == BRAODCAST_ID ) return 0;
		idx = smps_id * 2 + 1;
	}

	if( SMPSCMD[ idx ].smps_func == SMPS_CMD_NONE )
	{
		SMPSCMD[ idx ].smps_type = out_type;
		SMPSCMD[ idx ].smps_id = smps_id;
		SMPSCMD[ idx ].smps_func = smps_func;
		SMPSCMD[ idx ].mbm_addr = MBS_HOLDING_START;

		switch( smps_func )
		{
			case SMPS_CMD_RUN:
			case SMPS_CMD_STOP:
			case SMPS_CMD_RESET:
			case SMPS_CMD_WRITE:
				SMPSCMD[ idx ].mbm_qty = 3;
				SMPSCMD[ idx ].DBlock = &MBM_WRITE_DB[ smps_id ];
				SMPSCMD[ idx ].mbm_fcode = Modbus_FunCode_Preset_Multiple_Reg;
				SMPSCMD[ idx ].DBlock->Holdings[ SMPSCMD[ idx ].mbm_addr ] = cal_smps_mv( out_type, MV );
				SMPSCMD[ idx ].DBlock->Holdings[ SMPSCMD[ idx ].mbm_addr + 1 ] = set_smps_status( out_type, smps_func, smps_status );
				SMPSCMD[ idx ].DBlock->Holdings[ SMPSCMD[ idx ].mbm_addr + 2 ] = Q_format( leakagecurrent, 7 );
				break;
			case SMPS_CMD_READ:
				SMPSCMD[ idx ].mbm_qty = MBM_HR_CHPP_5521_REGS_CNT;
				if( out_type == OUT_SMPS_CHPP_8021 )
				{
					SMPSCMD[ idx ].mbm_qty = MBM_HR_CHPP_8021_REGS_CNT;
				}
				SMPSCMD[ idx ].DBlock = &MBM_READ_DB[ smps_id ];
				SMPSCMD[ idx ].mbm_fcode = Modbus_FunCode_Read_Holding_Reg;
				break;
			default:
				return 0;
		}
	}

	return 1;
}

/******************************************************************************
 * @brief Queue a read-only SMPS status request for diagnostics/logging.
 *        No RUN, STOP, RESET, or output-power write is generated here.
 *****************************************************************************/
uint8_t OutputRequestSmpsRead( output_et out_type, uint8_t smps_id )
{
	if( ( smps_id < 1 ) || ( smps_id >= MAX_SMPS_ID ) ) return 0;
	if( ( OUT_SMPS_CHPP_8021 != out_type ) && ( OUT_SMPS_CHPP_5521 != out_type ) ) return 0;

	return make_smps_mbm_query( smps_id, out_type, SMPS_CMD_READ, 0.0f, 0, 0 );
}

/******************************************************************************
 * @brief Set the dac object
 * 
 * @param out_type 
 * @param out_ch 
 * @param status 
 * @return uint8_t 
 *****************************************************************************/
static uint8_t set_dac( output_et out_type, int8_t out_ch, ... )
{
	uint8_t complete = 0;

	if( INVALID_CH != out_ch )
	{
		if( out_ch < MAX_DAC_CH )
		{
			if( AD5422.SetupRange( out_ch, match_dac_range( out_type ) ) != DAC_SET_ERR )
			{
				complete = 1;
			}
		}
		else if( out_ch < MAX_CONTROL_LOOP )
		{
			RemoteIO.SetAoRange( out_ch, match_remote_ao_range( out_type ) );
			complete = 1;
		}
	}

	return complete;
}

/******************************************************************************
 * @brief 
 * 
 * @param dac_range 
 * @param out_ch 
 * @param mv 
 *****************************************************************************/
static void run_dac( output_et dac_range, int8_t out_ch, float mv, ... )
{
	if( INVALID_CH != out_ch )
	{
		if( out_ch < MAX_DAC_CH )
		{
			AD5422.Output( out_ch, MV_to_dacValue( dac_range, mv, INTERNAL_DAC ) );
		}
		else if( out_ch < MAX_CONTROL_LOOP )
		{
			static AppTimerData_ut timer_can_tx_ao = {0};

			if( AppTimer.IsExpired( &timer_can_tx_ao ) )
			{
				AppTimer.Start( &timer_can_tx_ao, TIMER_CH_OUTPUT );

				RemoteIO.SetAoData( out_ch, MV_to_dacValue( dac_range, mv, EXTERNAL_DAC ) );
			}
		}
	}
}

/******************************************************************************
 * @brief 
 * 
 * @param type 
 * @param out_ch 
 * @return uint8_t 
 *****************************************************************************/
static uint8_t stop_dac( output_et type, int8_t out_ch, ... )
{
	uint8_t complete = 0;

	if( INVALID_CH != out_ch )
	{
		if( out_ch < MAX_DAC_CH )
		{
			if( type == OUT_V_M5_P5 || type == OUT_V_M10_P10 )
			{
				AD5422.Output( out_ch, AD5422_DATA_MAX_VALUE / 2 );
			}
			else
			{
				AD5422.Output( out_ch, 0 );
			}
			complete = 1;
		}
		else if( out_ch < MAX_CONTROL_LOOP )
		{
			RemoteIO.SetAoData( out_ch, MV_to_dacValue( type, 0, EXTERNAL_DAC ) );
			complete = 1;
		}
	}
	return complete;
}

/******************************************************************************
 * @brief Set the pwm object
 * 
 * @param out_type 
 * @param out_ch 
 * @param pwmfreq unit : 0.1Hz
 * @param status 
 * @return uint8_t 
 *****************************************************************************/
static uint8_t set_pwm( output_et out_type, int8_t out_ch, ... )
{
	uint8_t complete = 0;
	va_list argptr;
    va_start( argptr, out_ch );
    control_loop_config_st * pCfg = ( control_loop_config_st * )va_arg( argptr, int );
	va_end( argptr );
	if( INVALID_CH != out_ch )
	{
		if( out_ch < MAX_PWM_CHANNEL )
		{
			if( PWM.SetFreq( out_ch, (float)pCfg->PWMFreq / 10 ) && PWM.Start( out_ch ) )
			{
				complete = 1;
			}
		}
		else if( out_ch < MAX_CONTROL_LOOP )
		{
			RemoteIO.SetDoFreq( out_ch, pCfg->PWMFreq );
			complete = 1;
		}
	}
	return complete;
}

/******************************************************************************
 * @brief 
 * 
 * @param out_type 
 * @param out_ch 
 * @param mv 
 *****************************************************************************/
static void run_pwm( output_et out_type, int8_t out_ch, float mv, ... )
{
	if( INVALID_CH != out_ch )
	{
		if( out_ch < MAX_PWM_CHANNEL )					// internal PWM
		{
			PWM.SetDuty( out_ch, MV_to_PWMduty( mv ) );
		}
		else if( out_ch < MAX_CONTROL_LOOP )
		{
			static AppTimerData_ut timer_can_tx_do = {0};

			if( AppTimer.IsExpired( &timer_can_tx_do ) )
			{
				AppTimer.Start( &timer_can_tx_do, TIMER_CH_OUTPUT );
				RemoteIO.SetDoData( out_ch, mv * MULTIPLY_FLOAT_TO_PERMIL );
			}
		}
	}
}

/******************************************************************************
 * @brief 
 * 
 * @param type 
 * @param out_ch 
 * @return uint8_t 
 *****************************************************************************/
static uint8_t stop_pwm( output_et type, int8_t out_ch, ... )
{
	uint8_t complete = 0;
	if( INVALID_CH != out_ch )
	{
		if( out_ch < MAX_PWM_CHANNEL )
		{
			PWM.SetDuty( out_ch, MV_to_PWMduty( 0 ) );
			complete = PWM.Stop( out_ch );
		}
		else if( out_ch < MAX_CONTROL_LOOP )
		{
			RemoteIO.SetDoFreq( out_ch, 0 );
			RemoteIO.SetDoData( out_ch, 0 );
			complete = 1;
		}
	}
	return complete;
}

/******************************************************************************
 * @brief Set the smps analog object
 * 
 * @param out_type 
 * @param out_ch 
 * @param status 
 * @return uint8_t 
 *****************************************************************************/
static uint8_t set_smps_analog( output_et out_type, int8_t out_ch, ... )
{
	uint8_t complete = 0;

	if( INVALID_CH != out_ch )
	{
		if( out_ch <= 1 )
		{
			if( AD5422.SetupRange( out_ch, match_dac_range( OUT_V_P0_P5 ) ) != DAC_SET_ERR )
			{
				complete = 1;
			}
		}

		if( out_ch == 0 )
		{
			DIO.Output( SMPS_POLA1, SMPS_POLA_COOL );
		}
		else if( out_ch == 1 )
		{
			DIO.Output( SMPS_POLA2, SMPS_POLA_COOL );
		}
	}

	return complete;
}

/******************************************************************************
 * @brief 
 * 
 * @param out_type 
 * @param out_ch 
 * @param smps_mv 
 *****************************************************************************/
static void run_smps_analog( output_et out_type, int8_t out_ch, float smps_mv, ... )
{
	if( INVALID_CH != out_ch )
	{
		if( out_ch < MAX_DAC_CH )
		{
			if( 0 == out_ch )
			{
				if( smps_mv > 0 )
				{
					DIO.Output( SMPS_POLA1, SMPS_POLA_COOL );
				}
				else
				{
					DIO.Output( SMPS_POLA1, SMPS_POLA_HEAT );
					smps_mv = fabs( smps_mv );
				}
			}
			else if( 1 == out_ch )
			{
				if( smps_mv > 0 )
				{
					DIO.Output( SMPS_POLA2, SMPS_POLA_COOL );
				}
				else
				{
					DIO.Output( SMPS_POLA2, SMPS_POLA_HEAT );
					smps_mv = fabs( smps_mv );
				}
			}
			
			AD5422.Output( out_ch, MV_to_dacValue( OUT_V_P0_P5, smps_mv, INTERNAL_DAC ) );
		}
	}
}

/******************************************************************************
 * @brief 
 * 
 * @param out_type 
 * @param out_ch 
 * @return uint8_t 
 *****************************************************************************/
static uint8_t stop_smps_analog( output_et out_type, int8_t out_ch, ... )
{
	int complete = 0;

	if( INVALID_CH != out_ch )
	{
		AD5422.Output( out_ch, 0 );

		if( out_ch == 0 )
			DIO.Output( SMPS_POLA1, SMPS_POLA_COOL );
		else if( out_ch == 1 )
			DIO.Output( SMPS_POLA2, SMPS_POLA_COOL );

		complete = 1;
	}

	return complete;
}

/******************************************************************************
 * @brief Set the smps comm object
 * 
 * @param out_type 
 * @param out_ch 
 * @param status 
 * @return uint8_t 
 *****************************************************************************/
static uint8_t set_smps_comm( output_et out_type, int8_t out_ch, ... )
{
	uint8_t complete = 0;
	va_list argptr;
    va_start( argptr, out_ch );
    control_loop_config_st * pCfg = ( control_loop_config_st * )va_arg( argptr, int );
	va_end( argptr );

	if( INVALID_CH != out_ch )
	{
		complete = make_smps_mbm_query( out_ch, out_type, SMPS_CMD_RUN, 0, pCfg->SmpsLeakageCurrent, pCfg->SmpsConfig.all );

	}

	return complete;
}

/******************************************************************************
 * @brief 
 * 
 * @param out_type 
 * @param out_ch 
 * @param smps_mv 
 *****************************************************************************/
static void run_smps_comm( output_et out_type, int8_t out_ch, float smps_mv, ... )
{
	va_list argptr;
	va_start( argptr, smps_mv );
	control_loop_config_st * pCfg = ( control_loop_config_st * )va_arg( argptr, int );
	va_end( argptr );
	
	if( INVALID_CH != out_ch )
	{
		make_smps_mbm_query( out_ch, out_type, SMPS_CMD_READ, 0, pCfg->SmpsLeakageCurrent, pCfg->SmpsConfig.all );

		#if ENABLE_TRANSFER_FUNCTION_FOR_SMPS_LOW_POWER_RANGE_AND_RESPONSE_DELAY
		//if( pCfg->Reserved == 0 )
		{
			// Reference : from Jang Seokju of Chiller Development part

			int16_t iMV = smps_mv * 1000.0f;
			if( abs( iMV ) <= 50 )
			{
				// Use half the value if less than 5%.
				smps_mv /= 2.0f;
			}
			else
			{
				// iMV is pre-mille (1/1000) scale.
				// Since the formula below scales smps_mv by 1/8000,
				// it must be normalized back to a floating-point range of [-1.0, 1.0].
				if( iMV < 0 )
				{
					iMV *= -1;
					smps_mv = (-1.9424E-11f*iMV)*iMV*iMV*iMV*iMV + (4.41E-8f*iMV)*iMV*iMV*iMV - (3.3907E-5f*iMV)*iMV*iMV + (1.597E-2f*iMV)*iMV + 1.345f*iMV + 6.3486f;
					smps_mv *= -1.0f;
				}
				else
				{
					smps_mv = (-1.9424E-11f*iMV)*iMV*iMV*iMV*iMV + (4.41E-8f*iMV)*iMV*iMV*iMV - (3.3907E-5f*iMV)*iMV*iMV + (1.597E-2f*iMV)*iMV + 1.345f*iMV + 6.3486f;
				}
				smps_mv /= 8000.0f;
			}
		}
	#endif
		make_smps_mbm_query( out_ch, out_type, SMPS_CMD_WRITE, smps_mv, pCfg->SmpsLeakageCurrent, pCfg->SmpsConfig.all );
	}
}

/******************************************************************************
 * @brief 
 * 
 * @param out_type 
 * @param out_ch 
 * @return uint8_t 
 *****************************************************************************/
static uint8_t stop_smps_comm( output_et out_type, int8_t out_ch, ... )
{
	static uint8_t stopped = 0;
	va_list argptr;
	va_start( argptr, out_ch );
	control_loop_config_st * pCfg = ( control_loop_config_st * )va_arg( argptr, int );
	va_end( argptr );

	if( INVALID_CH != out_ch )
	{
		make_smps_mbm_query( out_ch, out_type, SMPS_CMD_STOP, 0, pCfg->SmpsLeakageCurrent, pCfg->SmpsConfig.all );
		make_smps_mbm_query( out_ch, out_type, SMPS_CMD_READ, 0, 0, 0 );
		switch( out_type )
		{
			case OUT_SMPS_CHPP_8021:
				stopped = !( MBM_READ_DB[ out_ch ].Holdings[ MBM_HR_CHPP_8021_STATUS_OF_SMPS ] & SMPS_CMD_STATUS_RUN );
				break;
			case OUT_SMPS_CHPP_5521:
				stopped = !( MBM_READ_DB[ out_ch ].Holdings[ MBM_HR_CHPP_5521_STATUS_OF_SMPS ] & SMPS_CMD_STATUS_RUN );
				break;
			default:
				break;
		}		
	}

	return stopped;	
}

/******************************************************************************
 * @brief 
 * 
 * @param type 
 * @param pOut 
 * @return uint8_t 
 *****************************************************************************/
static void funcSet_StartStop( output_et type, output_st* pOut )
{
	switch( type )
	{
		case OUT_V_P0_P5:
		case OUT_V_P0_P10:
		case OUT_V_M5_P5:
		case OUT_V_M10_P10:
		case OUT_A_4M_20M:
		case OUT_A_0M_20M:
		case OUT_A_0M_24M:
			pOut->SetCfg = set_dac;
			pOut->Stop = stop_dac;
			break;

		case OUT_PWM:
			pOut->SetCfg = set_pwm;
			pOut->Stop = stop_pwm;
			break;

		case OUT_SMPS_NHPP_6921:
		case OUT_SMPS_NHPP_7325:
			pOut->SetCfg = set_smps_analog;
			pOut->Stop = stop_smps_analog;
			break;
		case OUT_SMPS_CHPP_5521:
		case OUT_SMPS_CHPP_8021:
		case OUT_SMPS_NHPP_2032:
		case OUT_SMPS_NHPP_1531:
			pOut->SetCfg = set_smps_comm;
			pOut->Stop = stop_smps_comm;
			break;
		default:
			break;
	}
}

/******************************************************************************
 * @brief 
 * 
 * @param type 
 * @param pOut 
 * @return uint8_t 
 *****************************************************************************/
static void funcSet_Run( output_et type, output_st* pOut )
{
	switch( type )
	{
		case OUT_V_P0_P5:
		case OUT_V_P0_P10:
		case OUT_V_M5_P5:
		case OUT_V_M10_P10:
		case OUT_A_4M_20M:
		case OUT_A_0M_20M:
		case OUT_A_0M_24M:
			pOut->Run = run_dac;
			break;

		case OUT_PWM:
			pOut->Run = run_pwm;
			break;

		case OUT_SMPS_NHPP_6921:
		case OUT_SMPS_NHPP_7325:
			pOut->Run = run_smps_analog;
			break;
		case OUT_SMPS_CHPP_5521:
		case OUT_SMPS_CHPP_8021:
		case OUT_SMPS_NHPP_2032:
		case OUT_SMPS_NHPP_1531:
			pOut->Run = run_smps_comm;
			break;
		default:
			break;
	}
}

/******************************************************************************
 * @brief 
 * 
 *****************************************************************************/
void OutputTask( void )
{
	static uint8_t ctrl_ch = 0;
	static uint8_t output_cfg_valid[ MAX_CONTROL_LOOP ] = { 0 };
	static output_et applied_output_type[ MAX_CONTROL_LOOP ] = { OUT_NONE };
	static int8_t applied_output_ch[ MAX_CONTROL_LOOP ] = { 0 };
	static uint16_t applied_pwm_freq[ MAX_CONTROL_LOOP ] = { 0 };
	const control_loop_config_st * const pCfg = Controller.GetConfig( ctrl_ch );
	uint8_t cfg_changed = output_cfg_valid[ ctrl_ch ]
			&& ( ( applied_output_type[ ctrl_ch ] != pCfg->OutputType )
				|| ( applied_output_ch[ ctrl_ch ] != pCfg->OutputChannel )
				|| ( ( OUT_PWM == applied_output_type[ ctrl_ch ] )
					&& ( OUT_PWM == pCfg->OutputType )
					&& ( applied_pwm_freq[ ctrl_ch ] != pCfg->PWMFreq ) ) );

	if( cfg_changed )
	{
		if( OUTPUT[ ctrl_ch ].Stop )
		{
			OUTPUT[ ctrl_ch ].Stop( applied_output_type[ ctrl_ch ], applied_output_ch[ ctrl_ch ], pCfg );
		}

		OUTPUT[ ctrl_ch ].SetCfg = NULL;
		OUTPUT[ ctrl_ch ].Run = NULL;
		OUTPUT[ ctrl_ch ].Stop = NULL;
		applied_pwm_freq[ ctrl_ch ] = 0;
		output_cfg_valid[ ctrl_ch ] = 0;
	}

	if( pCfg->Enable )
	{
		if( OUTPUT[ ctrl_ch ].Run )
		{
			OUTPUT[ ctrl_ch ].Run( pCfg->OutputType, pCfg->OutputChannel, Controller.GetMV( ctrl_ch ), pCfg );
		}
		else
		{
			if( OUTPUT[ ctrl_ch ].SetCfg )
			{
				if( OUTPUT[ ctrl_ch ].SetCfg( pCfg->OutputType, pCfg->OutputChannel, pCfg ) )
				{
					applied_output_type[ ctrl_ch ] = pCfg->OutputType;
					applied_output_ch[ ctrl_ch ] = pCfg->OutputChannel;
					applied_pwm_freq[ ctrl_ch ] = pCfg->PWMFreq;
					output_cfg_valid[ ctrl_ch ] = 1;
					funcSet_Run( pCfg->OutputType, &OUTPUT[ ctrl_ch ] );
				}
			}
			else
			{
				if( pCfg->OutputType != OUT_NONE )
				{
					funcSet_StartStop( pCfg->OutputType, &OUTPUT[ ctrl_ch ] );
				}
			}
		}
	}
	else
	{
		if( OUTPUT[ ctrl_ch ].Stop )
		{
			OUTPUT[ ctrl_ch ].Stop(
					output_cfg_valid[ ctrl_ch ] ? applied_output_type[ ctrl_ch ] : pCfg->OutputType,
					output_cfg_valid[ ctrl_ch ] ? applied_output_ch[ ctrl_ch ] : pCfg->OutputChannel,
					pCfg );
		}

		OUTPUT[ ctrl_ch ].SetCfg = NULL;
		OUTPUT[ ctrl_ch ].Run = NULL;
		OUTPUT[ ctrl_ch ].Stop = NULL;
		applied_pwm_freq[ ctrl_ch ] = 0;
		output_cfg_valid[ ctrl_ch ] = 0;
	}

	TICK_UP( ctrl_ch, MAX_CONTROL_LOOP );
}
