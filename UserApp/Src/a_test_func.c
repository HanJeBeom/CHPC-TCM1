/******************************************************************************
 * @file a_test_func.c
 * @author GST (sw@gst-in.com)
 * @brief Production test command interface (machine-readable protocol)
 *        Reference: docs/harness/test_func.md, docs/harness/test_program.md
 * @version 0.1
 * @date 2026-06-15
 * 
 * @copyright Copyright (c) 2026 Global Standard Technology Co., Ltd.
 * 
 * All rights reserved.
 * This file is part of closed source software.
 * Unauthorized reproduction and redistribution prohibited.
 *****************************************************************************/

/*****************************************************************************/
/** INCLUDES *****************************************************************/
/*****************************************************************************/
#include <math.h>

#include "UserApp.h"

/*****************************************************************************/
/** MACRO DEFINITIONS ********************************************************/
/*****************************************************************************/

#define TF_TASK_CYCLE_TIME		0.01f	// 10ms line scan
#define TF_MEAS_PERIOD			0.5f	// continuous measurement stream period
#define TF_LOG_PERIOD			0.1f	// CL1/CHPP-8021 CSV snapshot period
#define TF_LOG_COLUMNS			44
#define TF_LOG_LINE_MAX		640
#define TF_CAL_HB_PERIOD		0.5f	// calibration heartbeat event period
#define TF_GET_SETTLE_TIME		0.5f	// wait for a fresh ADC value after sensor type change
#define TF_GET_TIMEOUT			3.0f	// FW single GET measurement timeout
#define TF_GET_HB_PERIOD		0.5f	// pending GET progress event period
#define TF_CAL_DONE_LINE_MAX	256		// RTD and RTD2X conversion fields in one CAL DONE line

#define TF_TC_DEFAULT_TYPE		SEN_TC_K
#define TF_MEAS_CHANNELS		4		// number of on-board temperature channels

/* Error codes (see test_func.md / test_program.md) */
#define TF_ERR_UNKNOWN			1		// unknown command
#define TF_ERR_ARG				2		// argument / line length error
#define TF_ERR_RANGE			3		// channel / value out of range
#define TF_ERR_NO_BOARD			10		// output board not present
#define TF_ERR_CAL_STATE		11		// not in calibration state / order error
#define TF_ERR_CAL_VALIDATION	12		// measured value out of ±1 % tolerance
#define TF_ERR_TOKEN			13		// dangerous action token mismatch
#define TF_ERR_EEPROM			20		// eeprom access failure
#define TF_ERR_INTERNAL			99		// internal error

#define TF_RTD_TEST_TOL_RATIO	0.01f	// RTD function test default tolerance ratio (1 %)
#define TF_TC_TEST_TOLERANCE_C	0.1f	// TC function test absolute tolerance (degC)

/*****************************************************************************/
/** DATATYPES ****************************************************************/
/*****************************************************************************/

typedef enum
{
	TF_GET_IDLE = 0,
	TF_GET_SETTLING,
	TF_GET_WAIT_SAMPLE,
} tf_get_state_et;

typedef struct test_func_state_struct_Tag
{
	char			line[ TF_LINE_MAX ];
	uint16_t		len;
	bool			overflow;
	bool			session;

	uint32_t		token;
	bool			token_issued;

	struct
	{
		bool		on;
		sensor_et	type;
	} meas;

	struct
	{
		bool		on;
		bool		manual;
		uint8_t		smps_id;
		uint32_t	sample_seq;
		uint32_t	sent_samples;
		uint32_t	usb_dropped_count;
	} log;

	struct
	{
		tf_get_state_et state;
		sensor_et	type;
		uint8_t		ch;
		uint32_t	start_generation;
		bool		override_active;
		sensor_et	original_type;
		uint16_t	original_sampling_period;
	} get;

	struct
	{
		bool		on;
		bool		pending_commit;		// CAL DONE sent, waiting for >CAL SUCCESS
		bool		point_on;
		bool		auto_on;
		bool		save_on;
		calib_session_mode_et auto_mode;
		uint8_t		auto_ch_mask;
		uint8_t		save_ch;
		uint8_t		save_mask;
		uint8_t		saved_mask;
		bool		save_diag_on;
		sensor_et	save_type;
		sensor_et	type;
		uint8_t		ch;
		sensor_et	point_type;
		uint8_t		point_ch;
		uint8_t		point_idx;
	} cal;
} test_func_state_st;

/*****************************************************************************/
/** FUNCTION DECLARATIONS ****************************************************/
/*****************************************************************************/

static bool test_func_is_active( void );
static bool test_func_is_sensor_override_active( uint8_t ch );
static void test_func_emit( const char * fmt, ... );
static void tf_emit_get_cal_diag( uint8_t ch, sensor_et type, int32_t value );
static bool tf_reset_eeprom_range( uint32_t start_addr, uint32_t end_addr );

/*****************************************************************************/
/** GLOBAL VARIABLES *********************************************************/
/*****************************************************************************/

const test_func_st TestFunc =
{
	.IsActive				= test_func_is_active,
	.IsSensorOverrideActive	= test_func_is_sensor_override_active,
	.Emit					= test_func_emit,
};

/*****************************************************************************/
/** LOCAL VARIABLES **********************************************************/
/*****************************************************************************/

extern GstRingBufHandle_t rbUsbRx;		// USB CDC receive ring buffer (a_usbcdc.c)

static CCMRAM test_func_state_st tf;

static AppTimerData_ut tf_task_timer = { .All = 0 };
static AppTimerData_ut tf_meas_timer = { .All = 0 };
static AppTimerData_ut tf_log_timer = { .All = 0 };
static AppTimerData_ut tf_cal_hb_timer = { .All = 0 };
static AppTimerData_ut tf_get_settle_timer = { .All = 0 };
static AppTimerData_ut tf_get_timeout_timer = { .All = 0 };
static AppTimerData_ut tf_get_hb_timer = { .All = 0 };

static const char tf_log_csv_header[] =
	"timestamp_ms,sample_seq,usb_dropped_count,system_run,system_ready,cl1_enable,input_type,input_channel,"
	"output_type,output_channel,control_period_ms,pb_tenth_pct,ti_100ms,td_10ms,"
	"output_min_permille,output_max_permille,control_type_u16,cool_heat_mode_u16,output_delay_s,start_delay_s,"
	"rtd_ch1_mdegc,rtd_ch2_mdegc,smps_id,sv_mdegc,pv_mdegc,error_mdegc,mv_permille,ctrl_fault_u16,"
	"smps_read_seq_u32,smps_data_age_ms,smps_power_cmd_w,smps_status_u16,"
	"smps_warning_u16,smps_fault_u16,smps_leak_ch1_q7,smps_leak_ch2_q7,"
	"smps_leak_ch3_q7,smps_leak_ch4_q7,smps_current_ch1_q7,smps_current_ch2_q7,"
	"smps_current_ch3_q7,smps_current_ch4_q7,smps_voltage_q5,smps_power_measured_w\r\n";
static CCMRAM char tf_log_line[ TF_LOG_LINE_MAX ];

/*****************************************************************************/
/** FUNCTION DEFINITIONS *****************************************************/
/*****************************************************************************/

/******************************************************************************
 * @brief Send one formatted line over USB CDC.
 *****************************************************************************/
static void test_func_emit( const char * fmt, ... )
{
	char buf[ TF_LINE_MAX ];
	va_list args;
	int n;

	va_start( args, fmt );
	n = vsnprintf( buf, sizeof( buf ), fmt, args );
	va_end( args );

	if( n < 0 )
	{
		return;
	}
	if( n >= ( int )sizeof( buf ) )
	{
		n = sizeof( buf ) - 1;
	}

	CDC.Write( ( const uint8_t * )buf, ( uint32_t )n );
}

/******************************************************************************
 * @brief Whether a test session is currently active.
 *****************************************************************************/
static bool test_func_is_active( void )
{
	return tf.session;
}

/******************************************************************************
 * @brief Whether a pending GET temporarily owns one physical ADC channel.
 *****************************************************************************/
static bool test_func_is_sensor_override_active( uint8_t ch )
{
	return tf.get.override_active && ( tf.get.ch == ch );
}

/******************************************************************************
 * @brief Whether normal control has been or is being requested.
 *****************************************************************************/
static bool tf_control_run_is_active( void )
{
	return ( 0 != MBSDB.Holdings[ MBS_HR_RUN ] ) || ( 0 != CFG.system.Run );
}

/******************************************************************************
 * @brief Acquire temporary ownership of one physical ADC channel for GET.
 *****************************************************************************/
static bool tf_sensor_override_acquire( uint8_t ch, sensor_et type )
{
	if( ( ch >= TF_MEAS_CHANNELS ) || tf.get.override_active )
	{
		return false;
	}

	tf.get.ch = ch;
	tf.get.type = type;
	tf.get.original_type = AD7124.GetType( ch );
	tf.get.original_sampling_period = AD7124.GetSamplePeriod( ch );
	tf.get.override_active = true;

	Temp.SetType( ( int16_t )ch, type, DEFAULT_SAMPLING_PERIOD_MS );
	return true;
}

/******************************************************************************
 * @brief Stop pending GET timers and release its physical ADC ownership.
 *
 * When RUN is active, Modbus Holding configuration has priority and the old
 * test snapshot must not overwrite a setting changed while GET was pending.
 *****************************************************************************/
static void tf_pending_get_finish( bool restore_original )
{
	AppTimer.Stop( &tf_get_settle_timer );
	AppTimer.Stop( &tf_get_timeout_timer );
	AppTimer.Stop( &tf_get_hb_timer );

	if( tf.get.override_active )
	{
		if( restore_original )
		{
			Temp.SetType( ( int16_t )tf.get.ch,
				tf.get.original_type, tf.get.original_sampling_period );
		}
		tf.get.override_active = false;
	}

	tf.get.state = TF_GET_IDLE;
}

/******************************************************************************
 * @brief Validate and start one asynchronous RTD/TC GET measurement.
 *****************************************************************************/
static void tf_pending_get_start( uint8_t ch, sensor_et type, const char * type_name )
{
	if( ( TF_GET_IDLE != tf.get.state ) || tf.get.override_active || tf.meas.on )
	{
		test_func_emit( "<GET ERR %d TYPE=%s CH=%d REASON=MEAS_BUSY\n",
			TF_ERR_CAL_STATE, type_name, ch );
		return;
	}
	if( tf.cal.on || tf.cal.point_on || tf.cal.save_on || Calib.status.On )
	{
		test_func_emit( "<GET ERR %d TYPE=%s CH=%d REASON=CAL_BUSY\n",
			TF_ERR_CAL_STATE, type_name, ch );
		return;
	}
	if( tf_control_run_is_active() )
	{
		test_func_emit( "<GET ERR %d TYPE=%s CH=%d REASON=RUN_ACTIVE\n",
			TF_ERR_CAL_STATE, type_name, ch );
		return;
	}
	if( !tf_sensor_override_acquire( ch, type ) )
	{
		test_func_emit( "<GET ERR %d TYPE=%s CH=%d REASON=INTERNAL_STATE\n",
			TF_ERR_INTERNAL, type_name, ch );
		return;
	}

	tf.get.state = TF_GET_SETTLING;
	tf.get.start_generation = Temp.GetSampleGeneration( ch );
	AppTimer.Start( &tf_get_settle_timer, TF_GET_SETTLE_TIME );
	AppTimer.Start( &tf_get_timeout_timer, TF_GET_TIMEOUT );
	AppTimer.Start( &tf_get_hb_timer, TF_GET_HB_PERIOD );
	test_func_emit( "<GET ACK TYPE=%s CH=%d\n", type_name, ch );
	test_func_emit( "!GET TYPE=%s CH=%d STATE=SETTLING GEN=%lu\n",
		type_name, ch, ( unsigned long )tf.get.start_generation );
}

/******************************************************************************
 * @brief Reply with an error status line. ( <CMD ERR code )
 *****************************************************************************/
static void tf_err( const char * cmd, int code )
{
	test_func_emit( "<%s ERR %d\n", cmd, code );
}

static int tf_cal_save_error_code( void )
{
	if( CALIB_SAVE_ERR_EEPROM == Calib.status.save_error )
	{
		return TF_ERR_EEPROM;
	}
	return TF_ERR_CAL_VALIDATION;
}

static const char * tf_cal_type_name( sensor_et type )
{
	return ( ( SEN_RTD == type ) || ( SEN_RTD2X == type ) ) ? "RTD" : "TC";
}

/******************************************************************************
 * @brief Copy the idx-th whitespace separated token of s into out.
 * 
 * @return true when a token exists at the given index.
 *****************************************************************************/
static bool tf_word( const char * s, int idx, char * out, int outsz )
{
	int i = 0;
	const char * p = s;

	while( 0 != *p )
	{
		while( ( ' ' == *p ) || ( '\t' == *p ) )
		{
			p++;
		}
		if( 0 == *p )
		{
			break;
		}

		const char * start = p;
		while( ( 0 != *p ) && ( ' ' != *p ) && ( '\t' != *p ) )
		{
			p++;
		}

		if( i == idx )
		{
			int n = ( int )( p - start );
			if( n >= outsz )
			{
				n = outsz - 1;
			}
			memcpy( out, start, n );
			out[ n ] = 0;
			return true;
		}
		i++;
	}

	return false;
}

/******************************************************************************
 * @brief Find a "KEY=VALUE" token and copy VALUE into out.
 *****************************************************************************/
static bool tf_find_val( const char * s, const char * key, char * out, int outsz )
{
	char tok[ TF_LINE_MAX ];
	int klen = ( int )strlen( key );

	for( int i = 0; tf_word( s, i, tok, sizeof( tok ) ); i++ )
	{
		if( ( 0 == strncmp( tok, key, klen ) ) && ( '=' == tok[ klen ] ) )
		{
			strncpy( out, tok + klen + 1, outsz - 1 );
			out[ outsz - 1 ] = 0;
			return true;
		}
	}

	return false;
}

/******************************************************************************
 * @brief Parse an integer value of "KEY=VALUE".
 *****************************************************************************/
static bool tf_get_int( const char * s, const char * key, int32_t * out )
{
	char val[ 24 ];

	if( !tf_find_val( s, key, val, sizeof( val ) ) )
	{
		return false;
	}

	*out = ( int32_t )strtol( val, NULL, 0 );
	return true;
}

/******************************************************************************
 * @brief Parse a float value of "KEY=VALUE".
 *****************************************************************************/
static bool tf_get_float( const char * s, const char * key, float * out )
{
	char val[ 24 ];

	if( !tf_find_val( s, key, val, sizeof( val ) ) )
	{
		return false;
	}

	*out = strtof( val, NULL );
	return true;
}

/******************************************************************************
 * @brief Issue a fresh one-shot token for dangerous actions.
 *****************************************************************************/
static uint32_t tf_make_token( void )
{
	static uint32_t seed = 0x12345678;

	seed = ( seed * 1664525u ) + 1013904223u + ( uint32_t )( AppTimer.GetCurrentSecs() * 1000.0f );
	return seed;
}

/******************************************************************************
 * @brief Validate the TOKEN= argument against the issued token.
 *****************************************************************************/
static bool tf_token_ok( const char * s )
{
	char val[ 24 ];
	const char * p;
	char * end;
	uint32_t given;

	if( !tf.token_issued )
	{
		return false;
	}
	if( !tf_find_val( s, "TOKEN", val, sizeof( val ) ) )
	{
		return false;
	}

	p = val;
	if( ( '0' == p[ 0 ] ) && ( ( 'x' == p[ 1 ] ) || ( 'X' == p[ 1 ] ) ) )
	{
		p += 2;
	}

	given = ( uint32_t )strtoul( p, &end, 16 );
	if( 0 != *end )
	{
		return false;
	}

	return ( given == tf.token );
}

/******************************************************************************
 * @brief Map a sensor type name to sensor_et. Falls back to numeric value.
 *****************************************************************************/
static sensor_et tf_sensor_from_name( const char * name )
{
	if( 0 == strcmp( name, "RTD2X" ) )	return SEN_RTD2X;
	if( 0 == strcmp( name, "RTD" ) )	return SEN_RTD;
	if( 0 == strcmp( name, "TC_K" ) )	return SEN_TC_K;
	if( 0 == strcmp( name, "TC_J" ) )	return SEN_TC_J;
	if( 0 == strcmp( name, "TC_E" ) )	return SEN_TC_E;
	if( 0 == strcmp( name, "TC_S" ) )	return SEN_TC_S;
	if( 0 == strcmp( name, "TC_T" ) )	return SEN_TC_T;
	if( 0 == strcmp( name, "TC_R" ) )	return SEN_TC_R;
	if( 0 == strcmp( name, "TC" ) )		return TF_TC_DEFAULT_TYPE;

	return ( sensor_et )strtol( name, NULL, 0 );
}

/******************************************************************************
 * @brief Whether a sensor type is one of the supported thermocouple types.
 *****************************************************************************/
static bool tf_is_tc_type( sensor_et type )
{
	return ( SEN_TC_K == type ) || ( SEN_TC_J == type ) || ( SEN_TC_E == type )
		|| ( SEN_TC_S == type ) || ( SEN_TC_T == type ) || ( SEN_TC_R == type );
}

/******************************************************************************
 * @brief Read one channel temperature for the selected test sensor type.
 *****************************************************************************/
static float tf_get_meas_temp( sensor_et type, int16_t ch )
{
	if( ( SEN_RTD == type ) || ( SEN_RTD2X == type ) )
	{
		return Temp.GetTemp( ch );
	}

	return Temp.GetTCTemp( type, ( uint8_t )ch, 0.0f );
}

/******************************************************************************
 * @brief Holding register index of a per-channel field for loop ch.
 *****************************************************************************/
static int tf_ch_reg( int base_ch1_reg, int ch )
{
	return base_ch1_reg + ( ch * MBS_HR_CH_SPAN );
}

/*****************************************************************************/
/** COMMAND HANDLERS *********************************************************/
/*****************************************************************************/

/******************************************************************************
 * @brief Session / identification commands.
 *****************************************************************************/
static void tf_cmd_hello( void )
{
	tf.session = true;
	test_func_emit( "<HELLO OK MCU=%04X REV=%04X BUILD=%06lX PLM=%08lX\n",
		CFG.system.MCU, CFG.system.REVISION,
		( unsigned long )CFG.system.BUILD_DATE, ( unsigned long )TCM1_PLM_CODE );
}

static void tf_cmd_ver( void )
{
	test_func_emit( "<VER OK MCU=%04X REV=%04X BUILD=%06lX\n",
		CFG.system.MCU, CFG.system.REVISION, ( unsigned long )CFG.system.BUILD_DATE );
}

static void tf_cmd_token( void )
{
	tf.token = tf_make_token();
	tf.token_issued = true;
	test_func_emit( "<TOKEN OK VAL=0x%08lX\n", ( unsigned long )tf.token );
}

/******************************************************************************
 * @brief Fill the selected EEPROM range with 0xFF.
 *****************************************************************************/
static bool tf_reset_eeprom_range( uint32_t start_addr, uint32_t end_addr )
{
	extern IWDG_HandleTypeDef hiwdg;
	static const uint8_t reset_data[ 16 ] = {
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

	for( uint32_t addr = start_addr; addr < end_addr; )
	{
		uint32_t remain = end_addr - addr;
		uint32_t len = ( remain > sizeof( reset_data ) ) ? sizeof( reset_data ) : remain;

		if( HAL_OK != EEPR.Write( addr, ( uint8_t * )reset_data, len ) )
		{
			return false;
		}
		HAL_IWDG_Refresh( &hiwdg );
		addr += len;
	}

	return true;
}

/******************************************************************************
 * @brief GET (read) commands.
 *****************************************************************************/
static void tf_cmd_get( const char * s )
{
	char sub[ 12 ];
	int32_t ch = 0;

	if( !tf_word( s, 1, sub, sizeof( sub ) ) )
	{
		tf_err( "GET", TF_ERR_ARG );
		return;
	}

	if( 0 == strcmp( sub, "TEMP" ) )
	{
		if( !tf_get_int( s, "CH", &ch ) || ( ch < 0 ) || ( ch >= TF_MEAS_CHANNELS ) )
		{
			tf_err( "GET", TF_ERR_RANGE );
			return;
		}
		test_func_emit( "<GET OK CH=%ld TEMP=%.3f\n", ( long )ch, Temp.GetTemp( ( int16_t )ch ) );
	}
	else if( 0 == strcmp( sub, "RTD" ) )
	{
		if( !tf_get_int( s, "CH", &ch ) || ( ch < 0 ) || ( ch >= TF_MEAS_CHANNELS ) )
		{
			tf_err( "GET", TF_ERR_RANGE );
			return;
		}
		tf_pending_get_start( ( uint8_t )ch, SEN_RTD, "RTD" );
	}
	else if( 0 == strcmp( sub, "TC" ) )
	{
		if( !tf_get_int( s, "CH", &ch ) || ( ch < 0 ) || ( ch >= TF_MEAS_CHANNELS ) )
		{
			tf_err( "GET", TF_ERR_RANGE );
			return;
		}
		tf_pending_get_start( ( uint8_t )ch, TF_TC_DEFAULT_TYPE, "TC" );
	}
	else if( 0 == strcmp( sub, "CJ" ) )
	{
		test_func_emit( "<GET OK CJ=%.3f\n", Temp.GetCjTemp() );
	}
	else if( 0 == strcmp( sub, "ADC" ) )
	{
		if( !tf_get_int( s, "CH", &ch ) || ( ch < 0 ) || ( ch >= TF_MEAS_CHANNELS ) )
		{
			tf_err( "GET", TF_ERR_RANGE );
			return;
		}
		test_func_emit( "<GET OK CH=%ld RAW=%ld\n", ( long )ch, ( long )AD7124.GetValue( ( uint8_t )ch ) );
	}
	else if( 0 == strcmp( sub, "BOARD" ) )
	{
		bool present = ( 0 == DIO.Input( BD_CHECK ) );
		test_func_emit( "<GET OK OUTBOARD=%s\n", present ? "OK" : "NONE" );
	}
	else if( 0 == strcmp( sub, "RESET" ) )
	{
		test_func_emit( "<GET OK CAUSE=%s COUNT=%d EXC=%s\n",
			Sys.GetResetCauseName(), Sys.GetResetCount(), Sys.GetLastExceptionName() );
	}
	else if( 0 == strcmp( sub, "CFG" ) )
	{
		if( !tf_get_int( s, "CH", &ch ) || ( ch < 0 ) || ( ch >= MAX_CONTROL_LOOP ) )
		{
			tf_err( "GET", TF_ERR_RANGE );
			return;
		}
		int32_t sv = ( int32_t )( ( uint16_t )MBSDB.Holdings[ tf_ch_reg( MBS_HR_CH1_SV_L, ch ) ]
				| ( ( uint32_t )MBSDB.Holdings[ tf_ch_reg( MBS_HR_CH1_SV_H, ch ) ] << 16 ) );
		test_func_emit( "<GET OK CH=%ld ENABLE=%d SV=%ld PV=%ld MV=%.3f PB=%d TI=%d TD=%d\n",
			( long )ch,
			MBSDB.Holdings[ tf_ch_reg( MBS_HR_CH1_ENABLE, ch ) ],
			( long )sv,
			( long )Controller.GetPV( ch ),
			Controller.GetMV( ch ),
			MBSDB.Holdings[ tf_ch_reg( MBS_HR_CH1_PB, ch ) ],
			MBSDB.Holdings[ tf_ch_reg( MBS_HR_CH1_TI, ch ) ],
			MBSDB.Holdings[ tf_ch_reg( MBS_HR_CH1_TD, ch ) ] );
	}
	else
	{
		tf_err( "GET", TF_ERR_ARG );
	}
}

/******************************************************************************
 * @brief SET (write to holding registers) commands.
 *****************************************************************************/
static void tf_cmd_set( const char * s )
{
	int32_t ch = 0;
	int32_t val = 0;
	char name[ 16 ];

	/* >SET RUN VAL=0|1 */
	{
		char sub[ 8 ];
		if( tf_word( s, 1, sub, sizeof( sub ) ) && ( 0 == strcmp( sub, "RUN" ) ) )
		{
			if( !tf_get_int( s, "VAL", &val ) )
			{
				tf_err( "SET", TF_ERR_ARG );
				return;
			}
			MBSDB.Holdings[ MBS_HR_RUN ] = ( uint16_t )( val ? 1 : 0 );
			test_func_emit( "<SET OK RUN=%d\n", MBSDB.Holdings[ MBS_HR_RUN ] );
			return;
		}
	}

	/* Per-channel parameter writes require CH= */
	if( !tf_get_int( s, "CH", &ch ) || ( ch < 0 ) || ( ch >= MAX_CONTROL_LOOP ) )
	{
		tf_err( "SET", TF_ERR_RANGE );
		return;
	}

	if( tf_get_int( s, "ENABLE", &val ) )
	{
		MBSDB.Holdings[ tf_ch_reg( MBS_HR_CH1_ENABLE, ch ) ] = ( uint16_t )( val ? 1 : 0 );
	}
	else if( tf_get_int( s, "SV", &val ) )
	{
		MBSDB.Holdings[ tf_ch_reg( MBS_HR_CH1_SV_L, ch ) ] = ( uint16_t )( val & 0xFFFF );
		MBSDB.Holdings[ tf_ch_reg( MBS_HR_CH1_SV_H, ch ) ] = ( uint16_t )( ( val >> 16 ) & 0xFFFF );
	}
	else if( tf_get_int( s, "PB", &val ) )
	{
		MBSDB.Holdings[ tf_ch_reg( MBS_HR_CH1_PB, ch ) ] = ( uint16_t )val;
	}
	else if( tf_get_int( s, "TI", &val ) )
	{
		MBSDB.Holdings[ tf_ch_reg( MBS_HR_CH1_TI, ch ) ] = ( uint16_t )val;
	}
	else if( tf_get_int( s, "TD", &val ) )
	{
		MBSDB.Holdings[ tf_ch_reg( MBS_HR_CH1_TD, ch ) ] = ( uint16_t )val;
	}
	else if( tf_find_val( s, "INTYPE", name, sizeof( name ) ) )
	{
		MBSDB.Holdings[ tf_ch_reg( MBS_HR_CH1_INPUT_TYPE, ch ) ] = ( uint16_t )tf_sensor_from_name( name );
	}
	else if( tf_get_int( s, "INCH", &val ) )
	{
		MBSDB.Holdings[ tf_ch_reg( MBS_HR_CH1_INPUT_CHANNEL, ch ) ] = ( uint16_t )val;
	}
	else if( tf_get_int( s, "OUTTYPE", &val ) )
	{
		MBSDB.Holdings[ tf_ch_reg( MBS_HR_CH1_OUTPUT_TYPE, ch ) ] = ( uint16_t )val;
	}
	else if( tf_get_int( s, "OUTCH", &val ) )
	{
		MBSDB.Holdings[ tf_ch_reg( MBS_HR_CH1_OUTPUT_CHANNEL, ch ) ] = ( uint16_t )val;
	}
	else
	{
		tf_err( "SET", TF_ERR_ARG );
		return;
	}

	test_func_emit( "<SET OK CH=%ld\n", ( long )ch );
}

/******************************************************************************
 * @brief SAVE parameters to EEPROM.
 *****************************************************************************/
static void tf_cmd_save( void )
{
	MBSDB.Holdings[ MBS_HR_SAVE_PARAMETER ] = 1;
	test_func_emit( "<SAVE OK\n" );
}

/******************************************************************************
 * @brief OUT (output / hardware) commands.
 *****************************************************************************/
static void tf_cmd_out( const char * s )
{
	char sub[ 12 ];
	char arg[ 12 ];
	int32_t ch = 0;
	float vf = 0.0f;

	if( !tf_word( s, 1, sub, sizeof( sub ) ) )
	{
		tf_err( "OUT", TF_ERR_ARG );
		return;
	}

	if( 0 == strcmp( sub, "DAC" ) )
	{
		if( !tf_get_int( s, "CH", &ch ) || ( ch < 0 ) || ( ch >= MAX_DAC_CH )
			|| !tf_find_val( s, "MODE", arg, sizeof( arg ) ) || !tf_get_float( s, "VAL", &vf ) )
		{
			tf_err( "OUT", TF_ERR_ARG );
			return;
		}

		dac_vi_set_et range;
		float code_f;

		if( 0 == strcmp( arg, "V" ) )
		{
			if( ( vf < -10.0f ) || ( vf > 10.0f ) )
			{
				tf_err( "OUT", TF_ERR_RANGE );
				return;
			}
			range = DAC_V_M10_to_10;
			code_f = ( vf + 10.0f ) / 20.0f * ( float )AD5422_DATA_MAX_VALUE;
		}
		else if( 0 == strcmp( arg, "I" ) )
		{
			if( ( vf < 0.0f ) || ( vf > 24.0f ) )
			{
				tf_err( "OUT", TF_ERR_RANGE );
				return;
			}
			range = DAC_I_0_to_24;
			code_f = vf / 24.0f * ( float )AD5422_DATA_MAX_VALUE;
		}
		else
		{
			tf_err( "OUT", TF_ERR_ARG );
			return;
		}

		if( DAC_SET_ERR == AD5422.SetupRange( ( uint8_t )ch, range ) )
		{
			tf_err( "OUT", TF_ERR_NO_BOARD );
			return;
		}
		AD5422.Output( ( uint8_t )ch, ( uint16_t )roundf( code_f ) );
		test_func_emit( "<OUT OK CH=%ld MODE=%s VAL=%.3f\n", ( long )ch, arg, vf );
	}
	else if( 0 == strcmp( sub, "PWM" ) )
	{
		float freq = 0.0f;
		float duty = 0.0f;

		if( !tf_get_int( s, "CH", &ch ) || ( ch < 0 ) || ( ch >= MAX_PWM_CHANNEL )
			|| !tf_get_float( s, "FREQ", &freq ) || !tf_get_float( s, "DUTY", &duty ) )
		{
			tf_err( "OUT", TF_ERR_ARG );
			return;
		}
		PWM.SetFreq( ( uint8_t )ch, freq );
		PWM.SetDuty( ( uint8_t )ch, duty );
		PWM.Start( ( uint8_t )ch );
		test_func_emit( "<OUT OK CH=%ld FREQ=%.3f DUTY=%.1f\n", ( long )ch, freq, duty );
	}
	else if( 0 == strcmp( sub, "SMPS" ) )
	{
		if( !tf_find_val( s, "POLA", arg, sizeof( arg ) ) )
		{
			tf_err( "OUT", TF_ERR_ARG );
			return;
		}
		bool cool = ( 0 == strcmp( arg, "COOL" ) );
		DIO.Output( SMPS_POLA1, cool ? SMPS_POLA_COOL : SMPS_POLA_HEAT );
		DIO.Output( SMPS_POLA2, cool ? SMPS_POLA_COOL : SMPS_POLA_HEAT );
		test_func_emit( "<OUT OK POLA=%s\n", cool ? "COOL" : "HEAT" );
	}
	else if( 0 == strcmp( sub, "LED" ) )
	{
		int32_t state = 0;

		if( !tf_find_val( s, "ID", arg, sizeof( arg ) ) || !tf_get_int( s, "STATE", &state ) )
		{
			tf_err( "OUT", TF_ERR_ARG );
			return;
		}
		bool on = ( 0 != state );
		if( 0 == strcmp( arg, "COM" ) )			DIO.Output( LED_COM, on );
		else if( 0 == strcmp( arg, "ERR" ) )	DIO.Output( LED_ERR, on );
		else if( 0 == strcmp( arg, "RUN" ) )	DIO.Output( LED_RUN, on );
		else
		{
			tf_err( "OUT", TF_ERR_ARG );
			return;
		}
		test_func_emit( "<OUT OK ID=%s STATE=%d\n", arg, on ? 1 : 0 );
	}
	else if( 0 == strcmp( sub, "FAULT" ) )
	{
		if( !tf_find_val( s, "MODE", arg, sizeof( arg ) ) )
		{
			tf_err( "OUT", TF_ERR_ARG );
			return;
		}
		MBSDB.Holdings[ MBS_HR_FAULT_NO_NC ] = ( 0 == strcmp( arg, "NC" ) ) ? 1 : 0;
		test_func_emit( "<OUT OK MODE=%s\n", ( 0 == strcmp( arg, "NC" ) ) ? "NC" : "NO" );
	}
	else
	{
		tf_err( "OUT", TF_ERR_ARG );
	}
}

/******************************************************************************
 * @brief TEST (communication / peripheral) commands.
 *****************************************************************************/
static void tf_cmd_test( const char * s )
{
	char sub[ 12 ];
	char arg[ 12 ];

	if( !tf_word( s, 1, sub, sizeof( sub ) ) )
	{
		tf_err( "TEST", TF_ERR_ARG );
		return;
	}
	if( ( TF_GET_IDLE != tf.get.state )
		&& ( ( 0 == strcmp( sub, "RTD" ) ) || ( 0 == strcmp( sub, "TC" ) ) ) )
	{
		tf_err( "TEST", TF_ERR_CAL_STATE );
		return;
	}

	if( 0 == strcmp( sub, "CAN" ) )
	{
		uint8_t data[ MAX_CAN_MSG_LEN ] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
		bool ok = CAN.SendMessage( 0x100, false, MAX_CAN_MSG_LEN, data );
		if( ok )	test_func_emit( "<TEST OK\n" );
		else		tf_err( "TEST", TF_ERR_INTERNAL );
	}
	else if( 0 == strcmp( sub, "UART" ) )
	{
		if( !tf_find_val( s, "PORT", arg, sizeof( arg ) ) )
		{
			tf_err( "TEST", TF_ERR_ARG );
			return;
		}
		UART_PORT_et port = ( 0 == strcmp( arg, "SMPS" ) ) ? UART_SMPS : UART_PLC;
		static const char msg[] = "TCM1 UART TEST\r\n";
		UART.Write( port, ( uint8_t * )msg, ( int32_t )( sizeof( msg ) - 1 ) );
		test_func_emit( "<TEST OK\n" );
	}
	else if( 0 == strcmp( sub, "EEPROM" ) )
	{
		uint8_t data[ 10 ] = { 0 };
		if( 0 == EEPR.Read( 0, data, sizeof( data ) ) )
		{
			tf_err( "TEST", TF_ERR_EEPROM );
			return;
		}
		test_func_emit( "<TEST OK DATA=%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X\n",
			data[ 0 ], data[ 1 ], data[ 2 ], data[ 3 ], data[ 4 ],
			data[ 5 ], data[ 6 ], data[ 7 ], data[ 8 ], data[ 9 ] );
	}
	else if( 0 == strcmp( sub, "RTD" ) )
	{
		float ref = 0.0f;
		int32_t ch = 0;

		if( !tf_get_int( s, "CH", &ch ) || ( ch < 0 ) || ( ch >= TF_MEAS_CHANNELS ) )
		{
			tf_err( "TEST", TF_ERR_RANGE );
			return;
		}
		if( !tf_get_float( s, "REF", &ref ) )
		{
			tf_err( "TEST", TF_ERR_ARG );
			return;
		}

		float tol = fabsf( ref ) * TF_RTD_TEST_TOL_RATIO;
		(void)tf_get_float( s, "TOL", &tol );

		Temp.SetType( ( int16_t )ch, SEN_RTD, DEFAULT_SAMPLING_PERIOD_MS );
		float meas = Temp.GetTemp( ( int16_t )ch );
		float err  = fabsf( meas - ref );
		bool  pass = ( err <= tol );

		test_func_emit( "!TEST TYPE=RTD CH=%ld REF=%.3f MEAS=%.3f ERR=%.3f TOL=%.3f RESULT=%s\n",
			( long )ch, ref, meas, err, tol, pass ? "PASS" : "FAIL" );

		if( pass )
		{
			test_func_emit( "<TEST OK TYPE=RTD CH=%ld\n", ( long )ch );
		}
		else
		{
			tf_err( "TEST", TF_ERR_CAL_VALIDATION );
		}
	}
	else if( 0 == strcmp( sub, "TC" ) )
	{
		float ref = 0.0f;
		int32_t ch = 0;
		sensor_et type = TF_TC_DEFAULT_TYPE;

		if( !tf_get_int( s, "CH", &ch ) || ( ch < 0 ) || ( ch >= TF_MEAS_CHANNELS ) )
		{
			tf_err( "TEST", TF_ERR_RANGE );
			return;
		}
		if( !tf_get_float( s, "REF", &ref ) )
		{
			tf_err( "TEST", TF_ERR_ARG );
			return;
		}

		if( tf_find_val( s, "TYPE", arg, sizeof( arg ) ) )
		{
			type = tf_sensor_from_name( arg );
			if( !tf_is_tc_type( type ) )
			{
				tf_err( "TEST", TF_ERR_ARG );
				return;
			}
		}

		float tol = TF_TC_TEST_TOLERANCE_C;
		(void)tf_get_float( s, "TOL", &tol );

		Temp.SetType( ( int16_t )ch, type, DEFAULT_SAMPLING_PERIOD_MS );
		float meas = Temp.GetTCTemp( type, ( uint8_t )ch, 0.0f );
		float err  = fabsf( meas - ref );
		bool  pass = ( err <= tol );

		test_func_emit( "!TEST TYPE=TC CH=%ld REF=%.3f MEAS=%.3f ERR=%.3f TOL=%.3f RESULT=%s\n",
			( long )ch, ref, meas, err, tol, pass ? "PASS" : "FAIL" );

		if( pass )
		{
			test_func_emit( "<TEST OK TYPE=TC CH=%ld\n", ( long )ch );
		}
		else
		{
			tf_err( "TEST", TF_ERR_CAL_VALIDATION );
		}
	}
	else
	{
		tf_err( "TEST", TF_ERR_ARG );
	}
}

/******************************************************************************
 * @brief MEAS (continuous temperature stream) commands.
 *****************************************************************************/
static void tf_cmd_meas( const char * s )
{
	char sub[ 8 ];
	char arg[ 8 ];

	if( !tf_word( s, 1, sub, sizeof( sub ) ) )
	{
		tf_err( "MEAS", TF_ERR_ARG );
		return;
	}

	if( 0 == strcmp( sub, "START" ) )
	{
		if( ( TF_GET_IDLE != tf.get.state ) || tf.get.override_active || Calib.status.On )
		{
			tf_err( "MEAS", TF_ERR_CAL_STATE );
			return;
		}
		if( !tf_find_val( s, "TYPE", arg, sizeof( arg ) ) )
		{
			tf_err( "MEAS", TF_ERR_ARG );
			return;
		}
		tf.meas.type = tf_sensor_from_name( arg );
		for( int16_t ch = 0; ch < TF_MEAS_CHANNELS; ch++ )
		{
			Temp.SetType( ch, tf.meas.type, DEFAULT_SAMPLING_PERIOD_MS );
		}
		tf.meas.on = true;
		AppTimer.Start( &tf_meas_timer, TF_MEAS_PERIOD );
		test_func_emit( "<MEAS OK\n" );
	}
	else if( 0 == strcmp( sub, "STOP" ) )
	{
		tf.meas.on = false;
		AppTimer.Stop( &tf_meas_timer );
		test_func_emit( "<MEAS OK\n" );
	}
	else
	{
		tf_err( "MEAS", TF_ERR_ARG );
	}
}

/******************************************************************************
 * @brief LOG START/STOP/STATUS for the CL1 and CHPP-8021 CSV stream.
 *****************************************************************************/
static void tf_cmd_log( const char * s )
{
	char sub[ 8 ];
	const control_loop_config_st * const pCfg = Controller.GetConfig( 0 );
	int32_t smps_id = 0;
	int32_t force = 0;

	if( !tf_word( s, 1, sub, sizeof( sub ) ) )
	{
		test_func_emit( "<LOG ERROR CODE=ARG MESSAGE=MISSING_SUBCOMMAND\r\n" );
		return;
	}

	if( 0 == strcmp( sub, "START" ) )
	{
		if( tf.log.on )
		{
			test_func_emit( "<LOG ERROR CODE=ALREADY_RUNNING MESSAGE=LOG_IS_RUNNING\r\n" );
			return;
		}
		if( NULL == pCfg )
		{
			test_func_emit( "<LOG ERROR CODE=CONFIG MESSAGE=CL1_CONFIG_UNAVAILABLE\r\n" );
			return;
		}
		bool has_smps_id = tf_get_int( s, "SMPS_ID", &smps_id );
		bool has_force = tf_get_int( s, "FORCE", &force );

		if( has_smps_id || has_force )
		{
			if( !has_force || ( 1 != force ) )
			{
				test_func_emit( "<LOG ERROR CODE=ARG MESSAGE=FORCE_REQUIRED\r\n" );
				return;
			}
			if( !has_smps_id )
			{
				test_func_emit( "<LOG ERROR CODE=ARG MESSAGE=SMPS_ID_REQUIRED\r\n" );
				return;
			}
			if( ( smps_id < 1 ) || ( smps_id >= MAX_SMPS_ID ) )
			{
				test_func_emit( "<LOG ERROR CODE=RANGE MESSAGE=SMPS_ID_OUT_OF_RANGE VALID_MIN=1 VALID_MAX=%u\r\n",
					( unsigned int )( MAX_SMPS_ID - 1 ) );
				return;
			}
			tf.log.manual = true;
		}
		else
		{
			if( OUT_SMPS_CHPP_8021 != pCfg->OutputType )
			{
				test_func_emit( "<LOG ERROR CODE=CONFIG MESSAGE=CL1_OUTPUT_NOT_CHPP8021 OUTPUT_TYPE=%u\r\n",
					( unsigned int )pCfg->OutputType );
				return;
			}
			if( ( pCfg->OutputChannel < 1 ) || ( pCfg->OutputChannel >= MAX_SMPS_ID ) )
			{
				test_func_emit( "<LOG ERROR CODE=CONFIG MESSAGE=CL1_OUTPUT_CHANNEL_INVALID OUTPUT_CHANNEL=%d VALID_MIN=1 VALID_MAX=%u\r\n",
					( int )pCfg->OutputChannel, ( unsigned int )( MAX_SMPS_ID - 1 ) );
				return;
			}
			smps_id = pCfg->OutputChannel;
			tf.log.manual = false;
		}

		tf.log.smps_id = ( uint8_t )smps_id;
		tf.log.sample_seq = 0;
		tf.log.sent_samples = 0;
		tf.log.usb_dropped_count = 0;
		tf.log.on = true;
		AppTimer.Start( &tf_log_timer, TF_LOG_PERIOD );
		if( tf.log.manual )
		{
			OutputRequestSmpsRead( OUT_SMPS_CHPP_8021, tf.log.smps_id );
		}

		test_func_emit( "<LOG START OK PERIOD_MS=100 COLUMNS=%d CL=1 SMPS_ID=%u SOURCE=%s\r\n",
			TF_LOG_COLUMNS, ( unsigned int )tf.log.smps_id,
			tf.log.manual ? "MANUAL" : "CL_CONFIG" );
		if( CDC.Write( ( const uint8_t * )tf_log_csv_header,
			( uint32_t )strlen( tf_log_csv_header ) ) != strlen( tf_log_csv_header ) )
		{
			tf.log.usb_dropped_count++;
		}
	}
	else if( 0 == strcmp( sub, "STOP" ) )
	{
		tf.log.on = false;
		AppTimer.Stop( &tf_log_timer );
		test_func_emit( "<LOG STOP OK SAMPLES=%lu DROPPED=%lu\r\n",
			( unsigned long )tf.log.sent_samples,
			( unsigned long )tf.log.usb_dropped_count );
	}
	else if( 0 == strcmp( sub, "STATUS" ) )
	{
		test_func_emit( "<LOG STATUS STATE=%s PERIOD_MS=100 CL=1 SMPS_ID=%u SOURCE=%s SAMPLES=%lu DROPPED=%lu\r\n",
			tf.log.on ? "RUN" : "STOP",
			( unsigned int )tf.log.smps_id,
			tf.log.manual ? "MANUAL" : "CL_CONFIG",
			( unsigned long )tf.log.sent_samples,
			( unsigned long )tf.log.usb_dropped_count );
	}
	else
	{
		test_func_emit( "<LOG ERROR CODE=ARG MESSAGE=UNKNOWN_SUBCOMMAND\r\n" );
	}
}

/******************************************************************************
 * @brief Capture and enqueue one 44-column CSV snapshot.
 *****************************************************************************/
static void tf_log_stream_task( void )
{
	if( !tf.log.on || !AppTimer.IsExpired( &tf_log_timer ) )
	{
		return;
	}

	AppTimer.Start( &tf_log_timer, TF_LOG_PERIOD );
	if( tf.log.manual )
	{
		/* Manual logging is read-only: refresh the selected SMPS data without
		 * changing its run state or commanded output. */
		OutputRequestSmpsRead( OUT_SMPS_CHPP_8021, tf.log.smps_id );
	}

	const control_loop_config_st * const pCfg = Controller.GetConfig( 0 );
	if( ( NULL == pCfg ) || ( tf.log.smps_id >= MAX_SMPS_ID ) )
	{
		tf.log.on = false;
		AppTimer.Stop( &tf_log_timer );
		test_func_emit( "<LOG ERROR CODE=CONFIG MESSAGE=CL1_CONFIG_LOST\r\n" );
		return;
	}

	uint8_t smps_id = tf.log.smps_id;
	const uint16_t * const rd = MBM_READ_DB[ smps_id ].Holdings;
	const uint16_t * const wr = MBM_WRITE_DB[ smps_id ].Holdings;
	int32_t sv = pCfg->SV;
	int32_t pv = Controller.GetPV( 0 );
	int32_t error = sv - pv;
	int32_t mv_permille = ( int32_t )( Controller.GetMV( 0 ) * MULTIPLY_FLOAT_TO_PERMIL );
	int32_t rtd_ch1_mdegc = ( int32_t )( Temp.GetTemp( 0 ) * 1000.0f );
	int32_t rtd_ch2_mdegc = ( int32_t )( Temp.GetTemp( 1 ) * 1000.0f );
	uint32_t seq = ++tf.log.sample_seq;

	int n = snprintf( tf_log_line, sizeof( tf_log_line ),
		"%lu,%lu,%lu,%u,%u,%u,%u,%d,%u,%d,"
		"%u,%u,%u,%u,%d,%d,%u,%u,%u,%u,"
		"%ld,%ld,%u,%ld,%ld,%ld,%ld,%u,"
		"%lu,%lu,%d,%u,%u,%u,"
		"%u,%u,%u,%u,%d,%d,%d,%d,%d,%u\r\n",
		( unsigned long )HAL_GetTick(),
		( unsigned long )seq,
		( unsigned long )tf.log.usb_dropped_count,
		( unsigned int )!!CFG.system.Run,
		( unsigned int )!!MBSDB.Inputs[ MBS_SYSTEM_READY ],
		( unsigned int )!!pCfg->Enable,
		( unsigned int )pCfg->InputType,
		( int )pCfg->InputChannel,
		( unsigned int )pCfg->OutputType,
		( int )pCfg->OutputChannel,
		( unsigned int )pCfg->ControlPeriod,
		( unsigned int )pCfg->Pb,
		( unsigned int )pCfg->Ti,
		( unsigned int )pCfg->Td,
		( int )pCfg->OutputMin,
		( int )pCfg->OutputMax,
		( unsigned int )pCfg->ControlType,
		( unsigned int )pCfg->CoolHeat,
		( unsigned int )pCfg->OutputDelay,
		( unsigned int )pCfg->StartDelay,
		( long )rtd_ch1_mdegc,
		( long )rtd_ch2_mdegc,
		( unsigned int )smps_id,
		( long )sv,
		( long )pv,
		( long )error,
		( long )mv_permille,
		( unsigned int )Controller.GetFault( 0 ).All,
		( unsigned long )ModbusMasterGetReadSequence( smps_id ),
		( unsigned long )ModbusMasterGetDataAgeMs( smps_id ),
		( int )( int16_t )wr[ MBM_HR_CHPP_8021_OUTPUT_POWER ],
		( unsigned int )rd[ MBM_HR_CHPP_8021_STATUS_OF_SMPS ],
		( unsigned int )rd[ MBM_HR_CHPP_8021_WARNING_STATUS_OF_SMPS ],
		( unsigned int )rd[ MBM_HR_CHPP_8021_FAULT_STATUS_OF_SMPS ],
		( unsigned int )rd[ MBM_HR_CHPP_8021_CH1_LEAKAGE_CURRENT ],
		( unsigned int )rd[ MBM_HR_CHPP_8021_CH2_LEAKAGE_CURRENT ],
		( unsigned int )rd[ MBM_HR_CHPP_8021_CH3_LEAKAGE_CURRENT ],
		( unsigned int )rd[ MBM_HR_CHPP_8021_CH4_LEAKAGE_CURRENT ],
		( int )( int16_t )rd[ MBM_HR_CHPP_8021_CH1_OUTPUT_CURRENT ],
		( int )( int16_t )rd[ MBM_HR_CHPP_8021_CH2_OUTPUT_CURRENT ],
		( int )( int16_t )rd[ MBM_HR_CHPP_8021_CH3_OUTPUT_CURRENT ],
		( int )( int16_t )rd[ MBM_HR_CHPP_8021_CH4_OUTPUT_CURRENT ],
		( int )( int16_t )rd[ MBM_HR_CHPP_8021_OUTPUT_VOLTAGE_MEASURED ],
		( unsigned int )rd[ MBM_HR_CHPP_8021_OUTPUT_POWER_MEASURED ] );

	if( ( n <= 0 ) || ( n >= ( int )sizeof( tf_log_line ) )
		|| ( CDC.Write( ( const uint8_t * )tf_log_line, ( uint32_t )n ) != ( uint32_t )n ) )
	{
		tf.log.usb_dropped_count++;
		return;
	}
	tf.log.sent_samples++;
}

/******************************************************************************
 * @brief Emit one completed calibration point while preserving the legacy
 *        CONV field and exposing the separately measured RTD2X value.
 *****************************************************************************/
static void tf_emit_cal_point_result( const char * prefix, sensor_et type,
		uint8_t ch, uint8_t idx )
{
	if( ( ( SEN_RTD == type ) || ( SEN_RTD2X == type ) )
		&& ( idx <= CALIB_RTD_2X_SECTION ) )
	{
		test_func_emit( "%s CH=%d IDX=%d CONV=%.3f CONV_RTD2X=%.3f\n",
			prefix, ch, idx,
			Calib.GetConv( SEN_RTD, ch, idx ),
			Calib.GetConv( SEN_RTD2X, ch, idx ) );
	}
	else
	{
		test_func_emit( "%s CH=%d IDX=%d CONV=%.3f\n",
			prefix, ch, idx, Calib.GetConv( type, ch, idx ) );
	}
}

/******************************************************************************
 * @brief Emit <CAL DONE with all measured conversion values appended.
 *        ABORT does not call this — it uses plain test_func_emit("<CAL DONE").
 *****************************************************************************/
static void tf_emit_cal_done_with_conv( void )
{
	char buf[ TF_CAL_DONE_LINE_MAX ];
	int pos;
	int n;
	bool is_rtd;

	is_rtd = ( SEN_RTD == tf.cal.type ) || ( SEN_RTD2X == tf.cal.type );
	n = is_rtd ? ( CALIB_RTD_SECTION + 1 ) : ( CALIB_TC_SECTION + 1 );

	pos = snprintf( buf, sizeof( buf ), "<CAL DONE CH=%d", tf.cal.ch );
	for( int i = 0; i < n && pos < ( int )sizeof( buf ) - 14; i++ )
	{
		pos += snprintf( buf + pos, sizeof( buf ) - pos,
						 " CONV%d=%.3f", i,
						 Calib.GetConv( is_rtd ? SEN_RTD : tf.cal.type,
							tf.cal.ch, ( uint8_t )i ) );
	}
	if( is_rtd )
	{
		for( int i = 0; i <= CALIB_RTD_2X_SECTION && pos < ( int )sizeof( buf ) - 22; i++ )
		{
			pos += snprintf( buf + pos, sizeof( buf ) - pos,
							 " CONV_RTD2X%d=%.3f", i,
							 Calib.GetConv( SEN_RTD2X, tf.cal.ch, ( uint8_t )i ) );
		}
	}
	if( pos < ( int )sizeof( buf ) - 1 )
	{
		buf[ pos++ ] = '\n';
	}
	CDC.Write( ( const uint8_t * )buf, ( uint32_t )pos );
}

/******************************************************************************
 * @brief Begin a calibration session bridged to Calib.Run().
 *****************************************************************************/
static void tf_cal_start( const char * s )
{
	char arg[ 8 ];
	char mode[ 12 ];
	int32_t ch = 0;

	if( tf_find_val( s, "MODE", mode, sizeof( mode ) ) )
	{
		calib_session_mode_et session_mode;
		uint8_t ch_mask;

		if( tf.cal.on || tf.cal.point_on || tf.cal.auto_on || tf.cal.save_on
			|| ( TF_GET_IDLE != tf.get.state ) || tf.get.override_active )
		{
			tf_err( "CAL", TF_ERR_CAL_STATE );
			return;
		}
		if( 0 == strcmp( mode, "GROUP" ) )
		{
			session_mode = CALIB_SESSION_GROUP;
			ch_mask = 0x0F;
			ch = 0;
		}
		else if( 0 == strcmp( mode, "CHANNEL" ) )
		{
			if( !tf_get_int( s, "CH", &ch ) || ( ch < 0 ) || ( ch >= TF_MEAS_CHANNELS ) )
			{
				tf_err( "CAL", TF_ERR_RANGE );
				return;
			}
			session_mode = CALIB_SESSION_CHANNEL;
			ch_mask = ( 1u << ch );
		}
		else
		{
			tf_err( "CAL", TF_ERR_ARG );
			return;
		}

		if( !Calib.StartSession( session_mode, ( uint8_t )ch ) )
		{
			tf_err( "CAL", tf_cal_save_error_code() );
			return;
		}
		tf.cal.auto_on = true;
		tf.cal.auto_mode = session_mode;
		tf.cal.auto_ch_mask = ch_mask;
		tf.cal.pending_commit = false;
		test_func_emit( ( CALIB_SESSION_GROUP == session_mode )
			? "<CAL OK MODE=GROUP\n"
			: "<CAL OK MODE=CHANNEL CH=%ld\n", ( long )ch );
		return;
	}

	if( !tf_find_val( s, "TYPE", arg, sizeof( arg ) )
		|| !tf_get_int( s, "CH", &ch ) || ( ch < 0 ) || ( ch >= TF_MEAS_CHANNELS ) )
	{
		tf_err( "CAL", TF_ERR_ARG );
		return;
	}
	if( tf.cal.on || tf.cal.point_on || tf.cal.auto_on || tf.cal.save_on
		|| ( TF_GET_IDLE != tf.get.state ) || tf.get.override_active )
	{
		tf_err( "CAL", TF_ERR_CAL_STATE );
		return;
	}

	tf.cal.type = ( 0 == strcmp( arg, "TC" ) ) ? TF_TC_DEFAULT_TYPE : tf_sensor_from_name( arg );
	tf.cal.ch = ( uint8_t )ch;

	/* Starting a new calibration invalidates the board's complete-calib flag. */
	if( !Calib.ClearDoneFlag() )
	{
		tf_err( "CAL", TF_ERR_EEPROM );
		return;
	}

	/* Reset the static state machine inside calibration_task before starting a
	 * new session. Without this, leftover stage/idx from a previous session can
	 * cause ENTER to land on the wrong calibration point. */
	Calib.Run( tf.cal.type, tf.cal.ch, ( char )0x1B );

	tf.cal.on = true;
	tf.cal.pending_commit = false;

	Calib.status.ch = ( uint8_t )ch;
	Calib.status.On = 1;

	AppTimer.Start( &tf_cal_hb_timer, TF_CAL_HB_PERIOD );
	test_func_emit( "<CAL OK CH=%ld\n", ( long )ch );
}

/******************************************************************************
 * @brief Complete a pending single-channel GET after the new sensor mode has
 *        produced a filtered ADC value and TemperatureTask has updated it.
 *****************************************************************************/
static void tf_pending_get_task( void )
{
	uint8_t dev;
	uint8_t ach;
	uint32_t generation;
	int32_t raw;
	int32_t value;
	uint32_t drdy;
	uint32_t adc_error;
	int valid_count;
	const char * type_name;
	const char * state_name;
	uint8_t ch;
	sensor_et type;
	float temp;
	float mv = 0.0f;

	if( TF_GET_IDLE == tf.get.state )
	{
		return;
	}

	ch = tf.get.ch;
	type = tf.get.type;
	dev = ch / MAX_ADC_CH;
	ach = ch % MAX_ADC_CH;
	type_name = tf_cal_type_name( type );
	state_name = ( TF_GET_SETTLING == tf.get.state ) ? "SETTLING" : "WAIT_SAMPLE";
	generation = Temp.GetSampleGeneration( ch );
	raw = AD7124.GetRawValue( ch );
	value = AD7124.GetValue( ch );
	drdy = ( 0 == ( AD7124.dev[ dev ].regs[ AD7124_Status ].value & AD7124_STATUS_REG_RDY ) ) ? 1u : 0u;
	adc_error = ( uint32_t )AD7124.dev[ dev ].regs[ AD7124_Error ].value;
	valid_count = AD7124.dev[ dev ].mf.valid_count[ ach ];

	if( tf_control_run_is_active() )
	{
		tf_pending_get_finish( false );
		test_func_emit( "<GET ERR %d TYPE=%s CH=%d REASON=RUN_ACTIVE\n",
			TF_ERR_CAL_STATE, type_name, ch );
		return;
	}

	if( AppTimer.IsExpired( &tf_get_timeout_timer ) )
	{
		tf_pending_get_finish( true );
		test_func_emit( "<GET ERR %d TYPE=%s CH=%d DEV=%d ACH=%d STATE=%s VALID=%d GEN=%lu RAW=%ld VALUE=%ld DRDY=%lu ADCERR=0x%06lX\n",
			TF_ERR_INTERNAL, type_name, ch, dev, ach, state_name,
			valid_count, ( unsigned long )generation,
			( long )raw, ( long )value, ( unsigned long )drdy, ( unsigned long )adc_error );
		return;
	}

	if( TF_GET_SETTLING == tf.get.state )
	{
		if( AppTimer.IsExpired( &tf_get_settle_timer )
			&& ( AD7124.GetType( ch ) == type ) )
		{
			tf.get.start_generation = generation;
			tf.get.state = TF_GET_WAIT_SAMPLE;
			AppTimer.Start( &tf_get_hb_timer, TF_GET_HB_PERIOD );
			test_func_emit( "!GET TYPE=%s CH=%d STATE=WAIT_SAMPLE GEN=%lu RAW=%ld VALUE=%ld\n",
				type_name, ch, ( unsigned long )generation, ( long )raw, ( long )value );
			return;
		}
		if( AppTimer.IsExpired( &tf_get_settle_timer ) )
		{
			tf_pending_get_finish( true );
			test_func_emit( "<GET ERR %d TYPE=%s CH=%d REASON=TYPE_MISMATCH\n",
				TF_ERR_INTERNAL, type_name, ch );
			return;
		}

		if( AppTimer.IsExpired( &tf_get_hb_timer ) )
		{
			AppTimer.Start( &tf_get_hb_timer, TF_GET_HB_PERIOD );
			test_func_emit( "!GET TYPE=%s CH=%d STATE=SETTLING GEN=%lu RAW=%ld VALUE=%ld\n",
				type_name, ch, ( unsigned long )generation, ( long )raw, ( long )value );
		}
		return;
	}

	if( AD7124.GetType( ch ) != type )
	{
		tf_pending_get_finish( true );
		test_func_emit( "<GET ERR %d TYPE=%s CH=%d REASON=TYPE_MISMATCH\n",
			TF_ERR_INTERNAL, type_name, ch );
		return;
	}

	if( generation == tf.get.start_generation )
	{
		if( AppTimer.IsExpired( &tf_get_hb_timer ) )
		{
			AppTimer.Start( &tf_get_hb_timer, TF_GET_HB_PERIOD );
			test_func_emit( "!GET TYPE=%s CH=%d STATE=WAIT_SAMPLE GEN=%lu RAW=%ld VALUE=%ld\n",
				type_name, ch, ( unsigned long )generation, ( long )raw, ( long )value );
		}
		return;
	}

	temp = tf_get_meas_temp( type, ( int16_t )ch );
	if( ( SEN_RTD != type ) && ( SEN_RTD2X != type ) )
	{
		mv = Temp.GetTCmV( ch );
	}
	tf_emit_get_cal_diag( ch, type, value );
	tf_pending_get_finish( true );

	if( ( SEN_RTD == type ) || ( SEN_RTD2X == type ) )
	{
		test_func_emit( "<GET OK TYPE=RTD CH=%d TEMP=%.3f RAW=%ld GEN=%lu\n",
			ch, temp, ( long )raw, ( unsigned long )generation );
	}
	else
	{
		test_func_emit( "<GET OK TYPE=TC CH=%d TEMP=%.3f RAW=%ld MV=%.3f CJ=0.000 GEN=%lu\n",
			ch, temp, ( long )raw, mv, ( unsigned long )generation );
	}
}

/******************************************************************************
 * @brief Emit the complete calibration path used by a single GET result.
 *
 * This must run before tf_pending_get_finish() restores the original sensor
 * type; otherwise the AD7124 registers may already contain another type's
 * gain/offset values.
 *****************************************************************************/
static void tf_emit_get_cal_diag( uint8_t ch, sensor_et type, int32_t value )
{
	Calib_Appl_st * app;
	Calib_AD7124_st * adc_cal;
	uint8_t section_count;
	uint8_t selected;
	uint8_t adc_type;
	uint8_t dev;
	uint8_t ach;
	float converted;
	float calibrated;

	if( ch >= TF_MEAS_CHANNELS ) return;

	if( SEN_RTD2X == type )
	{
		app = Calib.data[ ch ].RTD2X;
		section_count = CALIB_RTD_2X_SECTION;
		adc_type = SEN_RTD2X;
		converted = Temp.ConvToRes( type, ( uint32_t )value, AD7124.GetSamplePeriod( ch ) );
	}
	else if( SEN_RTD == type )
	{
		app = Calib.data[ ch ].RTD;
		section_count = CALIB_RTD_SECTION;
		adc_type = SEN_RTD;
		converted = Temp.ConvToRes( type, ( uint32_t )value, AD7124.GetSamplePeriod( ch ) );
	}
	else
	{
		app = Calib.data[ ch ].TC;
		section_count = CALIB_TC_SECTION;
		adc_type = SEN_TC_K;
		converted = Temp.ConvTomV( value, AD7124.GetSamplePeriod( ch ) );
	}

	selected = section_count - 1u;
	for( uint8_t i = 0; i < section_count; i++ )
	{
		if( value <= app[ i ].boundary )
		{
			selected = i;
			break;
		}
	}
	calibrated = ( converted - app[ selected ].offset ) * app[ selected ].gain;

	dev = ch / MAX_ADC_CH;
	ach = ch % MAX_ADC_CH;
	adc_cal = &Calib.data[ ch ].AD7124[ adc_type ];

	test_func_emit( "!GET DIAG TYPE=%s CH=%d VALUE=%ld SAMPLE_MS=%u APP_IN=%.6f APP_OUT=%.6f APP_IDX=%u\n",
		tf_cal_type_name( type ), ch, ( long )value,
		( unsigned int )AD7124.GetSamplePeriod( ch ), converted, calibrated, selected );
	test_func_emit( "!GET DIAG TYPE=%s CH=%d ADC_CAL_GAIN=0x%06lX ADC_CAL_OFFSET=0x%06lX ADC_REG_GAIN=0x%06lX ADC_REG_OFFSET=0x%06lX\n",
		tf_cal_type_name( type ), ch,
		( unsigned long )adc_cal->gain, ( unsigned long )adc_cal->offset,
		( unsigned long )AD7124.GetRegister( dev, AD7124_Gain_0 + ach ),
		( unsigned long )AD7124.GetRegister( dev, AD7124_Offset_0 + ach ) );
	test_func_emit( "!GET DIAG TYPE=%s CH=%d DEV=%u ACH=%u CHREG=0x%04lX CONFIG=0x%04lX FILTER=0x%06lX IOCON1=0x%06lX ERROR=0x%06lX\n",
		tf_cal_type_name( type ), ch, dev, ach,
		( unsigned long )AD7124.GetRegister( dev, AD7124_Channel_0 + ach ),
		( unsigned long )AD7124.GetRegister( dev, AD7124_Config_0 + ach ),
		( unsigned long )AD7124.GetRegister( dev, AD7124_Filter_0 + ach ),
		( unsigned long )AD7124.GetRegister( dev, AD7124_IOCon1 ),
		( unsigned long )AD7124.GetRegister( dev, AD7124_Error ) );

	for( uint8_t i = 0; i < section_count; i++ )
	{
		test_func_emit( "!GET DIAG TYPE=%s CH=%d APP_IDX=%u SELECTED=%u BOUNDARY=%ld GAIN=%.9f OFFSET=%.9f\n",
			tf_cal_type_name( type ), ch, i, ( i == selected ) ? 1u : 0u,
			( long )app[ i ].boundary, app[ i ].gain, app[ i ].offset );
	}
}

/******************************************************************************
 * @brief Parse a calibration TYPE= argument and normalize it for storage.
 *****************************************************************************/
static bool tf_cal_parse_type( const char * s, sensor_et * type )
{
	char arg[ 8 ];

	if( !tf_find_val( s, "TYPE", arg, sizeof( arg ) ) )
	{
		return false;
	}

	*type = ( 0 == strcmp( arg, "TC" ) ) ? TF_TC_DEFAULT_TYPE : tf_sensor_from_name( arg );

	if( ( SEN_RTD == *type ) || ( SEN_RTD2X == *type ) )
	{
		*type = SEN_RTD;
		return true;
	}
	if( tf_is_tc_type( *type ) )
	{
		*type = TF_TC_DEFAULT_TYPE;
		return true;
	}

	return false;
}

/******************************************************************************
 * @brief Number of measurement point indexes for a calibration type.
 *****************************************************************************/
static uint8_t tf_cal_point_max_idx( sensor_et type )
{
	return ( SEN_RTD == type ) ? CALIB_RTD_SECTION : CALIB_TC_SECTION;
}

/******************************************************************************
 * @brief Begin one explicit calibration point measurement.
 *****************************************************************************/
static void tf_cal_point_start( const char * s )
{
	sensor_et type;
	int32_t ch = 0;
	int32_t idx = 0;

	if( !tf_cal_parse_type( s, &type )
		|| !tf_get_int( s, "CH", &ch )
		|| !tf_get_int( s, "IDX", &idx ) )
	{
		tf_err( "CAL", TF_ERR_ARG );
		return;
	}
	if( tf.cal.on || tf.cal.point_on || tf.cal.save_on
		|| ( TF_GET_IDLE != tf.get.state ) || tf.get.override_active )
	{
		tf_err( "CAL", TF_ERR_CAL_STATE );
		return;
	}
	if( ( ch < 0 ) || ( ch >= TF_MEAS_CHANNELS )
		|| ( idx < 0 ) || ( idx > tf_cal_point_max_idx( type ) ) )
	{
		tf_err( "CAL", TF_ERR_RANGE );
		return;
	}
	if( !tf.cal.auto_on || ( 0 == ( tf.cal.auto_ch_mask & ( 1u << ch ) ) ) )
	{
		tf_err( "CAL", TF_ERR_CAL_STATE );
		return;
	}

	tf.cal.point_type = type;
	tf.cal.point_ch = ( uint8_t )ch;
	tf.cal.point_idx = ( uint8_t )idx;
	tf.cal.point_on = true;
	tf.cal.pending_commit = false;

	if( Calib.MeasurePoint( type, tf.cal.point_ch, tf.cal.point_idx, ( char )0x0D ) )
	{
		tf.cal.point_on = false;
		Calib.status.On = 0;
		tf_emit_cal_point_result( "<CAL POINT OK", type, ( uint8_t )ch, ( uint8_t )idx );
		return;
	}

	AppTimer.Start( &tf_cal_hb_timer, TF_CAL_HB_PERIOD );
	test_func_emit( "<CAL OK CH=%ld IDX=%ld\n", ( long )ch, ( long )idx );
}

/******************************************************************************
 * @brief Validate and save multiple channels selected by CHMASK.
 *****************************************************************************/
static void tf_cal_success_mask( const char * s )
{
	sensor_et type;
	int32_t mask = 0;
	int32_t diag = 0;

	Calib.status.save_diag_on = 0;

	if( !tf_cal_parse_type( s, &type ) || !tf_get_int( s, "CHMASK", &mask ) )
	{
		tf_err( "CAL", TF_ERR_ARG );
		return;
	}
	if( tf.cal.on || tf.cal.point_on || tf.cal.save_on )
	{
		tf_err( "CAL", TF_ERR_CAL_STATE );
		return;
	}
	if( !tf.cal.auto_on || ( ( uint8_t )mask != tf.cal.auto_ch_mask ) )
	{
		tf_err( "CAL", TF_ERR_CAL_STATE );
		return;
	}
	if( ( mask <= 0 ) || ( 0 != ( mask & ~0x0F ) ) )
	{
		tf_err( "CAL", TF_ERR_RANGE );
		return;
	}

	tf.cal.save_type = type;
	tf.cal.save_mask = ( uint8_t )mask;
	tf.cal.saved_mask = 0;
	tf.cal.save_ch = 0;
	tf.cal.save_diag_on = ( tf_get_int( s, "DIAG", &diag ) && ( 0 != diag ) );
	tf.cal.save_on = true;
}

/******************************************************************************
 * @brief Save at most one calibration channel per super-loop iteration.
 *****************************************************************************/
static void tf_cal_save_task( void )
{
	const char *type_name;

	if( !tf.cal.save_on )
	{
		return;
	}

	type_name = tf_cal_type_name( tf.cal.save_type );
	while( ( tf.cal.save_ch < TF_MEAS_CHANNELS )
		&& ( 0 == ( tf.cal.save_mask & ( 1u << tf.cal.save_ch ) ) ) )
	{
		tf.cal.save_ch++;
	}

	if( tf.cal.save_ch >= TF_MEAS_CHANNELS )
	{
		tf.cal.save_on = false;
		Calib.status.save_diag_on = 0;
		test_func_emit( "<CAL OK TYPE=%s CHMASK=0x%02X\n", type_name, tf.cal.saved_mask );
		return;
	}

	uint8_t ch = tf.cal.save_ch++;
	Calib.status.save_diag_on = tf.cal.save_diag_on ? 1 : 0;
	test_func_emit( "!CAL SAVE TYPE=%s CH=%d STATE=RUN\n", type_name, ch );
	if( !Calib.ValidateAndSave( tf.cal.save_type, ch ) )
	{
		int err = tf_cal_save_error_code();
		test_func_emit( "!CAL SAVE TYPE=%s CH=%d STATE=ERR CODE=%d\n", type_name, ch, err );
		tf.cal.save_on = false;
		Calib.status.save_diag_on = 0;
		tf_err( "CAL", err );
		return;
	}

	tf.cal.saved_mask |= ( 1u << ch );
	test_func_emit( "!CAL SAVE TYPE=%s CH=%d STATE=OK\n", type_name, ch );
}

/******************************************************************************
 * @brief CAL (calibration bridge) commands.
 *****************************************************************************/
static void tf_cmd_cal( const char * s )
{
	char sub[ 12 ];

	if( !tf_word( s, 1, sub, sizeof( sub ) ) )
	{
		tf_err( "CAL", TF_ERR_ARG );
		return;
	}

	if( 0 == strcmp( sub, "START" ) )
	{
		tf_cal_start( s );
	}
	else if( 0 == strcmp( sub, "POINT" ) )
	{
		tf_cal_point_start( s );
	}
	else if( 0 == strcmp( sub, "ENTER" ) )
	{
		if( !tf.cal.on || tf.cal.point_on )
		{
			tf_err( "CAL", TF_ERR_CAL_STATE );
			return;
		}
		if( Calib.Run( tf.cal.type, tf.cal.ch, ( char )0x0D ) )
		{
			tf.cal.on = false;
			Calib.status.On = 0;
			tf.cal.pending_commit = true;
			tf_emit_cal_done_with_conv();
		}
		else
		{
			test_func_emit( "<CAL OK CH=%d\n", tf.cal.ch );
		}
	}
	else if( 0 == strcmp( sub, "ABORT" ) )
	{
		if( tf.cal.on )
		{
			Calib.Run( tf.cal.type, tf.cal.ch, ( char )0x1B );
		}
		if( tf.cal.point_on )
		{
			Calib.MeasurePoint( tf.cal.point_type, tf.cal.point_ch, tf.cal.point_idx, ( char )0x1B );
		}
		tf.cal.on = false;
		tf.cal.point_on = false;
		tf.cal.save_on = false;
		tf.cal.auto_on = false;
		tf.cal.auto_ch_mask = 0;
		tf.cal.save_ch = 0;
		tf.cal.save_mask = 0;
		tf.cal.saved_mask = 0;
		tf.cal.save_diag_on = false;
		tf.cal.pending_commit = false;
		Calib.status.On = 0;
		Calib.status.conv_ready = 0;
		Calib.status.save_diag_on = 0;
		test_func_emit( "<CAL DONE CH=%d\n", tf.cal.ch );
	}
	else if( 0 == strcmp( sub, "COMP?" ) )
	{
		test_func_emit( Calib.IsDone() ? "<CAL OK\n" : "<CAL NO\n" );
	}
	else if( 0 == strcmp( sub, "STATUS?" ) )
	{
		test_func_emit( "<CAL STATUS GLOBAL=%d CHMASK=0x%02X\n",
			Calib.IsDone() ? 1 : 0, Calib.GetChannelDoneMask() );
	}
	else if( 0 == strcmp( sub, "COMPLETE" ) )
	{
		int32_t mask = 0;
		if( tf.cal.on || tf.cal.point_on || tf.cal.save_on || Calib.status.On )
		{
			test_func_emit( "<CAL ERR %d REASON=CAL_BUSY\n", TF_ERR_CAL_STATE );
			return;
		}
		if( tf_get_int( s, "CHMASK", &mask ) )
		{
			if( ( mask <= 0 ) || ( mask > 0x0F )
				|| ( tf.cal.auto_on && ( ( uint8_t )mask != tf.cal.auto_ch_mask ) ) )
			{
				tf_err( "CAL", TF_ERR_RANGE );
				return;
			}
		}
		else
		{
			mask = tf.cal.auto_on ? tf.cal.auto_ch_mask : 0x0F;
		}

		if( !Calib.Complete( ( uint8_t )mask ) )
		{
			int err = tf_cal_save_error_code();
			const char * reason = ( TF_ERR_EEPROM == err ) ? "EEPROM" : "INCOMPLETE";
			test_func_emit( "<CAL ERR %d REASON=%s\n", err, reason );
			return;
		}

		tf.cal.pending_commit = false;
		tf.cal.auto_on = false;
		tf.cal.auto_ch_mask = 0;
		test_func_emit( "<CAL OK COMPLETE=1 CHMASK=0x%02lX GLOBAL=%d\n",
			( unsigned long )mask, Calib.IsDone() ? 1 : 0 );
	}
	else if( 0 == strcmp( sub, "SUCCESS" ) )
	{
		int32_t diag = 0;
		const char * type_name;

		if( tf_find_val( s, "CHMASK", sub, sizeof( sub ) ) )
		{
			tf_cal_success_mask( s );
			return;
		}
		if( !tf.cal.pending_commit )
		{
			tf_err( "CAL", TF_ERR_CAL_STATE );
			return;
		}
		tf.cal.pending_commit = false;
		type_name = tf_cal_type_name( tf.cal.type );
		Calib.status.save_diag_on = ( tf_get_int( s, "DIAG", &diag ) && ( 0 != diag ) ) ? 1 : 0;
		test_func_emit( "!CAL SAVE TYPE=%s CH=%d STATE=RUN\n", type_name, tf.cal.ch );
		if( Calib.ValidateAndSave( tf.cal.type, tf.cal.ch ) )
		{
			test_func_emit( "!CAL SAVE TYPE=%s CH=%d STATE=OK\n", type_name, tf.cal.ch );
			Calib.status.save_diag_on = 0;
			test_func_emit( "<CAL OK CH=%d\n", tf.cal.ch );
		}
		else
		{
			int err = tf_cal_save_error_code();
			test_func_emit( "!CAL SAVE TYPE=%s CH=%d STATE=ERR CODE=%d\n", type_name, tf.cal.ch, err );
			Calib.status.save_diag_on = 0;
			tf_err( "CAL", err );
		}
	}
	else
	{
		tf_err( "CAL", TF_ERR_ARG );
	}
}

/******************************************************************************
 * @brief RESET (dangerous, token protected) commands.
 *****************************************************************************/
static void tf_cmd_reset( const char * s )
{
	char sub[ 12 ];

	if( !tf_word( s, 1, sub, sizeof( sub ) ) )
	{
		tf_err( "RESET", TF_ERR_ARG );
		return;
	}
	if( !tf_token_ok( s ) )
	{
		tf_err( "RESET", TF_ERR_TOKEN );
		return;
	}

	if( 0 == strcmp( sub, "CONFIG" ) )
	{
		if( !tf_reset_eeprom_range( EEPR_CONFIG_ADDR, EEPR_CONFIG_ADDR + EEPR_CONFIG_SZ ) )
		{
			tf_err( "RESET", TF_ERR_EEPROM );
			return;
		}
		tf.token_issued = false;
		test_func_emit( "<RESET OK TARGET=CONFIG\n" );
	}
	else if( 0 == strcmp( sub, "CALIB" ) )
	{
		if( !tf_reset_eeprom_range( EEPR_CALIIB_START_ADDRESS, EEPR_CALIIB_END_ADDRESS ) )
		{
			tf_err( "RESET", TF_ERR_EEPROM );
			return;
		}
		if( !Calib.StartSession( CALIB_SESSION_GROUP, 0 ) )
		{
			tf_err( "RESET", TF_ERR_EEPROM );
			return;
		}
		tf.token_issued = false;
		test_func_emit( "<RESET OK TARGET=CALIB\n" );
	}
	else
	{
		tf_err( "RESET", TF_ERR_ARG );
	}
}

/******************************************************************************
 * @brief REBOOT (dangerous, token protected) command.
 *****************************************************************************/
static void tf_cmd_reboot( const char * s )
{
	if( !tf_token_ok( s ) )
	{
		tf_err( "REBOOT", TF_ERR_TOKEN );
		return;
	}

	tf.token_issued = false;
	test_func_emit( "<REBOOT OK\n" );
	NVIC_SystemReset();
}

/******************************************************************************
 * @brief PRODUCT commands.
 *****************************************************************************/
static void tf_cmd_product( const char * s )
{
	char sub[ 12 ];
	char serial[ 24 ];
	char result[ 12 ];
	int32_t pass = 0;
	int32_t fail = 0;

	if( !tf_word( s, 1, sub, sizeof( sub ) ) || ( 0 != strcmp( sub, "COMPLETE" ) ) )
	{
		tf_err( "PRODUCT", TF_ERR_ARG );
		return;
	}
	if( !tf_find_val( s, "SERIAL", serial, sizeof( serial ) )
		|| !tf_find_val( s, "RESULT", result, sizeof( result ) )
		|| !tf_get_int( s, "PASS", &pass )
		|| !tf_get_int( s, "FAIL", &fail ) )
	{
		tf_err( "PRODUCT", TF_ERR_ARG );
		return;
	}
	if( ( 0 == serial[ 0 ] ) || ( 0 != strcmp( result, "PASS" ) ) || ( pass <= 0 ) || ( 0 != fail ) )
	{
		tf_err( "PRODUCT", TF_ERR_CAL_VALIDATION );
		return;
	}
	test_func_emit( "<PRODUCT OK\n" );
}

/******************************************************************************
 * @brief Parse and dispatch one complete request line.
 *****************************************************************************/
static void tf_process_line( const char * s )
{
	char cmd[ 12 ];

	/* The host frames every request with a leading '>' (see protocol). Skip it
	 * and any surrounding whitespace before extracting the command keyword,
	 * otherwise the '>' becomes part of the command and never matches. */
	while( ( '>' == *s ) || ( ' ' == *s ) || ( '\t' == *s ) )
	{
		s++;
	}

	if( !tf_word( s, 0, cmd, sizeof( cmd ) ) )
	{
		return;
	}

	if( 0 == strcmp( cmd, "HELLO" ) )		tf_cmd_hello();
	else if( 0 == strcmp( cmd, "VER" ) )	tf_cmd_ver();
	else if( 0 == strcmp( cmd, "PING" ) )	test_func_emit( "<PING OK\n" );
	else if( 0 == strcmp( cmd, "TOKEN" ) )	tf_cmd_token();
	else if( 0 == strcmp( cmd, "BYE" ) )
	{
		tf_pending_get_finish( !tf_control_run_is_active() );
		tf.log.on = false;
		AppTimer.Stop( &tf_log_timer );
		tf.session = false;
		test_func_emit( "<BYE OK\n" );
	}
	else if( 0 == strcmp( cmd, "GET" ) )	tf_cmd_get( s );
	else if( 0 == strcmp( cmd, "SET" ) )	tf_cmd_set( s );
	else if( 0 == strcmp( cmd, "SAVE" ) )	tf_cmd_save();
	else if( 0 == strcmp( cmd, "OUT" ) )	tf_cmd_out( s );
	else if( 0 == strcmp( cmd, "TEST" ) )	tf_cmd_test( s );
	else if( 0 == strcmp( cmd, "MEAS" ) )	tf_cmd_meas( s );
	else if( 0 == strcmp( cmd, "LOG" ) )	tf_cmd_log( s );
	else if( 0 == strcmp( cmd, "CAL" ) )	tf_cmd_cal( s );
	else if( 0 == strcmp( cmd, "RESET" ) )	tf_cmd_reset( s );
	else if( 0 == strcmp( cmd, "REBOOT" ) )	tf_cmd_reboot( s );
	else if( 0 == strcmp( cmd, "PRODUCT" ) )	tf_cmd_product( s );
	else									test_func_emit( "<ERR %d CMD=%s\n", TF_ERR_UNKNOWN, cmd );
}

/******************************************************************************
 * @brief Drain the USB receive ring into the line buffer.
 *****************************************************************************/
static void tf_read_lines( void )
{
	uint8_t b;

	while( Ring.Length( &rbUsbRx ) && Ring.Get( &rbUsbRx, &b, 1 ) )
	{
		if( '\r' == b )
		{
			continue;
		}

		if( '\n' == b )
		{
			if( tf.overflow )
			{
				tf_err( "", TF_ERR_ARG );
			}
			else if( tf.len > 0 )
			{
				tf.line[ tf.len ] = 0;
				tf_process_line( tf.line );
			}
			tf.len = 0;
			tf.overflow = false;
			continue;
		}

		if( b < 0x20 )
		{
			continue;
		}

		if( tf.len < ( TF_LINE_MAX - 1 ) )
		{
			tf.line[ tf.len++ ] = ( char )b;
		}
		else
		{
			tf.overflow = true;
		}
	}
}

/******************************************************************************
 * @brief Emit the continuous measurement / calibration event streams.
 *****************************************************************************/
static void tf_stream_task( void )
{
	tf_log_stream_task();

	if( tf.meas.on && AppTimer.IsExpired( &tf_meas_timer ) )
	{
		AppTimer.Start( &tf_meas_timer, TF_MEAS_PERIOD );

		for( int16_t ch = 0; ch < TF_MEAS_CHANNELS; ch++ )
		{
			float temp = tf_get_meas_temp( tf.meas.type, ch );
			test_func_emit( "!MEAS CH=%d TEMP=%.3f\n", ch, temp );
		}
	}

	if( tf.cal.on )
	{
		uint8_t cal_done = Calib.Run( tf.cal.type, tf.cal.ch, ( char )0 );

		if( Calib.status.conv_ready )
		{
			Calib.status.conv_ready = 0;
			tf_emit_cal_point_result( "!CAL", tf.cal.type,
				tf.cal.ch, Calib.status.conv_idx );
		}

		if( cal_done )
		{
			tf.cal.on = false;
			Calib.status.On = 0;
			tf.cal.pending_commit = true;
			tf_emit_cal_done_with_conv();
		}
		else if( AppTimer.IsExpired( &tf_cal_hb_timer ) )
		{
			AppTimer.Start( &tf_cal_hb_timer, TF_CAL_HB_PERIOD );
			test_func_emit( "!CAL CH=%d STATE=RUN\n", tf.cal.ch );
		}
	}
	else if( tf.cal.point_on )
	{
		uint8_t point_done = Calib.MeasurePoint( tf.cal.point_type, tf.cal.point_ch, tf.cal.point_idx, ( char )0 );

		if( point_done )
		{
			tf.cal.point_on = false;
			Calib.status.On = 0;
			tf_emit_cal_point_result( "<CAL POINT OK", tf.cal.point_type,
				tf.cal.point_ch, tf.cal.point_idx );
		}
		else if( AppTimer.IsExpired( &tf_cal_hb_timer ) )
		{
			AppTimer.Start( &tf_cal_hb_timer, TF_CAL_HB_PERIOD );
			test_func_emit( "!CAL CH=%d IDX=%d STATE=RUN\n",
				tf.cal.point_ch, tf.cal.point_idx );
		}
	}
}

/******************************************************************************
 * @brief Initialize the test function module.
 *****************************************************************************/
void TestFuncInit( void )
{
	memset( &tf, 0, sizeof( tf ) );
	tf.meas.type = SEN_RTD;
	tf.cal.type = SEN_RTD;
	tf.cal.point_type = SEN_RTD;

	AppTimer.Start( &tf_task_timer, TF_TASK_CYCLE_TIME );
}

/******************************************************************************
 * @brief Super-loop task. Replaces TerminalTask() when USE_LEGACY_TERMINAL
 *        is not defined.
 *****************************************************************************/
void TestFuncTask( void )
{
	if( !CDC.IsConnected() )
	{
		CDC.Purge();
		tf.session = false;
		tf.meas.on = false;
		tf.log.on = false;
		AppTimer.Stop( &tf_log_timer );
		tf_pending_get_finish( !tf_control_run_is_active() );
		tf.cal.on = false;
		tf.cal.point_on = false;
		tf.cal.save_on = false;
		tf.cal.auto_on = false;
		tf.cal.auto_ch_mask = 0;
		tf.cal.save_ch = 0;
		tf.cal.save_mask = 0;
		tf.cal.saved_mask = 0;
		tf.cal.save_diag_on = false;
		tf.cal.pending_commit = false;
		Calib.status.On = 0;
		Calib.status.conv_ready = 0;
		Calib.status.save_diag_on = 0;
		tf.len = 0;
		tf.overflow = false;
		return;
	}

	if( !AppTimer.IsExpired( &tf_task_timer ) )
	{
		return;
	}
	AppTimer.Start( &tf_task_timer, TF_TASK_CYCLE_TIME );

	tf_read_lines();
	tf_pending_get_task();
	tf_stream_task();
	tf_cal_save_task();
}
