/******************************************************************************
 * @file a_calib.c
 * @author Seo Yujeong (yjseo@gst-in.com)
 * @brief 
 * @version 0.1
 * @date 2023-08-17
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

#define CALIB_AVG_FILTER_SIZE	 				10
#define CALIB_FILTER_SETTLING_SAMPLES			5
#define CALIB_FILTER_MIN_VALID_SAMPLES			6
#define CALIB_FILTER_MAD_SCALE					1.4826f
#define CALIB_FILTER_OUTLIER_SIGMA				3.0f
#define CALIB_FILTER_SIGMA_FLOOR_RAW			1.0f
#define	CALIB_COMPELETED_STATUS					0xAAAA				// 캘리브레이션 완료 상태 변수
#define CALIB_COMPELETED_bp						0xffff
#define SLOPE_SECTION							2
#define CALIB_ADCSAMPLING_MS					100

/*****************************************************************************/
/** DATATYPES ****************************************************************/
/*****************************************************************************/

typedef struct EEPR_Appl_struct
{
	float 					offset;
	float 					gain;
	int32_t 				boundary;
	uint32_t 				status;
} EEPR_Appl_st;

typedef struct EEPR_Ad7124_struct
{
	uint32_t 				offset;
	uint32_t 				gain;
	uint32_t 				status;
} EEPR_Ad7124_st;

typedef struct EEPR_read_Struct
{
	EEPR_Appl_st Appl;
	EEPR_Ad7124_st AD7124;
} EEPR_read_st;

#if 1 /* MAD filter using unfiltered ADC raw data */
typedef struct Avgfilter_buffer_struct
{
	uint8_t settling_count;
	uint8_t sample_count;
	uint8_t valid_count;
	int32_t median;
	int32_t mad;
	float robust_sigma;
	int32_t peak_to_peak;
	int32_t buffer[ CALIB_AVG_FILTER_SIZE ];
} Avgfilter_buffer_st;
#elif 0 /* Legacy continuous median(3) result average */
typedef struct Avgfilter_buffer_struct
{
	uint8_t lastindex;
	uint8_t bufferOver;
	int32_t Sum;
	int32_t buffer[ CALIB_AVG_FILTER_SIZE ];
} Avgfilter_buffer_st;
#endif

typedef struct calculate_struct
{
	int32_t ADCraw;
	float Convert;
} calculate_st;

typedef struct calib_calc_struct
{
	calculate_st RTD2X[ CALIB_RTD_2X_SECTION + 1 ];
	calculate_st RTD[ CALIB_RTD_SECTION + 1 ];
	calculate_st TC[ CALIB_TC_SECTION + 1 ];
} calib_calc_st;

/*****************************************************************************/
/** FUNCTION DECLARATIONS ****************************************************/
/*****************************************************************************/
static uint8_t calibration_task( sensor_et type, uint8_t ch, char cmd );
static uint8_t calib_measure_point( sensor_et type, uint8_t ch, uint8_t idx, char cmd );
static float   calib_get_conv( sensor_et type, uint8_t ch, uint8_t idx );
static uint8_t calib_validate_and_save( sensor_et type, uint8_t ch );
static int32_t get_calibration_adc_value( uint8_t ch );
static uint8_t calib_clear_done_flag( void );
static uint8_t calib_start_session( calib_session_mode_et mode, uint8_t ch );
static uint8_t calib_is_done( void );
static uint8_t calib_complete_channels( uint8_t ch_mask );
static uint8_t calib_get_channel_done_mask( void );

/*****************************************************************************/
/** LOCAL VARIABLES **********************************************************/
/*****************************************************************************/

static const char* rtd_Str[ CALIB_RTD_SECTION + 1 ] =
{
	" 18.52Ω",
	"100.00Ω",
	"220.00Ω",
	"280.00Ω",
	"390.48Ω",
};

static const char* tc_Str[ CALIB_TC_SECTION + 1 ] =
{
	" 0.0mV",
	"15.5mV",
	"52.0mV",
};

static const float Rtd_Slope[ CALIB_RTD_SECTION ][ SLOPE_SECTION ] =
{
	{    18.52f, 	100.00f	},
	{	100.00f,	220.00f	},
	{	220.00f,	280.00f	},
	{	280.00f,    390.48f},
};

static const float TC_Slope[ CALIB_TC_SECTION ][ SLOPE_SECTION ] =
{
	{   0.00f,	    15.50f },
	{  15.50f,	    52.00f },
};

static const uint16_t RTD_2X_EEPRAdd[ CALIB_RTD_2X_SECTION ][ MAX_TEMP_CHANNEL ][ EEPR_NUM_1SECTION ] = 
{
	{
		{	EEPR_ADDR_2X_CH1_R1_OFFSET,	EEPR_ADDR_2X_CH1_R1_GAIN,	EEPR_ADDR_2X_CH1_R1_BD,	EEPR_ADDR_2X_CH1_R1_STATUS },		
		{	EEPR_ADDR_2X_CH2_R1_OFFSET,	EEPR_ADDR_2X_CH2_R1_GAIN,	EEPR_ADDR_2X_CH2_R1_BD,	EEPR_ADDR_2X_CH2_R1_STATUS },
		{	EEPR_ADDR_2X_CH3_R1_OFFSET,	EEPR_ADDR_2X_CH3_R1_GAIN,	EEPR_ADDR_2X_CH3_R1_BD,	EEPR_ADDR_2X_CH3_R1_STATUS },
		{	EEPR_ADDR_2X_CH4_R1_OFFSET,	EEPR_ADDR_2X_CH4_R1_GAIN,	EEPR_ADDR_2X_CH4_R1_BD,	EEPR_ADDR_2X_CH4_R1_STATUS },
	},
	{
		{	EEPR_ADDR_2X_CH1_R2_OFFSET,	EEPR_ADDR_2X_CH1_R2_GAIN,	EEPR_ADDR_2X_CH1_R2_BD,	EEPR_ADDR_2X_CH1_R2_STATUS	},
		{	EEPR_ADDR_2X_CH2_R2_OFFSET,	EEPR_ADDR_2X_CH2_R2_GAIN,	EEPR_ADDR_2X_CH2_R2_BD,	EEPR_ADDR_2X_CH2_R2_STATUS	},
		{	EEPR_ADDR_2X_CH3_R2_OFFSET,	EEPR_ADDR_2X_CH3_R2_GAIN,	EEPR_ADDR_2X_CH3_R2_BD,	EEPR_ADDR_2X_CH3_R2_STATUS	},
		{	EEPR_ADDR_2X_CH4_R2_OFFSET,	EEPR_ADDR_2X_CH4_R2_GAIN,	EEPR_ADDR_2X_CH4_R2_BD,	EEPR_ADDR_2X_CH4_R2_STATUS	},
	},
	{
		{	EEPR_ADDR_2X_CH1_R3_OFFSET,	EEPR_ADDR_2X_CH1_R3_GAIN,	EEPR_ADDR_2X_CH1_R3_BD,	EEPR_ADDR_2X_CH1_R3_STATUS	},
		{	EEPR_ADDR_2X_CH2_R3_OFFSET,	EEPR_ADDR_2X_CH2_R3_GAIN,	EEPR_ADDR_2X_CH2_R3_BD,	EEPR_ADDR_2X_CH2_R3_STATUS	},
		{	EEPR_ADDR_2X_CH3_R3_OFFSET,	EEPR_ADDR_2X_CH3_R3_GAIN,	EEPR_ADDR_2X_CH3_R3_BD,	EEPR_ADDR_2X_CH3_R3_STATUS	},
		{	EEPR_ADDR_2X_CH4_R3_OFFSET,	EEPR_ADDR_2X_CH4_R3_GAIN,	EEPR_ADDR_2X_CH4_R3_BD,	EEPR_ADDR_2X_CH4_R3_STATUS	},
	},
};

static const uint16_t RTD_EEPRAdd[ CALIB_RTD_SECTION ][ MAX_TEMP_CHANNEL ][ EEPR_NUM_1SECTION ] = 
{
	{
		{	EEPR_ADDR_CH1_R1_OFFSET,	EEPR_ADDR_CH1_R1_GAIN,	EEPR_ADDR_CH1_R1_BD,	EEPR_ADDR_CH1_R1_STATUS	},
		{	EEPR_ADDR_CH2_R1_OFFSET,	EEPR_ADDR_CH2_R1_GAIN,	EEPR_ADDR_CH2_R1_BD,	EEPR_ADDR_CH2_R1_STATUS	},
		{	EEPR_ADDR_CH3_R1_OFFSET,	EEPR_ADDR_CH3_R1_GAIN,	EEPR_ADDR_CH3_R1_BD,	EEPR_ADDR_CH3_R1_STATUS	},
		{	EEPR_ADDR_CH4_R1_OFFSET,	EEPR_ADDR_CH4_R1_GAIN,	EEPR_ADDR_CH4_R1_BD,	EEPR_ADDR_CH4_R1_STATUS	},
	},
	{
		{	EEPR_ADDR_CH1_R2_OFFSET,	EEPR_ADDR_CH1_R2_GAIN,	EEPR_ADDR_CH1_R2_BD,	EEPR_ADDR_CH1_R2_STATUS	},
		{	EEPR_ADDR_CH2_R2_OFFSET,	EEPR_ADDR_CH2_R2_GAIN,	EEPR_ADDR_CH2_R2_BD,	EEPR_ADDR_CH2_R2_STATUS	},
		{	EEPR_ADDR_CH3_R2_OFFSET,	EEPR_ADDR_CH3_R2_GAIN,	EEPR_ADDR_CH3_R2_BD,	EEPR_ADDR_CH3_R2_STATUS	},
		{	EEPR_ADDR_CH4_R2_OFFSET,	EEPR_ADDR_CH4_R2_GAIN,	EEPR_ADDR_CH4_R2_BD,	EEPR_ADDR_CH4_R2_STATUS	},
	},
	{
		{	EEPR_ADDR_CH1_R3_OFFSET,	EEPR_ADDR_CH1_R3_GAIN,	EEPR_ADDR_CH1_R3_BD,	EEPR_ADDR_CH1_R3_STATUS	},
		{	EEPR_ADDR_CH2_R3_OFFSET,	EEPR_ADDR_CH2_R3_GAIN,	EEPR_ADDR_CH2_R3_BD,	EEPR_ADDR_CH2_R3_STATUS	},
		{	EEPR_ADDR_CH3_R3_OFFSET,	EEPR_ADDR_CH3_R3_GAIN,	EEPR_ADDR_CH3_R3_BD,	EEPR_ADDR_CH3_R3_STATUS	},
		{	EEPR_ADDR_CH4_R3_OFFSET,	EEPR_ADDR_CH4_R3_GAIN,	EEPR_ADDR_CH4_R3_BD,	EEPR_ADDR_CH4_R3_STATUS	},
	},
	{
		{	EEPR_ADDR_CH1_R4_OFFSET,	EEPR_ADDR_CH1_R4_GAIN,	EEPR_ADDR_CH1_R4_BD,	EEPR_ADDR_CH1_R4_STATUS	},
		{	EEPR_ADDR_CH2_R4_OFFSET,	EEPR_ADDR_CH2_R4_GAIN,	EEPR_ADDR_CH2_R4_BD,	EEPR_ADDR_CH2_R4_STATUS	},
		{	EEPR_ADDR_CH3_R4_OFFSET,	EEPR_ADDR_CH3_R4_GAIN,	EEPR_ADDR_CH3_R4_BD,	EEPR_ADDR_CH3_R4_STATUS	},		
		{	EEPR_ADDR_CH4_R4_OFFSET,	EEPR_ADDR_CH4_R4_GAIN,	EEPR_ADDR_CH4_R4_BD,	EEPR_ADDR_CH4_R4_STATUS	},
	},
};

static const uint16_t TC_EEPRAdd[ CALIB_TC_SECTION ][ MAX_TEMP_CHANNEL ][ EEPR_NUM_1SECTION ] = 
{
	{
		{	EEPR_ADDR_CH1_TC1_OFFSET,	EEPR_ADDR_CH1_TC1_GAIN,	EEPR_ADDR_CH1_TC1_BD,	EEPR_ADDR_CH1_TC1_STATUS	},
		{	EEPR_ADDR_CH2_TC1_OFFSET,	EEPR_ADDR_CH2_TC1_GAIN,	EEPR_ADDR_CH2_TC1_BD,	EEPR_ADDR_CH2_TC1_STATUS	},
		{	EEPR_ADDR_CH3_TC1_OFFSET,	EEPR_ADDR_CH3_TC1_GAIN,	EEPR_ADDR_CH3_TC1_BD,	EEPR_ADDR_CH3_TC1_STATUS	},
		{	EEPR_ADDR_CH4_TC1_OFFSET,	EEPR_ADDR_CH4_TC1_GAIN,	EEPR_ADDR_CH4_TC1_BD,	EEPR_ADDR_CH4_TC1_STATUS	},
	},
	{
		{	EEPR_ADDR_CH1_TC2_OFFSET,	EEPR_ADDR_CH1_TC2_GAIN,	EEPR_ADDR_CH1_TC2_BD,	EEPR_ADDR_CH1_TC2_STATUS	},
		{	EEPR_ADDR_CH2_TC2_OFFSET,	EEPR_ADDR_CH2_TC2_GAIN,	EEPR_ADDR_CH2_TC2_BD,	EEPR_ADDR_CH2_TC2_STATUS	},
		{	EEPR_ADDR_CH3_TC2_OFFSET,	EEPR_ADDR_CH3_TC2_GAIN,	EEPR_ADDR_CH3_TC2_BD,	EEPR_ADDR_CH3_TC2_STATUS	},
		{	EEPR_ADDR_CH4_TC2_OFFSET,	EEPR_ADDR_CH4_TC2_GAIN,	EEPR_ADDR_CH4_TC2_BD,	EEPR_ADDR_CH4_TC2_STATUS	},
	},
};

static const uint16_t AD7124_EEPRAdd[ CALIB_PGA_NUM ][ MAX_TEMP_CHANNEL ][ EEPR_ADC_1SECTION ] = 
{
	{
		{	EEPR_ADDR_CH1_ADC_RTD2X_OFFSET, EEPR_ADDR_CH1_ADC_RTD2X_GAIN, EEPR_ADDR_CH1_ADC_RTD2X_STATUS },
		{	EEPR_ADDR_CH2_ADC_RTD2X_OFFSET, EEPR_ADDR_CH2_ADC_RTD2X_GAIN, EEPR_ADDR_CH2_ADC_RTD2X_STATUS },
		{	EEPR_ADDR_CH3_ADC_RTD2X_OFFSET, EEPR_ADDR_CH3_ADC_RTD2X_GAIN, EEPR_ADDR_CH3_ADC_RTD2X_STATUS },
		{	EEPR_ADDR_CH4_ADC_RTD2X_OFFSET, EEPR_ADDR_CH4_ADC_RTD2X_GAIN, EEPR_ADDR_CH4_ADC_RTD2X_STATUS },
	},
	{
		{	EEPR_ADDR_CH1_ADC_RTD_OFFSET, EEPR_ADDR_CH1_ADC_RTD_GAIN, EEPR_ADDR_CH1_ADC_RTD_STATUS },
		{	EEPR_ADDR_CH2_ADC_RTD_OFFSET, EEPR_ADDR_CH2_ADC_RTD_GAIN, EEPR_ADDR_CH2_ADC_RTD_STATUS },
		{	EEPR_ADDR_CH3_ADC_RTD_OFFSET, EEPR_ADDR_CH3_ADC_RTD_GAIN, EEPR_ADDR_CH3_ADC_RTD_STATUS },
		{	EEPR_ADDR_CH4_ADC_RTD_OFFSET, EEPR_ADDR_CH4_ADC_RTD_GAIN, EEPR_ADDR_CH4_ADC_RTD_STATUS },
	},
	{
		{	EEPR_ADDR_CH1_ADC_TC_OFFSET, EEPR_ADDR_CH1_ADC_TC_GAIN, EEPR_ADDR_CH1_ADC_TC_STATUS },
		{	EEPR_ADDR_CH2_ADC_TC_OFFSET, EEPR_ADDR_CH2_ADC_TC_GAIN, EEPR_ADDR_CH2_ADC_TC_STATUS },
		{	EEPR_ADDR_CH3_ADC_TC_OFFSET, EEPR_ADDR_CH3_ADC_TC_GAIN, EEPR_ADDR_CH3_ADC_TC_STATUS },
		{	EEPR_ADDR_CH4_ADC_TC_OFFSET, EEPR_ADDR_CH4_ADC_TC_GAIN, EEPR_ADDR_CH4_ADC_TC_STATUS },
	},
};

static const uint16_t EEPR_start_addr[ MAX_TEMP_CHANNEL ] = 
{ 
	EEPR_ADDR_2X_CH1_R1_OFFSET, EEPR_ADDR_2X_CH2_R1_OFFSET, EEPR_ADDR_2X_CH3_R1_OFFSET, EEPR_ADDR_2X_CH4_R1_OFFSET 
};

static const uint16_t EEPR_channel_done_addr[ MAX_TEMP_CHANNEL ] =
{
	EEPR_ADDR_CH1_CALIB_CHK, EEPR_ADDR_CH2_CALIB_CHK,
	EEPR_ADDR_CH3_CALIB_CHK, EEPR_ADDR_CH4_CALIB_CHK,
};

static const Calib_Appl_st default_calib = 
{
	.gain = 1.0f,
	.offset = 0.0f,
	.boundary = 0xffffffff,
};

static calib_calc_st calc_data[ MAX_TEMP_CHANNEL ];

static bool calib_chk_flag = false;
static uint8_t calib_channel_done_mask = 0;
static uint8_t calib_rtd_point_mask[ MAX_TEMP_CHANNEL ];
static uint8_t calib_rtd2x_point_mask[ MAX_TEMP_CHANNEL ];
static uint8_t calib_tc_point_mask[ MAX_TEMP_CHANNEL ];

#define CALIB_TOLERANCE_PCT		0.01f		// ±1 % of nominal
#define RTD_LOW_POINT_TOLERANCE_PCT	0.02f		// ±2 % for the 18.52 ohm point
#define TC_FULLSCALE_MV			52.00f		// TC full-scale for 0 mV point tolerance

static const float RTD_nominal[ CALIB_RTD_SECTION + 1 ] =
{
	18.52f, 100.00f, 220.00f, 280.00f, 390.48f,
};

static const float TC_nominal[ CALIB_TC_SECTION + 1 ] =
{
	0.00f, 15.50f, 52.00f,
};

/*****************************************************************************/
/** GLOBAL VARIABLES *********************************************************/
/*****************************************************************************/
calib_st Calib =
{
	.status = 
	{
		.ch = 0,
		.On = 0,
		.vaild_raw_value = 0,
	},
	.data = { { { { 0 }, }, }, },
	.Run             = calibration_task,
	.MeasurePoint    = calib_measure_point,
	.GetConv         = calib_get_conv,
	.ValidateAndSave = calib_validate_and_save,
	.ClearDoneFlag   = calib_clear_done_flag,
	.StartSession    = calib_start_session,
	.Complete         = calib_complete_channels,
	.IsDone          = calib_is_done,
	.GetChannelDoneMask = calib_get_channel_done_mask,
};

/*****************************************************************************/
/** FUNCTION DEFINITIONS *****************************************************/
/*****************************************************************************/

/******************************************************************************
 * @brief 
 * 
 *****************************************************************************/
void CalibrationInit( void )
{
	EEPR_read_st temp = { 0, };
	uint32_t     chk_val = 0;

	uint8_t all_section = CALIB_RTD_2X_SECTION + CALIB_RTD_SECTION + CALIB_TC_SECTION;

	EEPR.Read( EEPR_ADDR_CALIB_CHK, ( uint8_t * )&chk_val, sizeof( chk_val ) );
	calib_chk_flag = ( ( chk_val & CALIB_COMPELETED_bp ) == CALIB_COMPELETED_STATUS );
	calib_channel_done_mask = 0;
	for( uint8_t ch = 0; ch < MAX_TEMP_CHANNEL; ch++ )
	{
		uint32_t ch_done = 0;
		if( HAL_OK == EEPR.Read( EEPR_channel_done_addr[ ch ], ( uint8_t * )&ch_done, sizeof( ch_done ) )
			&& ( ch_done & CALIB_COMPELETED_bp ) == CALIB_COMPELETED_STATUS )
		{
			calib_channel_done_mask |= ( 1u << ch );
		}
	}
	calib_chk_flag = calib_chk_flag && ( calib_channel_done_mask == ( ( 1u << MAX_TEMP_CHANNEL ) - 1u ) );

	for( uint8_t ch = 0; ch < MAX_TEMP_CHANNEL; ch++ )
	{
		Calib_Appl_st *pData = Calib.data[ ch ].RTD2X;
		Calib_AD7124_st *pAD7124 = Calib.data[ ch ].AD7124;
		uint16_t eepr_addr = EEPR_start_addr[ ch ];

		for( uint8_t i = 0; i < all_section; i++ )
		{
			bool use_calib = false;
			if( calib_chk_flag )
			{
				EEPR.Read( eepr_addr, ( uint8_t * )&temp.Appl, sizeof( temp.Appl ) );
				use_calib = ( ( temp.Appl.status & CALIB_COMPELETED_bp ) == CALIB_COMPELETED_STATUS );
			}

			if( use_calib )
			{
				pData->offset   = temp.Appl.offset;
				pData->gain     = temp.Appl.gain;
				pData->boundary = temp.Appl.boundary;
			}
			else
			{
				pData->offset   = default_calib.offset;
				pData->gain     = default_calib.gain;
				pData->boundary = default_calib.boundary;
			}
			eepr_addr += EEPR_SIZE_R;
			pData++;
		}

		for( uint8_t i = 0; i < CALIB_PGA_NUM; i++ )
		{
			bool use_adc_calib = false;
			if( calib_chk_flag )
			{
				EEPR.Read( eepr_addr, ( uint8_t * )&temp.AD7124, sizeof( temp.AD7124 ) );
				use_adc_calib = ( ( temp.AD7124.status & CALIB_COMPELETED_bp ) == CALIB_COMPELETED_STATUS );
			}

			if( use_adc_calib )
			{
				pAD7124->offset = temp.AD7124.offset;
				pAD7124->gain   = temp.AD7124.gain;
			}
			else
			{
				pAD7124->offset = 0x800000;
				pAD7124->gain   = AD7124.GetRegister( ch / MAX_ADC_DEV, AD7124_Gain_0 + ( ch % MAX_ADC_CH ) );
			}
			eepr_addr += EEPR_SIZE_ADC;
			pAD7124++;
		}
	}
}

/******************************************************************************
 * @brief 
 * 
 * @param type 
 * @param idx 
 * @return const char* 
 *****************************************************************************/
static const char* data_string_of_type( sensor_et type, uint8_t idx )
{
	if( type == SEN_RTD )
	{
		return rtd_Str[ idx ];
	}
	else
	{
		return tc_Str[ idx ];
	}
}

/******************************************************************************
 * @brief Get the convert data
 * 
 * @param ch 
 * @param type 
 * @param data 
 * @return float 
 *****************************************************************************/
static float get_convert_data( sensor_et type, uint32_t data )
{
	if( type == SEN_RTD2X || type == SEN_RTD )
	{
		return Temp.ConvToRes( type, data, CALIB_ADCSAMPLING_MS );
	}
	else
	{
		return Temp.ConvTomV( data, CALIB_ADCSAMPLING_MS );
	}
}

#if 1 /* MAD filter using unfiltered ADC raw data */
/******************************************************************************
 * @brief Sort a small int32 array in ascending order.
 *****************************************************************************/
static void sort_int32( int32_t *data, uint8_t count )
{
	for( uint8_t i = 1; i < count; i++ )
	{
		int32_t value = data[ i ];
		int8_t j = ( int8_t )i - 1;

		while( ( j >= 0 ) && ( data[ j ] > value ) )
		{
			data[ j + 1 ] = data[ j ];
			j--;
		}
		data[ j + 1 ] = value;
	}
}

/******************************************************************************
 * @brief Return the median of an already sorted array.
 *****************************************************************************/
static int32_t median_of_sorted( const int32_t *data, uint8_t count )
{
	if( count & 1U )
	{
		return data[ count / 2U ];
	}

	return ( int32_t )( ( ( int64_t )data[ count / 2U - 1U ] + data[ count / 2U ] ) / 2 );
}

/******************************************************************************
 * @brief Collect raw samples and calculate a MAD-filtered inlier average.
 *****************************************************************************/
static uint8_t filter_adc_value( int32_t input, Avgfilter_buffer_st *data, int32_t *filtered )
{
	if( data->settling_count < CALIB_FILTER_SETTLING_SAMPLES )
	{
		data->settling_count++;
		return 0;
	}

	data->buffer[ data->sample_count++ ] = input;
	if( data->sample_count < CALIB_AVG_FILTER_SIZE )
	{
		return 0;
	}

	int32_t sorted[ CALIB_AVG_FILTER_SIZE ];
	int32_t deviation[ CALIB_AVG_FILTER_SIZE ];
	memcpy( sorted, data->buffer, sizeof( sorted ) );
	sort_int32( sorted, CALIB_AVG_FILTER_SIZE );
	data->median = median_of_sorted( sorted, CALIB_AVG_FILTER_SIZE );

	for( uint8_t i = 0; i < CALIB_AVG_FILTER_SIZE; i++ )
	{
		int32_t delta = data->buffer[ i ] - data->median;
		deviation[ i ] = ( delta < 0 ) ? -delta : delta;
	}
	sort_int32( deviation, CALIB_AVG_FILTER_SIZE );
	data->mad = median_of_sorted( deviation, CALIB_AVG_FILTER_SIZE );
	data->robust_sigma = CALIB_FILTER_MAD_SCALE * data->mad;
	if( data->robust_sigma < CALIB_FILTER_SIGMA_FLOOR_RAW )
	{
		data->robust_sigma = CALIB_FILTER_SIGMA_FLOOR_RAW;
	}

	float threshold = CALIB_FILTER_OUTLIER_SIGMA * data->robust_sigma;
	int64_t sum = 0;
	int32_t minimum = 0;
	int32_t maximum = 0;
	data->valid_count = 0;

	for( uint8_t i = 0; i < CALIB_AVG_FILTER_SIZE; i++ )
	{
		if( fabsf( ( float )( data->buffer[ i ] - data->median ) ) <= threshold )
		{
			int32_t value = data->buffer[ i ];
			if( 0 == data->valid_count )
			{
				minimum = value;
				maximum = value;
			}
			else
			{
				if( value < minimum ) minimum = value;
				if( value > maximum ) maximum = value;
			}
			sum += value;
			data->valid_count++;
		}
	}

	data->peak_to_peak = maximum - minimum;
	data->sample_count = 0;
	if( data->valid_count < CALIB_FILTER_MIN_VALID_SAMPLES )
	{
		return 0;
	}

	*filtered = ( int32_t )( sum / data->valid_count );
	return 1;
}

static int32_t get_calibration_adc_value( uint8_t ch )
{
	return AD7124.GetRawValue( ch );
}
#elif 0 /* Legacy continuous median(3) result average */
/******************************************************************************
 * @brief Legacy 10-sample average of the driver median output.
 *****************************************************************************/
static uint8_t Sum_AdcValue_Buffer( int32_t input, Avgfilter_buffer_st* data )
{
	if( data->lastindex >= CALIB_AVG_FILTER_SIZE )
	{
		data->lastindex = 0;
		data->bufferOver++;
	}
	data->buffer[ data->lastindex++ ] = input;
	if( data->bufferOver >= 2 )
	{
		data->Sum = 0;
		for( int i = 0; i < CALIB_AVG_FILTER_SIZE; i++ )
		{
			data->Sum += data->buffer[ i ];
		}
		return 1;
	}
	return 0;
}

static int32_t get_calibration_adc_value( uint8_t ch )
{
	return AD7124.GetValue( ch );
}
#endif

/******************************************************************************
 * @brief 
 * 
 * @param pData 
 * @param pResult 
 * @param section 
 *****************************************************************************/
static void calculate_calib( calculate_st *pData, Calib_Appl_st *pResult, const float (*slope_Array)[ SLOPE_SECTION ], const int section )
{
	for( int idx = 1; idx <= section; idx++ )
	{
		float slope = ( pData[ idx ].Convert - pData[ idx - 1 ].Convert ) / ( slope_Array[ idx - 1 ][ 1 ] - slope_Array[ idx - 1 ][ 0 ] );
		float offset = pData[ idx - 1 ].Convert - ( slope * slope_Array[ idx - 1 ][ 0 ] );

		pResult[ idx - 1 ].gain = 1 / slope;
		pResult[ idx - 1 ].offset = offset;
		pResult[ idx - 1 ].boundary = pData[ idx ].ADCraw;
	}
}

/******************************************************************************
 * @brief 
 * 
 * @param ch 
 * @param pResult 
 * @param pAd7124 
 * @param eeprAd7124 
 * @param section 
 *****************************************************************************/
static uint8_t save_calib( const char *type_name, uint8_t ch, Calib_Appl_st *pResult, Calib_AD7124_st pAd7124,
						const uint16_t (*eeprAddr)[ MAX_TEMP_CHANNEL ][ EEPR_NUM_1SECTION ], const uint16_t *eeprAd7124, const int section )
{
	const uint16_t status_buf[ 1 ] = { CALIB_COMPELETED_STATUS };
	static const char *idx_name[] = { "0", "1", "2", "3" };

	const char *field_name[] = { "OFFSET", "GAIN", "BD", "STATUS" };

	for( int idx = 0; idx < section; idx++ )
	{
		void *value[] = { &pResult[ idx ].offset, &pResult[ idx ].gain, &pResult[ idx ].boundary, ( void * )&status_buf[ 0 ] };
		uint32_t size[] = { EEPR_SIZE_OFFSET, EEPR_SIZE_GAIN, EEPR_SIZE_BOUNDARY, EEPR_SIZE_STATUS };

		for( int field = 0; field < EEPR_NUM_1SECTION; field++ )
		{
			uint8_t ret;
			uint16_t addr = eeprAddr[ idx ][ ch ][ field ];

			if( Calib.status.save_diag_on )
			{
				TestFunc.Emit( "!CAL EEPR TYPE=%s CH=%d IDX=%s FIELD=%s ADDR=0x%04X INIT=%lu STATE=RUN\n",
						type_name, ch, idx_name[ idx ], field_name[ field ], addr,
						( unsigned long )EEPR.Status->initiated );
			}
			ret = EEPR.Write( addr, ( uint8_t * )value[ field ], size[ field ] );
			if( Calib.status.save_diag_on )
			{
				TestFunc.Emit( "!CAL EEPR TYPE=%s CH=%d IDX=%s FIELD=%s ADDR=0x%04X INIT=%lu STATE=%s CODE=%d\n",
						type_name, ch, idx_name[ idx ], field_name[ field ], addr,
						( unsigned long )EEPR.Status->initiated,
						( HAL_OK == ret ) ? "OK" : "ERR", ret );
			}
			if( HAL_OK != ret ) return 0;
		}
	}

	{
		void *value[] = { &pAd7124.offset, &pAd7124.gain, ( void * )&status_buf[ 0 ] };
		uint32_t size[] = { EEPR_SIZE_ADC_OFFSET, EEPR_SIZE_ADC_GAIN, EEPR_SIZE_ADC_STATUS };
		const char *adc_field_name[] = { "ADC_OFFSET", "ADC_GAIN", "ADC_STATUS" };

		for( int field = 0; field < EEPR_ADC_1SECTION; field++ )
		{
			uint8_t ret;
			uint16_t addr = eeprAd7124[ field ];

			if( Calib.status.save_diag_on )
			{
				TestFunc.Emit( "!CAL EEPR TYPE=%s CH=%d IDX=ADC FIELD=%s ADDR=0x%04X INIT=%lu STATE=RUN\n",
						type_name, ch, adc_field_name[ field ], addr,
						( unsigned long )EEPR.Status->initiated );
			}
			ret = EEPR.Write( addr, ( uint8_t * )value[ field ], size[ field ] );
			if( Calib.status.save_diag_on )
			{
				TestFunc.Emit( "!CAL EEPR TYPE=%s CH=%d IDX=ADC FIELD=%s ADDR=0x%04X INIT=%lu STATE=%s CODE=%d\n",
						type_name, ch, adc_field_name[ field ], addr,
						( unsigned long )EEPR.Status->initiated,
						( HAL_OK == ret ) ? "OK" : "ERR", ret );
			}
			if( HAL_OK != ret ) return 0;
		}
	}

	return 1;
}

/******************************************************************************
 * @brief 
 * 
 * @param ch 
 * @param type 
 * @param idx 
 * 
 * @note  exception 필요 : ADC internal calib error 발생시, EEPR error시 (yjseo)
 *****************************************************************************/
static void ad7124_calib( uint8_t ch, sensor_et type, uint8_t idx )
{
	if( idx == 0 )
	{
		if( type == SEN_RTD2X )
		{
			tprintf( "\r\n\r\n   B. GAIN, OFFSET 조정중" );
			AD7124.CalibrateRTD( ch, AD7124_PGA_2X_RTD );
		}
		else if( type == SEN_RTD )
		{
			tprintf( "\r\n   A. GAIN, OFFSET 조정중" );
			AD7124.CalibrateRTD( ch, AD7124_PGA_RTD );
		}
		else
		{
			tprintf( "\r\n      GAIN, OFFSET 조정중" );
			AD7124.CalibrateTC( ch, AD7124_PGA_TC );
		}
		Calib.data[ ch ].AD7124[ type ].offset = AD7124.GetRegister( ch / MAX_ADC_DEV, AD7124_Offset_0 + ( ch % MAX_ADC_CH ) );
		Calib.data[ ch ].AD7124[ type ].gain = AD7124.GetRegister( ch / MAX_ADC_DEV, AD7124_Gain_0 + ( ch % MAX_ADC_CH ) );
		tprintf( "  -> 완료 offset : 0x%x, gain : 0x%x",  Calib.data[ ch ].AD7124[ type ].offset, Calib.data[ ch ].AD7124[ type ].gain );
	}
	else
	{
		AD7124.ApplyCalibData( ch, Calib.data[ ch ].AD7124[ type ].gain, Calib.data[ ch ].AD7124[ type ].offset );
	}
}

/******************************************************************************
 * @brief wait_avg_filter
 * 
 *****************************************************************************/
static uint8_t wait_avg_filter( sensor_et type, int32_t rawValue, Avgfilter_buffer_st* avg_buf, calculate_st* pData )
{
	int res = 0;

	if( Calib.status.vaild_raw_value )
	{
#if 1 /* MAD filter using unfiltered ADC raw data */
		int32_t filtered;
		if( filter_adc_value( rawValue, avg_buf, &filtered ) )
		{
			pData->ADCraw = filtered;
			pData->Convert = get_convert_data( type, pData->ADCraw );
			tprintf( VT100_CURSOR_POS_RESTORE "%.3f           ", pData->Convert );
			res = 1;
		}
#elif 0 /* Legacy continuous median(3) result average */
		if( Sum_AdcValue_Buffer( rawValue, avg_buf ) )
		{
			pData->ADCraw = avg_buf->Sum / CALIB_AVG_FILTER_SIZE;
			pData->Convert = get_convert_data( type, pData->ADCraw );
			tprintf( VT100_CURSOR_POS_RESTORE "%.3f           ", pData->Convert );
			res = 1;
		}
#endif
		Calib.status.vaild_raw_value = 0;
	}
	return res;
}
/******************************************************************************
 * @brief 
 * 
 * @param type 
 * @param ch 
 * @param cmd 
 * @return uint8_t 
 *****************************************************************************/
static uint8_t calibration_task( sensor_et type, uint8_t ch, char cmd )
{
	static enum { CALIB_INIT, CALIB_WAIT, CONV_TC, CONV_RTD, CONV_RTD2X, SAVE_CALIB, CALIB_COMPLETE } stage = CALIB_INIT;
	static Avgfilter_buffer_st avg_buf = {0};
	static uint8_t idx = 0;
	uint8_t calibration_end = 0;

	if( cmd == 0x1B )
	{
		stage = CALIB_INIT;
		idx = 0;
		Calib.status.conv_ready = 0;
		return 1;
	}

	switch( stage )
	{
		case	CALIB_INIT:
			Temp.SetType( ch, type, CALIB_ADCSAMPLING_MS );
			memset( &avg_buf, 0, sizeof( avg_buf ) );
			tprintf( VT100_ERASE_SCR_TO_END "\r\n %d) %s 연결 후 ENTER를 입력하세요 : " VT100_CURSOR_POS_SAVE, idx + 1, data_string_of_type( type, idx ) );
			stage = CALIB_WAIT;
			return 0;

		case	CALIB_WAIT:
			tprintf( VT100_CURSOR_POS_RESTORE "%.3f", get_convert_data( type, AD7124.GetValue( ch ) ) );
			if( cmd == 0x0D )
			{
				ad7124_calib( ch, type, idx );
				if( type == SEN_RTD )	stage = CONV_RTD;
				else					stage = CONV_TC;
			}
			break;

		case	CONV_TC:
			tprintf( VT100_CURSOR_POS_RESTORE "%.3f [T측정중]", get_convert_data( type, AD7124.GetValue( ch ) ) );
			if( wait_avg_filter( SEN_TC_K, get_calibration_adc_value( ch ), &avg_buf, &calc_data[ ch ].TC[ idx ] ) )
			{
				Calib.status.conv_ready = 1;
				Calib.status.conv_idx = idx;
				idx++;
				if( idx > CALIB_TC_SECTION )	stage = SAVE_CALIB;
				else	stage = CALIB_INIT;
			}
			break;

		case	CONV_RTD:
			tprintf( VT100_CURSOR_POS_RESTORE "%.3f [R측정중]", get_convert_data( SEN_RTD, AD7124.GetValue( ch ) ) );
			if( wait_avg_filter( SEN_RTD, get_calibration_adc_value( ch ), &avg_buf, &calc_data[ ch ].RTD[ idx ] ) )
			{
				if( idx <= CALIB_RTD_2X_SECTION )
				{
					ad7124_calib( ch, SEN_RTD2X, idx );
					Temp.SetType( ch, SEN_RTD2X, CALIB_ADCSAMPLING_MS );
					memset( &avg_buf, 0, sizeof( avg_buf ) );
					stage = CONV_RTD2X;
				}
				else
				{
					Calib.status.conv_ready = 1;
					Calib.status.conv_idx = idx;
					stage = SAVE_CALIB;
				}
			}
			break;

		case	CONV_RTD2X:
			tprintf( VT100_CURSOR_POS_RESTORE "%.3f [2X측정중]", get_convert_data( SEN_RTD2X, AD7124.GetValue( ch ) ) );
			if( wait_avg_filter( SEN_RTD2X, get_calibration_adc_value( ch ), &avg_buf, &calc_data[ ch ].RTD2X[ idx ] ) )
			{
				Calib.status.conv_ready = 1;
				Calib.status.conv_idx = idx;
				idx++;
				stage = CALIB_INIT;
			}
			break;

		case 	SAVE_CALIB:
			if( type == SEN_RTD )
			{
				calculate_calib( calc_data[ ch ].RTD, Calib.data[ ch ].RTD, Rtd_Slope, CALIB_RTD_SECTION );
				calculate_calib( calc_data[ ch ].RTD2X, Calib.data[ ch ].RTD2X, Rtd_Slope, CALIB_RTD_2X_SECTION );
			}
			else
			{
				calculate_calib( calc_data[ ch ].TC, Calib.data[ ch ].TC, TC_Slope, CALIB_TC_SECTION );
			}
			stage = CALIB_COMPLETE;
			break;

		case 	CALIB_COMPLETE:
			tprintf( "\r\n %dCH : Calibration Complete!\r\n" VT100_CURSOR_POS_RESTORE, ch + 1 );
			idx = 0;
			stage = CALIB_INIT;
			calibration_end = 1;
			break;
	}
	return calibration_end;
}

static uint8_t calib_measure_point( sensor_et type, uint8_t ch, uint8_t idx, char cmd )
{
	static enum { POINT_IDLE, POINT_CONV_TC, POINT_CONV_RTD, POINT_CONV_RTD2X } stage = POINT_IDLE;
	static Avgfilter_buffer_st avg_buf = {0};
	static uint8_t active_ch = 0;
	static uint8_t active_idx = 0;
	uint8_t point_done = 0;

	if( cmd == 0x1B )
	{
		stage = POINT_IDLE;
		memset( &avg_buf, 0, sizeof( avg_buf ) );
		Calib.status.On = 0;
		Calib.status.conv_ready = 0;
		return 1;
	}

	if( cmd == 0x0D )
	{
		if( stage != POINT_IDLE )
		{
			return 0;
		}

		if( ch >= MAX_TEMP_CHANNEL )
		{
			return 0;
		}

		if( ( type == SEN_RTD ) || ( type == SEN_RTD2X ) )
		{
			if( idx > CALIB_RTD_SECTION )
			{
				return 0;
			}
			type = SEN_RTD;
			stage = POINT_CONV_RTD;
		}
		else
		{
			if( idx > CALIB_TC_SECTION )
			{
				return 0;
			}
			if( ( type != SEN_TC_K ) && ( type != SEN_TC_J ) && ( type != SEN_TC_E )
				&& ( type != SEN_TC_S ) && ( type != SEN_TC_T ) && ( type != SEN_TC_R ) )
			{
				return 0;
			}
			type = SEN_TC_K;
			stage = POINT_CONV_TC;
		}

		active_ch = ch;
		active_idx = idx;
		Calib.status.ch = ch;
		Calib.status.On = 1;
		Calib.status.conv_ready = 0;
		memset( &avg_buf, 0, sizeof( avg_buf ) );
		Temp.SetType( ch, type, CALIB_ADCSAMPLING_MS );
		ad7124_calib( ch, type, idx );
	}

	switch( stage )
	{
		case POINT_CONV_TC:
			if( wait_avg_filter( SEN_TC_K, get_calibration_adc_value( active_ch ), &avg_buf, &calc_data[ active_ch ].TC[ active_idx ] ) )
			{
				calib_tc_point_mask[ active_ch ] |= ( 1u << active_idx );
				Calib.status.conv_ready = 1;
				Calib.status.conv_idx = active_idx;
				stage = POINT_IDLE;
				Calib.status.On = 0;
				point_done = 1;
			}
			break;

		case POINT_CONV_RTD:
			if( wait_avg_filter( SEN_RTD, get_calibration_adc_value( active_ch ), &avg_buf, &calc_data[ active_ch ].RTD[ active_idx ] ) )
			{
				if( active_idx <= CALIB_RTD_2X_SECTION )
				{
					ad7124_calib( active_ch, SEN_RTD2X, active_idx );
					Temp.SetType( active_ch, SEN_RTD2X, CALIB_ADCSAMPLING_MS );
					memset( &avg_buf, 0, sizeof( avg_buf ) );
					stage = POINT_CONV_RTD2X;
				}
				else
				{
					calib_rtd_point_mask[ active_ch ] |= ( 1u << active_idx );
					Calib.status.conv_ready = 1;
					Calib.status.conv_idx = active_idx;
					stage = POINT_IDLE;
					Calib.status.On = 0;
					point_done = 1;
				}
			}
			break;

		case POINT_CONV_RTD2X:
			if( wait_avg_filter( SEN_RTD2X, get_calibration_adc_value( active_ch ), &avg_buf, &calc_data[ active_ch ].RTD2X[ active_idx ] ) )
			{
				calib_rtd_point_mask[ active_ch ] |= ( 1u << active_idx );
				calib_rtd2x_point_mask[ active_ch ] |= ( 1u << active_idx );
				Calib.status.conv_ready = 1;
				Calib.status.conv_idx = active_idx;
				stage = POINT_IDLE;
				Calib.status.On = 0;
				point_done = 1;
			}
			break;

		case POINT_IDLE:
		default:
			break;
	}

	return point_done;
}

static void calib_calculate_channel( sensor_et type, uint8_t ch )
{
	if( ( SEN_RTD == type ) || ( SEN_RTD2X == type ) )
	{
		calculate_calib( calc_data[ ch ].RTD, Calib.data[ ch ].RTD, Rtd_Slope, CALIB_RTD_SECTION );
		calculate_calib( calc_data[ ch ].RTD2X, Calib.data[ ch ].RTD2X, Rtd_Slope, CALIB_RTD_2X_SECTION );
	}
	else
	{
		calculate_calib( calc_data[ ch ].TC, Calib.data[ ch ].TC, TC_Slope, CALIB_TC_SECTION );
	}
}

static uint8_t calib_validate_and_save( sensor_et type, uint8_t ch )
{
	float tol;
	float diff;

	Calib.status.save_error = CALIB_SAVE_ERR_NONE;

	if( ch >= MAX_TEMP_CHANNEL )
	{
		Calib.status.save_error = CALIB_SAVE_ERR_VALIDATION;
		return 0;
	}
	if( ( ( SEN_RTD == type ) || ( SEN_RTD2X == type ) )
		&& ( ( calib_rtd_point_mask[ ch ] != 0 ) || ( calib_rtd2x_point_mask[ ch ] != 0 ) )
		&& ( ( calib_rtd_point_mask[ ch ] != ( ( 1u << ( CALIB_RTD_SECTION + 1 ) ) - 1u ) )
			|| ( calib_rtd2x_point_mask[ ch ] != ( ( 1u << ( CALIB_RTD_2X_SECTION + 1 ) ) - 1u ) ) ) )
	{
		Calib.status.save_error = CALIB_SAVE_ERR_VALIDATION;
		return 0;
	}
	if( ( SEN_RTD != type ) && ( SEN_RTD2X != type )
		&& ( calib_tc_point_mask[ ch ] != 0 )
		&& ( calib_tc_point_mask[ ch ] != ( ( 1u << ( CALIB_TC_SECTION + 1 ) ) - 1u ) ) )
	{
		Calib.status.save_error = CALIB_SAVE_ERR_VALIDATION;
		return 0;
	}

	calib_calculate_channel( type, ch );

	if( ( SEN_RTD == type ) || ( SEN_RTD2X == type ) )
	{
		for( int i = 0; i <= CALIB_RTD_SECTION; i++ )
		{
			tol  = RTD_nominal[ i ] * ( ( i == 0 ) ? RTD_LOW_POINT_TOLERANCE_PCT : CALIB_TOLERANCE_PCT );
			diff = calc_data[ ch ].RTD[ i ].Convert - RTD_nominal[ i ];
			if( ( diff < -tol ) || ( diff > tol ) )
			{
				Calib.status.save_error = CALIB_SAVE_ERR_VALIDATION;
				return 0;
			}
		}
		for( int i = 0; i <= CALIB_RTD_2X_SECTION; i++ )
		{
			tol  = RTD_nominal[ i ] * ( ( i == 0 ) ? RTD_LOW_POINT_TOLERANCE_PCT : CALIB_TOLERANCE_PCT );
			diff = calc_data[ ch ].RTD2X[ i ].Convert - RTD_nominal[ i ];
			if( ( diff < -tol ) || ( diff > tol ) )
			{
				Calib.status.save_error = CALIB_SAVE_ERR_VALIDATION;
				return 0;
			}
		}
		if( !save_calib( "RTD", ch, Calib.data[ ch ].RTD, Calib.data[ ch ].AD7124[ SEN_RTD ],
					RTD_EEPRAdd, AD7124_EEPRAdd[ SEN_RTD ][ ch ], CALIB_RTD_SECTION ) )
		{
			Calib.status.save_error = CALIB_SAVE_ERR_EEPROM;
			return 0;
		}
		if( !save_calib( "RTD2X", ch, Calib.data[ ch ].RTD2X, Calib.data[ ch ].AD7124[ SEN_RTD2X ],
					RTD_2X_EEPRAdd, AD7124_EEPRAdd[ SEN_RTD2X ][ ch ], CALIB_RTD_2X_SECTION ) )
		{
			Calib.status.save_error = CALIB_SAVE_ERR_EEPROM;
			return 0;
		}
	}
	else
	{
		for( int i = 0; i <= CALIB_TC_SECTION; i++ )
		{
			tol  = ( i == 0 ) ? ( TC_FULLSCALE_MV * CALIB_TOLERANCE_PCT )
			                  : ( TC_nominal[ i ] * CALIB_TOLERANCE_PCT );
			diff = calc_data[ ch ].TC[ i ].Convert - TC_nominal[ i ];
			if( ( diff < -tol ) || ( diff > tol ) )
			{
				Calib.status.save_error = CALIB_SAVE_ERR_VALIDATION;
				return 0;
			}
		}
		if( !save_calib( "TC", ch, Calib.data[ ch ].TC, Calib.data[ ch ].AD7124[ SEN_TC_K ],
					TC_EEPRAdd, AD7124_EEPRAdd[ SEN_TC_K ][ ch ], CALIB_TC_SECTION ) )
		{
			Calib.status.save_error = CALIB_SAVE_ERR_EEPROM;
			return 0;
		}
	}

	return 1;
}

static float calib_get_conv( sensor_et type, uint8_t ch, uint8_t idx )
{
	if( ch >= MAX_TEMP_CHANNEL )
	{
		return 0.0f;
	}

	if( SEN_RTD2X == type )
	{
		return ( idx <= CALIB_RTD_2X_SECTION ) ? calc_data[ ch ].RTD2X[ idx ].Convert : 0.0f;
	}
	if( SEN_RTD == type )
	{
		return ( idx <= CALIB_RTD_SECTION ) ? calc_data[ ch ].RTD[ idx ].Convert : 0.0f;
	}
	return ( idx <= CALIB_TC_SECTION ) ? calc_data[ ch ].TC[ idx ].Convert : 0.0f;
}

/******************************************************************************
 * @brief Clear the calibration-complete flag in EEPROM and RAM.
 *        Call when a new calibration session starts (>CAL START).
 *****************************************************************************/
static uint8_t calib_clear_done_flag( void )
{
	const uint32_t cleared = 0;

	if( HAL_OK != EEPR.Write( EEPR_ADDR_CALIB_CHK, ( uint8_t * )&cleared, sizeof( cleared ) ) )
	{
		Calib.status.save_error = CALIB_SAVE_ERR_EEPROM;
		return 0;
	}

	calib_chk_flag = false;
	return 1;
}

static uint8_t calib_clear_eeprom_range( uint16_t start, uint16_t end )
{
	extern IWDG_HandleTypeDef hiwdg;
	const uint32_t cleared = 0;
	for( uint16_t addr = start; addr < end; addr += sizeof( cleared ) )
	{
		if( HAL_OK != EEPR.Write( addr, ( uint8_t * )&cleared, sizeof( cleared ) ) )
		{
			Calib.status.save_error = CALIB_SAVE_ERR_EEPROM;
			return 0;
		}
		HAL_IWDG_Refresh( &hiwdg );
	}
	return 1;
}

static uint8_t calib_start_session( calib_session_mode_et mode, uint8_t ch )
{
	uint8_t first_ch = 0;
	uint8_t last_ch = MAX_TEMP_CHANNEL;

	Calib.status.save_error = CALIB_SAVE_ERR_NONE;
	if( mode == CALIB_SESSION_CHANNEL )
	{
		if( ch >= MAX_TEMP_CHANNEL )
		{
			Calib.status.save_error = CALIB_SAVE_ERR_VALIDATION;
			return 0;
		}
		first_ch = ch;
		last_ch = ch + 1;
	}
	else if( mode != CALIB_SESSION_GROUP )
	{
		Calib.status.save_error = CALIB_SAVE_ERR_VALIDATION;
		return 0;
	}

	if( !calib_clear_done_flag() ) return 0;
	for( uint8_t target = first_ch; target < last_ch; target++ )
	{
		uint16_t end = ( target + 1u < MAX_TEMP_CHANNEL )
			? EEPR_start_addr[ target + 1u ]
			: ( EEPR_ADDR_CH4_ADC_TC_STATUS + EEPR_SIZE_ADC_STATUS );

		if( !calib_clear_eeprom_range( EEPR_start_addr[ target ], end ) ) return 0;
		if( !calib_clear_eeprom_range( EEPR_channel_done_addr[ target ],
			EEPR_channel_done_addr[ target ] + EEPR_SIZE_STATUS ) ) return 0;

		memset( &calc_data[ target ], 0, sizeof( calc_data[ target ] ) );
		memset( &Calib.data[ target ], 0, sizeof( Calib.data[ target ] ) );
		calib_rtd_point_mask[ target ] = 0;
		calib_rtd2x_point_mask[ target ] = 0;
		calib_tc_point_mask[ target ] = 0;
		calib_channel_done_mask &= ~( 1u << target );
	}
	return 1;
}

/******************************************************************************
 * @brief Return 1 if all channels (RTD + TC) have been validated and saved.
 *****************************************************************************/
static uint8_t calib_is_done( void )
{
	return calib_chk_flag ? 1u : 0u;
}

static uint8_t calib_get_channel_done_mask( void )
{
	return calib_channel_done_mask;
}

/******************************************************************************
 * @brief Check EEPROM RTD+TC sections for all channels; write the done flag
 *        if every section is valid.
 *****************************************************************************/
static uint8_t calib_complete_channels( uint8_t ch_mask )
{
	uint32_t status;
	const uint8_t all_mask = ( 1u << MAX_TEMP_CHANNEL ) - 1u;

	Calib.status.save_error = CALIB_SAVE_ERR_NONE;
	if( ( ch_mask == 0 ) || ( ch_mask & ~all_mask ) )
	{
		Calib.status.save_error = CALIB_SAVE_ERR_VALIDATION;
		return 0;
	}

	for( uint8_t i = 0; i < CALIB_RTD_2X_SECTION; i++ )
	{
		for( uint8_t ch = 0; ch < MAX_TEMP_CHANNEL; ch++ )
		{
			if( 0 == ( ch_mask & ( 1u << ch ) ) ) continue;
			status = 0;
			if( HAL_OK != EEPR.Read( RTD_2X_EEPRAdd[ i ][ ch ][ 3 ], ( uint8_t * )&status, sizeof( status ) ) )
			{
				Calib.status.save_error = CALIB_SAVE_ERR_EEPROM;
				return 0;
			}
			if( ( status & CALIB_COMPELETED_bp ) != CALIB_COMPELETED_STATUS )
			{
				Calib.status.save_error = CALIB_SAVE_ERR_VALIDATION;
				return 0;
			}
		}
	}

	for( uint8_t i = 0; i < CALIB_RTD_SECTION; i++ )
	{
		for( uint8_t ch = 0; ch < MAX_TEMP_CHANNEL; ch++ )
		{
			if( 0 == ( ch_mask & ( 1u << ch ) ) ) continue;
			status = 0;
			if( HAL_OK != EEPR.Read( RTD_EEPRAdd[ i ][ ch ][ 3 ], ( uint8_t * )&status, sizeof( status ) ) )
			{
				Calib.status.save_error = CALIB_SAVE_ERR_EEPROM;
				return 0;
			}
			if( ( status & CALIB_COMPELETED_bp ) != CALIB_COMPELETED_STATUS )
			{
				Calib.status.save_error = CALIB_SAVE_ERR_VALIDATION;
				return 0;
			}
		}
	}

	for( uint8_t i = 0; i < CALIB_TC_SECTION; i++ )
	{
		for( uint8_t ch = 0; ch < MAX_TEMP_CHANNEL; ch++ )
		{
			if( 0 == ( ch_mask & ( 1u << ch ) ) ) continue;
			status = 0;
			if( HAL_OK != EEPR.Read( TC_EEPRAdd[ i ][ ch ][ 3 ], ( uint8_t * )&status, sizeof( status ) ) )
			{
				Calib.status.save_error = CALIB_SAVE_ERR_EEPROM;
				return 0;
			}
			if( ( status & CALIB_COMPELETED_bp ) != CALIB_COMPELETED_STATUS )
			{
				Calib.status.save_error = CALIB_SAVE_ERR_VALIDATION;
				return 0;
			}
		}
	}

	for( uint8_t ch = 0; ch < MAX_TEMP_CHANNEL; ch++ )
	{
		if( 0 == ( ch_mask & ( 1u << ch ) ) ) continue;
		for( uint8_t type = 0; type < CALIB_PGA_NUM; type++ )
		{
			status = 0;
			if( HAL_OK != EEPR.Read( AD7124_EEPRAdd[ type ][ ch ][ 2 ], ( uint8_t * )&status, sizeof( status ) ) )
			{
				Calib.status.save_error = CALIB_SAVE_ERR_EEPROM;
				return 0;
			}
			if( ( status & CALIB_COMPELETED_bp ) != CALIB_COMPELETED_STATUS )
			{
				Calib.status.save_error = CALIB_SAVE_ERR_VALIDATION;
				return 0;
			}
		}
		const uint32_t channel_done = CALIB_COMPELETED_STATUS;
		if( HAL_OK != EEPR.Write( EEPR_channel_done_addr[ ch ], ( uint8_t * )&channel_done, sizeof( channel_done ) ) )
		{
			Calib.status.save_error = CALIB_SAVE_ERR_EEPROM;
			return 0;
		}
		calib_channel_done_mask |= ( 1u << ch );
	}

	if( calib_channel_done_mask == all_mask )
	{
		const uint32_t done_val = CALIB_COMPELETED_STATUS;
		if( HAL_OK != EEPR.Write( EEPR_ADDR_CALIB_CHK, ( uint8_t * )&done_val, sizeof( done_val ) ) )
		{
			Calib.status.save_error = CALIB_SAVE_ERR_EEPROM;
			return 0;
		}
		calib_chk_flag = true;
	}
	return 1;
}
