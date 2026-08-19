/******************************************************************************
 * @file d_ad7124.c
 * @author jylee1 (jylee1@gst-in.com)
 * @brief 
 * @version 0.1
 * @date 2020-08-13
 * 
 * @copyright Copyright (c) 2020
 * 
 *****************************************************************************/

/*****************************************************************************/
/** INCLUDES *****************************************************************/
/*****************************************************************************/
#include "UserDrivers.h"

/*****************************************************************************/
/** MACRO DEFINITIONS ********************************************************/
/*****************************************************************************/

/* Error codes */
#define INVALID_VAL 					-1 /* Invalid argument */
#define COMM_ERR   						-2 /* Communication error on receive */
#define TIMEOUT     					-3 /* A timeout has occured */

#define AD7124_POST_RESET_DELAY      	0.004f

#define MINIMUM_WAIT_TIME_FOR_DRDY	 	0.02f
#define EXCITATION_WAIT_TIME 			0.01f

#define AD7124_INTERNALCALIB_FS			2047 // 2.34sps in mid power mode

#define AD7124_SPI_TIMEOUT				100

#define FCLK_FULL_POWER_KHz				614.4f

#define SINC4_SETTILING_TIMES		4
#define SINC3_SETTILING_TIMES		3

/*****************************************************************************/
/** DATATYPES ****************************************************************/
/*****************************************************************************/

/*****************************************************************************/
/** FUNCTION DECLARATIONS ****************************************************/
/*****************************************************************************/
static void ad7124_rtd_setting( ad7124_dev_st *pDev, uint8_t AdcPga, uint16_t sps );
static void ad7124_tc_setting( ad7124_dev_st *pDev, uint16_t sps );
static int32_t get_ad7124_raw_value( uint8_t ch );
static int32_t get_ad7124_value( uint8_t ch );
static int32_t ad7124_read_register( ad7124_dev_st *pDev, int addr );
static int32_t ad7124_write_register( ad7124_dev_st *pDev, int addr );
static int32_t ad7124_no_check_read_register( ad7124_dev_st *pDev, int addr );
static int32_t ad7124_no_check_write_register( ad7124_dev_st *pDev, int addr );
static int32_t ad7124_reset( ad7124_dev_st *pDev );
static int32_t ad7124_wait_for_spi_ready( ad7124_dev_st *pDev );
static int32_t ad7124_wait_to_power_on( ad7124_dev_st *pDev, uint32_t timeout );
static int32_t ad7124_wait_for_conv_ready( ad7124_dev_st *pDev, uint32_t timeout ) __attribute__((unused));
static int32_t ad7124_read_data( ad7124_dev_st *pDev, int32_t *pData );
static int32_t ad7124_disable_channel( ad7124_dev_st *pDev );
static int32_t ad7124_enable_channel( ad7124_dev_st *pDev );
static uint8_t ad7124_compute_crc8( uint8_t *p_buf, uint8_t buf_size );
static void ad7124_update_crcsetting( ad7124_dev_st *pDev );
static void ad7124_update_dev_spi_settings( ad7124_dev_st *pDev );
static int32_t ad7124_setup( ad7124_dev_st *pDev );
static void set_sensor_type( uint8_t ch, sensor_et type );
static sensor_et get_sensor_type( uint8_t ch );
static void set_sampling_period( uint8_t ch, uint16_t sampling_period );
static uint16_t get_sampling_period( uint8_t ch );
static void ad7124_set_calibfactor( uint8_t ch, uint32_t gain, uint32_t offset );
static void ad7124_internalCalib_RTD( uint8_t ch, uint8_t pga );
static void ad7124_internalCalib_TC( uint8_t ch, uint8_t pga );
static uint32_t get_ad7124_register_value( uint8_t dev, uint8_t reg_no );
static float get_effective_bit( float sps, ad7124_PGA_gain_et pga );
/*****************************************************************************/
/** EXTERNAL VARIABLES *******************************************************/
/*****************************************************************************/

extern SPI_HandleTypeDef hspi2;

/*****************************************************************************/
/** LOCAL VARIABLES **********************************************************/
/*****************************************************************************/
static CCMRAM ad7124_reg_st ad7124_regs[ MAX_ADC_DEV ][ AD7124_REG_NO ] =
{
	{
		{ 0x00, 0x00, 1, AD7124_R }, /* AD7124_Status */
		{ 0x01, 0x0000, 2, AD7124_RW }, /* AD7124_ADC_Control */
		{ 0x02, 0x0000, 3, AD7124_R }, /* AD7124_Data */
		{ 0x03, 0x0000, 3, AD7124_RW }, /* AD7124_IOCon1 */
		{ 0x04, 0x0000, 2, AD7124_RW }, /* AD7124_IOCon2 */
		{ 0x05, 0x02, 1, AD7124_R }, /* AD7124_ID */
		{ 0x06, 0x0000, 3, AD7124_R }, /* AD7124_Error */
		{ 0x07, 0x0040, 3, AD7124_RW }, /* AD7124_Error_En */
		{ 0x08, 0x00, 1, AD7124_R }, /* AD7124_Mclk_Count */
		{ 0x09, 0x0001, 2, AD7124_RW }, /* AD7124_Channel_0 */
		{ 0x0A, 0x0001, 2, AD7124_RW }, /* AD7124_Channel_1 */
		{ 0x0B, 0x0001, 2, AD7124_RW }, /* AD7124_Channel_2 */
		{ 0x0C, 0x0001, 2, AD7124_RW }, /* AD7124_Channel_3 */
		{ 0x0D, 0x0001, 2, AD7124_RW }, /* AD7124_Channel_4 */
		{ 0x0E, 0x0001, 2, AD7124_RW }, /* AD7124_Channel_5 */
		{ 0x0F, 0x0001, 2, AD7124_RW }, /* AD7124_Channel_6 */
		{ 0x10, 0x0001, 2, AD7124_RW }, /* AD7124_Channel_7 */
		{ 0x11, 0x0001, 2, AD7124_RW }, /* AD7124_Channel_8 */
		{ 0x12, 0x0001, 2, AD7124_RW }, /* AD7124_Channel_9 */
		{ 0x13, 0x0001, 2, AD7124_RW }, /* AD7124_Channel_10 */
		{ 0x14, 0x0001, 2, AD7124_RW }, /* AD7124_Channel_11 */
		{ 0x15, 0x0001, 2, AD7124_RW }, /* AD7124_Channel_12 */
		{ 0x16, 0x0001, 2, AD7124_RW }, /* AD7124_Channel_13 */
		{ 0x17, 0x0001, 2, AD7124_RW }, /* AD7124_Channel_14 */
		{ 0x18, 0x0001, 2, AD7124_RW }, /* AD7124_Channel_15 */
		{ 0x19, 0x0860, 2, AD7124_RW }, /* AD7124_Config_0 */
		{ 0x1A, 0x0860, 2, AD7124_RW }, /* AD7124_Config_1 */
		{ 0x1B, 0x0860, 2, AD7124_RW }, /* AD7124_Config_2 */
		{ 0x1C, 0x0860, 2, AD7124_RW }, /* AD7124_Config_3 */
		{ 0x1D, 0x0860, 2, AD7124_RW }, /* AD7124_Config_4 */
		{ 0x1E, 0x0860, 2, AD7124_RW }, /* AD7124_Config_5 */
		{ 0x1F, 0x0860, 2, AD7124_RW }, /* AD7124_Config_6 */
		{ 0x20, 0x0860, 2, AD7124_RW }, /* AD7124_Config_7 */
		{ 0x21, 0x060180, 3, AD7124_RW }, /* AD7124_Filter_0 */
		{ 0x22, 0x060180, 3, AD7124_RW }, /* AD7124_Filter_1 */
		{ 0x23, 0x060180, 3, AD7124_RW }, /* AD7124_Filter_2 */
		{ 0x24, 0x060180, 3, AD7124_RW }, /* AD7124_Filter_3 */
		{ 0x25, 0x060180, 3, AD7124_RW }, /* AD7124_Filter_4 */
		{ 0x26, 0x060180, 3, AD7124_RW }, /* AD7124_Filter_5 */
		{ 0x27, 0x060180, 3, AD7124_RW }, /* AD7124_Filter_6 */
		{ 0x28, 0x060180, 3, AD7124_RW }, /* AD7124_Filter_7 */
		{ 0x29, 0x800000, 3, AD7124_RW }, /* AD7124_Offset_0 */
		{ 0x2A, 0x800000, 3, AD7124_RW }, /* AD7124_Offset_1 */
		{ 0x2B, 0x800000, 3, AD7124_RW }, /* AD7124_Offset_2 */
		{ 0x2C, 0x800000, 3, AD7124_RW }, /* AD7124_Offset_3 */
		{ 0x2D, 0x800000, 3, AD7124_RW }, /* AD7124_Offset_4 */
		{ 0x2E, 0x800000, 3, AD7124_RW }, /* AD7124_Offset_5 */
		{ 0x2F, 0x800000, 3, AD7124_RW }, /* AD7124_Offset_6 */
		{ 0x30, 0x800000, 3, AD7124_RW }, /* AD7124_Offset_7 */
		{ 0x31, 0x500000, 3, AD7124_RW }, /* AD7124_Gain_0 */
		{ 0x32, 0x500000, 3, AD7124_RW }, /* AD7124_Gain_1 */
		{ 0x33, 0x500000, 3, AD7124_RW }, /* AD7124_Gain_2 */
		{ 0x34, 0x500000, 3, AD7124_RW }, /* AD7124_Gain_3 */
		{ 0x35, 0x500000, 3, AD7124_RW }, /* AD7124_Gain_4 */
		{ 0x36, 0x500000, 3, AD7124_RW }, /* AD7124_Gain_5 */
		{ 0x37, 0x500000, 3, AD7124_RW }, /* AD7124_Gain_6 */
		{ 0x38, 0x500000, 3, AD7124_RW }, /* AD7124_Gain_7 */
	},
	{
		{ 0x00, 0x00, 1, AD7124_R }, /* AD7124_Status */
		{ 0x01, 0x0000, 2, AD7124_RW }, /* AD7124_ADC_Control */
		{ 0x02, 0x0000, 3, AD7124_R }, /* AD7124_Data */
		{ 0x03, 0x0000, 3, AD7124_RW }, /* AD7124_IOCon1 */
		{ 0x04, 0x0000, 2, AD7124_RW }, /* AD7124_IOCon2 */
		{ 0x05, 0x02, 1, AD7124_R }, /* AD7124_ID */
		{ 0x06, 0x0000, 3, AD7124_R }, /* AD7124_Error */
		{ 0x07, 0x0040, 3, AD7124_RW }, /* AD7124_Error_En */
		{ 0x08, 0x00, 1, AD7124_R }, /* AD7124_Mclk_Count */
		{ 0x09, 0x0001, 2, AD7124_RW }, /* AD7124_Channel_0 */
		{ 0x0A, 0x0001, 2, AD7124_RW }, /* AD7124_Channel_1 */
		{ 0x0B, 0x0001, 2, AD7124_RW }, /* AD7124_Channel_2 */
		{ 0x0C, 0x0001, 2, AD7124_RW }, /* AD7124_Channel_3 */
		{ 0x0D, 0x0001, 2, AD7124_RW }, /* AD7124_Channel_4 */
		{ 0x0E, 0x0001, 2, AD7124_RW }, /* AD7124_Channel_5 */
		{ 0x0F, 0x0001, 2, AD7124_RW }, /* AD7124_Channel_6 */
		{ 0x10, 0x0001, 2, AD7124_RW }, /* AD7124_Channel_7 */
		{ 0x11, 0x0001, 2, AD7124_RW }, /* AD7124_Channel_8 */
		{ 0x12, 0x0001, 2, AD7124_RW }, /* AD7124_Channel_9 */
		{ 0x13, 0x0001, 2, AD7124_RW }, /* AD7124_Channel_10 */
		{ 0x14, 0x0001, 2, AD7124_RW }, /* AD7124_Channel_11 */
		{ 0x15, 0x0001, 2, AD7124_RW }, /* AD7124_Channel_12 */
		{ 0x16, 0x0001, 2, AD7124_RW }, /* AD7124_Channel_13 */
		{ 0x17, 0x0001, 2, AD7124_RW }, /* AD7124_Channel_14 */
		{ 0x18, 0x0001, 2, AD7124_RW }, /* AD7124_Channel_15 */
		{ 0x19, 0x0860, 2, AD7124_RW }, /* AD7124_Config_0 */
		{ 0x1A, 0x0860, 2, AD7124_RW }, /* AD7124_Config_1 */
		{ 0x1B, 0x0860, 2, AD7124_RW }, /* AD7124_Config_2 */
		{ 0x1C, 0x0860, 2, AD7124_RW }, /* AD7124_Config_3 */
		{ 0x1D, 0x0860, 2, AD7124_RW }, /* AD7124_Config_4 */
		{ 0x1E, 0x0860, 2, AD7124_RW }, /* AD7124_Config_5 */
		{ 0x1F, 0x0860, 2, AD7124_RW }, /* AD7124_Config_6 */
		{ 0x20, 0x0860, 2, AD7124_RW }, /* AD7124_Config_7 */
		{ 0x21, 0x060180, 3, AD7124_RW }, /* AD7124_Filter_0 */
		{ 0x22, 0x060180, 3, AD7124_RW }, /* AD7124_Filter_1 */
		{ 0x23, 0x060180, 3, AD7124_RW }, /* AD7124_Filter_2 */
		{ 0x24, 0x060180, 3, AD7124_RW }, /* AD7124_Filter_3 */
		{ 0x25, 0x060180, 3, AD7124_RW }, /* AD7124_Filter_4 */
		{ 0x26, 0x060180, 3, AD7124_RW }, /* AD7124_Filter_5 */
		{ 0x27, 0x060180, 3, AD7124_RW }, /* AD7124_Filter_6 */
		{ 0x28, 0x060180, 3, AD7124_RW }, /* AD7124_Filter_7 */
		{ 0x29, 0x800000, 3, AD7124_RW }, /* AD7124_Offset_0 */
		{ 0x2A, 0x800000, 3, AD7124_RW }, /* AD7124_Offset_1 */
		{ 0x2B, 0x800000, 3, AD7124_RW }, /* AD7124_Offset_2 */
		{ 0x2C, 0x800000, 3, AD7124_RW }, /* AD7124_Offset_3 */
		{ 0x2D, 0x800000, 3, AD7124_RW }, /* AD7124_Offset_4 */
		{ 0x2E, 0x800000, 3, AD7124_RW }, /* AD7124_Offset_5 */
		{ 0x2F, 0x800000, 3, AD7124_RW }, /* AD7124_Offset_6 */
		{ 0x30, 0x800000, 3, AD7124_RW }, /* AD7124_Offset_7 */
		{ 0x31, 0x500000, 3, AD7124_RW }, /* AD7124_Gain_0 */
		{ 0x32, 0x500000, 3, AD7124_RW }, /* AD7124_Gain_1 */
		{ 0x33, 0x500000, 3, AD7124_RW }, /* AD7124_Gain_2 */
		{ 0x34, 0x500000, 3, AD7124_RW }, /* AD7124_Gain_3 */
		{ 0x35, 0x500000, 3, AD7124_RW }, /* AD7124_Gain_4 */
		{ 0x36, 0x500000, 3, AD7124_RW }, /* AD7124_Gain_5 */
		{ 0x37, 0x500000, 3, AD7124_RW }, /* AD7124_Gain_6 */
		{ 0x38, 0x500000, 3, AD7124_RW }, /* AD7124_Gain_7 */
	}
};

static AppTimerData_ut timerWaitIexc;
static AppTimerData_ut timerWaitDrdy;

static CCMRAM ad7124_dev_st ad7124_dev[ MAX_ADC_DEV ] =
{
	{
		.regs = ad7124_regs[ 0 ],
		.spi_rdy_poll_cnt = 10000,
		.cs =
		{
			.port = ADC12_NSS_PORT,
			.pin = ADC12_NSS_PIN,
		},
		.port = SPI_ADC,
		.hspi = &hspi2,
		.rxbuf = { 0 },
		.txbuf = { 0 },
		.RawValue = { 0 },
		.Value = { 0 },
		.Status =
		{
			.cycled = 0,
			.aquired = 0,
			.channel = 1,
			.use_crc = AD7124_DISABLE_CRC,
			.check_ready = 0,
		},
		.type = { 0 },
		.SamplePeriod = { 0 },
	},
	{
		.regs = ad7124_regs[ 1 ],
		.spi_rdy_poll_cnt = 10000,
		.cs =
		{
			.port = ADC34_NSS_PORT,
			.pin = ADC34_NSS_PIN,
		},
		.port = SPI_ADC,
		.hspi = &hspi2,
		.rxbuf = { 0 },
		.txbuf = { 0 },
		.RawValue = { 0 },
		.Value = { 0 },
		.Status =
		{
			.cycled = 0,
			.aquired = 0,
			.channel = 1,
			.use_crc = AD7124_DISABLE_CRC,
			.check_ready = 0,
		},
		.type = { 0 },
		.SamplePeriod = { 0 },
	}
};

#define SPS_ITEMS 15

static const float AD7124_SPS[ SPS_ITEMS ] = { 9.4, 10, 20, 40, 50, 60, 80, 160, 320, 640, 1280, 2400, 4800, 9600, 19200 };
static const float AD7124_PGA8_EFFECT_NUM[ SPS_ITEMS ] = { 1.11226E-07, 1.19209E-07, 1.68587E-07, 2.22452E-07, 2.55531E-07, 2.73871E-07, 3.14595E-07, 4.76837E-07, 6.2919E-07, 8.8981E-07, 1.3487E-06, 1.90735E-06, 3.0985E-06, 6.64177E-06, 2.15792E-05 };
static const float AD7124_PGA16_EFFECT_NUM[ SPS_ITEMS ] = { 1.46764E-07, 1.57298E-07, 2.22452E-07, 2.93528E-07, 3.37175E-07, 3.61375E-07, 4.15111E-07, 6.2919E-07, 8.30222E-07, 1.17411E-06, 1.77962E-06, 2.51676E-06, 4.08849E-06, 8.17698E-06, 2.3128E-05 };
static const float AD7124_PGA32_EFFECT_NUM[ SPS_ITEMS ] = { 1.93656E-07, 1.93656E-07, 2.73871E-07, 4.15111E-07, 4.44905E-07, 4.76837E-07, 5.87055E-07, 8.8981E-07, 1.17411E-06, 1.66044E-06, 2.34822E-06, 3.32089E-06, 5.3948E-06, 1.07896E-05, 2.84739E-05 };
static const float AD7124_PGA64_EFFECT_NUM[ SPS_ITEMS ] = { 3.14595E-07, 3.14595E-07, 4.44905E-07, 6.7435E-07, 7.2275E-07, 8.30222E-07, 9.53674E-07, 1.3487E-06, 1.90735E-06, 2.6974E-06, 3.8147E-06, 5.3948E-06, 8.76387E-06, 1.6354E-05, 4.02682E-05 };

/*****************************************************************************/
/** GLOBAL VARIABLES *********************************************************/
/*****************************************************************************/

const AD7124_st AD7124 =
{
	.dev = ad7124_dev,
	.SetType = set_sensor_type,
	.GetType =get_sensor_type,
	.SetSamplePeriod = set_sampling_period,
	.GetSamplePeriod = get_sampling_period,
	.GetRawValue = get_ad7124_raw_value,
	.GetValue = get_ad7124_value,
	.ApplyCalibData = ad7124_set_calibfactor,
	.CalibrateRTD = ad7124_internalCalib_RTD,
	.CalibrateTC = ad7124_internalCalib_TC,
	.GetRegister = get_ad7124_register_value,
	.GetEffectBit = get_effective_bit,
};

/*****************************************************************************/
/** FUNCTION DEFINITIONS *****************************************************/
/*****************************************************************************/
/******************************************************************************
 * @brief ad7124_set_calibfactor
 *****************************************************************************/
static void ad7124_set_calibfactor( uint8_t ch, uint32_t gain, uint32_t offset )
{
	if( ch >= MAX_TEMP_CHANNEL ) return;

	ad7124_regs[ ch / MAX_ADC_DEV ][ AD7124_Gain_0 + ( ch % MAX_ADC_CH ) ].value = gain;
	ad7124_write_register( &AD7124.dev[ ch / MAX_ADC_DEV ], AD7124_Gain_0 + ( ch % MAX_ADC_CH ) );

	ad7124_regs[ ch / MAX_ADC_DEV ][ AD7124_Offset_0 + ( ch % MAX_ADC_CH ) ].value = offset;
	ad7124_write_register( &AD7124.dev[ ch / MAX_ADC_DEV ], AD7124_Offset_0 + ( ch % MAX_ADC_CH ) );
}
/******************************************************************************
 * @brief get_effective_bit
 *****************************************************************************/
static float get_effective_bit( float sampling_period, ad7124_PGA_gain_et pga )
{
	uint8_t index = 0;
	uint16_t sps = 1000 / sampling_period * SINC4_SETTILING_TIMES;

	for( int i = 0; i < SPS_ITEMS; i++ )
	{
		if( sps <= AD7124_SPS[ i ] )
		{
			index = i;
			break;
		}
	}
	
	if( index > 0 )
	{
		if( ( sps - AD7124_SPS[ index - 1] ) < ( AD7124_SPS[ index ] - sps ) )
		{
			index = index - 1;
		}
	}

	switch( pga )
	{
		case AD7124_PGA_Gain_8:
			return AD7124_PGA8_EFFECT_NUM[ index ];
		case AD7124_PGA_Gain_16:
			return AD7124_PGA16_EFFECT_NUM[ index ];
		case AD7124_PGA_Gain_32:
			return AD7124_PGA32_EFFECT_NUM[ index ];
		case AD7124_PGA_Gain_64:
			return AD7124_PGA64_EFFECT_NUM[ index ];
		default:
			return AD7124_UNIPOLAR_STEP;
	}
}
/******************************************************************************
 * @brief set_sensor_type
 *****************************************************************************/
static void set_sensor_type( uint8_t ch, sensor_et type )
{
	if( ch >= MAX_TEMP_CHANNEL ) return;

	uint8_t dev = ch / MAX_ADC_DEV;
	uint8_t ach = ch % MAX_ADC_CH;

	if( AD7124.dev[ dev ].type[ ach ] != type )
	{
		AD7124.dev[ dev ].mf.valid_count[ ach ] = 0;
	}
	AD7124.dev[ dev ].type[ ach ] = type;
}
/******************************************************************************
 * @brief get_sensor_type
 *****************************************************************************/
static sensor_et get_sensor_type( uint8_t ch )
{
	if( ch >= MAX_TEMP_CHANNEL ) return SEN_NONE;

	return AD7124.dev[ ch / MAX_ADC_DEV ].type[ ch % MAX_ADC_CH ];
}

/******************************************************************************
 * @brief set_sampling_period
 * @note fadc = fclk / (4 * 32 * FS[10:0])
 * FS[10:0 ] : filter register
 * fclk = 614.4 [kHz]
 * sampling_period : [ms]
 *****************************************************************************/
static void set_sampling_period( uint8_t ch, uint16_t sampling_period )
{
	if( sampling_period == 0 ) return;

	AD7124.dev[ ch / MAX_ADC_DEV ].SamplePeriod[ ch % MAX_ADC_CH ] = sampling_period;
}

/******************************************************************************
 * @brief get_sampling_period
 * @note fadc = fclk / (4 * 32 * FS[10:0])
 * FS[10:0 ] : filter register
 * fclk = 614.4 [kHz]
 * sampling_period : [ms]
 *****************************************************************************/
static uint16_t get_sampling_period( uint8_t ch )
{
	if( ch >= MAX_TEMP_CHANNEL ) return 0xFFFF;

	return AD7124.dev[ ch / MAX_ADC_DEV ].SamplePeriod[ ch % MAX_ADC_CH ];
}
/******************************************************************************
 * @brief Return the latest ADC conversion before the median filter.
 *****************************************************************************/
static int32_t get_ad7124_raw_value( uint8_t channel )
{
	if( channel >= MAX_TEMP_CHANNEL ) return 0;

	uint8_t ic = channel / MAX_ADC_DEV;
	uint8_t ch = channel % MAX_ADC_CH;
	int32_t value = AD7124.dev[ ic ].RawValue[ ch ];

	if( SEN_TC_K <= AD7124.dev[ ic ].type[ ch ] && AD7124.dev[ ic ].type[ ch ] <= SEN_TC_R )
	{
		value -= 0x800000;
	}

	return value;
}
/******************************************************************************
 * @brief set_sensor_type
 * @note TC의 경우 bipolar로 adc가 설정되므로 value -0x800000 가 필요함.
 *****************************************************************************/
static int32_t get_ad7124_value( uint8_t channel )
{	
	if( channel >= MAX_TEMP_CHANNEL ) return 0;

	uint8_t ic = channel / MAX_ADC_DEV;
	uint8_t ch = channel % MAX_ADC_CH;

	int32_t value = AD7124.dev[ ic ].Value[ ch ];

	if( SEN_TC_K <= AD7124.dev[ ic ].type[ ch ] && AD7124.dev[ ic ].type[ ch ] <= SEN_TC_R )
	{
		value -= 0x800000;
	}
	
	return value;
}
/******************************************************************************
 * @brief get_ad7124_register_value
 *****************************************************************************/
static uint32_t get_ad7124_register_value( uint8_t dev, uint8_t reg_no )
{
	if( dev >= MAX_ADC_DEV ) return 0xffffffff;

	ad7124_read_register( &AD7124.dev[ dev ], reg_no );
	return ad7124_regs[ dev ][ reg_no ].value;
}
/******************************************************************************
 * @brief
 *
 * @param pDev
 *****************************************************************************/
static void ad7124_rtd_setting( ad7124_dev_st *pDev, uint8_t AdcPga, uint16_t sps )
{
	int channel;
	ad7124_reg_st *regs;

	if( NULL == pDev ) return;

	regs = pDev->regs;
	channel = pDev->Status.channel;

	if( channel == 0 )
	{
		regs[ AD7124_IOCon1 ].value = AD7124_IO_CTRL1_REG_IOUT_CH0( 0 )		// AIN0
									  | AD7124_IO_CTRL1_REG_IOUT_CH1( 1 )	// AIN1
									  | AD7124_IO_CTRL1_REG_IOUT0( 3 )		// 250uA
									  | AD7124_IO_CTRL1_REG_IOUT1( 3 );		// 250uA
	}
	else if( channel == 1 )
	{
		regs[ AD7124_IOCon1 ].value = AD7124_IO_CTRL1_REG_IOUT_CH0( 10 )	// AIN4
									  | AD7124_IO_CTRL1_REG_IOUT_CH1( 11 )	// AIN5
									  | AD7124_IO_CTRL1_REG_IOUT0( 3 )		// 250uA
									  | AD7124_IO_CTRL1_REG_IOUT1( 3 );		// 250uA
	}

	ad7124_write_register( pDev, AD7124_IOCon1 );

	regs[ AD7124_IOCon2 ].value = 0x0;					// VBIAS ALL OFF
	ad7124_write_register( pDev, AD7124_IOCon2 );

	regs[ AD7124_Channel_0 ].value = AD7124_CH_MAP_REG_SETUP( 0 )
									 | AD7124_CH_MAP_REG_AINP( 2 )
									 | AD7124_CH_MAP_REG_AINM( 3 );
	ad7124_write_register( pDev, AD7124_Channel_0 );

	regs[ AD7124_Channel_1 ].value = AD7124_CH_MAP_REG_SETUP( 1 )
									 | AD7124_CH_MAP_REG_AINP( 6 )
									 | AD7124_CH_MAP_REG_AINM( 7 );
	ad7124_write_register( pDev, AD7124_Channel_1 );

	regs[ AD7124_Config_0 ].value = AD7124_CFG_REG_PGA( AdcPga )
									| AD7124_CFG_REG_REF_BUFP
									| AD7124_CFG_REG_REF_BUFM
									| AD7124_CFG_REG_AIN_BUFP
									| AD7124_CFG_REG_AIN_BUFM
									| AD7124_CFG_REG_REF_SEL( 0 );
	ad7124_write_register( pDev, AD7124_Config_0 );

	regs[ AD7124_Config_1 ].value = AD7124_CFG_REG_PGA( AdcPga )
									| AD7124_CFG_REG_REF_BUFP
									| AD7124_CFG_REG_REF_BUFM
									| AD7124_CFG_REG_AIN_BUFP
									| AD7124_CFG_REG_AIN_BUFM
									| AD7124_CFG_REG_REF_SEL( 0 );
	ad7124_write_register( pDev, AD7124_Config_1 );

	regs[ AD7124_Filter_0 ].value = AD7124_FILT_REG_FS( sps );
	ad7124_write_register( pDev, AD7124_Filter_0 );
	regs[ AD7124_Filter_1 ].value = AD7124_FILT_REG_FS( sps );
	ad7124_write_register( pDev, AD7124_Filter_1 );
}

/******************************************************************************
 * @brief
 *
 * @param pDev
 *****************************************************************************/
static void ad7124_tc_setting( ad7124_dev_st *pDev, uint16_t sps )
{
	ad7124_reg_st *regs;

	if( NULL == pDev ) return;

	regs = pDev->regs;

	regs[ AD7124_IOCon1 ].value = 0;					// EXCITAION CURRENT ALL OFF
	ad7124_write_register( pDev, AD7124_IOCon1 );

	/* Enable Vbias on negative pin */
	regs[ AD7124_IOCon2 ].value = AD7124_IO_CTRL2_REG_GPIO_VBIAS7 | AD7124_IO_CTRL2_REG_GPIO_VBIAS3;
	ad7124_write_register( pDev, AD7124_IOCon2 );

	regs[ AD7124_Channel_0 ].value = AD7124_CH_MAP_REG_SETUP( 0 )
									 | AD7124_CH_MAP_REG_AINP( 2 )
									 | AD7124_CH_MAP_REG_AINM( 3 );
	ad7124_write_register( pDev, AD7124_Channel_0 );

	regs[ AD7124_Channel_1 ].value = AD7124_CH_MAP_REG_SETUP( 1 )
									 | AD7124_CH_MAP_REG_AINP( 6 )
									 | AD7124_CH_MAP_REG_AINM( 7 );
	ad7124_write_register( pDev, AD7124_Channel_1 );

	regs[ AD7124_Config_0 ].value = AD7124_CFG_REG_PGA( AD7124_PGA_TC )
									| AD7124_CFG_REG_REF_BUFP
									| AD7124_CFG_REG_REF_BUFM
									| AD7124_CFG_REG_AIN_BUFP
									| AD7124_CFG_REG_AIN_BUFM
									| AD7124_CFG_REG_REF_SEL( 2 )
									| AD7124_CFG_REG_BIPOLAR;			
	ad7124_write_register( pDev, AD7124_Config_0 );

	regs[ AD7124_Config_1 ].value = AD7124_CFG_REG_PGA( AD7124_PGA_TC )
									| AD7124_CFG_REG_REF_BUFP
									| AD7124_CFG_REG_REF_BUFM
									| AD7124_CFG_REG_AIN_BUFP
									| AD7124_CFG_REG_AIN_BUFM
									| AD7124_CFG_REG_REF_SEL( 2 )
									| AD7124_CFG_REG_BIPOLAR;
	ad7124_write_register( pDev, AD7124_Config_1 );

	regs[ AD7124_Filter_0 ].value = AD7124_FILT_REG_FS( sps );
	ad7124_write_register( pDev, AD7124_Filter_0 );
	regs[ AD7124_Filter_1 ].value = AD7124_FILT_REG_FS( sps );
	ad7124_write_register( pDev, AD7124_Filter_1 );
}

/******************************************************************************
 * @brief Reads the value of the specified register without checking if the device is ready to accept user requests.
 *
 * @param pDev Pointer to the ad7124 device to be read.
 * @param addr Number of register address to be read.
 * @return 0 for success or negative error code.
 *****************************************************************************/
static int32_t ad7124_no_check_read_register( ad7124_dev_st *pDev, int addr )
{
	int32_t ret = 0;
	uint8_t i = 0;
	uint8_t check8 = 0, add_status_length = 0;
	uint8_t msg_buf[ 8 ] = { 0, 0, 0, 0, 0, 0, 0, 0 };

	if( !pDev ) return INVALID_VAL;

	ad7124_reg_st *pReg = &pDev->regs[ addr ];

	if( !pReg ) return INVALID_VAL;

	/* Build the Command word */
	pDev->txbuf[ 0 ] = AD7124_COMM_REG_WEN | AD7124_COMM_REG_RD | AD7124_COMM_REG_RA( pReg->addr );

	/*
	 * If this is an AD7124_DATA register read, and the DATA_STATUS bit is set
	 * in ADC_CONTROL, need to read 4, not 3 bytes for DATA with STATUS
	 */
	if( ( pReg->addr == AD7124_DATA_REG ) &&
		( pDev->regs[ AD7124_ADC_Control ].value & AD7124_ADC_CTRL_REG_DATA_STATUS ) )
	{
		add_status_length = 1;
	}

	DIO.Output( pDev->cs.port, pDev->cs.pin, GPIO_LOW );
	/* Read data from the device */
	SPI.Transceive( pDev->port, pDev->txbuf, pDev->rxbuf,
					( ( pDev->Status.use_crc == AD7124_USE_CRC ) ? pReg->size + 1 : pReg->size ) + 1 + add_status_length, AD7124_SPI_TIMEOUT );
	SPI.WaitXferComplete( pDev->port, AD7124_SPI_TIMEOUT );
	DIO.Output( pDev->cs.port, pDev->cs.pin, GPIO_HIGH );

	if( ret < 0 ) return ret;

	/* Check the CRC */
	if( pDev->Status.use_crc == AD7124_USE_CRC )
	{
		msg_buf[ 0 ] = AD7124_COMM_REG_WEN | AD7124_COMM_REG_RD | AD7124_COMM_REG_RA( pReg->addr );
		for( i = 1; i < pReg->size + 2 + add_status_length; ++i )
		{
			msg_buf[ i ] = pDev->txbuf[ i ];
		}
		check8 = ad7124_compute_crc8( msg_buf, pReg->size + 2 + add_status_length );
	}

	if( check8 != 0 )
	{
		/* ReadRegister checksum failed. */
		return COMM_ERR;
	}

	/*
	 * if reading Data with 4 bytes, need to copy the status byte to the STATUS
	 * register struct value member
	 */
	if( add_status_length )
	{
		pDev->regs[ AD7124_Status ].value = pDev->txbuf[ pReg->size + 1 ];
	}

	/* Build the result */
	pReg->value = 0;
	for( i = 1; i < pReg->size + 1; i++ )
	{
		pReg->value <<= 8;
		pReg->value += pDev->rxbuf[ i ];
	}

	return ret;
}

/******************************************************************************
 * @brief Writes the value of the specified register without checking if the device is ready to accept user requests.
 *
 * @param pDev Pointer to the ad7124 device to be written.
 * @param addr Number of register address to be written.
 * @return 0 for success or negative error code.
 *****************************************************************************/
static int32_t ad7124_no_check_write_register( ad7124_dev_st *pDev, int addr )
{
	int32_t ret = 0;
	int32_t reg_value = 0;
	uint8_t wr_buf[ 8 ] = { 0, 0, 0, 0, 0, 0, 0, 0 };
	uint8_t crc8 = 0;

	if( NULL == pDev ) return INVALID_VAL;

	ad7124_reg_st reg = pDev->regs[ addr ];

	/* Build the Command word */
	pDev->txbuf[ 0 ] = AD7124_COMM_REG_WEN | AD7124_COMM_REG_WR | AD7124_COMM_REG_RA( reg.addr );

	/* Fill the write buffer */
	reg_value = reg.value;
	for( uint8_t idx = 0; idx < reg.size; idx++ )
	{
		pDev->txbuf[ reg.size - idx ] = reg_value & 0xFF;
		reg_value >>= 8;
	}

	/* Compute the CRC */
	if( AD7124.dev[ 0 ].Status.use_crc != AD7124_DISABLE_CRC )
	{
		crc8 = ad7124_compute_crc8( wr_buf, reg.size + 1 );
		pDev->txbuf[ reg.size + 1 ] = crc8;
	}

	uint32_t tickstart = HAL_GetTick();
	DIO.Output( pDev->cs.port, pDev->cs.pin, GPIO_LOW );
	/* Write data to the device */
	HAL_SPI_Transmit( pDev->hspi, pDev->txbuf,
					  ( AD7124.dev[ 0 ].Status.use_crc != AD7124_DISABLE_CRC ) ? reg.size + 2 : reg.size + 1,
					  AD7124_SPI_TIMEOUT );
	while( HAL_SPI_GetState( pDev->hspi ) != HAL_SPI_STATE_READY )
	{
		if( HAL_GetTick() - tickstart > AD7124_SPI_TIMEOUT )
		{
			ret = TIMEOUT;
			break;
		}
	}
	DIO.Output( pDev->cs.port, pDev->cs.pin, GPIO_HIGH );

	return ret;
}

/******************************************************************************
 * @brief Reads the value of the specified register only when the device is ready
 *  to accept user requests. If the device ready flag is deactivated the
 *  read operation will be executed without checking the device state.
 *
 * @param pDev Pointer to the ad7124 device to be read.
 * @param addr Number of register address to be read
 * @return 0 for success or negative error code.
 *****************************************************************************/
static int32_t ad7124_read_register( ad7124_dev_st *pDev, int addr )
{
	int32_t ret;

	if( pDev->regs[ addr ].addr != AD7124_ERR_REG && pDev->Status.check_ready )
	{
		ret = ad7124_wait_for_spi_ready( pDev );
		if( ret < 0 )
			return ret;
	}
	ret = ad7124_no_check_read_register( pDev, addr );

	return ret;
}

/******************************************************************************
 * @brief Writes the value of the specified register only when the device is ready
 *  to accept user requests. If the device ready flag is deactivated the write
 *  operation will be executed without checking the device state.
 *
 * @param pDev Pointer to the ad7124 device to be written.
 * @param addr Number of register address to be written.
 * @return 0 for success or negative error code.
 *
 *****************************************************************************/
static int32_t ad7124_write_register( ad7124_dev_st *pDev, int addr )
{
	int32_t ret;
//    ad7124_reg_st * pReg;

	if( NULL == pDev ) return INVALID_VAL;

//    pReg = &pDev->regs[ addr ];

	if( pDev->Status.check_ready )
	{
		ret = ad7124_wait_for_spi_ready( pDev );
		if( ret < 0 ) return ret;
	}
	ret = ad7124_no_check_write_register( pDev, addr );

	return ret;
}

/******************************************************************************
 * @brief Resets the device.
 *
 * @param pDev Pointer to the ad7124 device to be written.
 * @return 0 for success or negative error code.
 *****************************************************************************/
static int32_t ad7124_reset( ad7124_dev_st *pDev )
{
	int32_t ret = 0;

	for( int i = 0; i < 8; ++i )
	{
		pDev->txbuf[ i ] = 0xFF;
	}

	uint32_t tickstart = HAL_GetTick();

	DIO.Output( pDev->cs.port, pDev->cs.pin, GPIO_LOW );
	HAL_SPI_TransmitReceive( pDev->hspi, pDev->txbuf, pDev->rxbuf, 8, SPI_TIMEOUT );
	while( HAL_SPI_GetState( pDev->hspi ) != HAL_SPI_STATE_READY )
	{
		if( HAL_GetTick() - tickstart > AD7124_SPI_TIMEOUT )
		{
			ret = TIMEOUT;
			break;
		}
	}
	DIO.Output( pDev->cs.port, pDev->cs.pin, GPIO_HIGH );

	/* CRC is disabled after reset */
	pDev->Status.use_crc = AD7124_DISABLE_CRC;

	/* Read POR bit to clear */
	ret = ad7124_wait_to_power_on( pDev, pDev->spi_rdy_poll_cnt );

	/*
	 * Post reset delay required to ensure all internal config done
	 * A time of 2ms should be enough based on the data sheet, but 4ms
	 * chosen to provide enough margin.
	 */
	AppTimer.Delay( AD7124_POST_RESET_DELAY );

	return ret;
}

/******************************************************************************
 * @brief Waits until the device can accept read and write user actions.
 *
 * @param pDev Pointer to the device to wait for spi ready
 * @return 0 for success or negative error code.
 *****************************************************************************/
static int32_t ad7124_wait_for_spi_ready( ad7124_dev_st *pDev )
{
	ad7124_reg_st *regs;
	int32_t ret;
	int8_t ready = 0;
	uint32_t timeout;

	if( NULL == pDev ) return INVALID_VAL;

	regs = pDev->regs;
	timeout = pDev->spi_rdy_poll_cnt;

	while( !ready && --timeout )
	{
		/* Read the value of the Error Register */
		ret = ad7124_read_register( pDev, AD7124_Error );
		if( ret < 0 ) return ret;

		/* Check the SPI IGNORE Error bit in the Error Register */
		ready = ( regs[ AD7124_Error ].value & AD7124_ERR_REG_SPI_IGNORE_ERR ) == 0;
	}

	return timeout ? 0 : TIMEOUT;
}

/******************************************************************************
 * @brief Waits until the device finishes the power-on reset operation.
 *
 * @param pDev Pointer to the device to wait for power on.
 * @param timeout Count representing the number of polls to be done until the function returns.
 * @return 0 for success or negative error code.
 *****************************************************************************/
static int32_t ad7124_wait_to_power_on( ad7124_dev_st *pDev, uint32_t timeout )
{
	ad7124_reg_st *regs;
	int32_t ret;
	int8_t powered_on = 0;

	regs = pDev->regs;

	while( !powered_on && --timeout )
	{
		ret = ad7124_read_register( pDev, AD7124_Status );
		if( ret < 0 ) continue;

		/* Check the POR_FLAG bit in the Status Register */
		powered_on = ( regs[ AD7124_Status ].value & AD7124_STATUS_REG_POR_FLAG ) == 0;
	}

	return ( timeout || powered_on ) ? 0 : TIMEOUT;
}

/******************************************************************************
 * @brief Waits until a new conversion result is available.
 *
 * @param pDev Pointer to the device to wait for power on.
 * @param timeout Count representing the number of polls to be done until the function returns if no new data is available.
 * @return 0 for success or negative error code.
 *****************************************************************************/
static int32_t ad7124_wait_for_conv_ready( ad7124_dev_st *pDev, uint32_t timeout )
{
	ad7124_reg_st *regs;
	int32_t ret;
	int8_t ready = 0;

	if( NULL == pDev ) return INVALID_VAL;

	regs = pDev->regs;

	while( !ready && --timeout )
	{
		/* Read the value of the Status Register */
		ret = ad7124_read_register( pDev, AD7124_Status );
		if( ret < 0 ) return ret;

		extern IWDG_HandleTypeDef hiwdg;
		HAL_IWDG_Refresh(&hiwdg);

		/* Check the RDY bit in the Status Register */
		ready = ( regs[ AD7124_Status ].value & AD7124_STATUS_REG_RDY ) == 0;
	}

	return timeout ? 0 : TIMEOUT;
}

/******************************************************************************
 * @brief Reads the conversion result from the device.
 *
 * @param pDev Pointer to the device to be read data.
 * @param pData Pointer to store the read data.
 * @return 0 for success or negative error code.
 *****************************************************************************/
static int32_t ad7124_read_data( ad7124_dev_st *pDev, int32_t *pData )
{
	ad7124_reg_st *regs;
	int32_t ret;

	if( NULL == pDev || NULL == pData ) return INVALID_VAL;

	regs = pDev->regs;

	/* Read the value of the Status Register */
	ret = ad7124_read_register( pDev, AD7124_Data );

	/* Get the read result */
	*pData = regs[ AD7124_Data ].value;

	return ret;
}

/******************************************************************************
 * @brief Disable channel A/D conversion
 *
 * @param pDev Pointer to the device to stop A/D conversion
 * @return 0 for success or negative error code.
 *****************************************************************************/
static int32_t ad7124_disable_channel( ad7124_dev_st *pDev )
{
	ad7124_reg_st *regs;
	int32_t ret;
	uint8_t channel;

	if( NULL == pDev ) return INVALID_VAL;

	channel = pDev->Status.channel;
	regs = pDev->regs;

	regs[ AD7124_Channel_0 + channel ].value &= ~AD7124_CH_MAP_REG_CH_ENABLE;

	ret = ad7124_write_register( pDev, AD7124_Channel_0 + channel );

	return ret;
}

/******************************************************************************
 * @brief Enable channel A/D conversion
 *
 * @param pDev Pointer to the device to activate a specific channel
 * @return 0 for success or negative error code.
 *****************************************************************************/
static int32_t ad7124_enable_channel( ad7124_dev_st *pDev )
{
	ad7124_reg_st *regs;
	int32_t ret;
	uint8_t channel;

	if( NULL == pDev ) return INVALID_VAL;

	channel = pDev->Status.channel;
	regs = pDev->regs;

	regs[ AD7124_Channel_0 + channel ].value |= AD7124_CH_MAP_REG_CH_ENABLE;

	ret = ad7124_write_register( pDev, AD7124_Channel_0 + channel );

	return ret;
}

/******************************************************************************
 * @brief Computes the CRC checksum for a data buffer.
 *
 * @param p_buf Data buffer to calculate crc8.
 * @param buf_size Data buffer size in bytes.
 * @return Computed CRC checksum.
 *****************************************************************************/
static uint8_t ad7124_compute_crc8( uint8_t *p_buf, uint8_t buf_size )
{
	uint8_t i = 0;
	uint8_t crc = 0;

	while( buf_size )
	{
		for( i = 0x80; i != 0; i >>= 1 )
		{
			bool cmp1 = ( crc & 0x80 ) != 0;
			bool cmp2 = ( *p_buf & i ) != 0;
			if( cmp1 != cmp2 )
			{ /* MSB of CRC register XOR input Bit from Data */
				crc <<= 1;
				crc ^= AD7124_CRC8_POLYNOMIAL_REPRESENTATION;
			}
			else
			{
				crc <<= 1;
			}
		}
		p_buf++;
		buf_size--;
	}
	return crc;
}

/******************************************************************************
 * @brief Updates the CRC settings.
 *
 * @param pDev Pointer to the device to update crc setting
 *****************************************************************************/
static void ad7124_update_crcsetting( ad7124_dev_st *pDev )
{
	if( NULL == pDev ) return;

	/* Get CRC State. */
	if( pDev->regs[ AD7124_Error_En ].value & AD7124_ERREN_REG_SPI_CRC_ERR_EN )
	{
		pDev->Status.use_crc = AD7124_USE_CRC;
	}
	else
	{
		pDev->Status.use_crc = AD7124_DISABLE_CRC;
	}
}

/******************************************************************************
 * @brief Updates the device SPI interface settings.
 *
 * @param pDev Pointer to the device to update the device SPI interface settings.
 *****************************************************************************/
static void ad7124_update_dev_spi_settings( ad7124_dev_st *pDev )
{
	if( NULL == pDev ) return;

	if( pDev->regs[ AD7124_Error_En ].value & AD7124_ERREN_REG_SPI_IGNORE_ERR_EN )
	{
		pDev->Status.check_ready = 1;
	}
	else
	{
		pDev->Status.check_ready = 0;
	}
}

/******************************************************************************
 * @brief ad7124_internalCalib_RTD
 *
 *****************************************************************************/
static void ad7124_internalCalib_RTD( uint8_t ch, uint8_t pga )
{
	if( ch >= MAX_TEMP_CHANNEL ) return;

	ad7124_dev_st *pDev = &AD7124.dev[ ch / MAX_ADC_DEV ];
	ch = ch % MAX_ADC_CH;

	ad7124_reset( pDev );

	pDev->regs[ AD7124_ADC_Control ].value = AD7124_ADC_CTRL_REG_CS_EN
										| AD7124_ADC_CTRL_REG_REF_EN
										| AD7124_ADC_CTRL_REG_DATA_STATUS
										| AD7124_ADC_CTRL_REG_POWER_MODE( 1 )   	// 0:low, 1:mid, 2:full power
										| AD7124_ADC_CTRL_REG_MODE( 4 ) 			// 0:continuous, 1:single, 2:standby, 3:power-down, 4:idle, 5:offset cal, 6:gain cal
										| AD7124_ADC_CTRL_REG_CLK_SEL( 0 );
	ad7124_write_register( pDev, AD7124_ADC_Control );

	pDev->regs[ AD7124_Error_En ].value = AD7124_ERREN_REG_ADC_CAL_ERR_EN
										| AD7124_ERREN_REG_ADC_CONV_ERR_EN
										| AD7124_ERREN_REG_ADC_SAT_ERR_EN;
	ad7124_write_register( pDev, AD7124_Error_En );

	if( ch == 0 )
	{
		pDev->regs[ AD7124_Channel_1 ].value &= ~AD7124_CH_MAP_REG_CH_ENABLE;
		ad7124_write_register( pDev, AD7124_Channel_1 );

		pDev->regs[ AD7124_IOCon1 ].value = AD7124_IO_CTRL1_REG_IOUT_CH0( 0 )  			// AIN0
										| AD7124_IO_CTRL1_REG_IOUT_CH1( 1 ) 			// AIN1
										| AD7124_IO_CTRL1_REG_IOUT0( 3 )            	// 250uA
										| AD7124_IO_CTRL1_REG_IOUT1( 3 );           	// 250uA
		ad7124_write_register( pDev, AD7124_IOCon1 );

		pDev->regs[ AD7124_Channel_0 ].value = AD7124_CH_MAP_REG_CH_ENABLE
										| AD7124_CH_MAP_REG_SETUP( 0 )
										| AD7124_CH_MAP_REG_AINP( 2 )
										| AD7124_CH_MAP_REG_AINM( 3 );
		ad7124_write_register( pDev, AD7124_Channel_0 );

		pDev->regs[ AD7124_Config_0 ].value = AD7124_CFG_REG_PGA( pga )
										| AD7124_CFG_REG_REF_BUFP
										| AD7124_CFG_REG_REF_BUFM
										| AD7124_CFG_REG_AIN_BUFP
										| AD7124_CFG_REG_AIN_BUFM
										| AD7124_CFG_REG_REF_SEL( 0 );
		ad7124_write_register( pDev, AD7124_Config_0 );

		pDev->regs[ AD7124_Offset_0 ].value = 0x800000;
		ad7124_write_register( pDev, AD7124_Offset_0 );

		pDev->regs[ AD7124_Filter_0 ].value = AD7124_FILT_REG_FS( AD7124_INTERNALCALIB_FS );
		ad7124_write_register( pDev, AD7124_Filter_0 );
	}
	else if( ch == 1 )
	{
		pDev->regs[ AD7124_Channel_0 ].value &= ~AD7124_CH_MAP_REG_CH_ENABLE;
		ad7124_write_register( pDev, AD7124_Channel_0 );

		pDev->regs[ AD7124_IOCon1 ].value = AD7124_IO_CTRL1_REG_IOUT_CH0( 10 )	// AIN4
										| AD7124_IO_CTRL1_REG_IOUT_CH1( 11 )	// AIN5
										| AD7124_IO_CTRL1_REG_IOUT0( 3 )		// 250uA
										| AD7124_IO_CTRL1_REG_IOUT1( 3 );		// 250uA
		ad7124_write_register( pDev, AD7124_IOCon1 );

		pDev->regs[ AD7124_Channel_1 ].value = AD7124_CH_MAP_REG_CH_ENABLE
										| AD7124_CH_MAP_REG_SETUP( 1 )
										| AD7124_CH_MAP_REG_AINP( 6 )
										| AD7124_CH_MAP_REG_AINM( 7 );
		ad7124_write_register( pDev, AD7124_Channel_1 );

		pDev->regs[ AD7124_Config_1 ].value = AD7124_CFG_REG_PGA( pga )
										| AD7124_CFG_REG_REF_BUFP
										| AD7124_CFG_REG_REF_BUFM
										| AD7124_CFG_REG_AIN_BUFP
										| AD7124_CFG_REG_AIN_BUFM
										| AD7124_CFG_REG_REF_SEL( 0 );
		ad7124_write_register( pDev, AD7124_Config_1 );

		pDev->regs[ AD7124_Filter_1 ].value = AD7124_FILT_REG_FS( AD7124_INTERNALCALIB_FS );
		ad7124_write_register( pDev, AD7124_Filter_1 );

		pDev->regs[ AD7124_Offset_1 ].value = 0x800000;
		ad7124_write_register( pDev, AD7124_Offset_1 );
	}

	pDev->regs[ AD7124_ADC_Control ].value = AD7124_ADC_CTRL_REG_CS_EN
										| AD7124_ADC_CTRL_REG_REF_EN
										| AD7124_ADC_CTRL_REG_DATA_STATUS
										| AD7124_ADC_CTRL_REG_POWER_MODE( 1 )		// 0:low, 1:mid, 2:full power
										| AD7124_ADC_CTRL_REG_MODE( 6 ) 			// 0:continuous, 1:single, 2:standby, 3:power-down, 4:idle, 5:offset cal, 6:gain cal
										| AD7124_ADC_CTRL_REG_CLK_SEL( 0 );
	ad7124_write_register( pDev, AD7124_ADC_Control );
	ad7124_wait_for_conv_ready( pDev, 10000000UL );
	
	pDev->regs[ AD7124_ADC_Control ].value = AD7124_ADC_CTRL_REG_CS_EN
										| AD7124_ADC_CTRL_REG_REF_EN
										| AD7124_ADC_CTRL_REG_DATA_STATUS
										| AD7124_ADC_CTRL_REG_POWER_MODE( 1 )     	// 0:low, 1:mid, 2:full power
										| AD7124_ADC_CTRL_REG_MODE( 5 ) 			// 0:continuous, 1:single, 2:standby, 3:power-down, 4:idle, 5:offset cal, 6:gain cal
										| AD7124_ADC_CTRL_REG_CLK_SEL( 0 );
	ad7124_write_register( pDev, AD7124_ADC_Control );
	ad7124_wait_for_conv_ready( pDev, 10000000UL );

	ad7124_read_register( pDev, AD7124_Error );

	pDev->regs[ AD7124_ADC_Control ].value = AD7124_ADC_CTRL_REG_CS_EN
	                                    | AD7124_ADC_CTRL_REG_REF_EN
	                                    | AD7124_ADC_CTRL_REG_DATA_STATUS
	                                    | AD7124_ADC_CTRL_REG_POWER_MODE( 2 )     // 0:low, 1:mid, 2:full power
	                                    | AD7124_ADC_CTRL_REG_MODE( 0 ) 		 // 0:continuous, 1:single, 2:standby, 3:power-down, 4:idle, 5:offset cal, 6:gain cal
	                                    | AD7124_ADC_CTRL_REG_CLK_SEL( 0 );
	ad7124_write_register( pDev, AD7124_ADC_Control );
}

/******************************************************************************
 * @brief ad7124_internalCalib_TC
 *
 *****************************************************************************/
static void ad7124_internalCalib_TC( uint8_t ch, uint8_t pga )
{
	if( ch >= MAX_TEMP_CHANNEL ) return;

	ad7124_dev_st *pDev = &AD7124.dev[ ch / MAX_ADC_DEV ];
	ch = ch % MAX_ADC_CH;

	ad7124_reset( pDev );

	pDev->regs[ AD7124_ADC_Control ].value = AD7124_ADC_CTRL_REG_CS_EN
										| AD7124_ADC_CTRL_REG_REF_EN
										| AD7124_ADC_CTRL_REG_DATA_STATUS
										| AD7124_ADC_CTRL_REG_POWER_MODE( 1 )   	// 0:low, 1:mid, 2:full power
										| AD7124_ADC_CTRL_REG_MODE( 4 ) 			// 0:continuous, 1:single, 2:standby, 3:power-down, 4:idle, 5:offset cal, 6:gain cal
										| AD7124_ADC_CTRL_REG_CLK_SEL( 0 );
	ad7124_write_register( pDev, AD7124_ADC_Control );

	pDev->regs[ AD7124_Error_En ].value = AD7124_ERREN_REG_ADC_CAL_ERR_EN
										| AD7124_ERREN_REG_ADC_CONV_ERR_EN
										| AD7124_ERREN_REG_ADC_SAT_ERR_EN;
	ad7124_write_register( pDev, AD7124_Error_En );

	pDev->regs[ AD7124_IOCon1 ].value = 0;					// EXCITAION CURRENT ALL OFF
	ad7124_write_register( pDev, AD7124_IOCon1 );

	pDev->regs[ AD7124_IOCon2 ].value = AD7124_IO_CTRL2_REG_GPIO_VBIAS7 | AD7124_IO_CTRL2_REG_GPIO_VBIAS3;
	ad7124_write_register( pDev, AD7124_IOCon2 );

	if( ch == 0 )
	{
		pDev->regs[ AD7124_Channel_1 ].value &= ~AD7124_CH_MAP_REG_CH_ENABLE;
		ad7124_write_register( pDev, AD7124_Channel_1 );

		pDev->regs[ AD7124_Channel_0 ].value = AD7124_CH_MAP_REG_CH_ENABLE
										| AD7124_CH_MAP_REG_SETUP( 0 )
										| AD7124_CH_MAP_REG_AINP( 2 )
										| AD7124_CH_MAP_REG_AINM( 3 );
		ad7124_write_register( pDev, AD7124_Channel_0 );

		pDev->regs[ AD7124_Config_0 ].value = AD7124_CFG_REG_PGA( AD7124_PGA_TC )
									| AD7124_CFG_REG_AIN_BUFP
									| AD7124_CFG_REG_AIN_BUFM
									| AD7124_CFG_REG_REF_SEL( 2 )
									| AD7124_CFG_REG_BIPOLAR;			
		ad7124_write_register( pDev, AD7124_Config_0 );

		pDev->regs[ AD7124_Offset_0 ].value = 0x800000;
		ad7124_write_register( pDev, AD7124_Offset_0 );

		pDev->regs[ AD7124_Filter_0 ].value = AD7124_FILT_REG_FS( AD7124_INTERNALCALIB_FS );
		ad7124_write_register( pDev, AD7124_Filter_0 );
	}
	else if( ch == 1 )
	{
		pDev->regs[ AD7124_Channel_0 ].value &= ~AD7124_CH_MAP_REG_CH_ENABLE;
		ad7124_write_register( pDev, AD7124_Channel_0 );

		pDev->regs[ AD7124_Channel_1 ].value = AD7124_CH_MAP_REG_CH_ENABLE
										| AD7124_CH_MAP_REG_SETUP( 1 )
										| AD7124_CH_MAP_REG_AINP( 6 )
										| AD7124_CH_MAP_REG_AINM( 7 );
		ad7124_write_register( pDev, AD7124_Channel_1 );

		pDev->regs[ AD7124_Config_1 ].value = AD7124_CFG_REG_PGA( AD7124_PGA_TC )
										| AD7124_CFG_REG_AIN_BUFP
										| AD7124_CFG_REG_AIN_BUFM
										| AD7124_CFG_REG_REF_SEL( 2 )
										| AD7124_CFG_REG_BIPOLAR;
		ad7124_write_register( pDev, AD7124_Config_1 );

		pDev->regs[ AD7124_Offset_1 ].value = 0x800000;
		ad7124_write_register( pDev, AD7124_Offset_1 );

		pDev->regs[ AD7124_Filter_1 ].value = AD7124_FILT_REG_FS( AD7124_INTERNALCALIB_FS );
		ad7124_write_register( pDev, AD7124_Filter_1 );
	}

	pDev->regs[ AD7124_ADC_Control ].value = AD7124_ADC_CTRL_REG_CS_EN
										| AD7124_ADC_CTRL_REG_REF_EN
										| AD7124_ADC_CTRL_REG_DATA_STATUS
										| AD7124_ADC_CTRL_REG_POWER_MODE( 1 )		// 0:low, 1:mid, 2:full power
										| AD7124_ADC_CTRL_REG_MODE( 6 ) 			// 0:continuous, 1:single, 2:standby, 3:power-down, 4:idle, 5:offset cal, 6:gain cal
										| AD7124_ADC_CTRL_REG_CLK_SEL( 0 );
	ad7124_write_register( pDev, AD7124_ADC_Control );
	ad7124_wait_for_conv_ready( pDev, 10000000UL );
	
	pDev->regs[ AD7124_ADC_Control ].value = AD7124_ADC_CTRL_REG_CS_EN
										| AD7124_ADC_CTRL_REG_REF_EN
										| AD7124_ADC_CTRL_REG_DATA_STATUS
										| AD7124_ADC_CTRL_REG_POWER_MODE( 1 )     	// 0:low, 1:mid, 2:full power
										| AD7124_ADC_CTRL_REG_MODE( 5 ) 			// 0:continuous, 1:single, 2:standby, 3:power-down, 4:idle, 5:offset cal, 6:gain cal
										| AD7124_ADC_CTRL_REG_CLK_SEL( 0 );
	ad7124_write_register( pDev, AD7124_ADC_Control );
	ad7124_wait_for_conv_ready( pDev, 10000000UL );

	ad7124_read_register( pDev, AD7124_Error );

	pDev->regs[ AD7124_ADC_Control ].value = AD7124_ADC_CTRL_REG_CS_EN
	                                    | AD7124_ADC_CTRL_REG_REF_EN
	                                    | AD7124_ADC_CTRL_REG_DATA_STATUS
	                                    | AD7124_ADC_CTRL_REG_POWER_MODE( 2 )     // 0:low, 1:mid, 2:full power
	                                    | AD7124_ADC_CTRL_REG_MODE( 0 ) 		 // 0:continuous, 1:single, 2:standby, 3:power-down, 4:idle, 5:offset cal, 6:gain cal
	                                    | AD7124_ADC_CTRL_REG_CLK_SEL( 0 );
	ad7124_write_register( pDev, AD7124_ADC_Control );
}

/******************************************************************************
 * @brief Initializes the AD7124.
 *
 * @param pDev Pointer to the device to initialize
 * @return 0 for success or negative error code.
 *****************************************************************************/
static int32_t ad7124_setup( ad7124_dev_st *pDev )
{
	int32_t ret;
	ad7124_registers_et reg_nr;

	/* Initialize the SPI communication. */

	/*  Reset the device interface.*/
	ret = ad7124_reset( pDev );
	if( ret < 0 )
	{
		return ret;
	}

	/* Update the device structure with power-on/reset settings */
	pDev->Status.check_ready = 1;

	pDev->regs[ AD7124_ADC_Control ].value |= AD7124_ADC_CTRL_REG_CS_EN
	                                          | AD7124_ADC_CTRL_REG_REF_EN
	                                          | AD7124_ADC_CTRL_REG_DATA_STATUS
	                                          | AD7124_ADC_CTRL_REG_POWER_MODE( 2 )     // 0:low, 1:mid, 2:full power
	                                          | AD7124_ADC_CTRL_REG_MODE( 0 ) // 0:continuous, 1:single, 2:standby, 3:power-down, 4:idle, 5:offset cal, 6:gain cal
	                                          | AD7124_ADC_CTRL_REG_CLK_SEL( 0 );

	pDev->regs[ AD7124_Error_En ].value = AD7124_ERREN_REG_SPI_IGNORE_ERR_EN
											| AD7124_ERREN_REG_REF_DET_ERR_EN
											| AD7124_ERREN_REG_AINM_OV_ERR_EN
											| AD7124_ERREN_REG_AINM_UV_ERR_EN
											| AD7124_ERREN_REG_AINP_OV_ERR_EN
											| AD7124_ERREN_REG_AINP_UV_ERR_EN
											| AD7124_ERREN_REG_ADC_SAT_ERR_EN
											| AD7124_ERREN_REG_ADC_CONV_ERR_EN
											| AD7124_ERREN_REG_ADC_CAL_ERR_EN;

	/* Initialize registers AD7124_ADC_Control through AD7124_Filter_7. */
	for( reg_nr = AD7124_Status; ( reg_nr < AD7124_Offset_0 ) && !( ret < 0 ); reg_nr++ )
	{
		if( pDev->regs[ reg_nr ].rw == AD7124_RW )
		{
			ret = ad7124_write_register( pDev, reg_nr );
			if( ret < 0 ) break;
		}

		/* Get CRC State and device SPI interface settings */
		if( reg_nr == AD7124_Error_En )
		{
			ad7124_update_crcsetting( pDev );
			ad7124_update_dev_spi_settings( pDev );
		}
	}

	return ret;
}

/******************************************************************************
 * @brief
 *
 * @param a
 * @param b
 * @return
 *****************************************************************************/
static int compare_int32( const void *a, const void *b )
{
	return ( *( int* )a - *( int* )b );
}

/******************************************************************************
 * @brief
 *
 * @param buff
 * @param cnt
 * @return
 *****************************************************************************/
static int32_t median_filter( int32_t *buff, int cnt )
{
	int32_t res = 0;

	if( buff )
	{
		if( cnt > 1 )
		{
			int32_t copy_raw[ MEDIAN_FILTER_SZ ];

			memcpy( copy_raw, buff, sizeof(int32_t) * cnt );
			qsort( copy_raw, cnt, sizeof(int32_t), compare_int32 );
			if( cnt & 1 )  // 홀수 ?
				res = copy_raw[ cnt >> 1 ]; /*  정렬된 값의 중간값을 반환함.    */
			else
				// 짝수?
				res = ( ( copy_raw[ ( cnt >> 1 ) - 1 ] + copy_raw[ cnt >> 1 ] ) >> 1 ); /*  가운데 2개 합의 1/2한 값을 반환    */
		}
		else if( cnt == 1 )
		{
			res = buff[ 0 ];
		}
	}
	return res;
}
/******************************************************************************
 * @brief ad7124_sensor_setting
 *
 *****************************************************************************/
void ad7124_sensor_setting( ad7124_dev_st *pDev )
{
	uint8_t ch = pDev->Status.channel;
	uint16_t filter_SPS = pDev->SamplePeriod[ ch ] * FCLK_FULL_POWER_KHz / ( 32 * SINC4_SETTILING_TIMES );

	switch( pDev->type[ ch ] )
	{
		default:
		case SEN_RTD2X:
			ad7124_rtd_setting( pDev, AD7124_PGA_2X_RTD, filter_SPS );
			break;
		case SEN_RTD:
			ad7124_rtd_setting( pDev, AD7124_PGA_RTD, filter_SPS );
			break;
		case SEN_TC_K:
		case SEN_TC_J:
		case SEN_TC_E:
		case SEN_TC_S:
		case SEN_TC_T:
		case SEN_TC_R:
			ad7124_tc_setting( pDev, filter_SPS );
			break;
		case SEN_COMM:
			return;
	}
}
/******************************************************************************
 * @brief Initialize MCU GPIO for AD7124 and AD7124 device
 *
 *****************************************************************************/
void AD7124Init( void )
{
	uint32_t timeout = 1000;

	for( int ic = 0; ic < MAX_ADC_DEV; ic++ )
	{
		ad7124_dev_st *pDev = &AD7124.dev[ ic ];
		ad7124_reg_st *regs = pDev->regs;

		/* AD7124 wakeup */
		ad7124_wait_to_power_on( pDev, 1000 );

		do
		{
			timeout--;
			while( ad7124_no_check_read_register( pDev, AD7124_ID ) < 0 )
			{
				__NOP( );
			}
		} while( regs[ AD7124_ID ].value != 0xff && ( regs[ AD7124_ID ].value & 0x04 ) != 0x04 && timeout );

		if( 0 == timeout )
		{
			return;
		}

		ad7124_setup( pDev );
	}
}

/******************************************************************************
 * @brief AD7124 Task
 *
 * @note  It must be called by main-loop.
 *
 *****************************************************************************/
void AD7124Task( void )
{
	static uint32_t timeout = 0;
	static enum
	{
		SETUP_ADC_S, CONFIG_ADC_S, TRIGGER_S, WAIT_FOR_DRDY_S, ADC_AQUI_S, ADC_CHANNEL_ENABLE_S, WAIT_IEXC_S
	} state = SETUP_ADC_S;
	ad7124_dev_st *pDev0 = &AD7124.dev[ 0 ];
	ad7124_dev_st *pDev1 = &AD7124.dev[ 1 ];
	switch( state )
	{
		case SETUP_ADC_S:
			//ad7124_setup( NULL );
			state = CONFIG_ADC_S;
			break;
		case CONFIG_ADC_S:
			TICK_UP( pDev0->Status.channel, MAX_ADC_CH );
			TICK_UP( pDev1->Status.channel, MAX_ADC_CH );

			ad7124_sensor_setting( pDev0 );
			ad7124_sensor_setting( pDev1 );
			ad7124_enable_channel( pDev0 );
			ad7124_enable_channel( pDev1 );

			state = ADC_CHANNEL_ENABLE_S;
			break;

		case ADC_CHANNEL_ENABLE_S:
			state = WAIT_IEXC_S;

			AppTimer.Start( &timerWaitIexc, EXCITATION_WAIT_TIME );
			break;

		case WAIT_IEXC_S:
			if( AppTimer.IsExpired( &timerWaitIexc ) )
			{
				state = TRIGGER_S;
			}
			break;

		case TRIGGER_S:
			/* SYNC PIN TRIGGERING */
			DIO.Output( ADC_NSYNC, GPIO_LOW );
			/* delay 24/mclk(614.4kHz) at least */
			AppTimer.Delay( 0.0002f );
			DIO.Output( ADC_NSYNC, GPIO_HIGH );

			state = WAIT_FOR_DRDY_S;
			timeout = pDev0->spi_rdy_poll_cnt;

			AppTimer.Start( &timerWaitDrdy, MINIMUM_WAIT_TIME_FOR_DRDY );

			break;

		case WAIT_FOR_DRDY_S:
			if( !AppTimer.IsExpired( &timerWaitDrdy ) ) return;

			timeout--;
			if( timeout == 0 )
			{
				state = ADC_CHANNEL_ENABLE_S;
			}

#if 1
			/* Read the value of the Status Register */
			int32_t ret = ad7124_read_register( pDev0, AD7124_Status );
			if( ret < 0 ) return;
			ret = ad7124_read_register( pDev1, AD7124_Status );
			if( ret < 0 ) return;

//			ad7124_read_register( pDev0, AD7124_Error );
//			ad7124_read_register( pDev1, AD7124_Error );
//			if( pDev0->regs[ AD7124_Status ].value & AD7124_STATUS_REG_ERROR_FLAG )
//			{
//			}
//			if( pDev1->regs[ AD7124_Status ].value & AD7124_STATUS_REG_ERROR_FLAG )
//			{
//			}

			/* Check the RDY bit in the Status Register */
			int32_t dev0_drdy = pDev0->regs[ AD7124_Status ].value & AD7124_STATUS_REG_RDY;
			int32_t dev1_drdy = pDev1->regs[ AD7124_Status ].value & AD7124_STATUS_REG_RDY;
			if( dev0_drdy == 0 && dev1_drdy == 0 )
#else
			if( DIO.Read( ADC_MISO ) == false )
#endif
			{
				state = ADC_AQUI_S;
			}
			break;

		case ADC_AQUI_S:
			ad7124_read_data( pDev0, &pDev0->RawValue[ pDev0->Status.channel ] );
			ad7124_read_data( pDev1, &pDev1->RawValue[ pDev1->Status.channel ] );

			ad7124_disable_channel( pDev0 );
			ad7124_disable_channel( pDev1 );

			for( int dev = 0; dev < MAX_ADC_DEV; dev++ )
			{
				ad7124_dev_st *pDev = &AD7124.dev[ dev ];
				uint8_t ach = pDev->Status.channel;
				int32_t new_raw = pDev->RawValue[ ach ];

				if( pDev->mf.valid_count[ ach ] < MEDIAN_FILTER_SZ )
				{
					for( int i = 0; i < MEDIAN_FILTER_SZ; i++ )
						pDev->mf.buf[ ach ][ i ] = new_raw;
					pDev->mf.valid_count[ ach ] = MEDIAN_FILTER_SZ;
				}
				else
				{
					pDev->mf.buf[ ach ][ pDev->mf.idx ] = new_raw;
				}
				pDev->Value[ ach ] = median_filter( pDev->mf.buf[ ach ], MEDIAN_FILTER_SZ );

				pDev->Status.aquired = 1;

				if( pDev->Status.channel == MAX_ADC_CH - 1 )
				{
					pDev->Status.cycled = 1;
					TICK_UP( pDev->mf.idx, MEDIAN_FILTER_SZ );
				}
			}

			state = CONFIG_ADC_S;
			break;
	}
}
