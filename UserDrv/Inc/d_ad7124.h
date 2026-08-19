/******************************************************************************
 * @file d_ad7124.h
 * @author jylee1 (jylee1@gst-in.com)
 * @brief 
 * @version 0.1
 * @date 2020-08-13
 * 
 * @copyright Copyright (c) 2020
 * 
 *****************************************************************************/

#ifndef INC_D_AD7124_H_
#define INC_D_AD7124_H_

#ifdef __cplusplus
extern "C" {
#endif

/*****************************************************************************/
/** INCLUDES *****************************************************************/
/*****************************************************************************/

//#include <stdint.h>

/*****************************************************************************/
/** MACRO DEFINITIONS ********************************************************/
/*****************************************************************************/
#define AD7124_BITDEPTH 24
#define AD7124_UNIPOLAR_STEP (5.96046E-8F)      // 1/2^24
#define AD7124_BIPOLAR_STEP (1.19209E-7F)       // 1/2^23

#define AD7124_PGA_TC AD7124_PGA_Gain_8
#define AD7124_PGA_RTD AD7124_PGA_Gain_16
#define AD7124_PGA_2X_RTD AD7124_PGA_Gain_32

#define AD7124_UNCALIB_STATUS					0xaa

//#define RTD_PGA 5  // PGA = log 2 (GAIN 16)
//#define NTC_PGA 0  // PGA = log 2 (GAIN  1)
//#define TC_PGA 6   // PGA = log 2 (GAIN 64)

#define MEDIAN_FILTER_SZ 3

#define MAX_ADC_DEV 2
#define MAX_ADC_CH 2


//#define VADC_VREF_VOLT 2.408F //2.5F

//#define AIN_CH1 0
//#define AIN_CH2 1
//#define AIN_CH3 2
//#define AIN_CH4 3
//#define AIN_CH5 4
//#define AIN_CH6 5
//#define AIN_EXT_CH 6

/******************* Register map and register definitions ********************/

#define AD7124_RW 1   /* Read and Write */
#define AD7124_R  2   /* Read only */
#define AD7124_W  3   /* Write only */

/* AD7124 Register Map */
#define AD7124_COMM_REG      0x00
#define AD7124_STATUS_REG    0x00
#define AD7124_ADC_CTRL_REG  0x01
#define AD7124_DATA_REG      0x02
#define AD7124_IO_CTRL1_REG  0x03
#define AD7124_IO_CTRL2_REG  0x04
#define AD7124_ID_REG        0x05
#define AD7124_ERR_REG       0x06
#define AD7124_ERREN_REG     0x07
#define AD7124_CH0_MAP_REG   0x09
#define AD7124_CH1_MAP_REG   0x0A
#define AD7124_CH2_MAP_REG   0x0B
#define AD7124_CH3_MAP_REG   0x0C
#define AD7124_CH4_MAP_REG   0x0D
#define AD7124_CH5_MAP_REG   0x0E
#define AD7124_CH6_MAP_REG   0x0F
#define AD7124_CH7_MAP_REG   0x10
#define AD7124_CH8_MAP_REG   0x11
#define AD7124_CH9_MAP_REG   0x12
#define AD7124_CH10_MAP_REG  0x13
#define AD7124_CH11_MAP_REG  0x14
#define AD7124_CH12_MAP_REG  0x15
#define AD7124_CH13_MAP_REG  0x16
#define AD7124_CH14_MAP_REG  0x17
#define AD7124_CH15_MAP_REG  0x18
#define AD7124_CFG0_REG      0x19
#define AD7124_CFG1_REG      0x1A
#define AD7124_CFG2_REG      0x1B
#define AD7124_CFG3_REG      0x1C
#define AD7124_CFG4_REG      0x1D
#define AD7124_CFG5_REG      0x1E
#define AD7124_CFG6_REG      0x1F
#define AD7124_CFG7_REG      0x20
#define AD7124_FILT0_REG     0x21
#define AD7124_FILT1_REG     0x22
#define AD7124_FILT2_REG     0x23
#define AD7124_FILT3_REG     0x24
#define AD7124_FILT4_REG     0x25
#define AD7124_FILT5_REG     0x26
#define AD7124_FILT6_REG     0x27
#define AD7124_FILT7_REG     0x28
#define AD7124_OFFS0_REG     0x29
#define AD7124_OFFS1_REG     0x2A
#define AD7124_OFFS2_REG     0x2B
#define AD7124_OFFS3_REG     0x2C
#define AD7124_OFFS4_REG     0x2D
#define AD7124_OFFS5_REG     0x2E
#define AD7124_OFFS6_REG     0x2F
#define AD7124_OFFS7_REG     0x30
#define AD7124_GAIN0_REG     0x31
#define AD7124_GAIN1_REG     0x32
#define AD7124_GAIN2_REG     0x33
#define AD7124_GAIN3_REG     0x34
#define AD7124_GAIN4_REG     0x35
#define AD7124_GAIN5_REG     0x36
#define AD7124_GAIN6_REG     0x37
#define AD7124_GAIN7_REG     0x38

/* Communication Register bits */
#define AD7124_COMM_REG_WEN    (0 << 7)
#define AD7124_COMM_REG_WR     (0 << 6)
#define AD7124_COMM_REG_RD     (1 << 6)
#define AD7124_COMM_REG_RA(x)  ((x) & 0x3F)

/* Status Register bits */
#define AD7124_STATUS_REG_RDY          (1 << 7)
#define AD7124_STATUS_REG_ERROR_FLAG   (1 << 6)
#define AD7124_STATUS_REG_POR_FLAG     (1 << 4)
#define AD7124_STATUS_REG_CH_ACTIVE(x) ((x) & 0xF)

/* ADC_Control Register bits */
#define AD7124_ADC_CTRL_REG_DOUT_RDY_DEL   (1 << 12)
#define AD7124_ADC_CTRL_REG_CONT_READ      (1 << 11)
#define AD7124_ADC_CTRL_REG_DATA_STATUS    (1 << 10)
#define AD7124_ADC_CTRL_REG_CS_EN          (1 << 9)
#define AD7124_ADC_CTRL_REG_REF_EN         (1 << 8)
#define AD7124_ADC_CTRL_REG_POWER_MODE(x)  (((x) & 0x3) << 6)
#define AD7124_ADC_CTRL_REG_MODE(x)        (((x) & 0xF) << 2)
#define AD7124_ADC_CTRL_REG_CLK_SEL(x)     (((x) & 0x3) << 0)

/* IO_Control_1 Register bits */
#define AD7124_IO_CTRL1_REG_GPIO_DAT2     (1 << 23)
#define AD7124_IO_CTRL1_REG_GPIO_DAT1     (1 << 22)
#define AD7124_IO_CTRL1_REG_GPIO_CTRL2    (1 << 19)
#define AD7124_IO_CTRL1_REG_GPIO_CTRL1    (1 << 18)
#define AD7124_IO_CTRL1_REG_PDSW          (1 << 15)
#define AD7124_IO_CTRL1_REG_IOUT1(x)      (((x) & 0x7) << 11)
#define AD7124_IO_CTRL1_REG_IOUT0(x)      (((x) & 0x7) << 8)
#define AD7124_IO_CTRL1_REG_IOUT_CH1(x)   (((x) & 0xF) << 4)
#define AD7124_IO_CTRL1_REG_IOUT_CH0(x)   (((x) & 0xF) << 0)

/* IO_Control_1 AD7124-8 specific bits */
#define AD7124_8_IO_CTRL1_REG_GPIO_DAT4     (1 << 23)
#define AD7124_8_IO_CTRL1_REG_GPIO_DAT3     (1 << 22)
#define AD7124_8_IO_CTRL1_REG_GPIO_DAT2     (1 << 21)
#define AD7124_8_IO_CTRL1_REG_GPIO_DAT1     (1 << 20)
#define AD7124_8_IO_CTRL1_REG_GPIO_CTRL4    (1 << 19)
#define AD7124_8_IO_CTRL1_REG_GPIO_CTRL3    (1 << 18)
#define AD7124_8_IO_CTRL1_REG_GPIO_CTRL2    (1 << 17)
#define AD7124_8_IO_CTRL1_REG_GPIO_CTRL1    (1 << 16)

/* IO_Control_2 Register bits */
#define AD7124_IO_CTRL2_REG_GPIO_VBIAS7   (1 << 15)
#define AD7124_IO_CTRL2_REG_GPIO_VBIAS6   (1 << 14)
#define AD7124_IO_CTRL2_REG_GPIO_VBIAS5   (1 << 11)
#define AD7124_IO_CTRL2_REG_GPIO_VBIAS4   (1 << 10)
#define AD7124_IO_CTRL2_REG_GPIO_VBIAS3   (1 << 5)
#define AD7124_IO_CTRL2_REG_GPIO_VBIAS2   (1 << 4)
#define AD7124_IO_CTRL2_REG_GPIO_VBIAS1   (1 << 1)
#define AD7124_IO_CTRL2_REG_GPIO_VBIAS0   (1 << 0)

/* IO_Control_2 AD7124-8 specific bits */
#define AD7124_8_IO_CTRL2_REG_GPIO_VBIAS15  (1 << 15)
#define AD7124_8_IO_CTRL2_REG_GPIO_VBIAS14  (1 << 14)
#define AD7124_8_IO_CTRL2_REG_GPIO_VBIAS13  (1 << 13)
#define AD7124_8_IO_CTRL2_REG_GPIO_VBIAS12  (1 << 12)
#define AD7124_8_IO_CTRL2_REG_GPIO_VBIAS11  (1 << 11)
#define AD7124_8_IO_CTRL2_REG_GPIO_VBIAS10  (1 << 10)
#define AD7124_8_IO_CTRL2_REG_GPIO_VBIAS9   (1 << 9)
#define AD7124_8_IO_CTRL2_REG_GPIO_VBIAS8   (1 << 8)
#define AD7124_8_IO_CTRL2_REG_GPIO_VBIAS7   (1 << 7)
#define AD7124_8_IO_CTRL2_REG_GPIO_VBIAS6   (1 << 6)
#define AD7124_8_IO_CTRL2_REG_GPIO_VBIAS5   (1 << 5)
#define AD7124_8_IO_CTRL2_REG_GPIO_VBIAS4   (1 << 4)
#define AD7124_8_IO_CTRL2_REG_GPIO_VBIAS3   (1 << 3)
#define AD7124_8_IO_CTRL2_REG_GPIO_VBIAS2   (1 << 2)
#define AD7124_8_IO_CTRL2_REG_GPIO_VBIAS1   (1 << 1)
#define AD7124_8_IO_CTRL2_REG_GPIO_VBIAS0   (1 << 0)

/* ID Register bits */
#define AD7124_ID_REG_DEVICE_ID(x)   (((x) & 0xF) << 4)
#define AD7124_ID_REG_SILICON_REV(x) (((x) & 0xF) << 0)

/* Error Register bits */
#define AD7124_ERR_REG_LDO_CAP_ERR        (1 << 19)
#define AD7124_ERR_REG_ADC_CAL_ERR        (1 << 18)
#define AD7124_ERR_REG_ADC_CONV_ERR       (1 << 17)
#define AD7124_ERR_REG_ADC_SAT_ERR        (1 << 16)
#define AD7124_ERR_REG_AINP_OV_ERR        (1 << 15)
#define AD7124_ERR_REG_AINP_UV_ERR        (1 << 14)
#define AD7124_ERR_REG_AINM_OV_ERR        (1 << 13)
#define AD7124_ERR_REG_AINM_UV_ERR        (1 << 12)
#define AD7124_ERR_REG_REF_DET_ERR        (1 << 11)
#define AD7124_ERR_REG_DLDO_PSM_ERR       (1 << 9)
#define AD7124_ERR_REG_ALDO_PSM_ERR       (1 << 7)
#define AD7124_ERR_REG_SPI_IGNORE_ERR     (1 << 6)
#define AD7124_ERR_REG_SPI_SLCK_CNT_ERR   (1 << 5)
#define AD7124_ERR_REG_SPI_READ_ERR       (1 << 4)
#define AD7124_ERR_REG_SPI_WRITE_ERR      (1 << 3)
#define AD7124_ERR_REG_SPI_CRC_ERR        (1 << 2)
#define AD7124_ERR_REG_MM_CRC_ERR         (1 << 1)
#define AD7124_ERR_REG_ROM_CRC_ERR        (1 << 0)

/* Error_En Register bits */
#define AD7124_ERREN_REG_MCLK_CNT_EN           (1 << 22)
#define AD7124_ERREN_REG_LDO_CAP_CHK_TEST_EN   (1 << 21)
#define AD7124_ERREN_REG_LDO_CAP_CHK(x)        (((x) & 0x3) << 19)
#define AD7124_ERREN_REG_ADC_CAL_ERR_EN        (1 << 18)
#define AD7124_ERREN_REG_ADC_CONV_ERR_EN       (1 << 17)
#define AD7124_ERREN_REG_ADC_SAT_ERR_EN        (1 << 16)
#define AD7124_ERREN_REG_AINP_OV_ERR_EN        (1 << 15)
#define AD7124_ERREN_REG_AINP_UV_ERR_EN        (1 << 14)
#define AD7124_ERREN_REG_AINM_OV_ERR_EN        (1 << 13)
#define AD7124_ERREN_REG_AINM_UV_ERR_EN        (1 << 12)
#define AD7124_ERREN_REG_REF_DET_ERR_EN        (1 << 11)
#define AD7124_ERREN_REG_DLDO_PSM_TRIP_TEST_EN (1 << 10)
#define AD7124_ERREN_REG_DLDO_PSM_ERR_ERR      (1 << 9)
#define AD7124_ERREN_REG_ALDO_PSM_TRIP_TEST_EN (1 << 8)
#define AD7124_ERREN_REG_ALDO_PSM_ERR_EN       (1 << 7)
#define AD7124_ERREN_REG_SPI_IGNORE_ERR_EN     (1 << 6)
#define AD7124_ERREN_REG_SPI_SCLK_CNT_ERR_EN   (1 << 5)
#define AD7124_ERREN_REG_SPI_READ_ERR_EN       (1 << 4)
#define AD7124_ERREN_REG_SPI_WRITE_ERR_EN      (1 << 3)
#define AD7124_ERREN_REG_SPI_CRC_ERR_EN        (1 << 2)
#define AD7124_ERREN_REG_MM_CRC_ERR_EN         (1 << 1)
#define AD7124_ERREN_REG_ROM_CRC_ERR_EN        (1 << 0)

/* Channel Registers 0-15 bits */
#define AD7124_CH_MAP_REG_CH_ENABLE    (1 << 15)
#define AD7124_CH_MAP_REG_SETUP(x)     (((x) & 0x7) << 12)
#define AD7124_CH_MAP_REG_AINP(x)      (((x) & 0x1F) << 5)
#define AD7124_CH_MAP_REG_AINM(x)      (((x) & 0x1F) << 0)

/* Configuration Registers 0-7 bits */
#define AD7124_CFG_REG_BIPOLAR     (1 << 11)
#define AD7124_CFG_REG_BURNOUT(x)  (((x) & 0x3) << 9)
#define AD7124_CFG_REG_REF_BUFP    (1 << 8)
#define AD7124_CFG_REG_REF_BUFM    (1 << 7)
#define AD7124_CFG_REG_AIN_BUFP    (1 << 6)
#define AD7124_CFG_REG_AIN_BUFM    (1 << 5)
#define AD7124_CFG_REG_REF_SEL(x)  ((x) & 0x3) << 3
#define AD7124_CFG_REG_PGA(x)      (((x) & 0x7) << 0)

/* Filter Register 0-7 bits */
#define AD7124_FILT_REG_FILTER(x)         (((x) & 0x7) << 21)
#define AD7124_FILT_REG_REJ60             (1 << 20)
#define AD7124_FILT_REG_POST_FILTER(x)    (((x) & 0x7) << 17)
#define AD7124_FILT_REG_SINGLE_CYCLE      (1 << 16)
#define AD7124_FILT_REG_FS(x)             (((x) & 0x7FF) << 0)


#define AD7124_CRC8_POLYNOMIAL_REPRESENTATION 0x07 /* x8 + x2 + x + 1 */
#define AD7124_DISABLE_CRC 0
#define AD7124_USE_CRC 1


/*****************************************************************************/
/** DATATYPES ****************************************************************/
/*****************************************************************************/
typedef enum Temp_Channel_enum
{
	TEMP_CHANNEL_1,
	TEMP_CHANNEL_2,
	TEMP_CHANNEL_3,
	TEMP_CHANNEL_4,
	MAX_TEMP_CHANNEL,
} Temp_Channel_et;

typedef enum __packed sensor_type_enum_Tag
{
	SEN_NONE = 0xffff,
	SEN_RTD2X = 0,
	SEN_RTD = 1,
	SEN_TC_K = 2,
	SEN_TC_J = 3,
	SEN_TC_E = 4,
	SEN_TC_S = 5,
	SEN_TC_T = 6,
	SEN_TC_R = 7,

	SEN_COMM = 8,

	sensor_et_max = 0xffff
} sensor_et;

/*! Device register info */
typedef struct ad7124_reg_Tag
{
    int32_t addr;
    int32_t value;
    int32_t size;
    int32_t rw;
} ad7124_reg_st;

/*! AD7124 registers list*/
typedef enum ad7124_registers_enum_Tag
{
    AD7124_Status = 0x00,
    AD7124_ADC_Control,
    AD7124_Data,
    AD7124_IOCon1,
    AD7124_IOCon2,
    AD7124_ID,
    AD7124_Error,
    AD7124_Error_En,
    AD7124_Mclk_Count,
    AD7124_Channel_0,
    AD7124_Channel_1,
    AD7124_Channel_2,
    AD7124_Channel_3,
    AD7124_Channel_4,
    AD7124_Channel_5,
    AD7124_Channel_6,
    AD7124_Channel_7,
    AD7124_Channel_8,
    AD7124_Channel_9,
    AD7124_Channel_10,
    AD7124_Channel_11,
    AD7124_Channel_12,
    AD7124_Channel_13,
    AD7124_Channel_14,
    AD7124_Channel_15,
    AD7124_Config_0,
    AD7124_Config_1,
    AD7124_Config_2,
    AD7124_Config_3,
    AD7124_Config_4,
    AD7124_Config_5,
    AD7124_Config_6,
    AD7124_Config_7,
    AD7124_Filter_0,
    AD7124_Filter_1,
    AD7124_Filter_2,
    AD7124_Filter_3,
    AD7124_Filter_4,
    AD7124_Filter_5,
    AD7124_Filter_6,
    AD7124_Filter_7,
    AD7124_Offset_0,
    AD7124_Offset_1,
    AD7124_Offset_2,
    AD7124_Offset_3,
    AD7124_Offset_4,
    AD7124_Offset_5,
    AD7124_Offset_6,
    AD7124_Offset_7,
    AD7124_Gain_0,
    AD7124_Gain_1,
    AD7124_Gain_2,
    AD7124_Gain_3,
    AD7124_Gain_4,
    AD7124_Gain_5,
    AD7124_Gain_6,
    AD7124_Gain_7,
    AD7124_REG_NO
} ad7124_registers_et;

typedef enum ad7124_PGA_gain_enum_Tag
{
    AD7124_PGA_Gain_1,
    AD7124_PGA_Gain_2,
    AD7124_PGA_Gain_4,
    AD7124_PGA_Gain_8,
    AD7124_PGA_Gain_16,
    AD7124_PGA_Gain_32,
    AD7124_PGA_Gain_64,
    AD7124_PGA_Gain_128,
} ad7124_PGA_gain_et;

typedef struct ad7124_init_param_struct_Tag
{
    ad7124_reg_st *regs;
    int16_t spi_rdy_poll_cnt;
} ad7124_init_param_st;

typedef struct median_filter_struct_Tag
{
    int32_t buf[ MAX_ADC_CH ][ MEDIAN_FILTER_SZ ];
    int32_t idx;
    uint8_t valid_count[ MAX_ADC_CH ];
} mf_st;

typedef union ad7124_status_union_Tag
{
    struct
    {
        uint32_t cycled : 1;
        uint32_t channel : 3;
        uint32_t aquired : 1;
        uint32_t use_crc : 1;
        uint32_t check_ready : 1;
    };
    uint32_t all;
} ad7124_status_ut;

typedef struct ad7124_dev_struct_Tag
{
    ad7124_reg_st *regs;
    int16_t spi_rdy_poll_cnt;
    struct
    {
        int port;
        int pin;
    } cs;
    const spi_port_et port;
    SPI_HandleTypeDef *hspi;
    uint8_t rxbuf[ 32 ];
    uint8_t txbuf[ 32 ];
    int32_t RawValue[ MAX_ADC_CH ];
    int32_t Value[ MAX_ADC_CH ];
    mf_st mf;
    ad7124_status_ut Status;
    sensor_et type[ MAX_ADC_CH ];
    uint16_t SamplePeriod[ MAX_ADC_CH ];
} ad7124_dev_st;

typedef struct AD7124_struct_st
{
    ad7124_dev_st *dev;
    void (*SetType)( uint8_t ch, sensor_et type );
    sensor_et (*GetType)( uint8_t ch );
    void (*SetSamplePeriod)( uint8_t ch, uint16_t sampling_period );
    uint16_t (*GetSamplePeriod)( uint8_t ch );
    int32_t (*GetRawValue)( uint8_t ch );
    int32_t (*GetValue)( uint8_t ch );
    void (*ApplyCalibData)( uint8_t ch, uint32_t gain, uint32_t offset );
    void (*CalibrateRTD)( uint8_t ch, uint8_t pga );
    void (*CalibrateTC)( uint8_t ch, uint8_t pga );
    uint32_t ( *GetRegister )( uint8_t dev, uint8_t reg_no );
    float (*GetEffectBit)( float sps, ad7124_PGA_gain_et pga );
} AD7124_st;

/*****************************************************************************/
/** FUNCTION DECLARATIONS ****************************************************/
/*****************************************************************************/
void AD7124Init( void );
void AD7124Task( void );

/*****************************************************************************/
/** LOCAL VARIABLES **********************************************************/
/*****************************************************************************/

/*****************************************************************************/
/** GLOBAL VARIABLES *********************************************************/
/*****************************************************************************/
extern const AD7124_st AD7124;

/*****************************************************************************/
/** FUNCTION DEFINITIONS *****************************************************/
/*****************************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* INC_D_AD7124_H_ */
