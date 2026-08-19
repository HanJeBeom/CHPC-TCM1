/******************************************************************************
 * @file d_eeprom.h
 * @author jylee1 (jylee1@gst-in.com)
 * @brief 
 * @version 0.1
 * @date 2020-08-05
 * 
 * @copyright Copyright (c) 2020 Global Standard Technology Co., Ltd.
 * 
 * All rights reserved.
 * This file is part of closed source software.
 * Unauthorized reproduction and redistribution prohibited.
 *****************************************************************************/

#ifndef _D_EEPROM_H_
#define _D_EEPROM_H_

#ifdef __cplusplus
extern "C" {
#endif

/*****************************************************************************/
/** INCLUDES *****************************************************************/
/*****************************************************************************/
#include <stdint.h>

/*****************************************************************************/
/** MACRO DEFINITIONS ********************************************************/
/*****************************************************************************/

#define INVALID_CH -1

#define EEPR_CONFIG_ADDR 0
#define EEPR_CONFIG_SZ 4096

#define EEPR_MAX_SZ 8192

#define CFG_MCU 0xF405
#define CFG_REVISION 0x0005
#define CFG_BUILD_DATE BUILD_VERSION_BCD

/*****************************************************************************/
/** DATATYPES ****************************************************************/
/*****************************************************************************/


typedef enum __packed output_type_enum_Tag
{
	OUT_NONE = 0,
	OUT_V_P0_P5,
	OUT_V_P0_P10,
	OUT_V_M5_P5,
	OUT_V_M10_P10,
	OUT_A_4M_20M,
	OUT_A_0M_20M,
	OUT_A_0M_24M,
	OUT_PWM,
	OUT_SMPS_NHPP_6921,
	OUT_SMPS_NHPP_7325,
	OUT_SMPS_CHPP_5521,
	OUT_SMPS_CHPP_8021,
	OUT_SMPS_NHPP_2032,
	OUT_SMPS_NHPP_1531,

	output_et_max = 0xffff
} output_et;

typedef enum __packed control_type_enum_Tag
{
	CTRL_2DOF_PID = 0,
	CTRL_PID,
	CTRL_ONOFF,
	CTRL_BYPASS,
	CTRL_SLIDING,
	CTRL_NN,
	CTRL_FULL_TEST,
	CTRL_HALF_TEST,

	control_et_max = 0xffff
} control_et;

typedef enum __packed cool_heat_mode_enum_Tag
{
	COOL_CTRL_MODE,
	HEAT_CTRL_MODE,

	cool_heat_mode_et_max = 0xffff
} cool_heat_mode_et;

typedef union __packed smps_config_union_Tag
{
	struct
	{
		uint16_t reserved0 		:1;
		uint16_t run_stop 		:1;  // 1:run, 0:stop
		uint16_t software_reset :1;
		uint16_t ch1 			:1;
		uint16_t ch2 			:1;
		uint16_t ch3 			:1;
		uint16_t ch4 			:1;
		uint16_t cutoff_threashold_for_channel_fail :2;
	};
	uint16_t all;
} smps_config_ut;

typedef struct __packed control_loop_config_struct_Tag
{
	int32_t SV;                     // default: 0         / Range: -273150 ~ 2000000 (unit 0.001ºC)
	uint16_t Enable;                // default: 0         / Range: 0, 1
	uint16_t SamplePeriod;          // default: 50        / Range: 0 ~ 60000 ( ms )
	uint16_t ControlPeriod;         // default: 50        / Range: 0 ~ 60000 ( ms )
	sensor_et InputType : 16;       // default: RTD2X     / Range: 0 ~ 8
	int16_t InputChannel;           // default: -1        / Range: 0 ~ 19
	output_et OutputType : 16;      // default: N.A       / Range: 0 ~ 9
	int16_t OutputChannel;          // default: -1        / Range: 0 ~ 20
	int16_t HighOverAlarm;          // default: 2000      / Range: 0 ~ 20000 ( * 0.01K )
	int16_t LowUnderAlarm;          // default: 2000      / Range: 0 ~ 20000 ( * -0.01K )
	uint16_t OverCurrAlarm;         // default: 10000     / Range: 1 ~ 10000 ( mA )
	uint16_t ControlTimeOver;       // default: 3600      / Range: 0 ~ 3600 (sec)
	int16_t TempOffset;             // default: 0         / Range: -30000 ~ 30000 ( * 0.001K )
	uint16_t Pb;                    // default: 10000     / Range: 0 ~ 60000 ( 0.1 )
	uint16_t Ti;                    // default: 0         / Range: 0 ~ 60000 ( 100ms )
	uint16_t Td;                    // default: 0         / Range: 0 ~ 60000 ( 10ms )
	uint16_t Saturated_I;           // default: 5000      / Range: 0 ~ 60000 ( * 0.1 )
	int16_t InputFilterCoeff;       // default: 65        / Range: 0 ~ 30000 ( * 100ms )
	int16_t Reserved;               // default: 0         / Range: 
	uint16_t AutoTuneEnabled;       // default: 0         / Range: 0, 1
	int16_t OutputMax;              // default: 1000      / Range: -1000 ~ 1000 ( * 0.1% )
	int16_t OutputMin;              // default: -1000     / Range: -1000 ~ 1000 ( * 0.1% )
	uint16_t PWMFreq;               // default: 10        / Range: 0 ~ 60000 ( 0.1Hz )
	uint16_t OutputDelay;           // default: 0         / Range: 0 ~ 999 (sec)
	control_et ControlType:16;      // default: 2DOF PID  / Range: 2DOF PID, PID, ONOFF
	cool_heat_mode_et CoolHeat:16;  // default: Cool      / Range: COOL : 0, Heat : 1
	uint16_t StartDelay;            // default: 10        / Range: 0 ~ 999 (sec)
	uint16_t SmpsLeakageCurrent;    // default: 100       / Range: 0 ~ 65535 (mA)
	smps_config_ut SmpsConfig;		// default: 0         / Range: 
} control_loop_config_st;

typedef struct CONFIG_SYSTEM_struct_Tag
{
	uint16_t Run;                   // @note 0 [Ready] @note 1 [Run]
	uint16_t FaultRelayNc;          // @note 0 [NO] @note 1 [NC]
	uint16_t MCU;                   // @note 0xF405 [STM32F405RG]
	uint16_t REVISION;              // @note 0x0002 [PLM Revision]
	uint32_t BUILD_DATE;			// @note 0x00231010 [Build Date]
} CONFIG_SYSTEM_st;

#pragma pack(push, 1)
typedef struct EEPR_CONFIG_struct_Tag
{
	CONFIG_SYSTEM_st system;
	control_loop_config_st ch[ 20 ];
} __attribute__((aligned(1), packed)) EEPR_CONFIG_st;
#pragma pack(pop)

typedef union EEPR_Status_union_Tag
{
	struct
	{
		uint32_t initiated :1;
	};
	uint32_t all;
} EEPR_Status_ut;

typedef struct EEPR_struct_Tag
{
	uint8_t (*Write)( uint32_t dstAdd, void *rbuf, uint32_t len );
	uint8_t (*Read)( uint32_t srcAdd, void *rbuf, uint32_t len );
	EEPR_Status_ut *Status;
} EEPR_st;

/*****************************************************************************/
/** FUNCTION DECLARATIONS ****************************************************/
/*****************************************************************************/

void EepromInit( void );
void EepromTask( void );

/*****************************************************************************/
/** LOCAL VARIABLES **********************************************************/
/*****************************************************************************/

/*****************************************************************************/
/** GLOBAL VARIABLES *********************************************************/
/*****************************************************************************/

extern EEPR_CONFIG_st CFG;
extern const EEPR_st EEPR;

/*****************************************************************************/
/** FUNCTION DEFINITIONS *****************************************************/
/*****************************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* _D_EEPROM_H_ */
