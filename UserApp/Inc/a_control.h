/******************************************************************************
 * @file a_control.h
 * @author jylee1 (jylee1@gst-in.com)
 * @brief 
 * @version 0.1
 * @date 2023-05-31
 * 
 * @copyright Copyright (c) 2023 Global Standard Technology Co., Ltd.
 * 
 * All rights reserved.
 * This file is part of closed source software.
 * Unauthorized reproduction and redistribution prohibited.
 *****************************************************************************/

#ifndef _A_CONTROL_H_
#define _A_CONTROL_H_

#ifdef __cplusplus
extern "C" {
#endif

/*****************************************************************************/
/** INCLUDES *****************************************************************/
/*****************************************************************************/

/*****************************************************************************/
/** MACRO DEFINITIONS ********************************************************/
/*****************************************************************************/

#define MAX_CONTROL_LOOP 20

#define MIN_CONTROL_PERIOD 50

/*****************************************************************************/
/** DATATYPES ****************************************************************/
/*****************************************************************************/

typedef union system_fault_status_union_Tag
{
	struct {
		uint16_t McuTempHigh : 1;
	};
	uint16_t All;
} global_fault_status_ut;

typedef union fault_status_union_Tag
{
	struct {
		uint16_t HighTemp : 1;
		uint16_t LowTemp : 1;
		uint16_t ControlTimeOver :  1;
		uint16_t SmpsCommError : 1;
		uint16_t SVError : 1;
		uint16_t ConfigError : 1;
		uint16_t AutotuneError : 1;
		uint16_t OverCurrAlarm : 1;
	};
	uint16_t All;
} loop_fault_status_ut;

typedef struct controller_struct_Tag
{
	int32_t (*GetPV)( int loop_idx );       // @retval -273150 ~ 2000000 (unit 0.001ºC)
	float (*GetMV)( int loop_idx );       // @retval -1.0f ~ 1.0f
	loop_fault_status_ut (*GetFault)( int loop_idx );
	void (*SetFault)( int loop_idx, loop_fault_status_ut flt );
	void (*SetConfig)( uint16_t data[], int loop_idx );
	const control_loop_config_st * const (*GetConfig)( int loop_idx );
	void (*ClearAlarm)( uint16_t clear_bitmask_hi, uint16_t clear_bitmask_lo ); // @param clear_bitmask_hi Bitmask to clear fault status of control loop 11th to 20th @param clear_bitmask_lo Bitmask to clear fault status of control loop 1st to 10th
	void (*SetDefault)( int32_t loop_idx );
} controller_st;

/*****************************************************************************/
/** FUNCTION DECLARATIONS ****************************************************/
/*****************************************************************************/

void ControllerTaskInit( void );
void ControllerTask( void ) __attribute__((long_call, section(".RamFunc")));    // for fast RamFunc

/*****************************************************************************/
/** EXTERNAL VARIABLES *******************************************************/
/*****************************************************************************/

extern const controller_st Controller;

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

#endif /* _A_CONTROL_H_ */
