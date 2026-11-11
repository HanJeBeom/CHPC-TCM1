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

/* Control types the PLC may select. */
#define CONTROL_TYPE_IS_VALID( type )	( ( type ) <= CTRL_BYPASS )

/* Control types that need PB, Ti or Td from the PLC. CTRL_PI_ADAPTIVE decides its own gains. */
#define CONTROL_TYPE_USES_PLC_GAIN( type )	( ( ( type ) < CTRL_ONOFF ) && ( ( type ) != CTRL_PI_ADAPTIVE ) )

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

/* controller_diag_st.LimitFlags */
#define CTRL_LIMIT_MV_AT_MAX		( 1U << 0 )		// MV is at OutputMax
#define CTRL_LIMIT_MV_AT_MIN		( 1U << 1 )		// MV is at OutputMin
#define CTRL_LIMIT_SMPS_VOLTAGE		( 1U << 2 )		// SMPS output voltage is at or above the estimator freeze level
#define CTRL_LIMIT_MV_OVERRIDDEN	( 1U << 3 )		// MV was overwritten after the last PID calculation

/* Values actually used by the running controller, for logging. */
typedef struct controller_diag_Tag
{
	float AppliedKp;     // Kp used by the last calculation (per-unit MV / degC)
	float AppliedTi;     // Ti used by the last calculation (sec)
	float EstGain;       // CTRL_PI_ADAPTIVE: estimated plant gain (degC / per-unit MV)
	float EstTau;        // CTRL_PI_ADAPTIVE: estimated plant time constant (sec)
	uint16_t EstState;   // CTRL_PI_ADAPTIVE: last estimator result ( adaptive_state_et )

	float SvRef;         // Setpoint used by the last calculation (degC)
	float PTerm;         // Proportional term of the last calculation (per-unit MV)
	float ITerm;         // Integral term of the last calculation (per-unit MV)
	float DTerm;         // Derivative term of the last calculation (per-unit MV)
	float MvCalc;        // MV of the last calculation before the limit (per-unit)
	uint16_t LimitFlags; // CTRL_LIMIT_*

	uint32_t ControlCalcSeq;       // PID calculations since boot
	uint32_t MvOverrideCount;      // MV overwrites by start or output delay handling since boot
	uint32_t MvSignChangeCount;    // MV sign changes since boot
	uint32_t EnableSeq;            // Control starts since boot
} controller_diag_st;

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
	void (*GetDiag)( int loop_idx, controller_diag_st *diag );
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
