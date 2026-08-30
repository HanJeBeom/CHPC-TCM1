/*****************************************************************************
 * @file a_control.c
 * @author jylee1 (jylee1@gst-in.com)
 * @brief 
 * @version 0.1
 * @date 2023-05-31
 * 
 * @copyright Copyright (c) 2023
 * 
 ****************************************************************************/

/*****************************************************************************/
/** INCLUDES *****************************************************************/
/*****************************************************************************/
#include "UserApp.h"

/*****************************************************************************/
/** MACRO DEFINITIONS ********************************************************/
/*****************************************************************************/

// #define MULTIPLY_PERCENT_TO_PERMIL 10.0f
// #define MULTIPLY_PERMIL_TO_PERCENT 0.1f

#define MULTIPLY_K_TO_mK 1000.0f
#define MULTIPLY_mK_TO_K 0.001f

#define MULTIPLY_s_TO_ms 1000.0f
#define MULTIPLY_ms_TO_s 0.001f

#define MULTIPLY_Pb_FLOAT_TO_INT 10.0f
#define MULTIPLY_Pb_INT_TO_FLOAT 0.1f
#define MULTIPLY_Ti_FLOAT_TO_INT 10.0f
#define MULTIPLY_Ti_INT_TO_FLOAT 0.1f
#define MULTIPLY_Td_FLOAT_TO_INT 100.0f
#define MULTIPLY_Td_INT_TO_FLOAT 0.01f

#define MULTIPLY_SatI_FLOAT_TO_INT 1000.0f
#define MULTIPLY_SatI_INT_TO_FLOAT 0.001f

#define ERROR_VALUE_I_TERM_WINDING 40.0f

#define PID_HEATING_NEAR_ENTER_ERROR_K 1.0f
#define PID_HEATING_NEAR_EXIT_ERROR_K 1.3f
#define PID_COOLING_NEAR_ENTER_ERROR_K 1.0f
#define PID_COOLING_NEAR_EXIT_ERROR_K 1.3f
#define PID_HEATING_NEAR_APPROACH_TI_RAW 1200U
#define PID_HEATING_NEAR_HOLD_TI_RAW 2400U
#define PID_GAIN_TRANSITION_BIAS_TAU_S 8.0f
#define PID_COOLING_NEAR_MIN_MV 0.001f
#define PID_COOLING_REVERSE_HEAT_ENTER_K 0.15f
#define PID_COOLING_REVERSE_HEAT_EXIT_K 0.05f
#define PID_COOLING_REVERSE_HEAT_CONFIRM_S 3.0f
#define PID_COOLING_REVERSE_HEAT_MAX_MV 0.003f
#define PID_COOLING_STABLE_SLEW_RATE_PER_S 0.001f
#define PID_COOLING_TARGET_REACHED_ERROR_K 0.0f

/*****************************************************************************/
/** DATATYPES ****************************************************************/
/*****************************************************************************/

typedef struct pid_gain_profile_struct_Tag
{
	uint16_t Pb;
	uint16_t Ti;
	uint16_t Td;
	bool Valid;
} pid_gain_profile_st;

typedef struct control_loop_instance_Tag
{
	uint16_t CH;
	AppTimerData_ut timerPIDPeriod;
	AppTimerData_ut timerOutputDelay;
	AppTimerData_ut timerStartDelay;
	int32_t lastSV;
	int32_t OldSV;
	int32_t PV;     // -273150 ~ 2000000 (unit 0.001ºC)
	float MV;     // -1.0f ~ 1.0f
	loop_fault_status_ut FaultStatus;
	float IntegralSum;
	float lastError;
	float lastPV;
	float lastFilteredSV;
	float oldFilteredSV;
	float filter_alpha;
	int32_t directionSV;
	bool directionInited;
	enum
	{
		PID_DIRECTION_HEATING = 0,
		PID_DIRECTION_COOLING
	} Direction;
	enum
	{
		PID_REGION_FAR = 0,
		PID_REGION_NEAR
	} Region;
	pid_gain_profile_st FarGains;
	pid_gain_profile_st NearGains;
	struct
	{
		float Ku;
		float Pu;
		int32_t Min;    // unit 0.001
		int32_t Max;    // unit 0.001
		enum
		{
			AT_STOPPED,
			AT_START,
			RELAY_ON_FOR_PREPARATION,
			RELAY_OFF_FOR_PREPARATION,
			RELAY_ON_FOR_PREPARE_TO_FIND_DELTA,
			FIND_PV_HIGH,
			FIND_PV_LOW,
			AT_CALCULATION
		} Stage;
		float StartTime;
		float StageTime;
	} Autotune;
} control_loop_instance_st;

typedef struct controller_variable_struct_Tag
{
	control_loop_instance_st Inst[ MAX_CONTROL_LOOP ];
	control_loop_config_st Cfg[ MAX_CONTROL_LOOP ];
	bool Inited;
} controller_var_st;

typedef struct {
	float Kp, Ti, Td, alpha, lambda, Ts;
	float u_min, u_max;

	// Setpoint filter
	float b0, b1, c0, c1;
	float SP_prev, SP_ref_prev;

	// PI integrator
	float I;

	// Filtered PV and derivative state
	float PV_filtered;
	float p0, b;
	float v_prev, PV_prev;
	bool PVFilterInited;

	// Anti-windup gain
	float Kaw;
	float GainTransitionBias;
	float GainTransitionTargetMV;
	bool GainTransitionPending;
	bool CoolingReverseHeatAllowed;
	bool CoolingTargetReached;
	bool CoolingSlewActive;
	float CoolingReverseHeatCandidateTime;
	bool HeatingTargetReached;

	/* Diagnostic mirrors; these fields do not participate in PID calculation. */
	uint16_t EffectivePb, EffectiveTi, EffectiveTd;
	int16_t EffectiveInputFilterCoeff;
	float CalculatedMV, AppliedMV;
} pid_st;


/*****************************************************************************/
/** FUNCTION DECLARATIONS ****************************************************/
/*****************************************************************************/

static void set_default_config( int32_t loop_idx );
static int32_t control_loop_get_pv( int loop_idx );
static float control_loop_get_mv( int loop_idx );
static loop_fault_status_ut control_loop_get_fault( int loop_idx );
static void control_loop_set_fault( int loop_idx, loop_fault_status_ut flt );
static void control_loop_set_config( uint16_t data[ ], int loop_idx );
static const control_loop_config_st* const control_loop_get_config( int loop_idx );
static bool control_loop_get_pid_diagnostic_status( int loop_idx, pid_diagnostic_status_st *status );
static void control_loop_clear_alarm( uint16_t clear_bitmask_hi, uint16_t clear_bitmask_lo );
static __RAM_FUNC int32_t get_temperature( control_loop_config_st *pCfg, int loop_idx );                        // for fast run
static __RAM_FUNC int16_t pid_calculation( control_loop_config_st *pCfg, control_loop_instance_st *pInst );     // for fast run
static __RAM_FUNC int16_t onoff_calculation( control_loop_config_st *pCfg, control_loop_instance_st *pInst );   // for fast run
static void autotune_relay_method( control_loop_config_st *pCfg, control_loop_instance_st *pInst );


static __RAM_FUNC void PID_instance_Init( pid_st *pPid, control_loop_config_st *pCfg );
static __RAM_FUNC void PID_instance_UpdateGains( pid_st *pPid, control_loop_config_st const *pCfg,
	uint16_t effective_pb_raw, uint16_t effective_ti_raw, uint16_t effective_td_raw );
static __RAM_FUNC bool PID_update_operating_state( control_loop_instance_st *pInst, control_loop_config_st const *pCfg );
static __RAM_FUNC void PID_cache_gain_profile( control_loop_instance_st *pInst, control_loop_config_st const *pCfg );
static __RAM_FUNC const pid_gain_profile_st *PID_select_gain_profile( control_loop_instance_st const *pInst );
static __RAM_FUNC float PID2DOF_Calculate_backward_euler( pid_st *pPID, float sp, float pv,
	cool_heat_mode_et cool_heat, bool cooling_near );
static __RAM_FUNC float pid_calculation_ai_tustin( pid_st* pPid, float sp, float pv );
static __RAM_FUNC float pid_calculation_too_simple( control_loop_config_st *pCfg, control_loop_instance_st *pInst );

/*****************************************************************************/
/** GLOBAL VARIABLES *********************************************************/
/*****************************************************************************/

const controller_st Controller =
{
    .GetPV = control_loop_get_pv,
    .GetMV = control_loop_get_mv,
    .GetFault = control_loop_get_fault,
    .SetFault = control_loop_set_fault,
    .SetConfig = control_loop_set_config,
    .GetConfig = control_loop_get_config,
	.GetPidDiagnosticStatus = control_loop_get_pid_diagnostic_status,
    .ClearAlarm = control_loop_clear_alarm,
    .SetDefault = set_default_config,
};

/*****************************************************************************/
/** LOCAL VARIABLES **********************************************************/
/*****************************************************************************/

static CCMRAM pid_st pid_inst[ MAX_CONTROL_LOOP ];

static int loop_idx = 0;
static CCMRAM controller_var_st ControllerVars = { 0 };

/*****************************************************************************/
/** FUNCTION DEFINITIONS *****************************************************/
/*****************************************************************************/

/******************************************************************************
 * @fn void ControllerTaskInit(void)
 * @brief
 *
 *****************************************************************************/
void ControllerTaskInit( void )
{
	memset( &ControllerVars, 0, sizeof( ControllerVars ) );

	for( int loop_idx = 0; loop_idx < MAX_CONTROL_LOOP; ++loop_idx )
	{
		ControllerVars.Inst[ loop_idx ].CH = loop_idx;
	}

	if( EEPR.Status->initiated )
	{
		EEPR.Read( 0, ( uint8_t* ) &CFG, sizeof( CFG ) );
	}

	CFG.system.BUILD_DATE = CFG_BUILD_DATE;

	// config from EEPROM
	if( ( CFG.system.MCU == CFG_MCU ) && ( CFG.system.REVISION == CFG_REVISION ) )
	{
		for( int idx = 0; idx < MAX_CONTROL_LOOP; ++idx )
		{
			memcpy( &ControllerVars.Cfg[ idx ], &CFG.ch[ idx ], sizeof( CFG.ch[ idx ] ) );
			memcpy( &MBSDB.Holdings[ MBS_HR_CH1_SV_L + idx * MBS_HR_CH_SPAN ], &CFG.ch[ idx ], sizeof( CFG.ch[ idx ] ) );
			uint32_t SV = CFG.ch[ idx ].SV;
			MBSDB.Holdings[ MBS_HR_CH1_SV_H + idx * MBS_HR_CH_SPAN ] = SV / 65536;
			MBSDB.Holdings[ MBS_HR_CH1_SV_L + idx * MBS_HR_CH_SPAN ] = SV % 65536;
		}

		MBSDB.Holdings[ MBS_HR_FAULT_NO_NC ] = CFG.system.FaultRelayNc;
		MBSDB.Holdings[	MBS_HR_MCU ] = CFG.system.MCU;
		MBSDB.Holdings[	MBS_HR_REVISION ] = CFG.system.REVISION;
	}
	else
	{
		for( int idx = 0; idx < MAX_CONTROL_LOOP; ++idx )
		{
			set_default_config( idx );
			memcpy( &CFG.ch[ idx ], &ControllerVars.Cfg[ idx ], sizeof( CFG.ch[ idx ] ) );
			memcpy( &MBSDB.Holdings[ MBS_HR_CH1_SV_L + idx * MBS_HR_CH_SPAN ], &CFG.ch[ idx ], sizeof( CFG.ch[ idx ] ) );
		}

		CFG.system.Run = 0;
		CFG.system.FaultRelayNc = 0;

		if( EEPR.Status->initiated )
		{
			CFG.system.MCU = CFG_MCU;
			CFG.system.REVISION = CFG_REVISION;
			CFG.system.BUILD_DATE = CFG_BUILD_DATE;
			EEPR.Write( 0, ( uint8_t* ) &CFG, sizeof( CFG ) );
		}
	}

	ControllerVars.Inited = 1;
}

/******************************************************************************
 * @fn void ControllerTask(void)
 * @brief
 *
 *****************************************************************************/
void ControllerTask( void )
{
	do
	{
		control_loop_config_st *pCfg = &ControllerVars.Cfg[ loop_idx ];
		control_loop_instance_st *pInst = &ControllerVars.Inst[ loop_idx ];

		pInst->PV = get_temperature( pCfg, loop_idx ) + pCfg->TempOffset;

		if( pCfg->Enable && CFG.system.Run )
		{
			if( !AppTimer.IsRun( &pInst->timerStartDelay ) )
			{
				AppTimer.Start( &pInst->timerStartDelay, pCfg->StartDelay );
				pInst->MV = pCfg->OutputMin * MULTIPLY_PERMIL_TO_FLOAT;
				pInst->IntegralSum = 0;
				pInst->lastError = 0;
			}
			else if( AppTimer.IsExpired( &pInst->timerStartDelay ) )
			{
				switch( pCfg->ControlType )
				{
					case CTRL_PID:
					case CTRL_2DOF_PID:
						pid_calculation( pCfg, pInst );
						break;
					case CTRL_ONOFF:
						onoff_calculation( pCfg, pInst );
						break;
					case CTRL_BYPASS:
						pInst->MV = ( int16_t )MBSDB.Holdings[ MBS_HR_CH1_BYPASS_MV + pInst->CH ] * MULTIPLY_PERMYRIAD_TO_FLOAT;
						break;
					default:
						break;
				}

				if( pInst->lastSV != pCfg->SV )
				{
					pInst->lastSV = pCfg->SV;
					AppTimer.Start( &pInst->timerOutputDelay, pCfg->OutputDelay );
					pInst->MV = pCfg->OutputMin * MULTIPLY_PERMIL_TO_FLOAT;
				}
				if( !AppTimer.IsExpired( &pInst->timerOutputDelay ) )
				{
					pInst->MV = pCfg->OutputMin * MULTIPLY_PERMIL_TO_FLOAT;
					pInst->lastError = 0;
				}
			}
		}
		else
		{
			PID_instance_Init( &pid_inst[ loop_idx ], pCfg );
			pInst->directionInited = false;
			pInst->Region = PID_REGION_FAR;
			pInst->FarGains.Valid = false;
			pInst->NearGains.Valid = false;
			pInst->MV = 0;
			pInst->IntegralSum = 0;
			pInst->lastError = 0;
			pInst->lastFilteredSV = pInst->PV * MULTIPLY_mK_TO_K;
			pCfg->AutoTuneEnabled = 0;
			MBSDB.Holdings[ MBS_HR_CH1_AUTOTUNE_ENABLED + pInst->CH * MBS_HR_CH_SPAN ] = pCfg->AutoTuneEnabled;
			AppTimer.Stop( &pInst->timerStartDelay );
			AppTimer.Stop( &pInst->timerOutputDelay );
		}

		TICK_UP( loop_idx, MAX_CONTROL_LOOP );
	} while( 0 );//loop_idx != 0 );
}

/******************************************************************************
 * @fn void default_config(control_loop_config_st*)
 * @brief
 *
 * @param pCfg
 *****************************************************************************/
static void set_default_config( int32_t loop_idx )
{
	control_loop_config_st *pCfg = &ControllerVars.Cfg[ loop_idx ];

	pCfg->SV = 0;                           // default: 0         / Range: -273150 ~ 2000000 (unit 0.001)
	pCfg->Enable = 0;                       // default: 0         / Range: 0, 1
	pCfg->SamplePeriod = 50;                // default: 50        / Range: 0 ~ 60000 ( ms )
	pCfg->ControlPeriod = 50;               // default: 50        / Range: 0 ~ 60000 ( ms )
	pCfg->InputType = SEN_NONE;             // default: NONE      / Range: 0 ~ 8
	pCfg->InputChannel = INVALID_CH;        // default: -1        / Range: 0 ~ 19
	pCfg->OutputType = OUT_NONE;            // default: NONE      / Range: 0 ~ 9
	pCfg->OutputChannel = INVALID_CH;       // default: -1        / Range: 0 ~ 19
	pCfg->HighOverAlarm = 2000;             // default: 2000      / Range: 0 ~ 20000 ( * 0.01K )
	pCfg->LowUnderAlarm = 2000;             // default: 2000      / Range: 0 ~ 20000 ( * -0.01K )
	pCfg->OverCurrAlarm = 10000;            // default: 10000     / Range: 1 ~ 10000 ( mA )
	pCfg->ControlTimeOver = 3600;           // default: 3600      / Range: 0 ~ 3600 (sec)
	pCfg->TempOffset = 0;                   // default: 0         / Range: -10000 ~ 10000 ( * 0.01K )
	pCfg->Pb = 10000;                       // default: 10000     / Range: 0 ~ 60000 ( 0.1% )
	pCfg->Ti = 0;                           // default: 0         / Range: 0 ~ 60000 ( 0.1s )
	pCfg->Td = 0;                           // default: 0         / Range: 0 ~ 60000 ( 0.01s or 10ms )
	pCfg->Saturated_I = 0;                  // default: 0         / Range: 0 ~ 60000 ( * 0.001 )
	pCfg->InputFilterCoeff = 65;            // default: 65        / Range: 0 ~ 30000 ( * 100 )
	pCfg->Reserved = 0;                  	// default: 0         / Range: 
	pCfg->AutoTuneEnabled = 0;              // default: 0         / Range: 0, 1
	pCfg->OutputMax = 1000;                 // default: 1000      / Range: -1000 ~ 1000 ( * 0.1% )
	pCfg->OutputMin = -1000;                // default: -1000     / Range: -1000 ~ 1000 ( * 0.1% )
	pCfg->PWMFreq = 10;                     // default: 10        / Range: 0 ~ 60000 ( 0.1Hz )
	pCfg->OutputDelay = 0;                  // default: 0         / Range: 0 ~ 999 (sec)
	pCfg->ControlType = CTRL_2DOF_PID;      // default: 2DOF PID  / Range: 2DOF PID, PID, ONOFF
	pCfg->CoolHeat = COOL_CTRL_MODE;        // default: COOL      / Range: COOL, HEAT
	pCfg->StartDelay = 10;                  // default: 10        / Range: 0 ~ 999 (sec)
	pCfg->SmpsLeakageCurrent = 100;         // default: 100       / Range: 0 ~ 60000 (mA)
	pCfg->SmpsConfig.all = 0;               // default: 0         / Range:
}

/******************************************************************************
 * @brief
 *
 * @param loop_idx
 * @return int32_t [Process value(temperature) of loop_idx th control loop]
 *****************************************************************************/
static int32_t control_loop_get_pv( int loop_idx )
{
	if( ( 0 > loop_idx ) || ( loop_idx >= MAX_CONTROL_LOOP ) ) return MINIMUM_PV;

	return ControllerVars.Inst[ loop_idx ].PV;
}

/******************************************************************************
 * @brief
 *
 * @param loop_idx
 * @return int16_t [Manipulated value of loop_idx th control loop]
 *****************************************************************************/
static float control_loop_get_mv( int loop_idx )
{
	if( ( 0 > loop_idx ) || ( loop_idx >= MAX_CONTROL_LOOP ) ) return 0;

	return ControllerVars.Inst[ loop_idx ].MV;
}

/******************************************************************************
 * @brief Return PID diagnostics without changing the 1st-revision control.
 ******************************************************************************/
static bool control_loop_get_pid_diagnostic_status( int loop_idx, pid_diagnostic_status_st *status )
{
	if( ( NULL == status ) || ( 0 > loop_idx ) || ( loop_idx >= MAX_CONTROL_LOOP ) ) return false;

	const pid_st * const pPid = &pid_inst[ loop_idx ];
	const control_loop_instance_st * const pInst = &ControllerVars.Inst[ loop_idx ];
	const control_loop_config_st * const pCfg = &ControllerVars.Cfg[ loop_idx ];
	status->EffectivePb = pPid->EffectivePb;
	status->EffectiveTi = pPid->EffectiveTi;
	status->EffectiveTd = pPid->EffectiveTd;
	status->EffectiveInputFilterCoeff = pPid->EffectiveInputFilterCoeff;
	status->Direction = ( uint16_t )pInst->Direction;
	status->Region = ( uint16_t )pInst->Region;
	status->Pending = ( pCfg->Pb != pPid->EffectivePb )
		|| ( pCfg->Ti != pPid->EffectiveTi )
		|| ( pCfg->Td != pPid->EffectiveTd );
	status->CoolingReverseHeatAllowed = pPid->CoolingReverseHeatAllowed;
	status->CalculatedMV = pPid->CalculatedMV;
	status->AppliedMV = pPid->AppliedMV;
	return true;
}

/******************************************************************************
 * @fn int control_loop_get_fault(int)
 * @brief
 *
 * @param loop_idx
 * @return Fault status of loop_idx th control loop
 *****************************************************************************/
static loop_fault_status_ut control_loop_get_fault( int loop_idx )
{
	loop_fault_status_ut fs = { .All = 0xFFFF };
	if( ( 0 <= loop_idx ) && ( loop_idx < MAX_CONTROL_LOOP ) )
	{
		fs.All = ControllerVars.Inst[ loop_idx ].FaultStatus.All;
	}

	return fs;
}

/******************************************************************************
 * @brief 
 * 
 * @param loop_idx 
 * @param flt 
 *****************************************************************************/
static void control_loop_set_fault( int loop_idx, loop_fault_status_ut flt )
{
	if( ( 0 <= loop_idx ) && ( loop_idx < MAX_CONTROL_LOOP ) )
	{
		ControllerVars.Inst[ loop_idx ].FaultStatus = flt;
	}
}

/******************************************************************************
 * @brief Set configuration of control loop
 * Do not change configurations of Input and Output when Controller is running.
 * @param data
 * @param loop_idx index of control loop
 *****************************************************************************/
static void control_loop_set_config( uint16_t data[ ], int loop_idx )
{
	if( ( 0 > loop_idx ) || ( loop_idx >= MAX_CONTROL_LOOP ) ) return;

	control_loop_config_st * pCfg = &ControllerVars.Cfg[ loop_idx ];

	pCfg->SV = ( int32_t )( ( uint32_t )data[ 1 ] << 16 ) + data[ 0 ];
	pCfg->SamplePeriod = data[ 3 ];
	pCfg->ControlPeriod = data[ 4 ];
	if( !pCfg->Enable )
	{
		pCfg->InputType = data[ 5 ];
		pCfg->InputChannel = data[ 6 ];
		pCfg->OutputType = data[ 7 ];
		pCfg->OutputChannel = data[ 8 ];
	}
	pCfg->HighOverAlarm = data[ 9 ];
	pCfg->LowUnderAlarm = data[ 10 ];
	pCfg->OverCurrAlarm = data[ 11 ];
	pCfg->ControlTimeOver = data[ 12 ];
	pCfg->TempOffset = data[ 13 ];
	pCfg->Pb = data[ 14 ];
	pCfg->Ti = data[ 15 ];
	pCfg->Td = data[ 16 ];
	pCfg->Saturated_I = data[ 17 ];
	pCfg->InputFilterCoeff = data[ 18 ];
	pCfg->Reserved = data[ 19 ];
	pCfg->AutoTuneEnabled = data[ 20 ];
	pCfg->OutputMax = data[ 21 ];
	pCfg->OutputMin = data[ 22 ];
	if( !pCfg->Enable )
	{
		pCfg->PWMFreq = data[ 23 ];
	}
	pCfg->OutputDelay = data[ 24 ];
	pCfg->ControlType = data[ 25 ];
	pCfg->CoolHeat = data[ 26 ];
	pCfg->StartDelay = data[ 27 ];
	pCfg->SmpsLeakageCurrent = data[ 28 ];
	pCfg->SmpsConfig.all = data[ 29 ];
	pCfg->Enable = data[ 2 ];			// If it is updated at original order, then other datas cannot be updated when data[ 2 ] is true.
}

/******************************************************************************
 * @brief 
 * 
 * @param loop_idx 
 * @return const control_loop_config_st* 
 *****************************************************************************/
static const control_loop_config_st* const control_loop_get_config( int loop_idx )
{
	if( ( 0 > loop_idx ) || ( loop_idx >= MAX_CONTROL_LOOP ) ) return NULL;

	if( !ControllerVars.Inited )
	{
		ControllerTaskInit();
	}

	return &ControllerVars.Cfg[ loop_idx ];
}

/******************************************************************************
 * @brief Clear fault status of control loop.
 * @brief It should be cleared alarm by MODBUS command from PLC.
 *
 * @param clear_bitmask_hi Bitmask to clear fault status of control loop 11th to 20th
 * @param clear_bitmask_lo Bitmask to clear fault status of control loop 1st to 10th
 *****************************************************************************/
static void control_loop_clear_alarm( uint16_t clear_bitmask_hi, uint16_t clear_bitmask_lo )
{
	for( int loop_idx = 0; loop_idx < MAX_CONTROL_LOOP / 2; ++loop_idx )
	{
		if( clear_bitmask_hi & 1 )
		{
			ControllerVars.Inst[ loop_idx + 10 ].FaultStatus.All = 0;
			Reset_smps_comm( ControllerVars.Cfg[ loop_idx + 10 ].OutputType, ControllerVars.Cfg[ loop_idx + 10 ].OutputChannel );
		}
		if( clear_bitmask_lo & 1 )
		{
			ControllerVars.Inst[ loop_idx ].FaultStatus.All = 0;
			Reset_smps_comm( ControllerVars.Cfg[ loop_idx ].OutputType, ControllerVars.Cfg[ loop_idx ].OutputChannel );
		}

		clear_bitmask_hi >>= 1;
		clear_bitmask_lo >>= 1;
	}
}

/******************************************************************************
 * @brief Get temperature from ADC, CAN, PLC Serial Comm
 *
 * @param pCfg
 * @retval Temperature(Process Value) / unit 0.001ºC
 *****************************************************************************/
static int32_t get_temperature( control_loop_config_st *pCfg, int loop_idx )
{
	int32_t PV = MINIMUM_PV;

	if( ( !pCfg ) || ( 0 > loop_idx ) || ( loop_idx >= MAX_CONTROL_LOOP ) ) return PV;

	if( pCfg->InputType == SEN_COMM )
	{
		uint16_t pv_h_hr_idx = MBS_HR_CH1_PV_H + loop_idx * MBS_PV_WORDS;
		uint16_t pv_l_hr_idx = MBS_HR_CH1_PV_L + loop_idx * MBS_PV_WORDS;
		PV = ( ( ( uint32_t )MBSDB.Holdings[ pv_h_hr_idx ] ) << 16 ) | MBSDB.Holdings[ pv_l_hr_idx ];
	}
	else
	{
		int8_t ch = pCfg->InputChannel;

		if( INVALID_CH != ch )
		{
			if( ch < MAX_TEMP_CHANNEL )
			{
				PV = Temp.GetTemp( ch ) * MULTIPLY_K_TO_mK;
			}
			else
			{
				PV = RemoteIO.GetPvData( ch );
			}
		}
	}

	return PV;
}

/******************************************************************************
 * @brief Check whether the setpoint value is changed
 * 
 * @param pInst 
 * @return true 
 * @return false 
 *****************************************************************************/
static bool sv_changed( control_loop_instance_st *pInst )
{
	bool result = false;
	if( pInst->OldSV != pInst->lastSV )
	{
		pInst->OldSV = pInst->lastSV;
		result = true;
	}

	return result;
}

/******************************************************************************
 * @brief PID Controller Calculation
 *
 * @param pCfg Pointer to control_loop_config_st structure
 * @param pInst Pointer to control_loop_instance_st structure
 * @return int16_t 1: MV updated, 0: MV not updated
 ******************************************************************************/
static __RAM_FUNC int16_t pid_calculation( control_loop_config_st *pCfg, control_loop_instance_st *pInst )
{
	int retval = 0;

	if( !pCfg || !pInst ) return 0;

	const float dt = ( ( pCfg->ControlPeriod < MIN_CONTROL_PERIOD ) ? MIN_CONTROL_PERIOD : pCfg->ControlPeriod ) * MULTIPLY_ms_TO_s;

	if( AppTimer.IsExpired( &pInst->timerPIDPeriod ) )
	{
		AppTimer.Start( &pInst->timerPIDPeriod, dt );

		if( pCfg->AutoTuneEnabled )
		{
			autotune_relay_method( pCfg, pInst );

			if( !pCfg->AutoTuneEnabled )
			{
				MBSDB.Holdings[ MBS_HR_CH1_PB + pInst->CH * MBS_HR_CH_SPAN ] = pCfg->Pb;
				MBSDB.Holdings[ MBS_HR_CH1_TI + pInst->CH * MBS_HR_CH_SPAN ] = pCfg->Ti;
				MBSDB.Holdings[ MBS_HR_CH1_TD + pInst->CH * MBS_HR_CH_SPAN ] = pCfg->Td;
				MBSDB.Holdings[ MBS_HR_CH1_AUTOTUNE_ENABLED + pInst->CH * MBS_HR_CH_SPAN ] = 0;
				retval = 1;
			}
		}
		else
		{
			pid_st * const pPid = &pid_inst[ pInst->CH ];
			const bool setpoint_changed = PID_update_operating_state( pInst, pCfg );
			if( setpoint_changed )
			{
				/* A new demand starts with a clean integral; live PLC profile writes do not. */
				pPid->I = 0.0f;
				pPid->GainTransitionBias = 0.0f;
				pPid->GainTransitionPending = false;
				pPid->CoolingReverseHeatAllowed = false;
				pPid->CoolingTargetReached = false;
				pPid->CoolingSlewActive = false;
				pPid->CoolingReverseHeatCandidateTime = 0.0f;
				pPid->HeatingTargetReached = false;
			}

			PID_cache_gain_profile( pInst, pCfg );
			const pid_gain_profile_st * const effective = PID_select_gain_profile( pInst );
			const uint16_t effective_pb = effective ? effective->Pb : pCfg->Pb;
			uint16_t effective_ti = effective ? effective->Ti : pCfg->Ti;
			const uint16_t effective_td = effective ? effective->Td : pCfg->Td;
			if( ( PID_DIRECTION_HEATING == pInst->Direction ) && ( pInst->PV >= pCfg->SV ) )
			{
				pPid->HeatingTargetReached = true;
			}
			if( ( PID_DIRECTION_HEATING == pInst->Direction )
				&& ( PID_REGION_NEAR == pInst->Region ) && ( effective_ti > 0U ) )
			{
				effective_ti = pPid->HeatingTargetReached
					? PID_HEATING_NEAR_HOLD_TI_RAW : PID_HEATING_NEAR_APPROACH_TI_RAW;
			}

			PID_instance_UpdateGains( pPid, pCfg, effective_pb, effective_ti, effective_td );
			if( setpoint_changed )
			{
				/* Do not transfer the old demand's output into a new setpoint. */
				pPid->GainTransitionPending = false;
			}
			float sp = pCfg->SV * MULTIPLY_mK_TO_K;
			float pv = pInst->PV * MULTIPLY_mK_TO_K;
			const bool cooling_near = ( PID_DIRECTION_COOLING == pInst->Direction )
				&& ( PID_REGION_NEAR == pInst->Region );
			pInst->MV = PID2DOF_Calculate_backward_euler( pPid, sp, pv, pCfg->CoolHeat, cooling_near );
			retval = 1;
		}
	}
	return retval;
}

/******************************************************************************
 * @brief Latch the SV-change direction and update the FAR/NEAR region.
 *
 * Direction changes only when SV changes.  Region uses separate enter/exit
 * thresholds so measurement noise cannot repeatedly switch the effective Ti.
 ******************************************************************************/
static __RAM_FUNC bool PID_update_operating_state( control_loop_instance_st *pInst, control_loop_config_st const *pCfg )
{
	if( !pInst || !pCfg ) return false;
	bool setpoint_changed = false;

	if( !pInst->directionInited )
	{
		/* Equality defaults to cooling; direction is latched from the active demand. */
		pInst->Direction = ( pCfg->SV > pInst->PV ) ? PID_DIRECTION_HEATING : PID_DIRECTION_COOLING;
		pInst->directionSV = pCfg->SV;
		pInst->Region = PID_REGION_FAR;
		pInst->directionInited = true;
		setpoint_changed = true;
	}
	else if( pCfg->SV != pInst->directionSV )
	{
		pInst->Direction = ( pCfg->SV > pInst->directionSV ) ? PID_DIRECTION_HEATING : PID_DIRECTION_COOLING;
		pInst->directionSV = pCfg->SV;
		pInst->Region = PID_REGION_FAR;
		setpoint_changed = true;
	}

	const float abs_error = fabsf( ( pCfg->SV - pInst->PV ) * MULTIPLY_mK_TO_K );
	const bool heating = ( PID_DIRECTION_HEATING == pInst->Direction );
	const float near_enter_error = heating ? PID_HEATING_NEAR_ENTER_ERROR_K
		: PID_COOLING_NEAR_ENTER_ERROR_K;
	const float near_exit_error = heating ? PID_HEATING_NEAR_EXIT_ERROR_K
		: PID_COOLING_NEAR_EXIT_ERROR_K;
	if( PID_REGION_FAR == pInst->Region )
	{
		if( abs_error <= near_enter_error )
		{
			pInst->Region = PID_REGION_NEAR;
		}
	}
	else if( abs_error >= near_exit_error )
	{
		pInst->Region = PID_REGION_FAR;
	}

	return setpoint_changed;
}

/******************************************************************************
 * @brief Cache the two PLC gain profiles without letting each PLC write select
 *        the active controller. Ti=0 identifies FAR and Ti>0 identifies NEAR.
 ******************************************************************************/
static __RAM_FUNC void PID_cache_gain_profile( control_loop_instance_st *pInst, control_loop_config_st const *pCfg )
{
	if( !pInst || !pCfg ) return;

	pid_gain_profile_st * const profile = ( pCfg->Ti > 0 ) ? &pInst->NearGains : &pInst->FarGains;
	profile->Pb = pCfg->Pb;
	profile->Ti = pCfg->Ti;
	profile->Td = pCfg->Td;
	profile->Valid = true;
}

/******************************************************************************
 * @brief Select gains only from the internally latched FAR/NEAR region.
 *        Until the matching PLC profile has been observed, use the current
 *        configured gains as a temporary fallback.
 ******************************************************************************/
static __RAM_FUNC const pid_gain_profile_st *PID_select_gain_profile( control_loop_instance_st const *pInst )
{
	if( !pInst ) return NULL;

	const pid_gain_profile_st * const profile =
		( PID_REGION_NEAR == pInst->Region ) ? &pInst->NearGains : &pInst->FarGains;
	return profile->Valid ? profile : NULL;
}

/******************************************************************************
 * @brief 2DOF PID Controller Calculation (Backward Euler Discretization)
 *
 * @param pPID Pointer to pid_st structure
 * @param sp Setpoint Value (SP)
 * @param pv Process Value (PV)
 * @return Manipulated Value (MV)
 ******************************************************************************/
static __RAM_FUNC float PID2DOF_Calculate_backward_euler( pid_st *pPID, float sp, float pv,
	cool_heat_mode_et cool_heat, bool cooling_near )
{
	/****************************************************************************
	InputFilterCoeff filters PV only. P and I use the true setpoint error against
	the filtered PV. D is derivative-on-filtered-PV, so setpoint changes do not
	create derivative kick. This is the beta=1, gamma=0 form of a 2DOF PID.
	****************************************************************************/

	float Ti = pPID->Ti;
	float Ts = pPID->Ts;
	float alpha = pPID->alpha;

	/* 1. First-order low-pass filter on the measured PV. Raw 0 bypasses the
	 * filter; raw 100 follows the current sample directly. */
	if( !pPID->PVFilterInited )
	{
		pPID->PV_filtered = pv;
		pPID->PV_prev = pv;
		pPID->v_prev = 0.0f;
		pPID->PVFilterInited = true;
	}
	else if( ( alpha <= 0.0f ) || ( alpha >= 1.0f ) )
	{
		pPID->PV_filtered = pv;
	}
	else
	{
		pPID->PV_filtered += alpha * ( pv - pPID->PV_filtered );
	}

	/* 2. P and I use the real SV error; Ti no longer delays the setpoint. */
	float action = ( COOL_CTRL_MODE == cool_heat ) ? -1.0f : 1.0f;
	float error = action * ( sp - pPID->PV_filtered );

	/* 3. Proportional term (2DOF setpoint weight beta=1). */
	float P_term = pPID->Kp * error;

	/* 4. Filtered derivative on PV (2DOF derivative weight gamma=0). */
	float D_term = 0.0f;
	float Td = pPID->Td;
	float lambda = pPID->lambda;

	if( Td > 0.0f )
	{
		float denominator = Ts + ( lambda * Td );
		if( ( denominator < 0.0f ) || ( 0.0f < denominator ) )
		{
			float num_d = ( lambda * Td * pPID->v_prev )
				+ ( pPID->Kp * Td * ( pPID->PV_filtered - pPID->PV_prev ) );
			D_term = num_d / denominator;
		}
	}

	// 5. 계산된 조작량 (Calculated MV) - 아직 제한 전
	// 주의: 적분항은 '이전 스텝까지 누적된 값(I)'을 사용합니다.
	/* Match the previous applied output on a live FAR/NEAR gain change. The
	 * temporary bias then decays instead of becoming a long-lived I preload. */
	const float pid_base = P_term + ( ( Ti > 0.0f ) ? pPID->I : 0.0f ) - ( action * D_term );
	if( pPID->GainTransitionPending )
	{
		pPID->GainTransitionBias = pPID->GainTransitionTargetMV - pid_base;
		pPID->GainTransitionPending = false;
	}
	float mv_calc = pid_base + pPID->GainTransitionBias;

	// 6. 출력 제한 (Saturation) 및 실제 조작량 결정
	float mv_final = mv_calc;

	if( mv_final > pPID->u_max )
	{
		mv_final = pPID->u_max;
	}
	else if( mv_final < pPID->u_min )
	{
		mv_final = pPID->u_min;
	}

	/* Bound the intentional cooling limit cycle near SV. A small cooling floor
	 * keeps the SMPS out of zero during normal hold. Reverse heat requires a
	 * filtered 0.15 K over-cooling condition for three seconds and is limited to
	 * 0.3%, then all later recovery changes are slew-limited. */
	bool cooling_output_constrained = false;
	if( cooling_near && ( COOL_CTRL_MODE == cool_heat ) )
	{
		if( !pPID->CoolingTargetReached
			&& ( pPID->PV_filtered <= ( sp + PID_COOLING_TARGET_REACHED_ERROR_K ) ) )
		{
			pPID->CoolingTargetReached = true;
			pPID->CoolingSlewActive = true;
		}

		if( pPID->CoolingTargetReached && !pPID->CoolingReverseHeatAllowed )
		{
			if( pPID->PV_filtered <= ( sp - PID_COOLING_REVERSE_HEAT_ENTER_K ) )
			{
				pPID->CoolingReverseHeatCandidateTime += Ts;
				if( pPID->CoolingReverseHeatCandidateTime >= PID_COOLING_REVERSE_HEAT_CONFIRM_S )
				{
					pPID->CoolingReverseHeatAllowed = true;
					pPID->CoolingSlewActive = true;
					pPID->CoolingReverseHeatCandidateTime = 0.0f;
				}
			}
			else
			{
				pPID->CoolingReverseHeatCandidateTime = 0.0f;
			}
		}

		if( pPID->CoolingReverseHeatAllowed )
		{
			if( pPID->PV_filtered >= ( sp - PID_COOLING_REVERSE_HEAT_EXIT_K ) )
			{
				pPID->CoolingReverseHeatAllowed = false;
			}
			else if( mv_final < -PID_COOLING_REVERSE_HEAT_MAX_MV )
			{
				mv_final = -PID_COOLING_REVERSE_HEAT_MAX_MV;
				cooling_output_constrained = true;
			}
			else if( mv_final > 0.0f )
			{
				mv_final = 0.0f;
				cooling_output_constrained = true;
			}
		}

		if( !pPID->CoolingReverseHeatAllowed && ( mv_final < PID_COOLING_NEAR_MIN_MV ) )
		{
			mv_final = PID_COOLING_NEAR_MIN_MV;
			cooling_output_constrained = true;
		}

		if( pPID->CoolingTargetReached && pPID->CoolingSlewActive )
		{
			const float max_delta = PID_COOLING_STABLE_SLEW_RATE_PER_S * Ts;
			const float upper = pPID->AppliedMV + max_delta;
			const float lower = pPID->AppliedMV - max_delta;
			if( mv_final > upper )
			{
				mv_final = upper;
				cooling_output_constrained = true;
			}
			else if( mv_final < lower )
			{
				mv_final = lower;
				cooling_output_constrained = true;
			}
		}
	}
	else
	{
		pPID->CoolingReverseHeatAllowed = false;
		pPID->CoolingTargetReached = false;
		pPID->CoolingSlewActive = false;
		pPID->CoolingReverseHeatCandidateTime = 0.0f;
	}

	/* Keep both the unconstrained calculation and the actual applied output. */
	pPID->CalculatedMV = mv_calc;
	pPID->AppliedMV = mv_final;

	// 7. 적분항 anti-windup 업데이트 (Back-calculation)
	// 수식: I[k+1] = I[k] + (Ki * error * Ts) + (Kb * (mv_final - mv_calc) * Ts)
	// mv_final - mv_calc는 초과된 양(오차)을 의미합니다.
	float Ki_term = 0.0f;
	if( Ti > 0.0f )
	{
		Ki_term = ( pPID->Kp / Ti ) * error * Ts;
		if( cooling_output_constrained
			&& ( ( ( mv_final > mv_calc ) && ( Ki_term < 0.0f ) )
				|| ( ( mv_final < mv_calc ) && ( Ki_term > 0.0f ) ) ) )
		{
			/* Do not integrate farther into any applied-output constraint. */
			Ki_term = 0.0f;
		}
	}

	// Saturation 오차 (실제 출력 - 계산된 출력)
	float saturation_diff = mv_final - mv_calc;

	// 역계산 항 (Tracking Term)
	float back_calc_term = pPID->Kaw * saturation_diff * Ts;

	// 다음 주기를 위한 적분값 누적
	pPID->I += Ki_term + back_calc_term;

	/* Remove only the temporary gain-transition correction. With an 8 s time
	 * constant, the 3rd-test 17%p profile jump is converted to about 2%p/s. */
	if( PID_GAIN_TRANSITION_BIAS_TAU_S > Ts )
	{
		pPID->GainTransitionBias *= 1.0f - ( Ts / PID_GAIN_TRANSITION_BIAS_TAU_S );
	}
	else
	{
		pPID->GainTransitionBias = 0.0f;
	}

	// 8. 다음 스텝을 위해 현재 상태 저장
	pPID->SP_ref_prev = sp;
	pPID->SP_prev = sp;
	pPID->PV_prev = pPID->PV_filtered;
	pPID->v_prev = D_term;

	return mv_final;
}

/******************************************************************************
 * @brief PID Controller (supported 2DOF PID)
 * 
 * @param pCfg 
 * @param pInst 
 * @return int16_t
 ******************************************************************************/
static __RAM_FUNC float pid_calculation_too_simple( control_loop_config_st *pCfg, control_loop_instance_st *pInst )
{
	const float dt = ( ( pCfg->ControlPeriod < MIN_CONTROL_PERIOD ) ? MIN_CONTROL_PERIOD : pCfg->ControlPeriod ) * MULTIPLY_ms_TO_s;
	const float Kp = pCfg->Pb ? ( 100.0f / ( pCfg->Pb * MULTIPLY_Pb_INT_TO_FLOAT ) ) : 0.0f; // Kp = 100 / Pb
	const float Ti_sec = pCfg->Ti * MULTIPLY_Ti_INT_TO_FLOAT;
	const float Td_sec = pCfg->Td * MULTIPLY_Td_INT_TO_FLOAT;
	const float Ki = (Ti_sec > 0.0f) ? (Kp / Ti_sec) : 0.0f; // Ki = Kp / Ti
	const float Kd = Kp * Td_sec; // Kd = Kp * Td
	const float SV = pCfg->SV * MULTIPLY_mK_TO_K;
	const float PV = pInst->PV * MULTIPLY_mK_TO_K;
	const float error = SV - PV;
	const float alpha = pCfg->InputFilterCoeff >= 0 ? ( pCfg->InputFilterCoeff * MULTIPLY_PERCENT_TO_FLOAT ) : 1.0f;
	const float max_mv = pCfg->OutputMax * MULTIPLY_PERMIL_TO_FLOAT;
	const float min_mv = pCfg->OutputMin * MULTIPLY_PERMIL_TO_FLOAT;

	float P_term = Kp * ( alpha * SV - PV );
	float I_increment = error * dt;
	float D_term = Kd * ( pInst->lastPV - PV ) / dt;

	float MV_pre_saturation = P_term + Ki * ( pInst->IntegralSum + I_increment ) + D_term;

	// Anti-windup for I-term
	if( ( MV_pre_saturation > max_mv ) && ( error > 0 ) )
	{
			I_increment = 0.0f;
	}
	else if( ( MV_pre_saturation < min_mv ) && ( error < 0 ) )
	{
		I_increment = 0.0f;
	}

	pInst->IntegralSum += I_increment;
	float I_term = Ki * pInst->IntegralSum;

	float fMV = P_term + I_term + D_term;

	fMV = fmaxf( min_mv, fminf( max_mv, fMV ) );

	pInst->lastPV = PV;

	pInst->lastError = error;
	pInst->MV = fMV;

	return fMV;
}

/******************************************************************************
 * @brief 2DOF PID Controller Initialization
 *
 * @param pPid Pointer to pid_st structure
 * @param pCfg Pointer to control_loop_config_st structure
 ******************************************************************************/
static __RAM_FUNC void PID_instance_Init( pid_st *pPid, control_loop_config_st *pCfg )
{
	pPid->Kp = pCfg->Pb ? ( 100.0f / ( pCfg->Pb * MULTIPLY_Pb_INT_TO_FLOAT ) ) : 0.0f; // Kp = 1 / Pb
	pPid->Ti = pCfg->Ti * MULTIPLY_Ti_INT_TO_FLOAT;
	pPid->Td = pCfg->Td * MULTIPLY_Td_INT_TO_FLOAT;
	pPid->alpha = pCfg->InputFilterCoeff * MULTIPLY_PERCENT_TO_FLOAT;
	pPid->lambda = 0.2f;
	pPid->Ts = ( ( pCfg->ControlPeriod < MIN_CONTROL_PERIOD ) ? MIN_CONTROL_PERIOD : pCfg->ControlPeriod ) * MULTIPLY_ms_TO_s;
	pPid->u_min = pCfg->OutputMin * MULTIPLY_PERMIL_TO_FLOAT;
	pPid->u_max = pCfg->OutputMax * MULTIPLY_PERMIL_TO_FLOAT;
	pPid->Kaw = pCfg->Ti ? 1.0f / pPid->Ti : 0.0f; // Anti-windup gain

	float a = 2.0f * pPid->Ti / pPid->Ts;
	pPid->b0 = 1.0f + (1.0f - pPid->alpha) * a;
	pPid->b1 = 1.0f - (1.0f - pPid->alpha) * a;
	pPid->c0 = 1.0f + a;
	pPid->c1 = 1.0f - a;
	pPid->SP_prev = 0.0f;
	pPid->SP_ref_prev = 0.0f;

	pPid->I = 0.0f;

	pPid->p0 = (2.0f * pPid->Kp * pPid->Td) / pPid->Ts;
	pPid->b  = (2.0f * pPid->lambda * pPid->Td) / pPid->Ts;
	pPid->v_prev = 0.0f;
	pPid->PV_filtered = 0.0f;
	pPid->PV_prev = 0.0f;
	pPid->PVFilterInited = false;
	pPid->EffectivePb = pCfg->Pb;
	pPid->EffectiveTi = pCfg->Ti;
	pPid->EffectiveTd = pCfg->Td;
	pPid->EffectiveInputFilterCoeff = pCfg->InputFilterCoeff;
	pPid->CalculatedMV = 0.0f;
	pPid->AppliedMV = 0.0f;
	pPid->GainTransitionBias = 0.0f;
	pPid->GainTransitionTargetMV = 0.0f;
	pPid->GainTransitionPending = false;
	pPid->CoolingReverseHeatAllowed = false;
	pPid->CoolingTargetReached = false;
	pPid->CoolingSlewActive = false;
	pPid->CoolingReverseHeatCandidateTime = 0.0f;
	pPid->HeatingTargetReached = false;
}

/******************************************************************************
 * @brief Apply the profile selected by the internal FAR/NEAR state while
 *        preserving the running PID state.
 *
 * Ti=0 selects P-only calculation but does not delete the stored integral.
 * Integral reset is owned by run/setpoint state changes, not PLC profile writes.
 ******************************************************************************/
static __RAM_FUNC void PID_instance_UpdateGains( pid_st *pPid, control_loop_config_st const *pCfg,
	uint16_t effective_pb_raw, uint16_t effective_ti_raw, uint16_t effective_td_raw )
{
	if( !pPid || !pCfg ) return;

	const float kp = effective_pb_raw ? ( 100.0f / ( effective_pb_raw * MULTIPLY_Pb_INT_TO_FLOAT ) ) : 0.0f;
	const float ti = effective_ti_raw * MULTIPLY_Ti_INT_TO_FLOAT;
	const float td = effective_td_raw * MULTIPLY_Td_INT_TO_FLOAT;
	const float alpha = pCfg->InputFilterCoeff * MULTIPLY_PERCENT_TO_FLOAT;
	pPid->EffectivePb = effective_pb_raw;
	pPid->EffectiveTi = effective_ti_raw;
	pPid->EffectiveTd = effective_td_raw;
	pPid->EffectiveInputFilterCoeff = pCfg->InputFilterCoeff;

	if( ( pPid->Kp == kp ) && ( pPid->Ti == ti ) && ( pPid->Td == td ) && ( pPid->alpha == alpha ) ) return;

	/* The next PID sample computes a bias against the new P/I/D terms so the
	 * applied output begins at the previous sample instead of jumping. */
	pPid->GainTransitionTargetMV = pPid->AppliedMV;
	pPid->GainTransitionPending = true;

	pPid->Kp = kp;
	pPid->Ti = ti;
	pPid->Td = td;
	pPid->alpha = alpha;
	pPid->Kaw = ( ti > 0.0f ) ? ( 1.0f / ti ) : 0.0f;

	/* Keep the derived values coherent for the alternate PID implementation. */
	const float a = 2.0f * pPid->Ti / pPid->Ts;
	pPid->b0 = 1.0f + ( 1.0f - pPid->alpha ) * a;
	pPid->b1 = 1.0f - ( 1.0f - pPid->alpha ) * a;
	pPid->c0 = 1.0f + a;
	pPid->c1 = 1.0f - a;
	pPid->p0 = ( 2.0f * pPid->Kp * pPid->Td ) / pPid->Ts;
	pPid->b = ( 2.0f * pPid->lambda * pPid->Td ) / pPid->Ts;
}

static __RAM_FUNC float pid_calculation_ai_tustin( pid_st* pPid, float sp, float pv )
{
	// Setpoint filter
	float SP_ref = (pPid->b0 * sp + pPid->b1 * pPid->SP_prev - pPid->c1 * pPid->SP_ref_prev) / pPid->c0;

	float error = SP_ref - pv;

	// Proportional term
	float P = pPid->Kp * error;

	// Preceding derivative-type element
	float v = (pPid->p0 * (pv - pPid->PV_prev) - (1.0f - pPid->b) * pPid->v_prev) / (1.0f + pPid->b);

	// Unsaturated control output
	float u_unsat = P + pPid->I - v;
	float u = u_unsat;

	// Saturation
	if (u > pPid->u_max) u = pPid->u_max;
	else if (u < pPid->u_min) u = pPid->u_min;

	// Back-calculation anti-windup
	float aw_term = pPid->Kaw * (u - u_unsat);
	float I_new = pPid->I + (pPid->Kp * pPid->Ts / pPid->Ti) * error + aw_term;

	pPid->I = I_new;
	pPid->SP_prev = sp;
	pPid->SP_ref_prev = SP_ref;
	pPid->v_prev = v;
	pPid->PV_prev = pv;

	return u;
}

/******************************************************************************
 * @brief PID Controller (supported 2DOF PID)
 * 
 * @param pCfg 
 * @param pInst 
 * @return int16_t
 ******************************************************************************/
static int16_t pid_calculation_old( control_loop_config_st *pCfg, control_loop_instance_st *pInst )
{
	int retval = 0;

	if( !pCfg || !pInst ) return 0;

	const float dt = ( ( pCfg->ControlPeriod < MIN_CONTROL_PERIOD ) ? MIN_CONTROL_PERIOD : pCfg->ControlPeriod ) * MULTIPLY_ms_TO_s;

	if( AppTimer.IsExpired( &pInst->timerPIDPeriod ) )
	{
		AppTimer.Start( &pInst->timerPIDPeriod, dt );

		if( pCfg->AutoTuneEnabled )
		{
			autotune_relay_method( pCfg, pInst );

			if( !pCfg->AutoTuneEnabled )
			{
				MBSDB.Holdings[ MBS_HR_CH1_PB + pInst->CH * MBS_HR_CH_SPAN ] = pCfg->Pb;
				MBSDB.Holdings[ MBS_HR_CH1_TI + pInst->CH * MBS_HR_CH_SPAN ] = pCfg->Ti;
				MBSDB.Holdings[ MBS_HR_CH1_TD + pInst->CH * MBS_HR_CH_SPAN ] = pCfg->Td;
				MBSDB.Holdings[ MBS_HR_CH1_AUTOTUNE_ENABLED + pInst->CH * MBS_HR_CH_SPAN ] = 0;
				retval = 1;
			}
		}
		else
		{
			float fMV = 0.0f;
			float SV = pCfg->SV * MULTIPLY_mK_TO_K;
			const float PV = pInst->PV * MULTIPLY_mK_TO_K;
			const float Kp = pCfg->Pb ? ( 100.0f / ( pCfg->Pb * MULTIPLY_Pb_INT_TO_FLOAT ) ) : 0.0f;
			const float Ti = pCfg->Ti * MULTIPLY_Ti_INT_TO_FLOAT;
			const float Td = pCfg->Td * MULTIPLY_Td_INT_TO_FLOAT;
			const float fOutputMax = pCfg->OutputMax * MULTIPLY_PERMIL_TO_FLOAT;
			const float fOutputMin = pCfg->OutputMin * MULTIPLY_PERMIL_TO_FLOAT;

			/****************************************************************************
			       +-------------+             +-------+           +-------+
			SV --> |  Filtering  | --> (+) --> | P.I.D |--> MV --> | Plant | ---+---> PV
			       | using alpha |      |      +-------+           +-------+    |
			       +-------------+      +-<-----------<-----------<-----------<-+
			****************************************************************************/
			/* Smooth change SV (2 dimension of Freedom PID) */
			if( CTRL_2DOF_PID == pCfg->ControlType )
			{
				if( sv_changed( pInst ) )
				{
					if( pCfg->InputFilterCoeff >= 0 )
					{
						float Tu = fabs( PV - SV ) * 1.7320508f;
						float tau = 1.0f / ( 2.0f * ( float )M_PI / ( Tu * 10 ) );
						float alpha_gain = pCfg->InputFilterCoeff * MULTIPLY_PERCENT_TO_FLOAT;
						if( pCfg->InputFilterCoeff == 0 )
						{
							alpha_gain = 0.65f;
						}
						pInst->filter_alpha = pow( tau / ( tau + dt ),  alpha_gain );
					}
					else
					{
						pInst->filter_alpha = 0;
					}

					pInst->lastFilteredSV = PV;
				}

				pInst->lastFilteredSV = pInst->filter_alpha * pInst->lastFilteredSV + ( 1 - pInst->filter_alpha ) * SV;
				if( fabs( pInst->oldFilteredSV - pInst->lastFilteredSV ) < 2e-7 * dt )
				{
					pInst->lastFilteredSV = SV;
				}
				pInst->oldFilteredSV = pInst->lastFilteredSV;
				SV = fround( pInst->lastFilteredSV );
			}

			float error = SV - PV;

			if( COOL_CTRL_MODE == pCfg->CoolHeat )
			{
				error *= -1;
			}

			pInst->IntegralSum = ( Ti > 0.0f ) ? pInst->IntegralSum + Kp * ( error * dt / Ti ) : 0.0f;

			if( pCfg->Saturated_I )
			{
				const float SaturatedI = pCfg->Saturated_I * MULTIPLY_SatI_INT_TO_FLOAT;

				if( pInst->IntegralSum > SaturatedI )
				{
					pInst->IntegralSum = SaturatedI;
				}
				else if( pInst->IntegralSum < -SaturatedI )
				{
					pInst->IntegralSum = -SaturatedI;
				}
			}

#if 1
			float proportion = error;
			float derivate = ( error - pInst->lastError ) / dt * Td;
			float integral_temp = Kp * error * dt / Ti;

			/* To implement anti-windup, the currently computed integral term is not accumulated in pInst->iTerm at this. */
			/* 1. In other words, after the PID is computed, */
			/* 2. if the MV exceeds the limit, the integral calculation stops and operates like a PD controller */
			/* 3. And if the MV is within the limit, the integral calculation continues and operates like a PID controller. */
			/* The point is that the integral-term accumulation is stopped when the MV exceeds the limit. */
			fMV = Kp * ( proportion + derivate ) + pInst->IntegralSum + integral_temp;

			if( isfinite( fMV ) )	// 0 or positive and negative real number
			{
				// MV Limitation and Anti-Windup
				if( fMV > fOutputMax )
				{
					fMV = fOutputMax;
				}
				else if( fMV < fOutputMin )
				{
					fMV = fOutputMin;
				}
				else
				{
					pInst->IntegralSum += integral_temp;
				}
			}
			else
			{
				if( isnan( fMV ) )
				{
				}
				else if( signbit( fMV ) > 0 )	// positive infinity
				{
					fMV = fOutputMax;
				}
				else if( signbit( fMV ) < 0 )	// negative infinity
				{
					fMV = fOutputMin;
				}
			}
#else
			float proportion = error;
			float differential = ( error - pInst->lastError ) / dt * Td;

			/* P I D */
			fMV = Kp * ( proportion + differential ) + pInst->IntegralSum;

			if( isfinite( fMV ) )	// 0 or positive and negative real number
			{
				// MV Limitation and Anti-Windup
				if( fMV > fOutputMax )
				{
					pInst->IntegralSum = fOutputMax - Kp * ( proportion + differential );
					fMV = fOutputMax;
				}
				else if( fMV < fOutputMin )
				{
					pInst->IntegralSum = fOutputMin - Kp * ( proportion + differential );
					fMV = fOutputMin;
				}
			}
			else
			{
				if( isnan( fMV ) )
				{

				}
				else if( signbit( fMV ) > 0 )	// positive infinity
				{
					fMV = fOutputMax;
				}
				else if( signbit( fMV ) < 0 )	// negative infinity
				{
					fMV = fOutputMin;
				}
			}
#endif

			pInst->lastError = error;
			pInst->MV = fMV;

			retval = 1;
		}
	}

	return retval;
}

/******************************************************************************
 * @brief Autotuner through Relay method.
 * 
 * @param[in] pCfg
 * @param[in] pInst 
 *****************************************************************************/
static void autotune_relay_method( control_loop_config_st *pCfg, control_loop_instance_st *pInst )
{
	float tm_now = 0.0f;

	if( !pCfg || !pInst ) return;

	if( !pCfg->Enable )
	{
		pInst->Autotune.Stage = AT_STOPPED;
		return;
	}

	/* find MIN MAX value */

	int32_t PV = pInst->PV;     // -273.150 ~ 2000.000ºC
	int32_t SV = pCfg->SV;      // -273.150 ~ 2000.000ºC
	float fOutputMin = pCfg->OutputMin * MULTIPLY_PERMIL_TO_FLOAT;
	float fOutputMax = pCfg->OutputMax * MULTIPLY_PERMIL_TO_FLOAT;

	// Automatic tuning error checking at every stage
	tm_now = AppTimer.GetCurrentSecs();
	if( tm_now - pInst->Autotune.StageTime > 600.0f )     // the duration of every stage should be less than 10 minutes.
	{
		if( pInst->Autotune.Stage != AT_STOPPED )
		{
			pInst->Autotune.Stage = AT_STOPPED;
			pInst->FaultStatus.AutotuneError = 1;
		}
	}

	switch( pInst->Autotune.Stage )
	{
		default:
			break;
		case AT_STOPPED:
			pInst->Autotune.Stage = AT_START;
			pInst->Autotune.StageTime = tm_now;
			break;
		case AT_START:
			pInst->MV = ( HEAT_CTRL_MODE == pCfg->CoolHeat ) ? fOutputMin : fOutputMax;
			if( PV < SV * 0.9f )  // pv under 90% of sv
			{
				pInst->Autotune.Stage = RELAY_ON_FOR_PREPARATION;
				pInst->Autotune.StageTime = tm_now;
			}
			break;
		case RELAY_ON_FOR_PREPARATION:
			pInst->MV = ( HEAT_CTRL_MODE == pCfg->CoolHeat ) ? fOutputMax : fOutputMin;
			if( PV >= SV )
			{
				pInst->Autotune.Stage = RELAY_OFF_FOR_PREPARATION;
				pInst->Autotune.StageTime = tm_now;
			}
			break;
		case RELAY_OFF_FOR_PREPARATION:
			pInst->MV = ( HEAT_CTRL_MODE == pCfg->CoolHeat ) ? fOutputMin : fOutputMax;
			if( PV < SV )
			{
				pInst->Autotune.Stage = RELAY_ON_FOR_PREPARE_TO_FIND_DELTA;
				pInst->Autotune.StageTime = tm_now;
			}
			break;
		case RELAY_ON_FOR_PREPARE_TO_FIND_DELTA:
			pInst->MV = ( HEAT_CTRL_MODE == pCfg->CoolHeat ) ? fOutputMax : fOutputMin;
			if( PV >= SV )
			{
				pInst->Autotune.Max = MINIMUM_PV;
				pInst->Autotune.Min = MAXIMUM_PV;
				pInst->Autotune.StartTime = AppTimer.GetCurrentSecs();
				pInst->Autotune.Stage = FIND_PV_HIGH;
				pInst->Autotune.StageTime = tm_now;
			}
			break;
		case FIND_PV_HIGH:
			pInst->MV = ( HEAT_CTRL_MODE == pCfg->CoolHeat ) ? fOutputMin : fOutputMax;

			if( pInst->Autotune.Max < pInst->PV )
			{
				pInst->Autotune.Max = pInst->PV;
			}
			// if( pInst->Autotune.Min > pInst->PV )
			// {
			// 	pInst->Autotune.Min = pInst->PV;
			// }

			if( PV < SV )
			{
				pInst->Autotune.Stage = FIND_PV_LOW;
				pInst->Autotune.StageTime = tm_now;
			}
			break;
		case FIND_PV_LOW:
			pInst->MV = ( HEAT_CTRL_MODE == pCfg->CoolHeat ) ? fOutputMax : fOutputMin;
			// if( pInst->Autotune.Max < pInst->PV )
			// {
			// 	pInst->Autotune.Max = pInst->PV;
			// }
			if( pInst->Autotune.Min > pInst->PV )
			{
				pInst->Autotune.Min = pInst->PV;
			}

			if( PV >= SV )
			{
				float amp_mv = 0;
				float amp_pv = 0;
				pInst->Autotune.Pu = AppTimer.GetCurrentSecs() - pInst->Autotune.StartTime;
				amp_mv = ( fOutputMax - fOutputMin );
				amp_pv = ( pInst->Autotune.Max - pInst->Autotune.Min ) * MULTIPLY_mK_TO_K;
				if( fabsf( amp_pv ) > 0.01f )   // avoid division by zero
				{
					pInst->Autotune.Ku = ( 4.0f * amp_mv ) / ( (float)M_PI * amp_pv );
					pInst->Autotune.Stage = AT_CALCULATION;
					pInst->Autotune.StageTime = tm_now;
				}
				else
				{
					pInst->Autotune.Stage = AT_STOPPED;
					pInst->FaultStatus.AutotuneError = 1;
				}
			}
			break;
		case AT_CALCULATION:
			pInst->MV = pCfg->OutputMin;

			float Kp = pInst->Autotune.Ku * 0.6f;
			float Pb = 100.0f / Kp;
			float Ti = pInst->Autotune.Pu * 0.5f;
			float Td = pInst->Autotune.Pu * 0.125f;

			if( Pb < 6550.0f && Ti < 6550.0f && Td < 655.00f )
			{
				pCfg->Pb = Pb * MULTIPLY_Pb_FLOAT_TO_INT;
				pCfg->Ti = Ti * MULTIPLY_Ti_FLOAT_TO_INT;
				pCfg->Td = Td * MULTIPLY_Td_FLOAT_TO_INT;
			}
			else
			{
				pInst->FaultStatus.AutotuneError = 1;
			}

			pCfg->AutoTuneEnabled = 0;
			pInst->Autotune.Stage = AT_STOPPED;
			pInst->IntegralSum = 0.0f;
			pInst->lastFilteredSV = SV * MULTIPLY_mK_TO_K;
			break;
	}
}

/******************************************************************************
 * @brief On-Off Controller. MV set a maximum value if PV is less then SV, and minimum value if PV is greater then SV.
 * 
 * @param pCfg 
 * @param pInst 
 * @return int16_t 
 *****************************************************************************/
static int16_t onoff_calculation( control_loop_config_st *pCfg, control_loop_instance_st *pInst )
{
	int16_t MV = 0;

	if( !pCfg || !pInst ) return 0;

	int32_t SV = pCfg->SV;
	int32_t PV = pInst->PV;

	if( SV > PV )
	{
		MV = ( pCfg->CoolHeat == HEAT_CTRL_MODE ) ? pCfg->OutputMax : pCfg->OutputMin;
	}
	else
	{
		MV = ( pCfg->CoolHeat == HEAT_CTRL_MODE ) ? pCfg->OutputMin : pCfg->OutputMax;
	}

	pInst->MV = MV;

	return MV;
}
