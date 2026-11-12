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

/* CTRL_PI_SV_RATE. Provisional value: replace after measuring the plant rise rate. */
#define SV_RATE_DEGC_PER_SEC 0.5f

/* CTRL_PI_GS, CTRL_PI_GS_ERR. Number of points in each schedule table. */
#define GS_POINT_COUNT 4

/* CTRL_PI_ADAPTIVE. Provisional values: replace after measuring the plant. */
#define ADAPT_WINDOW_SEC 5.0f           // One model update period. MV is averaged over the whole window
#define ADAPT_END_SAMPLES 10            // PV is averaged over the last samples of the window
#define ADAPT_HISTORY_WINDOWS 2         // Previous windows needed for one model update
#define ADAPT_BLOCK_WINDOWS 3           // A blocked window is part of this many model updates
#define ADAPT_FORGETTING 0.995f         // Recursive least squares forgetting factor
#define ADAPT_INIT_POLE 0.9f
#define ADAPT_INIT_COVARIANCE 100.0f
#define ADAPT_MAX_COVARIANCE 1.0e6f
#define ADAPT_MIN_DPV_DEGC 0.02f        // Below this and ADAPT_MIN_DMV, the window has no information
#define ADAPT_MIN_DMV 0.005f
#define ADAPT_MIN_POLE 0.5f
#define ADAPT_MAX_POLE 0.995f
#define ADAPT_MIN_PLANT_GAIN 1.0f       // degC / per-unit MV
#define ADAPT_LAMBDA_SEC 10.0f          // Target closed-loop time constant
#define ADAPT_DELAY_SEC 0.2f            // Command delay used for tuning
#define ADAPT_GAIN_SMOOTHING 0.1f       // Gain movement per window
#define ADAPT_INIT_KP 0.05f             // 5 %/degC
#define ADAPT_INIT_TI_SEC 12.0f
#define ADAPT_KP_MIN 0.01f              // 1 %/degC
#define ADAPT_KP_MAX 0.10f              // 10 %/degC. Low for the first tests, raise after checking the logs
#define ADAPT_TI_MIN_SEC 5.0f
#define ADAPT_TI_MAX_SEC 300.0f
#define ADAPT_VOUT_FREEZE_V 240.0f      // Below the 245 V and 275 V limits of the CHPP-8022 modes

/* CTRL_PI_SPLIT. Provisional values: replace after measuring the plant. */
#define SPLIT_COOL_GAIN 2.0f            // Gain multiplier on the cooling side
#define SPLIT_ZERO_MV 0.02f             // Half width of the blend between the cooling and heating gains
#define SPLIT_ZERO_ERR 0.2f             // Integral holds when |MV| and |error| are both inside their bands

/* CTRL_PI_I_SEP. Provisional value: replace after measuring the plant. */
#define I_SEP_BAND_DEGC 5.0f            // Integrate only within this error

/* CTRL_PI_S_CURVE. Provisional values: replace after measuring the plant. */
#define S_RATE_DEGC_PER_SEC 0.5f
#define S_ACCEL_DEGC_PER_SEC2 0.05f

/* CTRL_PI_FUZZY. Provisional values: replace after measuring the plant. */
#define FUZZY_SIZE_COUNT 3              // Error rows: small, medium, large
#define FUZZY_RATE_COUNT 3              // Rate columns: approaching, still, leaving
#define FUZZY_ERR_MID_DEGC 2.5f
#define FUZZY_ERR_LARGE_DEGC 5.0f
#define FUZZY_RATE_BAND_DEGC_PER_SEC 0.2f
#define FUZZY_PV_RATE_TAU_SEC 1.0f      // PV rate filter time constant

/*****************************************************************************/
/** DATATYPES ****************************************************************/
/*****************************************************************************/

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
	/* Log counters. They stay across disable so one log can cover several starts. */
	uint32_t ControlCalcSeq;
	uint32_t MvOverrideCount;
	uint32_t MvSignChangeCount;
	uint32_t EnableSeq;
	int8_t LastMvSign;
	bool MvOverridden;
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

typedef enum
{
	AW_NONE,						// 포화 처리 없음 (I += Ki_term)
	AW_BACK_CALCULATION,			// 6번에서 사용
	AW_CONDITIONAL_INTEGRATION,		// 5번에서 사용
	AW_HOLD,
} anti_windup_et;

typedef enum
{
	GS_TEMP_RISING,
	GS_TEMP_FALLING,
	GS_DIRECTION_COUNT,
} gs_direction_et;

typedef enum
{
	ADAPT_STATE_WARMUP,         // Not enough windows yet
	ADAPT_STATE_BLOCKED,        // Saturation, SMPS voltage limit or MV sign change
	ADAPT_STATE_NO_EXCITATION,  // PV and MV are not moving
	ADAPT_STATE_REJECTED,       // Estimate is outside the expected plant
	ADAPT_STATE_RESET,          // Estimator was reset
	ADAPT_STATE_UPDATED,        // Gains moved toward the new estimate
} adaptive_state_et;

typedef struct
{
	// Gains in use
	float Kp, Ti;

	// One window
	float SumY, SumU;
	uint16_t Samples, EndSamples;
	bool WindowBlocked;

	// Previous windows
	float Y1, Y2, U1;
	uint8_t Count, BlockedWindows;

	// Recursive least squares: dy[k] = A * dy[k-1] + B * du[k]
	float A, B, P00, P01, P11;

	// Result
	float EstGain, EstTau;
	uint16_t State;
} adaptive_st;

typedef struct {
	float Kp, Ti, Td, alpha, lambda, Ts;
	float u_min, u_max;

	// Setpoint filter
	float b0, b1, c0, c1;
	float SP_prev, SP_ref_prev;

	// PI integrator
	float I;

	// Derivative filter
	float p0, b;
	float v_prev, PV_prev;

	// Anti-windup gain
	float Kaw;

	// First calculation after enable has not run yet
	bool Started;

	// CTRL_PI_GS
	bool GsValid;
	float GsSp, GsKpMult, GsTiMult;

	// CTRL_PI_ADAPTIVE
	adaptive_st Ad;

	// CTRL_PI_SPLIT: last controller output, heating positive
	float HeatMvPrev;

	// CTRL_PI_S_CURVE: speed of the internal setpoint (degC/s)
	float SvRate;

	// CTRL_PI_FUZZY: filtered PV rate (degC/s)
	float PvRate;

	// Values of the last calculation, for logging
	float AppliedKp, AppliedTi;
	float SvRef, PTerm, ITerm, DTerm, MvCalc;
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
static void control_loop_clear_alarm( uint16_t clear_bitmask_hi, uint16_t clear_bitmask_lo );
static __RAM_FUNC int32_t get_temperature( control_loop_config_st *pCfg, int loop_idx );                        // for fast run
static __RAM_FUNC int16_t pid_calculation( control_loop_config_st *pCfg, control_loop_instance_st *pInst );     // for fast run
static __RAM_FUNC int16_t onoff_calculation( control_loop_config_st *pCfg, control_loop_instance_st *pInst );   // for fast run
static void autotune_relay_method( control_loop_config_st *pCfg, control_loop_instance_st *pInst );


static __RAM_FUNC void PID_instance_Init( pid_st *pPid, control_loop_config_st *pCfg );
static __RAM_FUNC void PID_instance_UpdateGains( pid_st *pPid, control_loop_config_st const *pCfg );
static __RAM_FUNC float PID2DOF_Calculate_backward_euler( pid_st *pPID, float sp, float pv, cool_heat_mode_et cool_heat );
static __RAM_FUNC float PID_core( pid_st *pPID, float sp, float pv, cool_heat_mode_et cool_heat, float Kp, float Ti, anti_windup_et aw );
static __RAM_FUNC void PID_instance_Start( pid_st *pPid, float pv );
static __RAM_FUNC float PID_production_calculate( pid_st *pPid, control_loop_config_st const *pCfg, control_loop_instance_st *pInst, float dt, float sp, float pv );
static __RAM_FUNC float PI_sv_rate_calculate( pid_st *pPid, float sp, float pv, cool_heat_mode_et cool_heat );
static __RAM_FUNC float gs_interpolate( float const points[ GS_POINT_COUNT ], float const table[ GS_POINT_COUNT ], float x );
static __RAM_FUNC float PI_gain_scheduled_calculate( pid_st *pPid, float sp, float pv, cool_heat_mode_et cool_heat );
static __RAM_FUNC float PI_error_scheduled_calculate( pid_st *pPid, float sp, float pv, cool_heat_mode_et cool_heat );
static __RAM_FUNC float linear_interpolate( float const *points, float const *values, int count, float x );
static __RAM_FUNC float PI_split_calculate( pid_st *pPid, float sp, float pv, cool_heat_mode_et cool_heat );
static __RAM_FUNC float PI_integral_separation_calculate( pid_st *pPid, float sp, float pv, cool_heat_mode_et cool_heat );
static __RAM_FUNC float PI_s_curve_calculate( pid_st *pPid, float sp, float pv, cool_heat_mode_et cool_heat );
static __RAM_FUNC void fuzzy_gain_multipliers( float error, float pv_rate, float *kp_mult, float *ki_mult );
static __RAM_FUNC float PI_fuzzy_calculate( pid_st *pPid, float sp, float pv, cool_heat_mode_et cool_heat );
static __RAM_FUNC bool adaptive_smps_voltage_limited( control_loop_config_st const *pCfg );
static __RAM_FUNC void adaptive_update( pid_st *pPid, float y, float u, bool blocked, float action );
static __RAM_FUNC float PI_adaptive_calculate( pid_st *pPid, control_loop_config_st const *pCfg, float sp, float pv );
static __RAM_FUNC void adaptive_reset_estimator( adaptive_st *ad );
static void control_loop_get_diag( int loop_idx, controller_diag_st *diag );
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
    .ClearAlarm = control_loop_clear_alarm,
    .SetDefault = set_default_config,
    .GetDiag = control_loop_get_diag,
};

/*****************************************************************************/
/** LOCAL VARIABLES **********************************************************/
/*****************************************************************************/

static CCMRAM pid_st pid_inst[ MAX_CONTROL_LOOP ];

/*
 * CTRL_PI_GS multiplier tables. Provisional values, not measured: Kp is raised for
 * a move away from room temperature and lowered for a move toward it.
 * Ti stays 1.0 until the plant time constant is measured at each SV.
 */
static const float gs_sv_points[ GS_POINT_COUNT ] = { 10.0f, 30.0f, 50.0f, 70.0f };
static const float gs_kp_mult[ GS_DIRECTION_COUNT ][ GS_POINT_COUNT ] =
{
	[ GS_TEMP_RISING ]  = { 0.8f, 1.0f, 1.0f, 1.2f },
	[ GS_TEMP_FALLING ] = { 1.2f, 1.0f, 1.0f, 0.8f },
};
static const float gs_ti_mult[ GS_DIRECTION_COUNT ][ GS_POINT_COUNT ] =
{
	[ GS_TEMP_RISING ]  = { 1.0f, 1.0f, 1.0f, 1.0f },
	[ GS_TEMP_FALLING ] = { 1.0f, 1.0f, 1.0f, 1.0f },
};

/*
 * CTRL_PI_GS_ERR multiplier table by |SV - PV| (degC). Provisional values, not measured:
 * gain is lowered near the target. Both directions start with the same values.
 */
static const float gs_err_points[ GS_POINT_COUNT ] = { 0.0f, 1.0f, 3.0f, 5.0f };
static const float gs_err_mult[ GS_DIRECTION_COUNT ][ GS_POINT_COUNT ] =
{
	[ GS_TEMP_RISING ]  = { 0.5f, 0.5f, 0.75f, 1.0f },
	[ GS_TEMP_FALLING ] = { 0.5f, 0.5f, 0.75f, 1.0f },
};

/* CTRL_PI_SPLIT gain multiplier by the last heating-positive output. */
static const float split_mv_points[ 2 ] = { -SPLIT_ZERO_MV, SPLIT_ZERO_MV };
static const float split_kp_mult[ 2 ] = { SPLIT_COOL_GAIN, 1.0f };

/* CTRL_PI_FUZZY membership functions. Provisional values, not measured. */
static const float fuzzy_err_small_points[ 2 ] = { 0.0f, FUZZY_ERR_MID_DEGC };
static const float fuzzy_err_medium_points[ 3 ] = { 0.0f, FUZZY_ERR_MID_DEGC, FUZZY_ERR_LARGE_DEGC };
static const float fuzzy_err_large_points[ 2 ] = { FUZZY_ERR_MID_DEGC, FUZZY_ERR_LARGE_DEGC };
static const float fuzzy_rate_approach_points[ 2 ] = { 0.0f, FUZZY_RATE_BAND_DEGC_PER_SEC };
static const float fuzzy_rate_still_points[ 3 ] = { -FUZZY_RATE_BAND_DEGC_PER_SEC, 0.0f, FUZZY_RATE_BAND_DEGC_PER_SEC };
static const float fuzzy_rate_leave_points[ 2 ] = { -FUZZY_RATE_BAND_DEGC_PER_SEC, 0.0f };
static const float fuzzy_member_falling[ 2 ] = { 1.0f, 0.0f };
static const float fuzzy_member_rising[ 2 ] = { 0.0f, 1.0f };
static const float fuzzy_member_peak[ 3 ] = { 0.0f, 1.0f, 0.0f };

/* CTRL_PI_FUZZY rule tables. Provisional values, not measured. Columns: approaching, still, leaving. */
static const float fuzzy_kp_mult[ FUZZY_SIZE_COUNT ][ FUZZY_RATE_COUNT ] =
{
	{ 0.5f, 0.7f, 1.0f },	// small error
	{ 0.8f, 1.0f, 1.1f },	// medium error
	{ 1.0f, 1.0f, 1.2f },	// large error
};
static const float fuzzy_ki_mult[ FUZZY_SIZE_COUNT ][ FUZZY_RATE_COUNT ] =
{
	{ 0.5f, 1.0f, 1.2f },	// small error
	{ 0.5f, 0.8f, 1.0f },	// medium error
	{ 0.5f, 0.5f, 0.8f },	// large error
};

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
				pInst->EnableSeq++;
				pInst->MvOverrideCount++;
				pInst->MvOverridden = true;
			}
			else if( AppTimer.IsExpired( &pInst->timerStartDelay ) )
			{
				switch( pCfg->ControlType )
				{
					case CTRL_PID:
					case CTRL_2DOF_PID:
					case CTRL_PID_PROD:
					case CTRL_PI_SV_RATE:
					case CTRL_PI_S_CURVE:
					case CTRL_PI_COND_I:
					case CTRL_PI_BACK_CALC:
					case CTRL_PI_I_SEP:
					case CTRL_PI_GS:
					case CTRL_PI_GS_ERR:
					case CTRL_PI_ADAPTIVE:
					case CTRL_PI_SPLIT:
					case CTRL_PI_FUZZY:
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
					pInst->MvOverrideCount++;
					pInst->MvOverridden = true;
				}
				if( !AppTimer.IsExpired( &pInst->timerOutputDelay ) )
				{
					pInst->MV = pCfg->OutputMin * MULTIPLY_PERMIL_TO_FLOAT;
					pInst->lastError = 0;
					pInst->MvOverrideCount++;
					pInst->MvOverridden = true;
				}
			}

			/* MV 0 keeps the SMPS polarity, so only a nonzero sign counts. */
			if( 0.0f != pInst->MV )
			{
				int8_t mv_sign = ( pInst->MV > 0.0f ) ? 1 : -1;

				if( ( 0 != pInst->LastMvSign ) && ( mv_sign != pInst->LastMvSign ) )
				{
					pInst->MvSignChangeCount++;
				}
				pInst->LastMvSign = mv_sign;
			}
		}
		else
		{
			pInst->LastMvSign = 0;
			pInst->MvOverridden = false;
			PID_instance_Init( &pid_inst[ loop_idx ], pCfg );
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
	pCfg->ControlType = CTRL_PID_PROD;      // default: PID_PROD  / Range: 0 ~ 14 ( control_et )
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
			pid_st *pPid = &pid_inst[ pInst->CH ];
			float sp = pCfg->SV * MULTIPLY_mK_TO_K;
			float pv = pInst->PV * MULTIPLY_mK_TO_K;

			/* CTRL_PI_ADAPTIVE decides its own gains, so PLC gains are not applied to it. */
			if( CTRL_PI_ADAPTIVE != pCfg->ControlType )
			{
				/* Apply PB/Ti/Td written by the PLC without requiring CL Disable. */
				PID_instance_UpdateGains( pPid, pCfg );
			}

			PID_instance_Start( pPid, pv );

			switch( pCfg->ControlType )
			{
				case CTRL_PID_PROD:			// 0: 양산 PID (GitLab chpc-tcm1 R00의 계산식)
					pInst->MV = PID_production_calculate( pPid, pCfg, pInst, dt, sp, pv );
					break;
				case CTRL_PID:				// 1: 일반 PID
					pInst->MV = PID_core( pPid, sp, pv, pCfg->CoolHeat, pPid->Kp, pPid->Ti, AW_NONE );
					break;
				case CTRL_PI_SV_RATE:		// 3: SV 변화율 제한 PI (내부 SV를 일정 속도로 이동)
					pInst->MV = PI_sv_rate_calculate( pPid, sp, pv, pCfg->CoolHeat );
					break;
				case CTRL_PI_S_CURVE:		// 4: S자 SV 프로파일 PI (내부 SV를 가감속하며 이동)
					pInst->MV = PI_s_curve_calculate( pPid, sp, pv, pCfg->CoolHeat );
					break;
				case CTRL_PI_COND_I:		// 5: 조건부 적분 PI (포화를 키우는 방향이면 적분 정지)
					pInst->MV = PID_core( pPid, sp, pv, pCfg->CoolHeat, pPid->Kp, pPid->Ti, AW_CONDITIONAL_INTEGRATION );
					break;
				case CTRL_PI_BACK_CALC:		// 6: 역계산 PI (포화 시 잘린 양에 비례해 적분값을 되돌림)
					pInst->MV = PID_core( pPid, sp, pv, pCfg->CoolHeat, pPid->Kp, pPid->Ti, AW_BACK_CALCULATION );
					break;
				case CTRL_PI_I_SEP:			// 7: 적분 분리 PI (오차가 크면 적분 정지)
					pInst->MV = PI_integral_separation_calculate( pPid, sp, pv, pCfg->CoolHeat );
					break;
				case CTRL_PI_GS:			// 8: 게인 스케줄링 PI (SV와 승온/냉각 방향별 게인 배율)
					pInst->MV = PI_gain_scheduled_calculate( pPid, sp, pv, pCfg->CoolHeat );
					break;
				case CTRL_PI_GS_ERR:		// 9: 오차 구간 게인 스케줄링 PI (|SV - PV|와 승온/냉각 방향별 게인 배율)
					pInst->MV = PI_error_scheduled_calculate( pPid, sp, pv, pCfg->CoolHeat );
					break;
				case CTRL_PI_ADAPTIVE:		// 10: 적응형 PI (플랜트를 추정해 Kp, Ti를 직접 결정)
					pInst->MV = PI_adaptive_calculate( pPid, pCfg, sp, pv );
					break;
				case CTRL_PI_SPLIT:			// 11: 가열/냉각 분리 PI (직전 출력 방향별 게인 배율, 0 부근 적분 정지)
					pInst->MV = PI_split_calculate( pPid, sp, pv, pCfg->CoolHeat );
					break;
				case CTRL_PI_FUZZY:			// 12: 퍼지 게인 PI (오차와 PV 속도별 Kp, Ki 배율)
					pInst->MV = PI_fuzzy_calculate( pPid, sp, pv, pCfg->CoolHeat );
					break;
				case CTRL_2DOF_PID:			// 2: 2자유도 PID (SV 필터 적용)
				default:
					pInst->MV = PID2DOF_Calculate_backward_euler( pPid, sp, pv, pCfg->CoolHeat );
					break;
			}
			pInst->ControlCalcSeq++;
			pInst->MvOverridden = false;
			retval = 1;
		}
	}
	return retval;
}

/******************************************************************************
 * @brief 2DOF PID Controller Calculation (Backward Euler Discretization)
 *
 * @param pPID Pointer to pid_st structure
 * @param sp Setpoint Value (SP)
 * @param pv Process Value (PV)
 * @return Manipulated Value (MV)
 ******************************************************************************/
static __RAM_FUNC float PID2DOF_Calculate_backward_euler( pid_st *pPID, float sp, float pv, cool_heat_mode_et cool_heat )
{
	/****************************************************************************
	       +-------------+               +-----------------+
	SV --> |  Filtering  | --> (+) ----> |  P + I element  | ---> (+) --> MV
	       | using alpha |      ^        +-----------------+       ^
	       +-------------+      |                                  |
	                            |    +-------------------------+   |
	PV -------------------------+--> | Preceding               | --+
	                                 | Derivative-type element |
	                                 +-------------------------+

	SV filter : (1+(1-alpha)*Ti*s)/(1+Ti*s)
	P element : Kp
	I element : Kp/(Ti*s)
	Preceding D element : (Kp*Td*s)/(1+lambda*Td*s)

	# Note: The input for the preceding derivative-type element should be PV, not SV-PV.
	# Typically, lambda values between 0.05 and 0.2 are considered appropriate.

	Reference. OMRON CJ series Instruction manual, 3-518 PID command
	****************************************************************************/

	// 1. 목표값 필터 (Target Value Filter)
	// 수식: Y(s)/U(s) = (1 + (1-alpha)Ti*s) / (1 + Ti*s)
	// 이산화(Backward Euler): out[k] = (Ti * out[k-1] + Ts * sp[k] + (1-alpha)*Ti * (sp[k] - sp[k-1])) / (Ti + Ts)

	float Ti = pPID->Ti;
	float Ts = pPID->Ts;
	float alpha = pPID->alpha;

	float SP_ref;

	// Ti가 0이 아닐 때만 계산 (0으로 나누기 방지)
	if( Ti > 0.0f )
	{
		float numerator = ( Ti * pPID->SP_ref_prev ) + ( Ts * sp ) + ( ( 1.0f - alpha ) * Ti * ( sp - pPID->SP_prev ) );
		SP_ref = numerator / ( Ti + Ts );
	}
	else
	{
		SP_ref = sp; // Ti가 0이면 필터 없이 통과
	}

	float mv = PID_core( pPID, SP_ref, pv, cool_heat, pPID->Kp, Ti, AW_NONE );

	// 목표값 필터의 다음 스텝을 위해 현재 상태 저장
	pPID->SP_ref_prev = SP_ref;
	pPID->SP_prev = sp;

	return mv;
}

/******************************************************************************
 * @brief Common PI(D) calculation with setpoint weighting already applied.
 *
 * @param pPID Pointer to pid_st structure
 * @param sp Setpoint used for the error (after any filter or rate limit)
 * @param pv Process Value (PV)
 * @param cool_heat Control direction
 * @param Kp Proportional gain applied in this calculation
 * @param Ti Integral time applied in this calculation (sec)
 * @param aw Anti-windup method
 * @return Manipulated Value (MV)
 ******************************************************************************/
static __RAM_FUNC float PID_core( pid_st *pPID, float sp, float pv, cool_heat_mode_et cool_heat, float Kp, float Ti, anti_windup_et aw )
{
	float Ts = pPID->Ts;

	// 2. 오차 계산 (SV - 현재 PV)
	float action = ( COOL_CTRL_MODE == cool_heat ) ? -1.0f : 1.0f;
	float error = action * ( sp - pv );

	// 3. 비례항 (Proportional)
	float P_term = Kp * error;

	// 4. 선행 미분항 (Derivative on PV)
	// e[k] - e[k-1]이 아닌 PV[k] - PV[k-1]을 이용함
	// 수식: Y(s)/U(s) = (Kp * Td * s) / (1 + lambda * Td * s), 입력 U는 PV
	// 이산화: out[k] = (lambda*Td * out[k-1] + Kp*Td * (pv[k] - pv[k-1])) / (Ts + lambda*Td)

	float D_term = 0.0f;
	float Td = pPID->Td;
	float lambda = pPID->lambda;

	if( Td > 0.0f )
	{
		float denominator = Ts + ( lambda * Td );
		if( ( denominator < 0.0f ) || ( 0.0f < denominator ) )
		{
			float num_d = ( lambda * Td * pPID->v_prev ) + ( Kp * Td * ( pv - pPID->PV_prev ) );
			D_term = num_d / denominator;
		}
	}

	// 5. 계산된 조작량 (Calculated MV) - 아직 제한 전
	// 주의: 적분항은 '이전 스텝까지 누적된 값(I)'을 사용합니다.
	const float I_before = pPID->I;
	float mv_calc = P_term + I_before - ( action * D_term );

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

	// 7. 적분항 업데이트. 방식은 aw 인자로 정한다.
	float Ki_term = 0.0f;
	if( Ti > 0.0f )
	{
		Ki_term = ( Kp / Ti ) * error * Ts;
	}

	if( AW_HOLD == aw )
	{
		// 이번 주기에는 적분을 갱신하지 않는다 (Ki 항, 역계산 항 모두).
	}
	else if( AW_NONE == aw )
	{
		// 포화 여부와 상관없이 오차만 적분한다.
		pPID->I += Ki_term;
	}
	else if( AW_CONDITIONAL_INTEGRATION == aw )
	{
		// 포화 방향으로 오차가 더 밀어 올리는 동안에는 적분을 멈춘다.
		bool push_over_max = ( mv_calc > pPID->u_max ) && ( error > 0.0f );
		bool push_under_min = ( mv_calc < pPID->u_min ) && ( error < 0.0f );

		if( !push_over_max && !push_under_min )
		{
			pPID->I += Ki_term;
		}
	}
	else
	{
		// 역계산 anti-windup: MV가 상하한에 잘린 양에 비례해 적분값을 되돌린다. 안 잘리면 일반 적분과 같다.
		float saturation_diff = mv_final - mv_calc;
		float Kaw = ( Ti > 0.0f ) ? ( 1.0f / Ti ) : 0.0f;
		float back_calc_term = Kaw * saturation_diff * Ts;

		pPID->I += Ki_term + back_calc_term;
	}

	// 8. 로그용 값 저장. 적분을 갱신하기 전 값으로 mv_calc = P + I + D가 되게 한다.
	pPID->SvRef = sp;
	pPID->PTerm = P_term;
	pPID->ITerm = I_before;
	pPID->DTerm = -( action * D_term );
	pPID->MvCalc = mv_calc;
	pPID->AppliedKp = Kp;
	pPID->AppliedTi = Ti;

	// 9. 다음 스텝을 위해 현재 상태 저장
	pPID->PV_prev = pv;
	pPID->v_prev = D_term;

	return mv_final;
}

/******************************************************************************
 * @brief Set controller history from the first PV after the loop is enabled.
 *
 * A setpoint filter that starts from 0 degC treats the first calculation as a
 * step from 0 degC, so all history starts from the current PV instead.
 *
 * @param pPid Pointer to pid_st structure
 * @param pv Process Value (PV)
 ******************************************************************************/
static __RAM_FUNC void PID_instance_Start( pid_st *pPid, float pv )
{
	if( pPid->Started ) return;

	pPid->Started = true;
	pPid->HeatMvPrev = 0.0f;
	pPid->SvRate = 0.0f;
	pPid->PvRate = 0.0f;
	pPid->SP_prev = pv;
	pPid->SP_ref_prev = pv;
	pPid->PV_prev = pv;
	pPid->v_prev = 0.0f;
}

/******************************************************************************
 * @brief Production PID. Formula of pid_calculation() in GitLab chpc-tcm1 tag R00
 * (the "#if 1" branch), with two differences:
 * - Kp uses the same PB unit as the other controllers.
 * - With Ti = 0 the second integral step is 0. R00 divides by Ti there.
 *
 * As in R00, the integral step is added twice per cycle while MV is inside the
 * limits, and once per cycle while MV is limited (no anti-windup). Saturated_I
 * limits only the first step.
 *
 * @param pPid Pointer to pid_st structure (diagnostics only)
 * @param pCfg Pointer to control_loop_config_st structure
 * @param pInst Pointer to control_loop_instance_st structure
 * @param dt Control period (sec)
 * @param sp Setpoint Value (SP)
 * @param pv Process Value (PV)
 * @return Manipulated Value (MV)
 ******************************************************************************/
static __RAM_FUNC float PID_production_calculate( pid_st *pPid, control_loop_config_st const *pCfg, control_loop_instance_st *pInst, float dt, float sp, float pv )
{
	const float Kp = pCfg->Pb ? ( 100.0f / ( pCfg->Pb * MULTIPLY_Pb_INT_TO_FLOAT ) ) : 0.0f;
	const float Ti = pCfg->Ti * MULTIPLY_Ti_INT_TO_FLOAT;
	const float Td = pCfg->Td * MULTIPLY_Td_INT_TO_FLOAT;
	const float fOutputMax = pCfg->OutputMax * MULTIPLY_PERMIL_TO_FLOAT;
	const float fOutputMin = pCfg->OutputMin * MULTIPLY_PERMIL_TO_FLOAT;
	float fMV;

	float error = sp - pv;

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

	float proportion = error;
	float derivate = ( error - pInst->lastError ) / dt * Td;
	float integral_temp = ( Ti > 0.0f ) ? ( Kp * error * dt / Ti ) : 0.0f;

	fMV = Kp * ( proportion + derivate ) + pInst->IntegralSum + integral_temp;

	pPid->SvRef = sp;
	pPid->PTerm = Kp * proportion;
	pPid->ITerm = pInst->IntegralSum + integral_temp;
	pPid->DTerm = Kp * derivate;
	pPid->MvCalc = fMV;

	if( isfinite( fMV ) )
	{
		// MV Limitation. As in R00, the integral keeps the first step while MV is limited.
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
		else if( signbit( fMV ) > 0 )
		{
			fMV = fOutputMax;
		}
		else if( signbit( fMV ) < 0 )
		{
			fMV = fOutputMin;
		}
	}

	pInst->lastError = error;
	pPid->AppliedKp = Kp;
	pPid->AppliedTi = Ti;

	return fMV;
}

/******************************************************************************
 * @brief PI with a setpoint that moves toward SV at a fixed rate.
 *
 * @param pPid Pointer to pid_st structure
 * @param sp Setpoint Value (SP)
 * @param pv Process Value (PV)
 * @param cool_heat Control direction
 * @return Manipulated Value (MV)
 ******************************************************************************/
static __RAM_FUNC float PI_sv_rate_calculate( pid_st *pPid, float sp, float pv, cool_heat_mode_et cool_heat )
{
	const float step = SV_RATE_DEGC_PER_SEC * pPid->Ts;
	float sp_ref = pPid->SP_ref_prev;

	if( sp > sp_ref )
	{
		sp_ref = fminf( sp_ref + step, sp );
	}
	else
	{
		sp_ref = fmaxf( sp_ref - step, sp );
	}

	pPid->SP_ref_prev = sp_ref;

	return PID_core( pPid, sp_ref, pv, cool_heat, pPid->Kp, pPid->Ti, AW_NONE );
}

/******************************************************************************
 * @brief Linear interpolation over GS_POINT_COUNT points.
 *
 * @param points Input value of each point, in increasing order
 * @param table Value at each point
 * @param x Input value
 * @return Interpolated value. Clamped to the first and last points.
 ******************************************************************************/
static __RAM_FUNC float gs_interpolate( float const points[ GS_POINT_COUNT ], float const table[ GS_POINT_COUNT ], float x )
{
	if( x <= points[ 0 ] ) return table[ 0 ];

	for( int idx = 1; idx < GS_POINT_COUNT; ++idx )
	{
		if( x <= points[ idx ] )
		{
			float ratio = ( x - points[ idx - 1 ] ) / ( points[ idx ] - points[ idx - 1 ] );

			return table[ idx - 1 ] + ratio * ( table[ idx ] - table[ idx - 1 ] );
		}
	}

	return table[ GS_POINT_COUNT - 1 ];
}

/******************************************************************************
 * @brief Gain-scheduled PI. PLC gains are multiplied by a table selected by SV
 * and by direction. The multipliers change only when SV changes, so the gains
 * stay fixed during one response.
 *
 * @param pPid Pointer to pid_st structure
 * @param sp Setpoint Value (SP)
 * @param pv Process Value (PV)
 * @param cool_heat Control direction
 * @return Manipulated Value (MV)
 ******************************************************************************/
static __RAM_FUNC float PI_gain_scheduled_calculate( pid_st *pPid, float sp, float pv, cool_heat_mode_et cool_heat )
{
	if( !pPid->GsValid || ( pPid->GsSp != sp ) )
	{
		int direction = ( sp > pv ) ? GS_TEMP_RISING : GS_TEMP_FALLING;

		pPid->GsKpMult = gs_interpolate( gs_sv_points, gs_kp_mult[ direction ], sp );
		pPid->GsTiMult = gs_interpolate( gs_sv_points, gs_ti_mult[ direction ], sp );
		pPid->GsSp = sp;
		pPid->GsValid = true;
	}

	return PID_core( pPid, sp, pv, cool_heat, pPid->Kp * pPid->GsKpMult, pPid->Ti * pPid->GsTiMult, AW_NONE );
}

/******************************************************************************
 * @brief PI with gains scheduled by SV - PV. The multiplier is recalculated every
 * cycle. Kp and Ti get the same multiplier, so Kp / Ti (integral speed) stays the
 * same and only the P term changes with the error.
 *
 * @param pPid Pointer to pid_st structure
 * @param sp Setpoint Value (SP)
 * @param pv Process Value (PV)
 * @param cool_heat Control direction
 * @return Manipulated Value (MV)
 ******************************************************************************/
static __RAM_FUNC float PI_error_scheduled_calculate( pid_st *pPid, float sp, float pv, cool_heat_mode_et cool_heat )
{
	const float diff = sp - pv;
	const int direction = ( diff > 0.0f ) ? GS_TEMP_RISING : GS_TEMP_FALLING;
	const float mult = gs_interpolate( gs_err_points, gs_err_mult[ direction ], fabsf( diff ) );

	return PID_core( pPid, sp, pv, cool_heat, pPid->Kp * mult, pPid->Ti * mult, AW_NONE );
}

/******************************************************************************
 * @brief Linear interpolation over any number of points.
 *
 * @param points Input value of each point, in increasing order
 * @param values Value at each point
 * @param count Number of points
 * @param x Input value
 * @return Interpolated value. Clamped to the first and last points.
 ******************************************************************************/
static __RAM_FUNC float linear_interpolate( float const *points, float const *values, int count, float x )
{
	if( x <= points[ 0 ] ) return values[ 0 ];

	for( int idx = 1; idx < count; ++idx )
	{
		if( x <= points[ idx ] )
		{
			float ratio = ( x - points[ idx - 1 ] ) / ( points[ idx ] - points[ idx - 1 ] );

			return values[ idx - 1 ] + ratio * ( values[ idx ] - values[ idx - 1 ] );
		}
	}

	return values[ count - 1 ];
}

/******************************************************************************
 * @brief PI with separate cooling and heating gains. The multiplier follows the
 * last heating-positive output. The integral holds near zero output and error.
 *
 * HeatMvPrev comes from the PID_core result, so a start or output delay that
 * overwrites pInst->MV does not change it.
 *
 * @param pPid Pointer to pid_st structure
 * @param sp Setpoint Value (SP)
 * @param pv Process Value (PV)
 * @param cool_heat Control direction
 * @return Manipulated Value (MV)
 ******************************************************************************/
static __RAM_FUNC float PI_split_calculate( pid_st *pPid, float sp, float pv, cool_heat_mode_et cool_heat )
{
	const float action = ( COOL_CTRL_MODE == cool_heat ) ? -1.0f : 1.0f;
	const float error = sp - pv;
	const float mult = linear_interpolate( split_mv_points, split_kp_mult, 2, pPid->HeatMvPrev );
	const anti_windup_et aw = ( ( fabsf( pPid->HeatMvPrev ) < SPLIT_ZERO_MV ) && ( fabsf( error ) < SPLIT_ZERO_ERR ) ) ? AW_HOLD : AW_NONE;
	const float mv = PID_core( pPid, sp, pv, cool_heat, pPid->Kp * mult, pPid->Ti, aw );

	pPid->HeatMvPrev = action * mv;

	return mv;
}

/******************************************************************************
 * @brief PI that holds the accumulated integral while the error is large.
 *
 * @param pPid Pointer to pid_st structure
 * @param sp Setpoint Value (SP)
 * @param pv Process Value (PV)
 * @param cool_heat Control direction
 * @return Manipulated Value (MV)
 ******************************************************************************/
static __RAM_FUNC float PI_integral_separation_calculate( pid_st *pPid, float sp, float pv, cool_heat_mode_et cool_heat )
{
	const anti_windup_et aw = ( fabsf( sp - pv ) > I_SEP_BAND_DEGC ) ? AW_HOLD : AW_NONE;

	return PID_core( pPid, sp, pv, cool_heat, pPid->Kp, pPid->Ti, aw );
}

/******************************************************************************
 * @brief PI with an internal setpoint that accelerates, cruises and decelerates
 * toward SV. The internal setpoint starts from the PV at enable.
 *
 * @param pPid Pointer to pid_st structure
 * @param sp Setpoint Value (SP)
 * @param pv Process Value (PV)
 * @param cool_heat Control direction
 * @return Manipulated Value (MV)
 ******************************************************************************/
static __RAM_FUNC float PI_s_curve_calculate( pid_st *pPid, float sp, float pv, cool_heat_mode_et cool_heat )
{
	const float max_step = S_ACCEL_DEGC_PER_SEC2 * pPid->Ts;
	float ref = pPid->SP_ref_prev;
	const float dist = sp - ref;
	const float target = copysignf( fminf( S_RATE_DEGC_PER_SEC, sqrtf( 2.0f * S_ACCEL_DEGC_PER_SEC2 * fabsf( dist ) ) ), dist );

	pPid->SvRate += fmaxf( -max_step, fminf( max_step, target - pPid->SvRate ) );
	ref += pPid->SvRate * pPid->Ts;

	if( ( sp - ref ) * dist <= 0.0f )
	{
		ref = sp;
		pPid->SvRate = 0.0f;
	}

	pPid->SP_ref_prev = ref;

	return PID_core( pPid, ref, pv, cool_heat, pPid->Kp, pPid->Ti, AW_NONE );
}

/******************************************************************************
 * @brief Weighted average of the fuzzy rule table.
 *
 * @param error Physical error, SV - PV (degC)
 * @param pv_rate Filtered PV rate (degC/s)
 * @param kp_mult Kp multiplier
 * @param ki_mult Ki multiplier
 ******************************************************************************/
static __RAM_FUNC void fuzzy_gain_multipliers( float error, float pv_rate, float *kp_mult, float *ki_mult )
{
	const float size = fabsf( error );
	const float rate = copysignf( 1.0f, error ) * pv_rate;	// rate > 0: moving toward SV
	const float size_member[ FUZZY_SIZE_COUNT ] =
	{
		linear_interpolate( fuzzy_err_small_points, fuzzy_member_falling, 2, size ),
		linear_interpolate( fuzzy_err_medium_points, fuzzy_member_peak, 3, size ),
		linear_interpolate( fuzzy_err_large_points, fuzzy_member_rising, 2, size ),
	};
	const float rate_member[ FUZZY_RATE_COUNT ] =
	{
		linear_interpolate( fuzzy_rate_approach_points, fuzzy_member_rising, 2, rate ),
		linear_interpolate( fuzzy_rate_still_points, fuzzy_member_peak, 3, rate ),
		linear_interpolate( fuzzy_rate_leave_points, fuzzy_member_falling, 2, rate ),
	};
	float total = 0.0f;
	float kp_sum = 0.0f;
	float ki_sum = 0.0f;

	for( int row = 0; row < FUZZY_SIZE_COUNT; ++row )
	{
		for( int col = 0; col < FUZZY_RATE_COUNT; ++col )
		{
			const float weight = size_member[ row ] * rate_member[ col ];

			total += weight;
			kp_sum += weight * fuzzy_kp_mult[ row ][ col ];
			ki_sum += weight * fuzzy_ki_mult[ row ][ col ];
		}
	}

	*kp_mult = kp_sum / total;
	*ki_mult = ki_sum / total;
}

/******************************************************************************
 * @brief PI with Kp and Ki multipliers from a fuzzy rule table of the error size
 * and the PV rate toward SV.
 *
 * @param pPid Pointer to pid_st structure
 * @param sp Setpoint Value (SP)
 * @param pv Process Value (PV)
 * @param cool_heat Control direction
 * @return Manipulated Value (MV)
 ******************************************************************************/
static __RAM_FUNC float PI_fuzzy_calculate( pid_st *pPid, float sp, float pv, cool_heat_mode_et cool_heat )
{
	const float Ts = pPid->Ts;
	float kp_mult, ki_mult;

	/* PV_prev is updated inside PID_core, so the rate is calculated before the call. */
	pPid->PvRate += ( Ts / ( FUZZY_PV_RATE_TAU_SEC + Ts ) ) * ( ( ( pv - pPid->PV_prev ) / Ts ) - pPid->PvRate );

	fuzzy_gain_multipliers( sp - pv, pPid->PvRate, &kp_mult, &ki_mult );

	return PID_core( pPid, sp, pv, cool_heat, pPid->Kp * kp_mult, pPid->Ti * kp_mult / ki_mult, AW_NONE );
}

/******************************************************************************
 * @brief Check whether the SMPS output voltage is close to its limit.
 * Output no longer follows MV there, so the sample is not used for estimation.
 *
 * @param pCfg Pointer to control_loop_config_st structure
 * @return true when the voltage is at or above ADAPT_VOUT_FREEZE_V
 ******************************************************************************/
static __RAM_FUNC bool adaptive_smps_voltage_limited( control_loop_config_st const *pCfg )
{
	int8_t smps_id = pCfg->OutputChannel;

	if( ( OUT_SMPS_CHPP_8021 != pCfg->OutputType ) || ( smps_id < 0 ) || ( smps_id >= MAX_SMPS_ID ) ) return false;

	int16_t vout_q5 = ( int16_t )MBM_READ_DB[ smps_id ].Holdings[ MBM_HR_CHPP_8021_OUTPUT_VOLTAGE_MEASURED ];

	return fabsf( vout_q5 / 32.0f ) >= ADAPT_VOUT_FREEZE_V;
}

/******************************************************************************
 * @brief Update the plant estimate with one window of PV and MV.
 *
 * Model (velocity form, offset free): dy[k] = a * dy[k-1] + b * du[k]
 * y[k] is PV at the end of window k and u[k] is the MV average over window k,
 * because that MV is what moves PV from y[k-1] to y[k].
 * Plant gain K = b / (1 - a), time constant tau = T / (1 - a) - T / 2
 * Gains (lambda tuning): Kp = tau / ( K * ( lambda + delay ) ), Ti = tau
 *
 * @param pPid Pointer to pid_st structure
 * @param y PV at the end of the window (degC)
 * @param u Window average of MV (per-unit)
 * @param blocked The window contains saturation or SMPS voltage limit
 * @param action Control direction sign
 ******************************************************************************/
static __RAM_FUNC void adaptive_update( pid_st *pPid, float y, float u, bool blocked, float action )
{
	adaptive_st *ad = &pPid->Ad;

	/* A sign change restarts the SMPS power stage, so it is treated like saturation. */
	if( blocked || ( ( ad->Count > 0 ) && ( ( u * ad->U1 ) < 0.0f ) ) )
	{
		ad->BlockedWindows = ADAPT_BLOCK_WINDOWS;
	}

	float y2 = ad->Y2, y1 = ad->Y1, u1 = ad->U1;
	bool history_ok = ( ad->Count >= ADAPT_HISTORY_WINDOWS );

	ad->Y2 = y1;
	ad->Y1 = y;
	ad->U1 = u;
	if( ad->Count < ADAPT_HISTORY_WINDOWS ) ad->Count++;

	if( ad->BlockedWindows > 0 )
	{
		ad->BlockedWindows--;
		ad->State = ADAPT_STATE_BLOCKED;
		return;
	}
	if( !history_ok )
	{
		ad->State = ADAPT_STATE_WARMUP;
		return;
	}

	const float dy = y - y1;
	const float phi0 = y1 - y2;
	const float phi1 = u - u1;

	if( ( fabsf( phi0 ) < ADAPT_MIN_DPV_DEGC ) && ( fabsf( phi1 ) < ADAPT_MIN_DMV ) )
	{
		ad->State = ADAPT_STATE_NO_EXCITATION;
		return;
	}

	/* Recursive least squares with forgetting factor */
	const float Pphi0 = ad->P00 * phi0 + ad->P01 * phi1;
	const float Pphi1 = ad->P01 * phi0 + ad->P11 * phi1;
	const float denom = ADAPT_FORGETTING + phi0 * Pphi0 + phi1 * Pphi1;
	const float k0 = Pphi0 / denom;
	const float k1 = Pphi1 / denom;
	const float pred_err = dy - ( ad->A * phi0 + ad->B * phi1 );
	const float A_new = ad->A + k0 * pred_err;
	const float B_new = ad->B + k1 * pred_err;
	const float P00_new = ( ad->P00 - k0 * Pphi0 ) / ADAPT_FORGETTING;
	const float P01_new = ( ad->P01 - k0 * Pphi1 ) / ADAPT_FORGETTING;
	const float P11_new = ( ad->P11 - k1 * Pphi1 ) / ADAPT_FORGETTING;

	if( !isfinite( A_new ) || !isfinite( B_new ) || !isfinite( P00_new ) || !isfinite( P01_new ) || !isfinite( P11_new )
		|| ( P00_new <= 0.0f ) || ( P11_new <= 0.0f ) || ( ( P00_new + P11_new ) > ADAPT_MAX_COVARIANCE ) )
	{
		adaptive_reset_estimator( ad );
		ad->State = ADAPT_STATE_RESET;
		return;
	}

	ad->A = A_new;
	ad->B = B_new;
	ad->P00 = P00_new;
	ad->P01 = P01_new;
	ad->P11 = P11_new;

	/* Keep the current gains when the estimate does not describe a stable plant of the expected sign. */
	if( ( ad->A <= ADAPT_MIN_POLE ) || ( ad->A >= ADAPT_MAX_POLE ) )
	{
		ad->State = ADAPT_STATE_REJECTED;
		return;
	}

	const float plant_gain = action * ad->B / ( 1.0f - ad->A );

	if( plant_gain < ADAPT_MIN_PLANT_GAIN )
	{
		ad->State = ADAPT_STATE_REJECTED;
		return;
	}

	const float tau = ADAPT_WINDOW_SEC / ( 1.0f - ad->A ) - ( 0.5f * ADAPT_WINDOW_SEC );
	const float kp_target = fmaxf( ADAPT_KP_MIN, fminf( ADAPT_KP_MAX, tau / ( plant_gain * ( ADAPT_LAMBDA_SEC + ADAPT_DELAY_SEC ) ) ) );
	const float ti_target = fmaxf( ADAPT_TI_MIN_SEC, fminf( ADAPT_TI_MAX_SEC, tau ) );

	ad->Kp += ADAPT_GAIN_SMOOTHING * ( kp_target - ad->Kp );
	ad->Ti += ADAPT_GAIN_SMOOTHING * ( ti_target - ad->Ti );
	ad->EstGain = action * plant_gain;
	ad->EstTau = tau;
	ad->State = ADAPT_STATE_UPDATED;
}

/******************************************************************************
 * @brief Adaptive PI. Firmware decides Kp and Ti from an online plant estimate.
 * PB, Ti and Td from the PLC are not used.
 *
 * @param pPid Pointer to pid_st structure
 * @param pCfg Pointer to control_loop_config_st structure
 * @param sp Setpoint Value (SP)
 * @param pv Process Value (PV)
 * @return Manipulated Value (MV)
 ******************************************************************************/
static __RAM_FUNC float PI_adaptive_calculate( pid_st *pPid, control_loop_config_st const *pCfg, float sp, float pv )
{
	adaptive_st *ad = &pPid->Ad;
	const float action = ( COOL_CTRL_MODE == pCfg->CoolHeat ) ? -1.0f : 1.0f;

	const uint16_t window_samples = ( uint16_t )fmaxf( 1.0f, ( ADAPT_WINDOW_SEC / pPid->Ts ) + 0.5f );

	float mv = PID_core( pPid, sp, pv, pCfg->CoolHeat, ad->Kp, ad->Ti, AW_NONE );

	ad->SumU += mv;
	ad->Samples++;

	if( ( ad->Samples + ADAPT_END_SAMPLES ) > window_samples )
	{
		ad->SumY += pv;
		ad->EndSamples++;
	}

	if( ( mv >= pPid->u_max ) || ( mv <= pPid->u_min ) || adaptive_smps_voltage_limited( pCfg ) )
	{
		ad->WindowBlocked = true;
	}

	if( ad->Samples >= window_samples )
	{
		adaptive_update( pPid, ad->SumY / ad->EndSamples, ad->SumU / ad->Samples, ad->WindowBlocked, action );

		ad->SumY = 0.0f;
		ad->SumU = 0.0f;
		ad->Samples = 0;
		ad->EndSamples = 0;
		ad->WindowBlocked = false;
	}

	return mv;
}

/******************************************************************************
 * @brief Reset the plant estimate. Gains stay as they are.
 *
 * @param ad Pointer to adaptive_st structure
 ******************************************************************************/
static __RAM_FUNC void adaptive_reset_estimator( adaptive_st *ad )
{
	ad->A = ADAPT_INIT_POLE;
	ad->B = 0.0f;
	ad->P00 = ADAPT_INIT_COVARIANCE;
	ad->P01 = 0.0f;
	ad->P11 = ADAPT_INIT_COVARIANCE;
}

/******************************************************************************
 * @brief Report the values used by the running controller.
 *
 * @param loop_idx Index of control loop
 * @param diag Output values
 ******************************************************************************/
static void control_loop_get_diag( int loop_idx, controller_diag_st *diag )
{
	if( !diag || ( 0 > loop_idx ) || ( loop_idx >= MAX_CONTROL_LOOP ) ) return;

	pid_st const *pPid = &pid_inst[ loop_idx ];
	control_loop_config_st const *pCfg = &ControllerVars.Cfg[ loop_idx ];
	control_loop_instance_st const *pInst = &ControllerVars.Inst[ loop_idx ];
	uint16_t limit_flags = 0;

	if( pCfg->Enable && CFG.system.Run )
	{
		if( pInst->MV >= ( pCfg->OutputMax * MULTIPLY_PERMIL_TO_FLOAT ) ) limit_flags |= CTRL_LIMIT_MV_AT_MAX;
		if( pInst->MV <= ( pCfg->OutputMin * MULTIPLY_PERMIL_TO_FLOAT ) ) limit_flags |= CTRL_LIMIT_MV_AT_MIN;
	}
	if( adaptive_smps_voltage_limited( pCfg ) ) limit_flags |= CTRL_LIMIT_SMPS_VOLTAGE;
	if( pInst->MvOverridden ) limit_flags |= CTRL_LIMIT_MV_OVERRIDDEN;

	diag->AppliedKp = pPid->AppliedKp;
	diag->AppliedTi = pPid->AppliedTi;
	diag->EstGain = pPid->Ad.EstGain;
	diag->EstTau = pPid->Ad.EstTau;
	diag->EstState = pPid->Ad.State;
	diag->SvRef = pPid->SvRef;
	diag->PTerm = pPid->PTerm;
	diag->ITerm = pPid->ITerm;
	diag->DTerm = pPid->DTerm;
	diag->MvCalc = pPid->MvCalc;
	diag->LimitFlags = limit_flags;
	diag->ControlCalcSeq = pInst->ControlCalcSeq;
	diag->MvOverrideCount = pInst->MvOverrideCount;
	diag->MvSignChangeCount = pInst->MvSignChangeCount;
	diag->EnableSeq = pInst->EnableSeq;
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
	pPid->PV_prev = 0.0f;

	pPid->Started = false;
	pPid->HeatMvPrev = 0.0f;
	pPid->SvRate = 0.0f;
	pPid->PvRate = 0.0f;
	pPid->GsValid = false;
	pPid->GsSp = 0.0f;
	pPid->GsKpMult = 1.0f;
	pPid->GsTiMult = 1.0f;
	pPid->AppliedKp = 0.0f;
	pPid->AppliedTi = 0.0f;
	pPid->SvRef = 0.0f;
	pPid->PTerm = 0.0f;
	pPid->ITerm = 0.0f;
	pPid->DTerm = 0.0f;
	pPid->MvCalc = 0.0f;

	memset( &pPid->Ad, 0, sizeof( pPid->Ad ) );
	adaptive_reset_estimator( &pPid->Ad );
	pPid->Ad.Kp = ADAPT_INIT_KP;
	pPid->Ad.Ti = ADAPT_INIT_TI_SEC;
	pPid->Ad.State = ADAPT_STATE_WARMUP;
}

/******************************************************************************
 * @brief Apply changed PLC PID gains while preserving the running PID state.
 *
 * The current controller caches PB/Ti/Td as floating-point gains in pid_st.
 * R00 calculated them directly from the control configuration every cycle,
 * but the current implementation previously refreshed them only while the
 * loop was disabled.  Refresh only the gain-dependent values here so a PLC
 * tuning write takes effect on the next PID calculation without resetting the
 * setpoint/PV history.  When Ti is set to zero, clear the accumulated integral
 * because integral action has explicitly been disabled.
 ******************************************************************************/
static __RAM_FUNC void PID_instance_UpdateGains( pid_st *pPid, control_loop_config_st const *pCfg )
{
	if( !pPid || !pCfg ) return;

	const float kp = pCfg->Pb ? ( 100.0f / ( pCfg->Pb * MULTIPLY_Pb_INT_TO_FLOAT ) ) : 0.0f;
	const float ti = pCfg->Ti * MULTIPLY_Ti_INT_TO_FLOAT;
	const float td = pCfg->Td * MULTIPLY_Td_INT_TO_FLOAT;

	if( ( pPid->Kp == kp ) && ( pPid->Ti == ti ) && ( pPid->Td == td ) ) return;

	pPid->Kp = kp;
	pPid->Ti = ti;
	pPid->Td = td;
	pPid->Kaw = ( ti > 0.0f ) ? ( 1.0f / ti ) : 0.0f;

	if( ti <= 0.0f )
	{
		pPid->I = 0.0f;
	}

	/* Keep the derived values coherent for the alternate PID implementation. */
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
