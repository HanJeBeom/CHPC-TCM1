/******************************************************************************
 * @file d_pwm.c
 * @author jylee1 (jylee1@gst-in.com)
 * @brief 
 * @version 0.1
 * @date 2023-05-22
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
#include "UserDrivers.h"

/*****************************************************************************/
/** MACRO DEFINITIONS ********************************************************/
/*****************************************************************************/
#define EPSILON 1e-5f

/*****************************************************************************/
/** DATATYPES ****************************************************************/
/*****************************************************************************/
typedef struct PWM_out_ch_struct {
	TIM_HandleTypeDef *inst;
    uint16_t freq;
    float duty;
} pwm_out_ch_st;

/*****************************************************************************/
/** FUNCTION DECLARATIONS ****************************************************/
/*****************************************************************************/
static int8_t fcmp32( float a, float b, float epsilon );
static float pwm_get_freq( uint8_t ch );
static uint8_t pwm_set_freq( uint8_t ch, float freq );
static void pwm_set_duty( uint8_t ch, float duty );
static uint8_t pwm_start( uint8_t ch );
static uint8_t pwm_stop( uint8_t ch );
static void PwmError_Handler( void );

/*****************************************************************************/
/** EXTERNAL VARIABLES *******************************************************/
/*****************************************************************************/
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

/*****************************************************************************/
/** LOCAL VARIABLES **********************************************************/
/*****************************************************************************/
static pwm_out_ch_st pwm_out[ MAX_PWM_CHANNEL ] = 
{
	{
		.inst = &htim2,
		.freq = 0,
		.duty = 0.0f,
	},
	{
		.inst = &htim3,
		.freq = 0,
		.duty = 0.0f,
	},
};

/*****************************************************************************/
/** GLOBAL VARIABLES *********************************************************/
/*****************************************************************************/

const PWM_st PWM = {
	.GetFreq = pwm_get_freq,
	.SetFreq = pwm_set_freq,
	.SetDuty = pwm_set_duty,
	.Start = pwm_start,
	.Stop = pwm_stop
};

/*****************************************************************************/
/** FUNCTION DEFINITIONS *****************************************************/
/*****************************************************************************/

/******************************************************************************
 * @brief Compares two floating-point values with a specified precision
 * 
 * @param a First value to compare
 * @param b Second value to compare
 * @param epsilon Allowed difference between the values
 * @return int8_t 1 if a > b, -1 if a < b, 0 if a == b
 *****************************************************************************/
static int8_t fcmp32( float a, float b, float epsilon )
{
	if( fabsf( a - b ) <= epsilon )
	{
		return 0; // a와 b가 거의 같음
	}
	else if ( a > b )
	{
		return 1; // a가 b보다 큼
	}
	else
	{
		return -1; // a가 b보다 작음
	}
}

/******************************************************************************
 * @brief Get the frequency of the specified PWM channel
 * 
 * @param ch The PWM channel number
 * @return float The frequency of the PWM channel
 *****************************************************************************/
static float pwm_get_freq( uint8_t ch )
{
	uint32_t apb1_clock = HAL_RCC_GetPCLK1Freq() * 2;
	return apb1_clock / (float)( pwm_out[ ch ].inst->Init.Prescaler * pwm_out[ ch ].inst->Init.Period );
}

/******************************************************************************
 * @brief Set the frequency of the specified PWM channel
 * 
 * @param freq The frequency to set
 * @return uint8_t 1 if the frequency is set, 0 otherwise
 *****************************************************************************/
static uint8_t pwm_set_freq( uint8_t ch, float freq )
{
	TIM_MasterConfigTypeDef sMasterConfig;
	TIM_OC_InitTypeDef sConfigOC = {0};
	TIM_HandleTypeDef * pTim = pwm_out[ ch ].inst;

	if( ( 0 > freq ) || 0 == fcmp32( 0, freq, EPSILON ) ) return 0;
	if( 0 == fcmp32( pwm_out[ ch ].freq, freq, EPSILON ) ) return 0;

	uint32_t apb1_clock = HAL_RCC_GetPCLK1Freq() * 2;

	pTim->Init.Prescaler = apb1_clock / freq / 10000;
	pTim->Init.Period = 10000 - 1;
	
	pTim->Instance = pwm_out[ ch ].inst->Instance;
	pTim->Init.CounterMode = TIM_COUNTERMODE_UP;
	pTim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	pTim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if( HAL_TIM_PWM_Init( pTim ) != HAL_OK )
	{
		PwmError_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if( HAL_TIMEx_MasterConfigSynchronization( pTim, &sMasterConfig ) != HAL_OK )
	{
		PwmError_Handler();
	}

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	if( HAL_TIM_PWM_ConfigChannel( pTim, &sConfigOC, TIM_CHANNEL_1 ) != HAL_OK )
	{
		PwmError_Handler();
	}
	
	return 1;
}

/******************************************************************************
 * @brief 
 * 
 * @param ch 
 * @param duty 
 *****************************************************************************/
static void pwm_set_duty( uint8_t ch, float duty )
{
	if( 0 == fcmp32( pwm_out[ ch ].duty, duty, EPSILON ) ) return;

	TIM_HandleTypeDef *pTim = pwm_out[ ch ].inst;

	if( duty < 0 ) duty = 0.0f;
	if( duty > 1 ) duty = 1.0f;

	__HAL_TIM_SET_COMPARE( pTim, TIM_CHANNEL_1, ( pTim->Init.Period + 1 ) * duty );
	pwm_out[ ch ].duty = duty;
}

/******************************************************************************
 * @brief 
 * 
 * @param ch 
 *****************************************************************************/
static uint8_t pwm_start( uint8_t ch )
{
	if( HAL_TIM_PWM_Start( pwm_out[ ch ].inst, TIM_CHANNEL_1 ) == HAL_OK )
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

/******************************************************************************
 * @brief 
 * 
 * @param ch 
 *****************************************************************************/
uint8_t pwm_stop( uint8_t ch )
{
	if( ch < MAX_PWM_CHANNEL )
	{
		if( HAL_TIM_PWM_Stop( pwm_out[ ch ].inst, TIM_CHANNEL_1 ) == HAL_OK )
		{
			pwm_out[ ch ].duty = 0.0f;
			return 1;
		}
	}
	return 0;
}

/******************************************************************************
 * @brief 
 * 
 *****************************************************************************/
static void PwmError_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
		__NOP();
	}
	/* USER CODE END Error_Handler_Debug */
}
