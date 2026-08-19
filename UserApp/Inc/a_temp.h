/******************************************************************************
 * @file a_temp.c
 * @author Seo Yujeong (yjseo@gst-in.com)
 * @brief 
 * @version 0.1
 * @date 2023-07-26
 * 
 * @copyright Copyright (c) 2023 Global Standard Technology Co., Ltd.
 * 
 * All rights reserved.
 * This file is part of closed source software.
 * Unauthorized reproduction and redistribution prohibited.
 *****************************************************************************/

#ifndef INC_A_TEMP_H_
#define INC_A_TEMP_H_
/*****************************************************************************/
/** INCLUDES *****************************************************************/
/*****************************************************************************/

/*****************************************************************************/
/** MACRO DEFINITIONS ********************************************************/
/*****************************************************************************/

#define DEFAULT_SAMPLING_PERIOD_MS		50

/*****************************************************************************/
/** DATATYPES ****************************************************************/
/*****************************************************************************/

typedef struct temp_struct
{
	void (*SetType)( int16_t ch, sensor_et type, uint16_t sampling_period );
	float (*GetTemp)( int16_t ch );
	float (*ConvToRes)( sensor_et type, uint32_t Value, float sampling_period );
	float (*ConvTomV)( int32_t Value, float sampling_period );
	float (*GetCjTemp)( void );
	float (*GetTCTemp)( sensor_et type, uint8_t ch, float ColdJTemp );
	float (*GetTCmV)( uint8_t ch );
	uint32_t (*GetSampleGeneration)( uint8_t ch );
} temp_st;

/*****************************************************************************/
/** GLOBAL VARIABLES *********************************************************/
/*****************************************************************************/

extern const temp_st Temp;

/*****************************************************************************/
/** LOCAL VARIABLES **********************************************************/
/*****************************************************************************/

#define fround( num ) ( roundf( ( num ) * 1000.0f ) / 1000.0f )

/*****************************************************************************/
/** FUNCTION DEFINITIONS *****************************************************/
/*****************************************************************************/

void TempInit( void );
void TemperatureTask( void );

#endif /* INC_A_TEMP_H_ */
