/******************************************************************************
 * @file d_dio.h
 * @author jylee1 (jylee1@gst-in.com)
 * @brief 
 * @version 0.1
 * @date 2023-05-03
 * 
 * @copyright Copyright (c) 2023 Global Standard Technology Co., Ltd.
 * 
 * All rights reserved.
 * This file is part of closed source software.
 * Unauthorized reproduction and redistribution prohibited.
 *****************************************************************************/

#ifndef INC_D_DIO_H_
#define INC_D_DIO_H_

/*****************************************************************************/
/** INCLUDES *****************************************************************/
/*****************************************************************************/

/*****************************************************************************/
/** MACRO DEFINITIONS ********************************************************/
/*****************************************************************************/

#define GPIO_PORT_PIN(x) x##_PORT, x##_PIN

#define USART2_RE_Pin GPIO_PIN_0
#define USART2_RE_GPIO_Port GPIOA
#define USART2_DE_Pin GPIO_PIN_1
#define USART2_DE_GPIO_Port GPIOA
#define USART3_DE_Pin GPIO_PIN_1
#define USART3_DE_GPIO_Port GPIOB
#define USART3_RE_Pin GPIO_PIN_2
#define USART3_RE_GPIO_Port GPIOB

#define ADC_CJ_PIN 2
#define ADC_CJ_PORT 'C'
#define DAC_NSS_PIN 4
#define DAC_NSS_PORT 'A'
#define DAC_NCLR_PIN 4
#define DAC_NCLR_PORT 'C'
#define DAC_ALARM_PIN 5
#define DAC_ALARM_PORT 'C'
#define ADC12_NSS_PIN 12
#define ADC12_NSS_PORT 'B'
#define ADC_NSYNC_PIN 6
#define ADC_NSYNC_PORT 'C'
#define ADC34_NSS_PIN 7
#define ADC34_NSS_PORT 'C'
#define FLT_RLY_PIN 3
#define FLT_RLY_PORT 'C'
#define LED_ERR_PIN 2
#define LED_ERR_PORT 'D'
#define LED_COM_PIN 12
#define LED_COM_PORT 'C'
#define LED_RUN_PIN 3
#define LED_RUN_PORT 'B'
#define BD_CHECK_PIN 11
#define BD_CHECK_PORT 'C'
#define BDID0_PIN 10
#define BDID0_PORT 'A'
#define BDID1_PIN 9
#define BDID1_PORT 'A'
#define BDID2_PIN 8
#define BDID2_PORT 'A'
#define BDID3_PIN 9
#define BDID3_PORT 'C'
#define UART2_RE_PIN 0
#define UART2_RE_PORT 'A'
#define UART2_DE_PIN 1
#define UART2_DE_PORT 'A'
#define UART3_RE_PIN 2
#define UART3_RE_PORT 'B'
#define UART3_DE_PIN 1
#define UART3_DE_PORT 'B'
#define SMPS_POLA1_PIN 1
#define SMPS_POLA1_PORT 'C'
#define SMPS_POLA2_PIN 0
#define SMPS_POLA2_PORT 'C'

#define GPIO_HIGH 1
#define GPIO_LOW 0

#define ADC_CJ GPIO_PORT_PIN(ADC_CJ)
#define DAC_NSS GPIO_PORT_PIN(DAC_NSS)
#define DAC_NCLR GPIO_PORT_PIN(DAC_NCLR)
#define DAC_ALARM GPIO_PORT_PIN(DAC_ALARM)
#define ADC12_NSS GPIO_PORT_PIN(ADC12_NSS)
#define ADC_NSYNC GPIO_PORT_PIN(ADC_NSYNC)
#define ADC34_NSS GPIO_PORT_PIN(ADC34_NSS)
#define FLT_RLY GPIO_PORT_PIN(FLT_RLY)
#define LED_RUN GPIO_PORT_PIN(LED_RUN)
#define LED_ERR GPIO_PORT_PIN(LED_ERR)
#define LED_COM GPIO_PORT_PIN(LED_COM)
#define BD_CHECK GPIO_PORT_PIN(BD_CHECK)
#define BDID0 GPIO_PORT_PIN(BDID0)
#define BDID1 GPIO_PORT_PIN(BDID1)
#define BDID2 GPIO_PORT_PIN(BDID2)
#define BDID3 GPIO_PORT_PIN(BDID3)
#define UART2_RE GPIO_PORT_PIN(UART2_RE)
#define UART2_DE GPIO_PORT_PIN(UART2_DE)
#define UART3_RE GPIO_PORT_PIN(UART3_RE)
#define UART3_DE GPIO_PORT_PIN(UART3_DE)
#define SMPS_POLA1 GPIO_PORT_PIN(SMPS_POLA1)
#define SMPS_POLA2 GPIO_PORT_PIN(SMPS_POLA2)

#define LED_ON GPIO_HIGH
#define LED_OFF GPIO_LOW
#define FLT_RLY_ON GPIO_HIGH
#define FLT_RLY_OFF GPIO_LOW
#define SMPS_POLA_COOL GPIO_HIGH
#define SMPS_POLA_HEAT GPIO_LOW

/*****************************************************************************/
/** DATATYPES ****************************************************************/
/*****************************************************************************/

typedef struct {
    void (*Output)( int port, int pin, bool high );     // @param port Use MACRO @param pin Use MACRO @param high true [Output high] @param high false [Output low]
    bool (*Input)( int port, int pin );                 // @param port Use MACRO @param pin Use MACRO
    uint8_t (*GetBoardID)( void );                      // @return uint8_t @retval Value read from rotary switch
} DIO_t;

/*****************************************************************************/
/** EXTERNAL VARIABLES *******************************************************/
/*****************************************************************************/

extern DIO_t DIO;

/*****************************************************************************/
/** GLOBAL VARIABLES *********************************************************/
/*****************************************************************************/

/*****************************************************************************/
/** FUNCTION DEFINITIONS *****************************************************/
/*****************************************************************************/



#endif /* INC_D_DIO_H_ */
