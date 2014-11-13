/*
 *****************************************************************************
 * Copyright @ 2009 by austriamicrosystems AG                                *
 * All rights are reserved.                                                  *
 *                                                                           *
 * Reproduction in whole or in part is prohibited without the written consent*
 * of the copyright owner. Austriamicrosystems reserves the right to make    *
 * changes without notice at any time. The software is provided as is and    *
 * Austriamicrosystems makes no warranty, expressed, implied or statutory,   *
 * including but not limited to any implied warranty of merchantability or   *
 * fitness for any particular purpose, or that the use will not infringe any *
 * third party patent, copyright or trademark. Austriamicrosystems should    *
 * not be liable for any loss or damage arising from its use.                *
 *****************************************************************************
 */

/*
 * PROJECT: AS911 firmware
 * $Revision: $
 * LANGUAGE: ANSI C
 */

/*! \file led.c
 *
 * \author Oliver Regenfelder
 *
 * \brief Control the LEDs on the EMV board.
 */

/*
******************************************************************************
* INCLUDES
******************************************************************************
*/
#include <p24FJ64GB002.h>

#include "led.h"

/*
******************************************************************************
* DEFINES
******************************************************************************
*/

/*
******************************************************************************
* MACROS
******************************************************************************
*/

/*
******************************************************************************
* LOCAL DATA TYPES
******************************************************************************
*/

/*
******************************************************************************
* LOCAL VARIABLES
******************************************************************************
*/

/*
******************************************************************************
* LOCAL TABLES
******************************************************************************
*/

/*
******************************************************************************
* LOCAL FUNCTION PROTOTYPES
******************************************************************************
*/

/*
******************************************************************************
* GLOBAL VARIABLE DEFINITIONS
******************************************************************************
*/

/*
******************************************************************************
* GLOBAL FUNCTIONS
******************************************************************************
*/

void ledOn(u8 leds)
{
    if (leds & LED_GREEN)
        _LATB15 = 1;
    if (leds & LED_ORANGE)
        _LATB14 = 1;
    if (leds & LED_RED)
        _LATB13 = 1;
    if (leds & LED_BLUE)
        _LATB5 = 1;
}

void ledOff(u8 leds)
{
    if (leds & LED_GREEN)
        _LATB15 = 0;
    if (leds & LED_ORANGE)
        _LATB14 = 0;
    if (leds & LED_RED)
        _LATB13 = 0;
    if (leds & LED_BLUE)
        _LATB5 = 0;
}

/*
******************************************************************************
* LOCAL FUNCTIONS
******************************************************************************
*/
