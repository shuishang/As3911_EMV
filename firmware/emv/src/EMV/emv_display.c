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
 * PROJECT: AS3911 firmware
 * $Revision: $
 * LANGUAGE: ANSI C
 */

/*! \file emv_display.c
 *
 * \author Oliver Regenfelder
 *
 * \brief Methods to display EMV data on the GUI.
 */

/*
******************************************************************************
* INCLUDES
******************************************************************************
*/

#include "emv_display.h"

#include <stddef.h>

#include "logger.h"

#include "emv_layer4.h"
#include "emv_response_buffer.h"

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

#if USE_LOGGER == LOGGER_ON
#define EMV_LOG dbgLog
#else
#define EMV_LOG(...)
#endif

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

void emvDisplayString(const char *string)
{
    EMV_LOG(string);
}

void emvDisplayByteArray(const u8 *array, size_t length)
{
    size_t index = 0;
    for (index = 0; index < length; index++)
        EMV_LOG("%hhx", array[index]);
}

void emvDisplayUid(const u8 *uid, size_t length)
{
    size_t index = length - 1;

    do
    {
        EMV_LOG("%hhx", uid[index]);
        index--;
    } while (index > 0);
}

void emvDisplayError(s16 errorCode)
{
    switch (errorCode)
    {
    case EMV_ERR_OK:
        EMV_LOG("EMV: no error\n");
        break;
    case EMV_ERR_COLLISION:
        EMV_LOG("EMV: collision error\n");
        break;
    case EMV_ERR_PROTOCOL:
        EMV_LOG("EMV: protocol error\n");
        break;
    case EMV_ERR_TRANSMISSION:
        EMV_LOG("EMV: transmission error\n");
        break;
    case EMV_ERR_TIMEOUT:
        EMV_LOG("EMV: timeout error\n");
        break;
    case EMV_ERR_INTERNAL:
        EMV_LOG("EMV: internal error\n");
        break;
    case EMV_ERR_STOPPED:
        EMV_LOG("EMV: stopped error\n");
        break;
    default:
        EMV_LOG("EMV: unkown error code\n");
    }
}

void emvDisplayMessage(s16 messageCode)
{
    switch (messageCode)
    {
    case EMV_M_POLLING:
        EMV_LOG("EMV: Polling ...\n");
        break;
    case EMV_M_REMOVE_CARD:
        EMV_LOG("EMV: Remove card ...\n");
        break;
    default:
        EMV_LOG("EMV: unkown message code\n");
    }
}

void emvDisplayCAPDU(const u8 *apdu, size_t length)
{
    emvDisplayString("EMV: C-APDU ");
    emvDisplayByteArray(apdu, length);
    emvDisplayString("\n");
}

void emvDisplayRAPDU(const u8 *apdu, size_t length)
{
    emvDisplayString("EMV: R-APDU ");
    emvDisplayByteArray(apdu, length);
    emvDisplayString("\n");
}

/*
******************************************************************************
* LOCAL FUNCTIONS
******************************************************************************
*/
