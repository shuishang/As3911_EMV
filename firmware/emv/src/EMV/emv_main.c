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

/*! \file emv_main.c
 *
 * \author Oliver Regenfelder
 *
 * \brief EMV terminal application.
 */

/*
******************************************************************************
* INCLUDES
******************************************************************************
*/

#include <stddef.h>

#include "usb_hid_stream_appl_handler.h"

#include "emv_hal.h"
#include "emv_standard.h"
#include "emv_poll.h"
#include "emv_layer4.h"
#include "emv_display.h"
#include "sleep.h"

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

/*! Indicates whether a stop request has been received from the GUI or not. */
static volatile bool_t emvStopRequestReceivedFlag = FALSE;

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

/*!
 *****************************************************************************
 * \brief Convert 13.56 MHz carrier cycle numbers to milliseconds.
 *
 * \note \a numCarrierCycles must be <= 888720133 (65535 ms).
 *
 * \param numCarrierCycles Number of carrier cycles.

 * \return \a numCarrierCycles converted to milliseconds.
 *****************************************************************************
 */
static u16 emvConvertCarrierCyclesToMilliseconds(u32 numCarrierCycles);

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

s16 emvStartTerminalApplication(s16 (*application)(void))
{
    s16 error = EMV_ERR_OK;

    /* Reset stop request received flag. */
    emvStopRequestReceivedFlag = FALSE;

    /* Implementation:
     * Error checking is done at the beginning of the while loop because
     * error handling is the same for all stages of the terminal main loop.
     * If any stage encounters an error it uses a continue statement to fall
     * through the rest of the while loop and reach the error handling code.
     */
    while(1)
    {
        EmvPicc_t picc;

        if (EMV_ERR_STOPPED == error)
        {
            /* Received stop request, stop terminal main loop. */
            return EMV_ERR_STOPPED;
        }
        if (EMV_ERR_OK != error)
        {
            emvDisplayError(error);

            /* Reset field and continue with polling. */
            emvHalResetField();
        }

        /* Polling. */
        emvDisplayMessage(EMV_M_POLLING);
        error = emvPoll();
        if (EMV_ERR_OK != error)
            continue;

        /* Anticollision. */
        sleepMilliseconds(EMV_T_P);
        error = emvCollisionDetection(&picc);
        if (EMV_ERR_OK != error)
            continue;

        /* Activation. */
        error = emvActivate(&picc);
        if (EMV_ERR_OK != error)
            continue;

        /* Wait for SFGT. */
        if(picc.sfgi > 0)
        {
            u32 sfgtCycles = (4096UL + 384) << picc.sfgi;
            u16 sfgtMilliseconds = emvConvertCarrierCyclesToMilliseconds(sfgtCycles);
            sleepMilliseconds(sfgtMilliseconds);
        }

        /* Initialize layer 4. */
        error = emvInitLayer4(&picc);
        if (EMV_ERR_OK != error)
            continue;

        /* Start terminal application. */
        if (application != NULL)
            error = application();
        else
            error = EMV_ERR_OK;

        if (EMV_ERR_OK != error)
            continue;

        /* Card removal. */
        emvDisplayMessage(EMV_M_REMOVE_CARD);
        error = emvRemove(&picc);
        if (EMV_ERR_OK != error)
            continue;
    }
}

bool_t emvStopRequestReceived()
{
    ProcessIO();

    if (emvStopRequestReceivedFlag)
    {
        emvStopRequestReceivedFlag = FALSE;
        return TRUE;
    }
	
    return FALSE;
}

void emvStopTerminalApplication()
{
    emvStopRequestReceivedFlag = TRUE;
}


/*
******************************************************************************
* LOCAL FUNCTIONS
******************************************************************************
*/

static u16 emvConvertCarrierCyclesToMilliseconds(u32 num_cycles)
{
    return (num_cycles / 13560) + 1;
}
