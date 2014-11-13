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

/*! \file emv_prevalidation.c
 *
 * \author Oliver Regenfelder
 *
 * \brief EMV TTA L1 prevalidation application callback.
 */

/*
******************************************************************************
* INCLUDES
******************************************************************************
*/

#include <stddef.h>

#include "emv_display.h"
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

/*! EMV prevalidation test application initial SELECT PPSE command APDU. */
static const u8 emvPrevalidationApdu[] = { 0x00, 0xA4, 0x04, 0x00, 0x0E
    , '2', 'P', 'A', 'Y', '.', 'S', 'Y', 'S', '.', 'D', 'D', 'F', '0', '1', 0x00 };

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

s16 emvPrevalidationApplication()
{
    s16 error = EMV_ERR_OK;
    size_t responseLength = 0;
    emvDisplayCAPDU(emvPrevalidationApdu, sizeof(emvPrevalidationApdu));

    error = emvTransceiveApdu(emvPrevalidationApdu, sizeof(emvPrevalidationApdu),
                emvResponseBuffer, EMV_RESPONSE_BUFFER_SIZE, &responseLength);

    if (EMV_ERR_OK == error)
        emvDisplayRAPDU(emvResponseBuffer, responseLength);
    else
        emvDisplayError(error);

    return error;
}

/*
******************************************************************************
* LOCAL FUNCTIONS
******************************************************************************
*/
