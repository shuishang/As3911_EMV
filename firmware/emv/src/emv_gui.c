/*
 *****************************************************************************
 * Copyright @ 2011 by austriamicrosystems AG                                *
 * All rights are reserved.                                                  *
 *                                                                           *
 * IMPORTANT - PLEASE READ CAREFULLY BEFORE COPYING, INSTALLING OR USING     *
 * THE SOFTWARE.                                                             *
 *                                                                           *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       *
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT         *
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS         *
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT  *
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,     *
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT          *
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,     *
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY     *
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT       *
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE     *
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.      *
 *****************************************************************************
 */
/*
 *      PROJECT:   AS3911 firmware
 *      $Revision: $
 *      LANGUAGE:  ANSI C
 */

/*! \file emv_gui.c
 *
 *  \author Oliver Regenfelder
 *
 *  \brief EMV TTA L1 GUI commands.
 */

/*
******************************************************************************
* INCLUDES
******************************************************************************
*/

#include "EMV/emv_standard.h"
#include "EMV/emv_poll.h"
#include "EMV/emv_hal.h"
#include "EMV/emv_typeA.h"
#include "EMV/emv_typeB.h"
#include "EMV/emv_picc.h"
#include "EMV/emv_error_codes.h"
#include "EMV/emv_display.h"
#include "EMV/emv_main.h"
#include "EMV/emv_prevalidation.h"
#include "EMV/emv_digital.h"

/*
******************************************************************************
* GLOBAL VARIABLES
******************************************************************************
*/

/*
******************************************************************************
* GLOBAL FUNCTION
******************************************************************************
*/

void emvGuiToggleCarrier()
{
    /* EMV Testpad toggle CARRIER command. */
    if (emvHalFieldIsActivated())
    {
        emvDisplayString("EMV: deactivating carrier\n");
        emvHalActivateField(FALSE);
    }
    else
    {
        emvDisplayString("EMV: activating carrier\n");
        emvHalActivateField(TRUE);
    }
}

void emvGuiPoll()
{
    if (!emvHalFieldIsActivated())
    {
        emvDisplayString("EMV: carrier not activated!\n");
        return;
    }

    emvDisplayString("EMV: single poll\n");
    emvPollSingleIteration();
}

void emvGuiReset()
{
    if (!emvHalFieldIsActivated())
    {
        emvDisplayString("EMV: carrier not activated!\n");
        return;
    }

    emvDisplayString("EMV: reseting operating field\n");
    emvHalResetField();
}

void emvGuiWupa()
{
    /* EMV TTA L1 WUPA command. */
    size_t numBytesReceived;
    s8 error;
    u8 atqa[2];

    if (!emvHalFieldIsActivated())
    {
        emvDisplayString("EMV: carrier not activated!\n");
        return;
    }

    emvHalSetStandard(EMV_HAL_TYPE_A);
    emvHalSetErrorHandling(EMV_HAL_PREACTIVATION_ERROR_HANDLING);

    emvDisplayString("EMV: WUPA\n");
    error = emvHalTransceive(NULL, 0, &atqa[0], sizeof(atqa), &numBytesReceived, 1800, EMV_HAL_TRANSCEIVE_WUPA);

    if (EMV_HAL_ERR_OK == error)
    {
        emvDisplayString("EMV: ");
        emvDisplayByteArray(&atqa[0], numBytesReceived);
        emvDisplayString("\n");
    }
    else if (EMV_HAL_ERR_TIMEOUT == error)
    {
        emvDisplayString("EMV: timeout error\n");
    }
    else if (EMV_HAL_ERR_OVERFLOW == error)
    {
        emvDisplayString("EMV: buffer overflow error\n");
    }
    else if (EMV_HAL_ERR_ENCODING == error)
    {
        emvDisplayString("EMV: transmission error\n");
    }
    else if (EMV_HAL_ERR_PARAM == error)
    {
        emvDisplayString("EMV: function call parameter error\n");
    }
}

void emvGuiWupb()
{
    /* EMV TTA L1 WUPB command. */
    u8 wupb[3];
    u8 atqb[32];
    size_t numBytesReceived;
    s8 error;

    if (!emvHalFieldIsActivated())
    {
        emvDisplayString("EMV: carrier not activated!\n");
        return;
    }

    emvHalSetStandard(EMV_HAL_TYPE_B);
    emvHalSetErrorHandling(EMV_HAL_PREACTIVATION_ERROR_HANDLING);

    /* Setup WUPB command according to section 6.3.1 of the EMV standard. */
    wupb[0] = 0x05;
    wupb[1] = 0x00;
    wupb[2] = 0x08;

    emvDisplayString("EMV: WUPB\n");

    error = emvHalTransceive(wupb, 3, atqb, sizeof(atqb), &numBytesReceived, EMV_FWT_ATQB_PCD, EMV_HAL_TRANSCEIVE_WITH_CRC);

    if (EMV_HAL_ERR_OK == error)
    {
        emvDisplayString("EMV: ");
        emvDisplayByteArray(&atqb[0], numBytesReceived);
        emvDisplayString("\n");
    }
    else if (EMV_HAL_ERR_TIMEOUT == error)
    {
        emvDisplayString("EMV: timeout error\n");
    }
    else if (EMV_HAL_ERR_OVERFLOW == error)
    {
        emvDisplayString("EMV: buffer overflow error\n");
    }
    else if (EMV_HAL_ERR_ENCODING == error)
    {
        emvDisplayString("EMV: transmission error\n");
    }
    else if (EMV_HAL_ERR_PARAM == error)
    {
        emvDisplayString("EMV: function call parameter error\n");
    }
}

void emvGuiRats()
{
    /* EMV Testpad RATS command. */
    int error = 0;
    EmvPicc_t emvPicc;

    if (!emvHalFieldIsActivated())
    {
        emvDisplayString("EMV: carrier not activated!\n");
        return;
    }

    error = emvTypeAAnticollision(&emvPicc);
    if (error != EMV_ERR_OK)
    {
        emvDisplayError(error);
        return;
    }

    emvDisplayString("EMV: card selected ");
    emvDisplayUid(emvPicc.uid, emvPicc.uidLength);
    emvDisplayString("\n");

    error = emvTypeAActivation(&emvPicc);
    if (error == EMV_ERR_OK)
        emvDisplayString("EMV: card activated\n");
    else
        emvDisplayError(error);
}

void emvGuiAttrib()
{
    /* EMV Testpad ATTRIB command. */
    int error = 0;
    EmvPicc_t emvPicc;

    if (!emvHalFieldIsActivated())
    {
        emvDisplayString("EMV: carrier not activated!\n");
        return;
    }

    error = emvTypeBAnticollision(&emvPicc);
    if (error != EMV_ERR_OK)
    {
        emvDisplayError(error);
        return;
    }

    emvDisplayString("EMV: card selected ");
    emvDisplayUid(emvPicc.uid, emvPicc.uidLength);
    emvDisplayString("\n");

    error = emvTypeBActivation(&emvPicc);
    if (error == EMV_ERR_OK)
        emvDisplayString("EMV: card activated\n");
    else
        emvDisplayError(error);
}

void emvGuiPrevalidation()
{
    /* EMV Testpad pre validation application. */
    s16 error;

    if (!emvHalFieldIsActivated())
    {
        emvDisplayString("EMV: activating carrier\n");
        emvHalActivateField(TRUE);
    }

    emvDisplayString("EMV: starting prevalidation application.\n");
    error = emvStartTerminalApplication(emvPrevalidationApplication);
    if (error == EMV_ERR_OK)
        emvDisplayString("EMV: prevalidation application finished.\n");
    else if (error == EMV_ERR_STOPPED)
        emvDisplayString("EMV: prevalidation application stopped.\n");
    else
        emvDisplayError(error);
}

void emvGuiDigital()
{
    /* EMV Testpad digital application. */
    s16 error;

    if (!emvHalFieldIsActivated())
    {
        emvDisplayString("EMV: activating carrier\n");
        emvHalActivateField(TRUE);
    }

    emvDisplayString("EMV: starting digital application.\n");
    error = emvStartTerminalApplication(emvDigitalApplication);
    if (error == EMV_ERR_OK)
        emvDisplayString("EMV: digital application finished.\n");
    else if (error == EMV_ERR_STOPPED)
        emvDisplayString("EMV: digital application stopped.\n");
    else
        emvDisplayError(error);
}

void emvGuiStop()
{
    emvStopTerminalApplication();
}
