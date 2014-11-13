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

/*! \file main.h
 *
 *  \author F. Lobmaier
 *
 *  \brief Application header file
 *
 */

#ifndef MAIN_H
#define MAIN_H

/*
******************************************************************************
* INCLUDES
******************************************************************************
*/

/*
******************************************************************************
* DEFINES
******************************************************************************
*/

/*! AS3911 EMV firmware version string. */
#define AS3911_FW_VERSION   "AS3911 FW v1.0.4 (Built: " __DATE__ " " __TIME__ ")";

/*
 *****************************************************************************
 * APPLICATION-SPECIFIC COMMANDS
 * PLEASE CHECK IF CMD_ID IS FREE TO USE. REFER TO FILE
 * common/firmware/microchip/include/usb/AmsCom.h
 *****************************************************************************
 */
#define APPL_COM_EMV_TOGGLE_CARRIER 0xE0 /*!< EMV TTA L1 carrier button pressed. */
#define APPL_COM_EMV_POLL           0xE1 /*!< EMV TTA L1 poll button prossed. */
#define APPL_COM_EMV_RESET          0xE2 /*!< EMV TTA L1 reset button presed. */
#define APPL_COM_EMV_WUPA           0xE3 /*!< EMV TTA L1 WUPA button pressed. */
#define APPL_COM_EMV_WUPB           0xE4 /*!< EMV TTA L1 WUPB button pressed. */
#define APPL_COM_EMV_RATS           0xE5 /*!< EMV TTA L1 RATS button pressed. */
#define APPL_COM_EMV_ATTRIB         0xE6 /*!< EMV TTA L1 ATTRIB button pressed. */
#define APPL_COM_EMV_PREVALIDATION  0xE7 /*!< EMV TTA L1 prevalidation test start button pressed. */
#define APPL_COM_EMV_DIGITAL        0xE8 /*!< EMV TTA L1 digital test start button pressed. */
#define APPL_COM_EMV_STOP           0xEA /*!< EMV TTA L1 prevalidation or digital test application stop button pressed. */
#define APPL_COM_EMV_INIT           0xEF /*!< EMV mode initialization command. */
#define APPL_COM_REG_DUMP           0xF0 /*!< Dump register content. */
#define APPL_COM_REG_WRITE          0xF1 /*!< Write register. */
#define APPL_COM_REG_READ           0xF2 /*!< Read register. */
#define APPL_COM_DIRECT_COMMAND		0xF3 /*!< Execute an AS3911 direct command. */

/*!
 *****************************************************************************
 * \brief Application main routine.
 *
 * \return 0.
 *****************************************************************************
 */
int main();

#endif /* MAIN_H */
