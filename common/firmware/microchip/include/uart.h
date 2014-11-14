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
 *      PROJECT:   AS1130 MCU board firmware
 *      $Revision: $
 *      LANGUAGE:  ANSI C
 */

/*! \file uart.h
 *
 *  \author Wolfgang Reichart (based on work of Christian Eisendle)
 *
 *  \brief uart driver declaration file
 *  
 */
/*!
 * \defgroup uart UART driver
 * The UART driver provides basic functionalty for sending
 * data via serial interface. It is needed for logging and debugging.
 *
 * API:
 * - Initialize UART driver: #uartTxInitialize
 * - Deinitialize UART driver: #uartTxDeinitialize
 * - Transmit data: #uartTxNBytes
 * - Receive data: #uartRxNBytes
 */

#ifndef UART_H__
#define UART_H__

/*
******************************************************************************
* INCLUDES
******************************************************************************
*/
#include "ams_types.h"

/*
******************************************************************************
* GLOBAL FUNCTION PROTOTYPES
******************************************************************************
*/
/*! \ingroup uart
 *****************************************************************************
 *  \brief  Initialize UART TX interface
 *
 *  This function initializes the UART interface, i.e. sets the requested
 *  baudrate and enables transmission only.
 *  \note System frequency must not be changed since baudrate calculation is
 *  done on base of a fixed frequency.
 *
 *  \param[in] baudrate: Baudrate to set, e.g. 115200
 *  \param[out] actbaudrate: Actual set baudrate which normally differs from
 *                    requested \a baudrate due to divider error.
 *  \return ERR_NONE : No error, UART initialized.
 *
 *****************************************************************************
 */
extern s8 uartTxInitialize(u32 sysclk, u32 baudrate, u32* actbaudrate);

/*! \ingroup uart
 *****************************************************************************
 *  \brief  Deinitialize UART TX interface
 *
 *  Calling this function deinitializes the UART interface. 
 *  Transmitting is then not possible any more.
 *
 *  \return ERR_NONE : No error, UART deinitialized.
 *
 *****************************************************************************
 */
extern s8 uartTxDeinitialize();

/*! \ingroup uart
 *****************************************************************************
 *  \brief  Transmit a given number of bytes
 *
 *  This function is used to transmit \a length bytes via the UART interface.
 *  \note blocking implementation, i.e. Function doesn't return before all
 *  data has been sent.
 *
 *  \param[in] buffer: Buffer of size \a length to be transmitted.
 *  \param[in] length: Number of bytes to be transmitted.
 *
 *  \return ERR_NONE : No error, all data sent.
 *
 *****************************************************************************
 */
extern s8 uartTxNBytes(const u8* buffer, u16 length);

/*! \ingroup uart
 *****************************************************************************
 *  \brief  transmit a single byte
 *
 *  This function is used to transmit a single byte.
 *
 *  \param[in] dat: Byte to transmit.
 *
 *  \return ERR_NONE : No error, byte sent.
 *
 *****************************************************************************
 */
extern s8 uartTxByte(u8 dat);

#endif /* UART_H__ */

