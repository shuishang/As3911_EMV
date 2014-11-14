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

/*! \file uart.c
 *
 *  \author Wolfgang Reichart (based on work of Christian Eisendle)
 *
 *  \brief uart driver implementation for PIC24FJ64
 *
 */
/*
******************************************************************************
* INCLUDES
******************************************************************************
*/
#include <p24Fxxxx.h>
#include "errno.h"
#include "uart.h"
/*
******************************************************************************
* LOCAL DEFINES
******************************************************************************
*/

/*
******************************************************************************
* LOCAL VARIABLES
******************************************************************************
*/

/*
******************************************************************************
* LOCAL MACROS
******************************************************************************
*/
/*! \ingroup uart
 * macro which writes to uart config/status registers depending on selected
 * instance.
 */
#define UART_WRITE_REG(REG, VALUE) U1##REG = (VALUE)

/*! \ingroup uart
 * macro which reads from uart config/status registers depending on selected
 * instance.
 */
#define UART_READ_REG(REG) U1##REG

/*
******************************************************************************
* GLOBAL FUNCTIONS
******************************************************************************
*/
s8 uartTxInitialize(u32 sysclk, u32 baudrate, u32* actbaudrate)
{
    u32 br1, br2;
    u16 breg;

    /* Disable UART for configuration */
    UART_WRITE_REG(MODE, 0x0);
    UART_WRITE_REG(STA, 0x0);

    /* Setup UART registers */
    /* equation according to the datasheet:
       (sysclk / (16 * baudrate)) - 1
     */
    breg = (sysclk / (16 * baudrate)) - 1;

    /* round up/down w/o using floating point maths */
    br1 = sysclk / (16 * (breg + 1));
    br2 = sysclk / (16 * (breg + 2));

    /* check which of the two values produce fewer error rate */
    if ((br1 - baudrate) > (baudrate - br2))
    {
        U1BRG = breg + 1;
        *actbaudrate = br2;
    }
    else
    {
        U1BRG = breg;
        *actbaudrate = br1;
    }
    /* Enable UART */
    UART_WRITE_REG(MODE, UART_READ_REG(MODE) | 0x8000);
    UART_WRITE_REG(STA, UART_READ_REG(STA) | 0x400);

    return ERR_NONE;
}

s8 uartTxDeinitialize()
{
    /* disable UART */
    UART_WRITE_REG(MODE, 0x0);
    UART_WRITE_REG(STA, 0x0);;

    return ERR_NONE;
}

s8 uartTxByte(u8 dat)
{
    return uartTxNBytes(&dat, sizeof(u8));
}

s8 uartTxNBytes(const u8* buffer, u16 length)
{
    while (length--)
    {
        UART_WRITE_REG(TXREG, *buffer++);
        /* Wait until transmit shift register is empty */
        while (!(UART_READ_REG(STA) & 0x100))
            ;
    }

    return ERR_NONE;
}

