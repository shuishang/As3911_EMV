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

/*! \file logger.c
 *
 *  \author Wolfgang Reichart (based on work of Christian Eisendle)
 *
 *  \brief debug log output utility.
 *
 */

/*
******************************************************************************
* INCLUDES
******************************************************************************
*/
#include "logger.h"
#include "uart.h"
#include "errno.h"
#include <p24Fxxxx.h>
#include <stdarg.h>

/*
******************************************************************************
* LOCAL DEFINES
******************************************************************************
*/
#define NO_OF_NIBBLES 4
#define NULL_CHARACTER '\0'
#define ALPHA_LC_TO_INTEGER 87  /*!< small case conversion value              */
#define ALPHA_UC_TO_INTEGER 55  /*!< upper case conversion value              */
#define NUMCHAR_TO_INTEGER 48
#define CON_putchar uartTxByte

/*
******************************************************************************
* GLOBAL FUNCTIONS
******************************************************************************
*/
#if (USE_LOGGER == LOGGER_ON)
s8 dbgLog(const char* format, ...)
{
    u16 i = 0;
    u16 argint = 0;
    bool_t PercentageFound = FALSE;
    bool_t ZeroFound = FALSE;
    u16 NoOfNonzeroChars, NoOfZeros;
    u16 NoOfChars = 0;
    bool_t upperCase = FALSE;   /*user ABCDEF of abcdef for hex digits  */
    u8 CharSent[NO_OF_NIBBLES + 1];
    s16 hs = 0;
    u16 nibbles;
    u16 current_cpu_ipl;

    /* the USB interrupt is not handled via disi command -> use a "more direct" way */
    SET_AND_SAVE_CPU_IPL(current_cpu_ipl, 7);  /* disable interrupts */

    va_list argptr;
    va_start(argptr, format);

    while (*((u8*)format) != NULL_CHARACTER)
    {
        switch (*((u8*)format))
        {
        case '%':
            hs = 0;
            if (PercentageFound == TRUE)
            {
                PercentageFound = FALSE;
                CON_putchar('%');
            }
            else
            {
                PercentageFound = TRUE;
            }

            break;
        case 'h':
            if (PercentageFound)
            {
                hs++;
            }
            else
            {
                CON_putchar(*((u8*)format));
            }
            break;
        case 'X':
            upperCase = TRUE;
            /* fall through                                               */
        case 'c':
        case 'l':
        case 'D':
        case 'U':
        case 'd':
        case 'u':
        case 'x':

            /* Initialise the counters to 0 */
            NoOfNonzeroChars = 0;
            NoOfZeros = 0;

            /* if d/x was preceeded by % it represents some format */
            if (PercentageFound == TRUE)
            {
                nibbles = NO_OF_NIBBLES;
                if (hs == 2)
                {
                    nibbles = 2;
                    argint = (u16)va_arg(argptr, u16);
                }
                else if (hs == 1)
                {
                    argint = (u16)va_arg(argptr, u16);
                }
                else 
                {
                    argint = (s16)va_arg(argptr, s16);
                }

                /* shift and take nibble by nibble from MSB to LSB */
                for (i = nibbles; i > 0; i--)
                {
                    CharSent[i-1]=(u8)((argint & ((0xf)<<(4*(nibbles-1)))) >> (4*(nibbles-1)));

                    if ((CharSent[i-1] >= 0x0a) && (CharSent[i-1] <= 0x0f))
                    {
                        /*if the nibble is greater than 9 */
                        CharSent[i-1] = (u8)
                            (   (u16)CharSent[i-1]
                                + (upperCase ? ALPHA_UC_TO_INTEGER
                                    : ALPHA_LC_TO_INTEGER)
                            );
                    }
                    else
                    {
                        /*if nibble lies from 0 to 9*/
                        CharSent[i-1]=(u8)((u16)CharSent[i-1]+NUMCHAR_TO_INTEGER);
                    }
                    /* see how many nibbles the number is actually filling */
                    if (CharSent[i-1] == '0')
                    {
                        if (NoOfNonzeroChars == 0)
                        {
                            NoOfZeros++;
                        }
                        else
                        {
                            NoOfNonzeroChars++;
                        }
                    }
                    else
                    {
                        NoOfNonzeroChars++;
                    }
                    argint = argint << 4;
                }

                CharSent[nibbles] = '\0' ;

                PercentageFound = FALSE;

                /* If the user hasnt specified any number along with format display by default
                word size */
                if (NoOfChars == 0)
                {
                    NoOfChars = nibbles;
                }

                /* print the number displaying as many nibbles as the user has given in print format */
                if (NoOfChars >= NoOfNonzeroChars)
                {
                    for (i = 0; i < (NoOfChars - NoOfNonzeroChars); i++)
                    {
                        CON_putchar('0');
                    }

                    for (i = NoOfNonzeroChars; i > 0; i--)
                    {
                        CON_putchar(CharSent[i-1]);
                    }
                }
                else
                {
                    for (i = NoOfNonzeroChars; i > 0; i--)
                    {
                        CON_putchar(CharSent[i-1]);
                    }
                }
            }

            /* if d/x wasnt preceeded by % it is not a format but a string;so just print it */
            else
            {
                CON_putchar(*((u8*)format));
            }
            /*clean the number of s8 for next cycle                     */
            NoOfChars = 0;
            /*use lower case by default                                   */
            upperCase = FALSE;
            break;
        case '0':

            /*if 0 was preceeded by % it represents some format */
            if (PercentageFound == TRUE)
            {
                ZeroFound=TRUE;
            }

            /* if 0 wasnt preceeded by % it is not a format but a string;so just print it */
            else
            {
                CON_putchar(*((u8*)format));
            }
            break;
        case '1':
        case '2':
        case '3':
        case '4':
        case '5':
        case '6':
        case '7':
        case '8':
        case '9':

            /* if the number was preceeded by % and then a 0 it represents some format */
            if (ZeroFound == TRUE)
            {
                NoOfChars = (u16) (*((u8*)format) - NUMCHAR_TO_INTEGER);
                ZeroFound = FALSE;
            }

            /* if the number wasnt preceeded by % it is not a format but a string;so just print it */
            else
            {
                CON_putchar(*((u8*)format));
            }
            break;

        case '\n':
            /* Implicitly print a carriage return before each newline */
            CON_putchar('\r');
            CON_putchar(*((u8*)format));
            break;
            /*if the character doesnt represent any of these formats just print it*/
        default:
            CON_putchar(*((u8*)format));
            break;
        }
        format = ((char*)format) + 1;
    }
    va_end(argptr);

    RESTORE_CPU_IPL(current_cpu_ipl);

    return ERR_NONE;
}

// ******************************************************************************************************
// ************** USB Debugging Functions ***************************************************************
// ******************************************************************************************************
void dbgHexDump(unsigned char *buffer, u16 length)
{
    u16 i, rest;

    rest = length % 8;

    if (length >= 8)
    {
        for (i = 0; i < length/8; i++)
        {
            dbgLog("%hhx %hhx %hhx %hhx %hhx %hhx %hhx %hhx\n",
                     buffer[i*8],
                     buffer[i*8+1],
                     buffer[i*8+2],
                     buffer[i*8+3],
                     buffer[i*8+4],
                     buffer[i*8+5],
                     buffer[i*8+6],
                     buffer[i*8+7]
                    );
        }
    }
    if (rest > 0)
    {
        for (i = length/8 * 8; i < length; i++)
        {
            dbgLog("%hhx ", buffer[i]);
        }
        dbgLog("\n");
    }
}

#endif //#if USE_LOGGER == LOGGER_ON

