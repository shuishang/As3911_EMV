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
 *      PROJECT:   ASxxxx firmware
 *      $Revision: $
 *      LANGUAGE:  ANSI C
 */

/*! \file logger.h
 *
 *  \author Wolfgang Reichart (based on work of Christian Eisendle)
 *
 *  \brief serial output log declaration file
 *
 */
/*!
 * \defgroup logeer Serial output log driver
 * This driver provides a printf-like way to output log messages
 * via the UART interface. It makes use of the uart driver.
 *
 * API:
 * - Write a log message to UART output: #DEBUG
 */

#ifndef LOGGER_H
#define LOGGER_H

/*
******************************************************************************
* INCLUDES
******************************************************************************
*/
#include "ams_types.h"

/*
******************************************************************************
* DEFINES
******************************************************************************
*/
#define LOGGER_ON   1
#define LOGGER_OFF  0

/*! Enable dbgPrint() called in USB modules via UART1 with this define */
#define USE_USB_LOGGER  LOGGER_OFF

/*! Enable dbgPrint() called in I2C modules via UART1 with this define */
#define USE_I2C_LOGGER  LOGGER_OFF

/*! Enable dbgPrint() called in SPI modules via UART1 with this define */
#define USE_SPI_LOGGER  LOGGER_OFF

/*
 * Selektive enabling of the logger module is disabled for EMV. Instead
 * logging is always tunred on. Because the EMV GUI uses the logging API
 * to provide additional diagnostic output to the user.
 */
/*
#if (  (USE_USB_LOGGER == LOGGER_ON) \
    || (USE_I2C_LOGGER == LOGGER_ON) \
    || (USE_SPI_LOGGER == LOGGER_ON) \
    )
    #define USE_LOGGER  LOGGER_ON
#else
    #define USE_LOGGER  LOGGER_OFF
#endif
*/
#define USE_LOGGER  LOGGER_ON

/*
******************************************************************************
* GLOBAL MACROS
******************************************************************************
*/
#if (USE_LOGGER == LOGGER_ON)
#define LOG dbgLog /*!< macro used for printing debug messages */
#define LOGDUMP dbgHexDump /*!< macro used for dumping buffers */
/* debug assertion macro: 
 * If condition mustBeTrue is not fulfilled, an endless (idle) loop is entered
 */
#define DBG_ASSERT(mustBeTrue) {                                            \
    if (!(mustBeTrue))                                                      \
    {                                                                       \
        while (1);                                                          \
    }                                                                       \
}
#else
#define LOG(...) /*!< macro used for printing debug messages if USE_LOGGER is set */
#define LOGDUMP(...) /*!< macro used for dumping buffers if USE_LOGGER is set */
#define DBG_ASSERT(mustBeTrue)  {}
#endif

#if (USE_USB_LOGGER == LOGGER_ON)
#define USB_LOG dbgLog /*!< macro used for printing debug messages */
#define USB_LOGDUMP dbgHexDump /*!< macro used for dumping buffers */
#else
#define USB_LOG(...) /*!< macro used for printing debug messages if USE_LOGGER is set */
#define USB_LOGDUMP(...) /*!< macro used for dumping buffers if USE_LOGGER is set */
#endif

#if (USE_I2C_LOGGER == LOGGER_ON)
#define I2C_LOG dbgLog /*!< macro used for printing debug messages */
#define I2C_LOGDUMP dbgHexDump /*!< macro used for dumping buffers */
#else
#define I2C_LOG(...) /*!< macro used for printing debug messages if USE_LOGGER is set */
#define I2C_LOGDUMP(...) /*!< macro used for dumping buffers if USE_LOGGER is set */
#endif

#if (USE_SPI_LOGGER == LOGGER_ON)
#define SPI_LOG dbgLog /*!< macro used for printing debug messages */
#define SPI_LOGDUMP dbgHexDump /*!< macro used for dumping buffers */
#else
#define SPI_LOG(...) /*!< macro used for printing debug messages if USE_LOGGER is set */
#define SPI_LOGDUMP(...) /*!< macro used for dumping buffers if USE_LOGGER is set */
#endif

/*! \ingroup logger
 *****************************************************************************
 *  \brief  Writes out a formated string via UART interface
 *
 *  This function is used to write a formated string via the UART interface.
 *  \note This function shall not be called directly. Instead the #DEBUG
 *  macro should be used.
 *
 *  \param[in] format : 0 terminated string to be written out to UART interface.
 *                      the following format tags are supported:
 *                      - %x
 *  \param[in] ... : variable parameter which depends on \a format.
 *
 *  \return ERR_NONE : No error, debug output sent.
 *
 *****************************************************************************
 */
extern s8 dbgLog(const char* format, ...);

/*! \ingroup debug
 *****************************************************************************
 *  \brief  dumps a buffer to console, 8 values per line are dumped.
 *
 *  Buffer is hexdumped to console using dbgPrint(). Please note that the
 *  buffer size must be a multiple of 8.
 *  \param[in] buffer : pointer to buffer to be dumped.
 *
 *  \param[in] length : buffer length (multiple of 8 )
 *
 *  \return n/a
 *
 *****************************************************************************
 */
extern void dbgHexDump (unsigned char *buffer, u16 length);

#endif /* LOGGER_H */

