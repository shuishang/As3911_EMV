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

/*! \file led.h
 *
 * \author Oliver Regenfelder
 *
 * \brief Control the LEDs on the EMV board.
 */

#ifndef LED_H
#define LED_H

/*! \defgroup led LED Driver
 *****************************************************************************
 * \brief Driver to control the LEDs on the EMV demo board.
 *****************************************************************************
 */

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

/*! \name Available LEDs */
/**@{*/
/*! \ingroup led */
#define LED_GREEN   (1) /*!< Green LED (schematic LED4). */
#define LED_ORANGE  (2) /*!< Orange LED (schematic LED3). */
#define LED_RED     (4) /*!< Red LED (schematic LED2). */
#define LED_BLUE    (8) /*!< Blue LED (schematic LED1). */

/*! All LEDs available on the EMV board. */
#define LED_ALL (LED_GREEN | LED_ORANGE | LED_RED | LED_BLUE)
/**@}*/

/*
******************************************************************************
* MACROS
******************************************************************************
*/

/*
******************************************************************************
* GLOBAL FUNCTION PROTOTYPES
******************************************************************************
*/

/*! \ingroup led
 *****************************************************************************
 * \brief Turn on LEDs on the EMV board.
 *
 * \param[in] leds The LED(s) to turn on.
 *****************************************************************************
 */
void ledOn(u8 leds);


/*! \ingroup led
 *****************************************************************************
 * \brief Turn off LEDs on the EMV board.
 *
 * \param[in] leds The LED(s) to turn off.
 *****************************************************************************
 */
void ledOff(u8 leds);

#endif /* LED_H */
