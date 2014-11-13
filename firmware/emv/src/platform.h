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

/*! \file platform.h
 *
 * \author F. Lobmaier
 * \author Oliver Regenfelder
 *
 *  \brief Platform specific header file
 */

#ifndef PLATFORM_H
#define PLATFORM_H

/*
******************************************************************************
* INCLUDES
******************************************************************************
*/
#include <p24FJ64GB002.h>
#include "ams_types.h"
#include "GenericTypeDefs.h"
#include "Compiler.h"
#include "errno.h"

/*
******************************************************************************
* DEFINES
******************************************************************************
*/
#define SYSCLK_16MHZ                16000000UL
#define SYSCLK_8MHZ                 8000000UL
#define SYSCLK_4MHZ                 4000000UL
#define SYSCLK                      SYSCLK_16MHZ   /*!< SYSCLK frequency in Hz */

/*
******************************************************************************
* GLOBAL VARIABLES
******************************************************************************
*/
extern umword IRQ_COUNT;

/*
******************************************************************************
* GLOBAL MACROS
******************************************************************************
*/

/*! macro which globally disables interrupts and increments interrupt count */
#define IRQ_INC_DISABLE() do {                                              \
    __builtin_disi(0x3FFF);                                                 \
    IRQ_COUNT++;                                                            \
} while(0)

/*! macro to globally enable interrupts again if interrupt count is 0 */
#define IRQ_DEC_ENABLE() do {                                               \
   if (IRQ_COUNT != 0) IRQ_COUNT--;                                         \
   if (IRQ_COUNT == 0)  __builtin_disi(0);                                  \
} while(0)

#endif /* PLATFORM_H */

