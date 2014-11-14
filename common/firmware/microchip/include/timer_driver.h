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
 *      PROJECT:   PIC firmware
 *      $Revision: $
 *      LANGUAGE:  ANSI C
 */

/*! \file timer_driver.h
 *
 *  \author M. Arpa
 *
 *  \brief simple timer driver
 *
 *  This is the interface file for the timer driver. 
 *  
 */


#ifndef TIMER_DRIVER_H
#define TIMER_DRIVER_H

/*
******************************************************************************
* INCLUDES
******************************************************************************
*/
#include "ams_types.h"


/*! \ingroup timerdriver
 *****************************************************************************
 *  \brief  Initializes the timer module
 * 
 * A typical value for clockCyclesPerSecond is e.g. 16000000 for 16 MHz.
 *****************************************************************************
 */
void timerInitialize( u32 clockCyclesPerSecond );

/*! \ingroup timerdriver
 *****************************************************************************
 *  \brief  Wind up timer for the given period in milli-seconds.
 *****************************************************************************
 */
void timerStart( u16 mSec );

/*! \ingroup timerdriver
 *****************************************************************************
 *  \brief Stops the timer if it was running.
 *****************************************************************************
 */
void timerStop( );

/*! \ingroup timerdriver
 *****************************************************************************
 *  \brief Returns TRUE if the timer is running else FALSE.
 *****************************************************************************
 */
BOOL timerIsRunning( );

#endif /* TIMER_DRIVER_H */

