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

/*! \file timer_driver.c
 *
 *  \author M. Arpa
 *
 *  \brief simple timer driver
 *
 *  This is the implementation file for the timer driver. 
 *  
 */

/*
******************************************************************************
* INCLUDES
******************************************************************************
*/
#define USE_AND_OR
#include <p24Fxxxx.h>
#include "timer_driver.h"

#define TIMER_MAX_VALUE 1000   /* real max is 1048 */

/*
******************************************************************************
* variables
******************************************************************************
*/
/* clock cycles per milliseconds */
static u32 clkCyclesPerMSec;

/* flag to indicate whether timer is running or not */
static volatile s8 timer1Running;

/* timer value - useful for timer values > 1000 */
static volatile u16 timerValue;

/*
******************************************************************************
* interrupt service routines
******************************************************************************
*/

void __attribute__ ((interrupt,no_auto_psv)) _T1Interrupt ( void )
{
    _T1IF = 0;
    if ( timerValue > 0 )
    { /* reload */
        if ( timerValue < TIMER_MAX_VALUE )
	{
  	    u32 milliSec = ( ((u32)timerValue) * clkCyclesPerMSec ) / 256; /* prescaler used */
            PR1 = (u16)milliSec;  /* load timer 1 compare register */
	    timerValue = 0; /* end of outer loop */
	}    
	else /* not the last time we let timer overflow */
	{
	    timerValue -= TIMER_MAX_VALUE;
	} 
    }
    else /* timer has expired */
    {
        timerStop( );
    }
}

/*
******************************************************************************
* global functions
******************************************************************************
*/

void timerInitialize( u32 systemFrequency )
{
    clkCyclesPerMSec = systemFrequency / 1000;
    timerStop( );
}

void timerStart( u16 mSec )
{
    u32 milliSec;
    timerValue = mSec;
    if ( mSec > TIMER_MAX_VALUE ) /* cannot handle values bigger than 1048 */
    {
        mSec = TIMER_MAX_VALUE;
    }
    timerValue -= mSec;
    timer1Running = 1;    /* set running flag */
    milliSec = ( ((u32)mSec) * clkCyclesPerMSec ) / 256; /* prescaler used */
    PR1 = (u16)milliSec;  /* load timer 1 compare register */
    TMR1 = 0;             /* reset timer counter */
    _T1IF = 0;            /* clear interrupt flag */
    _T1IE = 1;            /* enable interrupt */
    T1CON = 0x8030;       /* start timer , prescaler 1:256*/
}

void timerStop( )
{
    _T1IE = 0; /* disable interrupt */
    T1CON = 0; /* stop timer */
    _T1IF = 0; /* clear interrupt flag */
    timer1Running = 0; /* timer is not running */
} 

BOOL timerIsRunning( )
{
    return ( timer1Running == 1 );
}


