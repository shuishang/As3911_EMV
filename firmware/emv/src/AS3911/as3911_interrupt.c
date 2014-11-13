/*
 *****************************************************************************
 * Copyright @ 2009 by austriamicrosystems AG                                *
 * All rights are reserved.                                                  *
 *                                                                           *
 * Reproduction in whole or in part is prohibited without the written consent*
 * of the copyright owner. Austriamicrosystems reserves the right to make    *
 * changes without notice at any time. The software is provided as is and    *
 * Austriamicrosystems makes no warranty, expressed, implied or statutory,   *
 * including but not limited to any implied warranty of merchantability or   *
 * fitness for any particular purpose, or that the use will not infringe any *
 * third party patent, copyright or trademark. Austriamicrosystems should    *
 * not be liable for any loss or damage arising from its use.                *
 *****************************************************************************
 */

/*
 * PROJECT: AS3911 firmware
 * $Revision: $
 * LANGUAGE: ANSI C
 */

/*! \file as3911_interrupt.c
 *
 * \author Oliver Regenfelder
 *
 * \brief AS3911 interrupt handling and ISR
 */

/*
******************************************************************************
* INCLUDES
******************************************************************************
*/

#include "as3911_interrupt.h"

#include "as3911.h"
#include "timer_driver.h"
#include "errno.h"

/*
******************************************************************************
* DEFINES
******************************************************************************
*/

/*
******************************************************************************
* MACROS
******************************************************************************
*/

/*!
 *****************************************************************************
 * \brief Clear the interrupt flag associated with the as3911 interrupt.
 *****************************************************************************
 */
#define AS3911_IRQ_CLR() { _IC1IF = 0; }

/*!
 *****************************************************************************
 * \brief Evaluates to true if there is a pending interrupt request from the
 * AS3911.
 *****************************************************************************
 */
#define AS3911_IRQ_IS_SET() ( _RB9 != 0)

/*
******************************************************************************
* LOCAL DATA TYPES
******************************************************************************
*/

/*
******************************************************************************
* LOCAL VARIABLES
******************************************************************************
*/

/*! AS3911 interrutp mask. */
static volatile u32 as3911InterruptMask = 0;
/*! Accumulated AS3911 interrupt status. */
static volatile u32 as3911InterruptStatus = 0;

/*
******************************************************************************
* LOCAL TABLES
******************************************************************************
*/

/*
******************************************************************************
* LOCAL FUNCTION PROTOTYPES
******************************************************************************
*/

/*
******************************************************************************
* GLOBAL VARIABLE DEFINITIONS
******************************************************************************
*/

/*
******************************************************************************
* GLOBAL FUNCTIONS
******************************************************************************
*/

s8 as3911EnableInterrupts(u32 mask)
{
    s8 error = ERR_NONE;
    u32 irqMask = 0;

    AS3911_IRQ_OFF();

    error |= as3911ContinuousRead(AS3911_REG_IRQ_MASK_MAIN, (u8*) &irqMask, 3);
    irqMask &= ~mask;
    as3911InterruptMask |= mask;
    error |= as3911ContinuousWrite(AS3911_REG_IRQ_MASK_MAIN, (u8*) &irqMask, 3);

    AS3911_IRQ_ON();

    if (ERR_NONE == error)
        return ERR_NONE;
    else
        return ERR_IO;
}

s8 as3911DisableInterrupts(u32 mask)
{
    s8 error = ERR_NONE;
    u32 irqMask = 0;

    AS3911_IRQ_OFF();

    error |= as3911ContinuousRead(AS3911_REG_IRQ_MASK_MAIN, (u8*) &irqMask, 3);
    irqMask |= mask;
    as3911InterruptMask &=  ~mask;
    error |= as3911ContinuousWrite(AS3911_REG_IRQ_MASK_MAIN, (u8*) &irqMask, 3);

    AS3911_IRQ_ON();

    if (ERR_NONE == error)
        return ERR_NONE;
    else
        return ERR_IO;
}

s8 as3911ClearInterrupts(u32 mask)
{
    s8 error = ERR_NONE;
    u32 irqStatus = 0;

    AS3911_IRQ_OFF();

    error |= as3911ContinuousRead(AS3911_REG_IRQ_MAIN, (u8*) &irqStatus, 3);
    as3911InterruptStatus |= irqStatus & as3911InterruptMask;
    as3911InterruptStatus &= ~mask;

    AS3911_IRQ_ON();

    if (ERR_NONE == error)
        return ERR_NONE;
    else
        return ERR_IO;
}

s8 as3911WaitForInterruptTimed(u32 mask, u16 timeout, u32 *irqs)
{
    bool_t timerExpired = FALSE;
    u32 irqStatus = 0;

    if (timeout > 0)
        timerStart(timeout);

    do
    {
        irqStatus = as3911InterruptStatus & mask;

        if (timeout > 0)
        {
            if (!timerIsRunning())
                timerExpired = TRUE;
        }
    } while (!irqStatus && !timerExpired);

    AS3911_IRQ_OFF();
    as3911InterruptStatus &= ~irqStatus;
    AS3911_IRQ_ON();

    *irqs = irqStatus;

    return ERR_NONE;
}

s8 as3911GetInterrupts(u32 mask, u32 *irqs)
{
    AS3911_IRQ_OFF();

    *irqs = as3911InterruptStatus & mask;
    as3911InterruptStatus &= ~mask;

    AS3911_IRQ_ON();

    return ERR_NONE;
}

void __attribute__((interrupt, no_auto_psv)) _IC1Interrupt(void)
// void as3911Isr(void)
{
    do
    {
        u32 irqStatus = 0;

        AS3911_IRQ_CLR();

        as3911ContinuousRead(AS3911_REG_IRQ_MAIN, (u8*) &irqStatus, 3);
        as3911InterruptStatus |= irqStatus & as3911InterruptMask;
    } while (AS3911_IRQ_IS_SET());
}

/*
******************************************************************************
* LOCAL FUNCTIONS
******************************************************************************
*/
