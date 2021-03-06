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
 
/*! \file as3911_irq.h
 *
 * \author Oliver Regenfelder
 *
 * \brief Handling of the external interrupt generated by the AS3911 in the PIC.
 *
 * Mandatory detailed description of the file
 */

#ifndef AS3911_IRQ_H
#define AS3911_IRQ_H

/*
******************************************************************************
* INCLUDES
******************************************************************************
*/

#include <p24FJ64GB002.h>

/*
******************************************************************************
* DEFINES
******************************************************************************
*/

/*
******************************************************************************
* GLOBAL MACROS
******************************************************************************
*/

#define AS3911_IRQ_ON()  { _IC1IE = 1; }
#define AS3911_IRQ_OFF() { _IC1IE = 0; }
#define AS3911_IRQ_CLR() { _IC1IF = 0; }
#define AS3911_IRQ_IS_SET() ( _RB9 != 0)

/*
******************************************************************************
* GLOBAL DATA TYPES
******************************************************************************
*/

/*
******************************************************************************
* GLOBAL VARIABLE DECLARATIONS
******************************************************************************
*/

/*
******************************************************************************
* GLOBAL FUNCTION PROTOTYPES
******************************************************************************
*/

#endif /* AS3911_IRQ_H_ */
