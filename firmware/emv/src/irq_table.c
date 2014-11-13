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
 * PROJECT: ASxxxx firmware
 * $Revision: 2588 $
 * LANGUAGE: ANSI C
 */

/*! \file irq_table.c
 *
 * \author F. Lobmaier
 *
 * \brief Interrupt table for bootloader images.
 *
 * IRQ table which is used for bootloaded images, i.e. when bootloader is used.
 * Interrupt vector table of bootloader jumps to the table listed below.
 */

/*
******************************************************************************
* INCLUDES
******************************************************************************
*/

/*
******************************************************************************
* GLOBAL VARIABLES
******************************************************************************
*/
void __attribute__ ((space(prog), section(".irqtable"))) IRQTable()
{
    asm("reset");               // reset instruction to prevent runaway code
    asm("nop");
    asm("nop");                 // ReservedTrap0
    asm("nop");
    asm("nop");                 // OscillatorFail
    asm("nop");
    asm("nop");                 // AddressError
    asm("nop");
    asm("nop");                 // StackError
    asm("nop");
    asm("nop");                 // MathError
    asm("nop");
    asm("nop");                 // ReservedTrap5
    asm("nop");
    asm("nop");                 // ReservedTrap6
    asm("nop");
    asm("nop");                 // ReservedTrap7
    asm("nop");
    asm("nop");                 // INT0Interrupt
    asm("nop");
    asm("goto __IC1Interrupt"); // IC1Interrupt
    asm("nop");
    asm("nop");                 // OC1Interrupt
    asm("nop");
    asm("goto __T1Interrupt");   // T1Interrupt
    asm("nop");
    asm("nop");                 // Interrupt4
    asm("nop");
    asm("nop");                 // IC2Interrupt
    asm("nop");
    asm("nop");                 // OC2Interrupt
    asm("nop");
    asm("nop");                 // T2Interrupt
    asm("nop");
    asm("nop");                 // T3Interrupt
    asm("nop");
    asm("nop");                 // SPI1ErrInterrupt
    asm("nop");
    asm("nop");                 // SPI1Interrupt
    asm("nop");
    asm("nop");  				// U1RXInterrupt
    asm("nop");
    asm("nop");                 // U1TXInterrupt
    asm("nop");
    asm("nop");                 // ADC1Interrupt
    asm("nop");
    asm("nop");                 // Interrupt14
    asm("nop");
    asm("nop");                 // Interrupt15
    asm("nop");
    asm("nop");                 // SI2C1Interrupt
    asm("nop");
    asm("nop");                 // MI2C1Interrupt
    asm("nop");
    asm("nop");                 // CompInterrupt
    asm("nop");
    asm("nop");                 // CNInterrupt
    asm("nop");
    asm("nop");                 // INT1Interrupt
    asm("nop");
    asm("nop");                 // Interrupt21
    asm("nop");
    asm("nop");                 // Interrupt22
    asm("nop");
    asm("nop");                 // Interrupt23
    asm("nop");
    asm("nop");                 // Interrupt24
    asm("nop");
    asm("nop");                 // OC3Interrupt
    asm("nop");
    asm("nop");                 // OC4Interrupt
    asm("nop");
    asm("nop");                 // T4Interrupt
    asm("nop");
    asm("nop");  				// T5Interrupt
    asm("nop");
    asm("nop");                 // INT2Interrupt
    asm("nop");
    asm("nop");                 // U2RXInterrupt
    asm("nop");
    asm("nop");                 // U2TXInterrupt
    asm("nop");
    asm("nop");                 // SPI2ErrInterrupt
    asm("nop");
    asm("nop");                 // SPI2Interrupt
    asm("nop");
    asm("nop");                 // Interrupt34
    asm("nop");
    asm("nop");                 // Interrupt35
    asm("nop");
    asm("nop");                 // Interrupt36
    asm("nop");
    asm("nop");                 // IC3Interrupt
    asm("nop");
    asm("nop");                 // IC4Interrupt
    asm("nop");
    asm("nop");                 // IC5Interrupt
    asm("nop");
    asm("nop");                 // Interrupt40
    asm("nop");
    asm("nop");                 // OC5Interrupt
    asm("nop");
    asm("nop");                 // Interrupt42
    asm("nop");
    asm("nop");                 // Interrupt43
    asm("nop");
    asm("nop");                 // Interrupt44
    asm("nop");
    asm("nop");                 // PMPInterrupt
    asm("nop");
    asm("nop");                 // Interrupt46
    asm("nop");
    asm("nop");                 // Interrupt47
    asm("nop");
    asm("nop");                 // Interrupt48
    asm("nop");
    asm("nop");                 // SI2C2Interrupt
    asm("nop");
    asm("nop");                 // MI2C2Interrupt
    asm("nop");
    asm("nop");                 // Interrupt51
    asm("nop");
    asm("nop");                 // Interrupt52
    asm("nop");
    asm("nop");                 // Interrupt53
    asm("nop");
    asm("nop");                 // Interrupt54
    asm("nop");
    asm("nop");                 // Interrupt55
    asm("nop");
    asm("nop");                 // Interrupt56
    asm("nop");
    asm("nop");                 // Interrupt57
    asm("nop");
    asm("nop");                 // Interrupt58
    asm("nop");
    asm("nop");                 // Interrupt59
    asm("nop");
    asm("nop");                 // Interrupt60
    asm("nop");
    asm("nop");                 // Interrupt61
    asm("nop");
    asm("nop");                 // RTCCInterrupt
    asm("nop");
    asm("nop");                 // Interrupt63
    asm("nop");
    asm("nop");                 // Interrupt64
    asm("nop");
    asm("nop");                 // U1ErrInterrupt
    asm("nop");
    asm("nop");                 // U2ErrInterrupt
    asm("nop");
    asm("nop");                 // CRCInterrupt
    asm("nop");
    asm("nop");                 // Interrupt68
    asm("nop");
    asm("nop");                 // Interrupt69
    asm("nop");
    asm("nop");                 // Interrupt70
    asm("nop");
    asm("nop");                 // Interrupt71
    asm("nop");
    asm("nop");                 // LVDInterrupt
    asm("nop");
    asm("nop");                 // Interrupt73
    asm("nop");
    asm("nop");                 // Interrupt74
    asm("nop");
    asm("nop");                 // Interrupt75
    asm("nop");
    asm("nop");                 // Interrupt76
    asm("nop");
    asm("nop");                 // Interrupt77
    asm("nop");
    asm("nop");                 // Interrupt78
    asm("nop");
    asm("nop");                 // Interrupt79
    asm("nop");
    asm("nop");                 // Interrupt80
    asm("nop");
    asm("nop");                 // Interrupt81
    asm("nop");
    asm("nop");                 // Interrupt82
    asm("nop");
    asm("nop");                 // Interrupt83
    asm("nop");
    asm("nop");                 // Interrupt84
    asm("nop");
    asm("nop");                 // Interrupt85
    asm("goto __USB1Interrupt"); // USB1Interrupt
}

