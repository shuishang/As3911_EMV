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

/*! \file as3911_io.c
 *
 * \author Oliver Regenfelder
 *
 * \brief AS3911 SPI communication.
 *
 * Implementation of the AS3911 SPI communication. The PIC is set to IPL 7 to disable
 * interrupts while accessing the SPI.
 */

/*
******************************************************************************
* INCLUDES
******************************************************************************
*/
#include <p24FJ64GB002.h>

#include "as3911_io.h"
#include "as3911_def.h"
#include "errno.h"
#include "spi_driver.h"

/*
******************************************************************************
* DEFINES
******************************************************************************
*/

#define AS3911_SPI_ADDRESS_MASK         (0x3F)
#define AS3911_SPI_CMD_READ_REGISTER    (0x40)
#define AS3911_SPI_CMD_WRITE_REGISTER   (0x00)
#define AS3911_SPI_CMD_READ_FIFO        (0xBF)
#define AS3911_SPI_CMD_WRITE_FIFO       (0x80)
#define AS3911_SPI_CMD_DIREC_CMD        (0xC0)

/*
******************************************************************************
* MACROS
******************************************************************************
*/

#define AS3911_SEN_ON() { _LATB8 = 0; }
#define AS3911_SEN_OFF() { _LATB8 = 1; }

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

s8 as3911WriteRegister(u8 address, u8 data)
{
    s8 error = ERR_NONE;
    int current_cpu_ipl = 0;
    u8 as3911WriteCommand[2] = { address & AS3911_SPI_ADDRESS_MASK, data };

    SET_AND_SAVE_CPU_IPL(current_cpu_ipl, 7);

    AS3911_SEN_ON();
    error |= spiRxTx(2, &as3911WriteCommand[0], 0, NULL, TRUE);
    AS3911_SEN_OFF();

    SET_CPU_IPL(current_cpu_ipl);

    if (ERR_NONE != error)
        return ERR_IO;
    else
        return ERR_NONE;
}

s8 as3911ReadRegister(u8 address, u8 *data)
{
    s8 error = ERR_NONE;
    int current_cpu_ipl = 0;
    u8 as3911ReadCommand = AS3911_SPI_CMD_READ_REGISTER | (address & AS3911_SPI_ADDRESS_MASK);

    SET_AND_SAVE_CPU_IPL(current_cpu_ipl, 7);

    AS3911_SEN_ON();
    error |= spiRxTx(1, &as3911ReadCommand, 1, data, TRUE);
    AS3911_SEN_OFF();

    SET_CPU_IPL(current_cpu_ipl);

    if (ERR_NONE != error)
        return ERR_IO;
    else
        return ERR_NONE;
}

s8 as3911WriteTestRegister(u8 address, u8 data)
{
    s8 error = ERR_NONE;
    int current_cpu_ipl = 0;
    u8 as3911WriteCommand[3] = { AS3911_SPI_CMD_DIREC_CMD | AS3911_CMD_TEST_ACCESS, address & AS3911_SPI_ADDRESS_MASK, data };

    SET_AND_SAVE_CPU_IPL(current_cpu_ipl, 7);

    AS3911_SEN_ON();
    error |= spiRxTx(3, &as3911WriteCommand[0], 0, NULL, TRUE);
    AS3911_SEN_OFF();

    SET_CPU_IPL(current_cpu_ipl);

    if (ERR_NONE != error)
        return ERR_IO;
    else
        return ERR_NONE;
}

s8 as3911ReadTestRegister(u8 address, u8 *data)
{
    s8 error = ERR_NONE;
    int current_cpu_ipl = 0;
    u8 as3911ReadCommand[2] = { AS3911_SPI_CMD_DIREC_CMD | AS3911_CMD_TEST_ACCESS, AS3911_SPI_CMD_READ_REGISTER | (address & AS3911_SPI_ADDRESS_MASK) };

    SET_AND_SAVE_CPU_IPL(current_cpu_ipl, 7);

    AS3911_SEN_ON();
    error |= spiRxTx(2, &as3911ReadCommand[0], 1, data, TRUE);
    AS3911_SEN_OFF();

    SET_CPU_IPL(current_cpu_ipl);

    if (ERR_NONE != error)
        return ERR_IO;
    else
        return ERR_NONE;
}

s8 as3911ModifyRegister(u8 address, u8 mask, u8 data)
{
    s8 error = ERR_NONE;
    u8 registerValue = 0;

    error |= as3911ReadRegister(address, &registerValue);
    registerValue = (registerValue & ~mask) | data;
    error |= as3911WriteRegister(address, registerValue);

    if (ERR_NONE == error)
        return ERR_NONE;
    else
        return ERR_IO;
}

s8 as3911ContinuousWrite(u8 address, const u8 *data, u8 length)
{
    s8 error = ERR_NONE;
    int current_cpu_ipl = 0;

    SET_AND_SAVE_CPU_IPL(current_cpu_ipl, 7);

    if (length == 0)
        return ERR_NONE;

    u8 as3911WriteCommand = AS3911_SPI_CMD_WRITE_REGISTER | (address & AS3911_SPI_ADDRESS_MASK);
    AS3911_SEN_ON();
    error |= spiRxTx(1, &as3911WriteCommand, 0, NULL, FALSE);
    error |= spiRxTx(length, data, 0, NULL, TRUE);
    AS3911_SEN_OFF();

    SET_CPU_IPL(current_cpu_ipl);

    if (ERR_NONE != error)
        return ERR_IO;
    else
        return ERR_NONE;
}

s8 as3911ContinuousRead(u8 address, u8 *data, u8 length)
{
    s8 error = ERR_NONE;
    int current_cpu_ipl = 0;
    u8 as3911ReadCommand = AS3911_SPI_CMD_READ_REGISTER | (address & AS3911_SPI_ADDRESS_MASK);

    SET_AND_SAVE_CPU_IPL(current_cpu_ipl, 7);

    AS3911_SEN_ON();
    error |= spiRxTx(1, &as3911ReadCommand, length, data, TRUE);
    AS3911_SEN_OFF();

    SET_CPU_IPL(current_cpu_ipl);

    if (ERR_NONE != error)
        return ERR_IO;
    else
        return ERR_NONE;
}

s8 as3911WriteFifo(const u8 *data, u8 length)
{
    s8 error = ERR_NONE;
    int current_cpu_ipl = 0;
    u8 as3911WriteFifoCommand = AS3911_SPI_CMD_WRITE_FIFO;

    if (0 == length)
        return ERR_NONE;

    SET_AND_SAVE_CPU_IPL(current_cpu_ipl, 7);

    AS3911_SEN_ON();
    error |= spiRxTx(1, &as3911WriteFifoCommand, 0, NULL, TRUE);
    error |= spiRxTx(length, data, 0, NULL, TRUE);
    AS3911_SEN_OFF();

    SET_CPU_IPL(current_cpu_ipl);

    if (ERR_NONE != error)
        return ERR_IO;
    else
        return ERR_NONE;
}

s8 as3911ReadFifo(u8 *data, u8 length)
{
    s8 error = ERR_NONE;
    int current_cpu_ipl = 0;
    u8 as3911ReadFifoCommand = AS3911_SPI_CMD_READ_FIFO;

    if (length == 0)
        return ERR_NONE;

    SET_AND_SAVE_CPU_IPL(current_cpu_ipl, 7);

    AS3911_SEN_ON();
    error |= spiRxTx(1, &as3911ReadFifoCommand, length, data, TRUE);
    AS3911_SEN_OFF();

    SET_CPU_IPL(current_cpu_ipl);

    if (ERR_NONE != error)
        return ERR_IO;
    else
        return ERR_NONE;
}

s8 as3911ExecuteCommand(u8 directCommand)
{
    s8 error = ERR_NONE;
    int current_cpu_ipl = 0;
    u8 as3911DirectCommand = AS3911_SPI_CMD_DIREC_CMD | (directCommand & AS3911_SPI_ADDRESS_MASK);

    SET_AND_SAVE_CPU_IPL(current_cpu_ipl, 7);

    AS3911_SEN_ON();
    error |= spiRxTx(1, &as3911DirectCommand, 0, NULL, TRUE);
    AS3911_SEN_OFF();

    SET_CPU_IPL(current_cpu_ipl);

    if (ERR_NONE != error)
        return ERR_IO;
    else
        return ERR_NONE;
}

/*
******************************************************************************
* LOCAL FUNCTIONS
******************************************************************************
*/
