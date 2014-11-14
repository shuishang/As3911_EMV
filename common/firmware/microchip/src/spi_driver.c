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
 *      $Revision: $
 *      LANGUAGE:  ANSI C
 */

/*! \file spi_driver.c
 *
 *  \author Wolfgang Reichart
 *
 *  \brief SPI driver for PIC24F.
 *
 */

/*
******************************************************************************
* INCLUDES
******************************************************************************
*/
#define USE_AND_OR
#include <p24Fxxxx.h>
#include <spi.h>
#include <string.h>
#include "ams_types.h"
#include "GenericTypeDefs.h"
#include "errno.h"
#include "spi_driver.h"
#include "logger.h"

/*
******************************************************************************
* LOCAL MACROS
******************************************************************************
*/
#define SPI_FIFO_DEPTH                  8

#define SPI_MAX_FREQ_CONF_TABLE_ENTRIES 5

/*
******************************************************************************
* LOCAL DATATYPES
******************************************************************************
*/
typedef struct
{
    u32 systemFrequency;
    u32 frequency;
    u32 actFrequency;
    u8 priPreScalerReg;
    u16 secPreScaler;
} spiFrequencyConfiguration_t;

/*
******************************************************************************
* LOCAL VARIABLES
******************************************************************************
*/
static umword spiTableIndex = 0;
static spiFrequencyConfiguration_t spiFreqConfTable[SPI_MAX_FREQ_CONF_TABLE_ENTRIES];

/*
******************************************************************************
* LOCAL VARIABLES
******************************************************************************
*/
static spiConfig_t myCfgData;

/*
******************************************************************************
* LOCAL FUNCTIONS
******************************************************************************
*/
/*
u16 spiReadStat()
{
    switch (myCfgData.instance)
    {
    case SPI1:
        return SPI1STAT;
    case SPI2:
        return SPI2STAT;
    default:
        return 0;
    }
}

u16 spiReadCon1()
{
    switch (myCfgData.instance)
    {
    case SPI1:
        return SPI1CON1;
    case SPI2:
        return SPI2CON1;
    default:
        return 0;
    }
}

u16 spiReadCon2()
{
    switch (myCfgData.instance)
    {
    case SPI1:
        return SPI1CON2;
    case SPI2:
        return SPI2CON2;
    default:
        return 0;
    }
}

u16 spiReadBuf()
{
    switch (myCfgData.instance)
    {
    case SPI1:
        return SPI1BUF;
    case SPI2:
        return SPI2BUF;
    default:
        return 0;
    }
}

void spiWriteStat(u16 value)
{
    switch (myCfgData.instance)
    {
    case SPI1:
        SPI1STAT = value;
        break;
    case SPI2:
        SPI2STAT = value;
        break;
    default:
        break;
    }
}

void spiWriteCon1(u16 value)
{
    switch (myCfgData.instance)
    {
    case SPI1:
        SPI1CON1 = value;
        break;
    case SPI2:
        SPI2CON1 = value;
        break;
    default:
        break;
    }
}

void spiWriteCon2(u16 value)
{
    switch (myCfgData.instance)
    {
    case SPI1:
        SPI1CON2 = value;
        break;
    case SPI2:
        SPI2CON2 = value;
        break;
    default:
        break;
    }
}

void spiWriteBuf(u16 value)
{
    switch (myCfgData.instance)
    {
    case SPI1:
        SPI1BUF = value;
        break;
    case SPI2:
        SPI2BUF = value;
        break;
    default:
        break;
    }
}
*/

#define spiReadStat() (SPI1STAT)
#define spiReadCon1() (SPI1CON1)
#define spiReadCon2() (SPI1CON2)
#define spiReadBuf() (SPI1BUF)
#define spiWriteStat(value) do { SPI1STAT = (value); } while(0)
#define spiWriteCon1(value) do { SPI1CON1 = (value); } while(0)
#define spiWriteCon2(value) do { SPI1CON2 = (value); } while(0)
#define spiWriteBuf(value) do { SPI1BUF = (value); } while(0)

/*
******************************************************************************
* GLOBAL FUNCTIONS
******************************************************************************
*/



void spiSerialiseConfig( const spiConfig_t * config, u8 * data )
{
    data[0] = config->frequency >> 24;
    data[1] = config->frequency >> 16;
    data[2] = config->frequency >> 8;
    data[3] = config->frequency;
    data[4] = config->instance;
    data[5] = config->deviceId;
    data[6] = config->clockPhase;
    data[7] = config->clockPolarity;
}

void spiDeserialiseConfig( spiConfig_t * config, const u8 * data )
{
    config->frequency = (((u32) data[0]) << 24) | (((u32) data[1]) << 16) | (((u32) data[2]) << 8) | (((u32) data[3]));
    config->instance = data[4];
    config->deviceId = data[5];
    config->clockPhase = data[6];
    config->clockPolarity = data[7];
}

s8 spiInitialize(u32 sysClk, const spiConfig_t* const spiConfigIn, spiConfig_t *spiConfigOut)
{
    u32 systemClock;
    u16 priprescaler;
    u8 priprescalerreg;
    u16 secprescaler;
    u32 actFrequency;
    spiFrequencyConfiguration_t* tblptr;
    umword i;
    BOOL configFound = FALSE;

    if (spiConfigOut != NULL)
    {
        spiConfigOut->clockPhase    = myCfgData.clockPhase;
        spiConfigOut->clockPolarity = myCfgData.clockPolarity;
        spiConfigOut->deviceId      = myCfgData.deviceId;
        spiConfigOut->frequency     = myCfgData.frequency;
        spiConfigOut->instance      = myCfgData.instance;
    }
    myCfgData.clockPhase    = spiConfigIn->clockPhase;
    myCfgData.clockPolarity = spiConfigIn->clockPolarity;
    myCfgData.deviceId      = spiConfigIn->deviceId;
    myCfgData.frequency     = spiConfigIn->frequency;
    myCfgData.instance      = spiConfigIn->instance;

    systemClock = sysClk;

    /* disable SPI for configuration */
    spiWriteStat(0);

    /* first check the cache table for already calculated frequencies */
    for (i = 0; i < SPI_MAX_FREQ_CONF_TABLE_ENTRIES; i++)
    {
        if ((spiFreqConfTable[i].frequency == myCfgData.frequency) && (spiFreqConfTable[i].systemFrequency == systemClock))
        {
            /* found a previously configured setting */
            configFound = TRUE;
            tblptr = &spiFreqConfTable[i];
            break;
        }
    }

    if (!configFound)
    {
        /* equation according to the datasheet for calculating spi clock speed:
         * fsck = SYSCLK / (pri_prescaler * sec_prescaler)
         */
        priprescaler = 1;
        priprescalerreg = 3;
        do
        {
            secprescaler = systemClock / (myCfgData.frequency * priprescaler);
            if (secprescaler > 8)
            {
                if (priprescaler >= 64)
                {
                    secprescaler = (secprescaler > 8) ? 8 : secprescaler;
                    break;
                }
                else
                {
                    priprescaler <<= 2;
                    priprescalerreg--;
                    continue;
                }
            }
            else
            {
                break;
            }

        }
        while(1);

        secprescaler = (secprescaler == 0) ? 1 : secprescaler;
        actFrequency = systemClock / (priprescaler * secprescaler);

        /* save this new configuration in the cache table */
        if (spiTableIndex >= SPI_MAX_FREQ_CONF_TABLE_ENTRIES)
        {
            spiTableIndex = 0;
        }
        spiFreqConfTable[spiTableIndex].actFrequency = actFrequency;
        spiFreqConfTable[spiTableIndex].frequency = myCfgData.frequency;
        spiFreqConfTable[spiTableIndex].priPreScalerReg = priprescalerreg;
        spiFreqConfTable[spiTableIndex].secPreScaler = secprescaler;
        spiFreqConfTable[spiTableIndex].systemFrequency = systemClock;
        spiTableIndex++;
    }
    else
    {
        actFrequency = tblptr->actFrequency;
        myCfgData.frequency = tblptr->frequency;
        priprescalerreg = tblptr->priPreScalerReg;
        secprescaler = tblptr->secPreScaler;
    }

    /* set pre-scaler */
    spiWriteCon1(priprescalerreg | ((8 - secprescaler) << 2));

    /* set clock and polarity */
    spiWriteCon1(spiReadCon1() |
                 (myCfgData.clockPhase << 8) |
                 (myCfgData.clockPolarity << 6));

    /* set enhanced buffer master mode and enable SPI */
    spiWriteCon2(1);
    spiWriteCon1(spiReadCon1() | (1 << 5));
    spiWriteStat((1 << 15));
	
    SPI_LOG( "\nSPI M=%hhx, des.f=%u%u, act.f=%u%u, clkPhase=%hhx, clkPol=%hhx, ss=%hhx\n"
            , myCfgData.instance
            , (u16)(myCfgData.frequency >> 16)
            , (u16)(myCfgData.frequency)
            , (u16)(actFrequency >> 16)
            , (u16)(actFrequency)
            , myCfgData.clockPhase
            , myCfgData.clockPolarity
            , myCfgData.deviceId
            );

    return ERR_NONE;
}

s8 spiDeinitialize()
{
    u8 tmp;

    /* clear receive buffer */
    if (spiReadStat() & 1)
    {
        tmp = spiReadBuf();
    }
    /* clear RX FIFO when SRXMPT bit is set (only valid in enhancled buffer mode) */
    while (!(spiReadStat() & (1 << 5)))
    {
        tmp = spiReadBuf();
    }
    /* clear receive overflow flag as it might be set */
    spiWriteStat(spiReadStat() & ~((u16)(1 << 6)));
    spiWriteStat(0);
    spiWriteCon1(0);
    spiWriteCon2(0);

    return ERR_NONE;
}

s8 spiTx(const u8* txData, u8* rxData, u16 length, BOOL sync)
{
    umword i;
    umword a;
    u16 ridx = 0;
    u16 widx = 0;
    u8 tmp;

    if (rxData != NULL)
    {
        while (!(spiReadStat() & (1 << 5)))
        {
            tmp = spiReadBuf();
        }
        /* clear receive overflow flag as it might be set */
        spiWriteStat(spiReadStat() & ~((u16)(1 << 6)));
    }

    while (length)
    {
        a = length > SPI_FIFO_DEPTH ? SPI_FIFO_DEPTH : length;
        SPI_LOG("SPI Write:");
        /* fill the FIFO */
        for (i = 0; i < a; i++)
        {
            /* loop as long as FIFO is full */
            while ((((spiReadStat()) >> 8) & 0x7) == 0x7)
                ;
            spiWriteBuf(txData[widx++]);
            SPI_LOG(" %hhx", txData[widx-1]);
        }
        SPI_LOG("\n");
        if ((rxData != NULL) || sync)
        {
            SPI_LOG("SPI Read:");
            for (i = 0; i < a; i++)
            {
                /* wait for received data */
                while (spiReadStat() & 0x20)
                    ;
                if (rxData != NULL)
                {
                    if (spiReadStat() & (1 << 6))
                    {
                        return ERR_IO;
                    }
                    rxData[ridx++] = spiReadBuf();
                    SPI_LOG(" %hhx", rxData[ridx-1]);
                }
                else
                {
                    tmp = spiReadBuf();
                }
            }
            SPI_LOG("\n");
        }
        length -= a;
    }

    if (sync)
    {
        spiSync();
    }

    return ERR_NONE;
}

s8 spiRxTx(u16 numberOfBytesToTx, const u8 * txData, u16 numberOfBytesToRx, u8 * rxData, BOOL sync)
{
    umword i;
    umword a;
    u16 ridx = 0;
    u16 rBufIdx = 0;
    u16 widx = 0;
    u16 length;
    u8 tmp;

    length = numberOfBytesToTx + numberOfBytesToRx; // overall length of transfer

    /* weak symbol spiActivateSEN must be defined by application and
     * must pull SEN line of corresponding spi device */
    if (spiActivateSEN)
    {
        spiActivateSEN(myCfgData.deviceId);
    }
    else
    {
        SPI_LOG("SPI: weak symbol spiActivateSEN must be defined by application and must pull SEN line of corresponding spi device\n");
    }

    if (rxData != NULL)
    {
        while (!(spiReadStat() & (1 << 5)))
        {
            tmp = spiReadBuf();
        }
        /* clear receive overflow flag as it might be set */
        spiWriteStat(spiReadStat() & ~((u16)(1 << 6)));
    }

    while (length)
    {
        a = length > SPI_FIFO_DEPTH ? SPI_FIFO_DEPTH : length;
        SPI_LOG("SPI Write:");
        /* fill the FIFO */
        for (i = 0; i < a; i++)
        {
            /* loop as long as FIFO is full */
            while ((((spiReadStat()) >> 8) & 0x7) == 0x7)
                ;
            if (widx >= numberOfBytesToTx)
            {
                spiWriteBuf(0xFF);
                widx++;
            }
            else
            {
                spiWriteBuf(txData[widx++]);
                SPI_LOG(" %hhx", txData[widx-1]);
            }
        }
        SPI_LOG("\n");
        if ((rxData != NULL) || sync)
        {
            SPI_LOG("SPI Read:");
            for (i = 0; i < a; i++)
            {
                /* wait for received data */
                while (spiReadStat() & 0x20)
                    ;
                if (rxData != NULL)
                {
                    if (spiReadStat() & (1 << 6))
                    {
                        return ERR_IO;
                    }
                    if (ridx >= numberOfBytesToTx)
                    {
                        rxData[rBufIdx++] = spiReadBuf();
                        SPI_LOG(" %hhx", rxData[rBufIdx-1]);
                        ridx++;
                    }
                    else
                    {
                        tmp = spiReadBuf();
                        ridx++;
                    }
                }
                else
                {
                    tmp = spiReadBuf();
                }
            }
            SPI_LOG("\n");
        }
        length -= a;
    }

    if (sync)
    {
        spiSync();
    }

    /* weak symbol spiDeactivateSEN must be defined by application and
     * must pull SEN line of corresponding spi device */
    if (spiDeactivateSEN)
    {
        spiDeactivateSEN(myCfgData.deviceId);
    }
    else
    {
        SPI_LOG("SPI: weak symbol spiDeactivateSEN must be defined by application and must pull SEN line of corresponding spi device\n");
    }
    
    return ERR_NONE;
}

s8 spiSync()
{
    u8 tmp;

    while (!(spiReadStat() & 0x80))
        ;
    while (!(spiReadStat() & (1 << 5)))
    {
        tmp = spiReadBuf();
    }
    /* clear receive buffer */
    if (spiReadStat() & 1)
    {
        tmp = spiReadBuf();
    }
    /* clear receive overflow flag as it might be set */
    spiWriteStat(spiReadStat() & ~((u16)(1 << 6)));

    return ERR_NONE;
}

