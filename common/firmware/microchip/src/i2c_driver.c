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

/*! \file i2c_driver.c
 *
 *  \author M. Arpa
 *  \author Wolfgang Reichart
 *
 *  \brief i2c driver implementation for Microchip PIC24F series
 *
 *
 *  This is the implementation file for the i2c driver which supports the
 *  Microchip PIC24F series.
 *
 */

/*!
 * \defgroup i2cdriver
 * i2c driver module
 */
/*
******************************************************************************
* INCLUDES
******************************************************************************
*/
#define USE_AND_OR
#include <p24Fxxxx.h>
#include <i2c.h>
#include <string.h>
#include "ams_types.h"
#include "GenericTypeDefs.h"
#include "errno.h"
#include "i2c_driver.h"
#include "logger.h"

/*
******************************************************************************
* DEFINES
******************************************************************************
*/

#if (SYSCLK == SYSCLK_16MHZ)
    #define BRG_FOR_100KHZ          0x9D
    #define BRG_FOR_400KHZ          0x25
    #define BRG_FOR_1000KHZ         0x0D
    #define BRG_FOR_3400KHZ         0x01
#else
    #error "SYSCLK value is not SYSCLK_16MHZ"
#endif

/*
******************************************************************************
* LOCAL VARIABLES
******************************************************************************
*/
static i2cConfig_t myCfgData;
static u32 systemClock;

/*
******************************************************************************
* LOCAL FUNCTIONS
******************************************************************************
*/
static u8 getI2cBrgValue (u8 clockMode)
{
    u8 i2cxBrg;
    switch(clockMode)
    {
    case I2C_SCK_100KHZ:
        i2cxBrg = BRG_FOR_100KHZ;
        break;
    case I2C_SCK_400KHZ:
        i2cxBrg = BRG_FOR_400KHZ;
        break;
    case I2C_SCK_1000KHZ:
        i2cxBrg = BRG_FOR_1000KHZ;
        break;
    case I2C_SCK_3400KHZ:
        i2cxBrg = BRG_FOR_3400KHZ;
        break;
    default:
        i2cxBrg = BRG_FOR_100KHZ;
        break;
    }
    return i2cxBrg;
}

static void i2cStart (void)
{
    if (I2C1_MODULE == myCfgData.i2cModule)
    {
        IdleI2C1(); /* wait for idle bus */
        StartI2C1();
        while (I2C1CONbits.SEN); /* wait until start sequence is completed */
    }
    else if (I2C2_MODULE == myCfgData.i2cModule)
    {
        IdleI2C2(); /* wait for idle bus */
        StartI2C2();
        while (I2C2CONbits.SEN); /* wait until start sequence is completed */
    }
    else
    {
        DBG_ASSERT(0);
    }

}

static void i2cStop (void)
{
    if (I2C1_MODULE == myCfgData.i2cModule)
    {
        StopI2C1(); /* send stop condition */
        while (I2C1CONbits.PEN); /* wait until stop sequence is completed */
    }
    else if (I2C2_MODULE == myCfgData.i2cModule)
    {
        StopI2C2(); /* send stop condition */
        while (I2C2CONbits.PEN); /* wait until stop sequence is completed */
    }
    else
    {
        DBG_ASSERT(0);
    }
}

static void i2cRestart (void)
{
    if (I2C1_MODULE == myCfgData.i2cModule)
    {
        RestartI2C1(); /* repeated start */
        while (I2C1CONbits.RSEN); /* wait until start sequence is completed */
    }
    else if (I2C2_MODULE == myCfgData.i2cModule)
    {
        RestartI2C2(); /* repeated start */
        while (I2C2CONbits.RSEN); /* wait until start sequence is completed */
    }
    else
    {
        DBG_ASSERT(0);
    }
}

static void i2cIdle (void)
{
    if (I2C1_MODULE == myCfgData.i2cModule)
    {
        IdleI2C1();
    }
    else if (I2C2_MODULE == myCfgData.i2cModule)
    {
        IdleI2C2();
    }
    else
    {
        DBG_ASSERT(0);
    }
}

static u8 i2cWriteByte (u8 data)
{
    u8 retVal = 0;
    
    if (I2C1_MODULE == myCfgData.i2cModule)
    {
        MasterWriteI2C1(data);
        while (I2C1STATbits.TRSTAT); /* wait until byte is ack/nak ed */
        if (I2C1STATbits.ACKSTAT)
        {
            retVal = -1; /* NAK case */
        }
    }
    else if (I2C2_MODULE == myCfgData.i2cModule)
    {
        MasterWriteI2C2(data);
        while (I2C2STATbits.TRSTAT); /* wait until byte is ack/nak ed */
        if (I2C2STATbits.ACKSTAT)
        {
            retVal = -1; /* NAK case */
        }
    }
    else
    {
        DBG_ASSERT(0);
    }
    return retVal;
}

static u16 i2cReadNBytes (u16 nRead, u8 *read)
{
    u16 rxed = 0;

    if (I2C1_MODULE == myCfgData.i2cModule)
    {
        rxed = MastergetsI2C1(nRead, read, 10000);
    }
    else if (I2C2_MODULE == myCfgData.i2cModule)
    {
        rxed = MastergetsI2C2(nRead, read, 10000);
    }
    else
    {
        DBG_ASSERT(0);
    }
    return (rxed == 0 ? nRead : 0);
}


static u16 i2cTxAllButStopCondition (u8 slaveAddr, const u8 * txData, u16 numberOfBytesToTx)
{
    u16 txed = 0; /* failure */

    if (I2C1_MODULE == myCfgData.i2cModule)
    {
        IdleI2C1(); /* wait for idle bus */
        StartI2C1();
        while (I2C1CONbits.SEN); /* wait until start sequence is completed */
        MasterWriteI2C1(slaveAddr << 1);
        while (I2C1STATbits.TRSTAT); /* wait until address is txed and ack/nak rxed */
        if (I2C1STATbits.ACKSTAT == 0) /* address accepted by slave */
        {
            /* transmit data */
            while(txed < numberOfBytesToTx)
            {
                MasterWriteI2C1(*txData);
                while (I2C1STATbits.TRSTAT); /* wait until byte is ack/nak ed */
                if (I2C1STATbits.ACKSTAT)
                {
                    break; /* early exit */
                }
                txed++;
                txData++;
            }
        }
        IdleI2C1();
    }
    else if (I2C2_MODULE == myCfgData.i2cModule)
    {
        IdleI2C2(); /* wait for idle bus */
        StartI2C2();
        while (I2C2CONbits.SEN); /* wait until start sequence is completed */
        MasterWriteI2C2(slaveAddr << 1);
        while (I2C2STATbits.TRSTAT); /* wait until address is txed and ack/nak rxed */
        if (I2C2STATbits.ACKSTAT == 0) /* address accepted by slave */
        {
            /* transmit data */
            while(txed < numberOfBytesToTx)
            {
                MasterWriteI2C2(*txData);
                while (I2C2STATbits.TRSTAT); /* wait until byte is ack/nak ed */
                if (I2C2STATbits.ACKSTAT)
                {
                    break; /* early exit */
                }
                txed++;
                txData++;
            }
        }
        IdleI2C2();
    }
    return txed;
}

/*
******************************************************************************
* GLOBAL FUNCTIONS
******************************************************************************
*/

void i2cDeserialiseConfig( i2cConfig_t * config, const u8 * data )
{
    config->i2cModule = data[0];
    config->i2cAddressMode = data[1];
    config->i2cClockMode = data[2];
}

void i2cSerialiseConfig( const i2cConfig_t * config, u8 * data )
{
    data[0] = config->i2cModule;
    data[1] = config->i2cAddressMode;
    data[2] = config->i2cClockMode;
}


s8 i2cInitialize (u32 sysClk, const i2cConfig_t* const cfgDataIn, i2cConfig_t *cfgDataOut)
{
    u16 myI2cAddressMode = I2C_7BIT_ADD;
    u8  i2cxBrg;
    s8 retVal = ERR_NONE;

    // FIXME: currently we assume a sysclk of 16MHz!! For further use
    systemClock = sysClk;

    if (cfgDataOut != NULL)
    {
        /* fill outgoing config with current configuration values */
        cfgDataOut->i2cAddressMode = myCfgData.i2cAddressMode;
        cfgDataOut->i2cClockMode   = myCfgData.i2cClockMode;
        cfgDataOut->i2cModule      = myCfgData.i2cModule;
    }
    myCfgData.i2cAddressMode = ((i2cConfig_t*)cfgDataIn)->i2cAddressMode;
    myCfgData.i2cClockMode   = ((i2cConfig_t*)cfgDataIn)->i2cClockMode;
    myCfgData.i2cModule      = ((i2cConfig_t*)cfgDataIn)->i2cModule;

    if (I2C_ADDRESS_MODE_10_BIT == myCfgData.i2cAddressMode)
    {
        myI2cAddressMode = I2C_10BIT_ADD;
    }

    i2cxBrg = getI2cBrgValue(myCfgData.i2cClockMode);
    
    if (I2C1_MODULE == myCfgData.i2cModule)
    {
        /* configure i2c1 block to be i2c master */
        CloseI2C1(); /* make sure i2c is off before (re-)configuring */
        OpenI2C1((I2C_ON | myI2cAddressMode) , i2cxBrg);
        IdleI2C1();
    }
    else if (I2C2_MODULE == myCfgData.i2cModule)
    {
        /* configure i2c2 block to be i2c master */
        CloseI2C2(); /* make sure i2c is off before (re-)configuring */
        OpenI2C2((I2C_ON | myI2cAddressMode) , i2cxBrg);
        IdleI2C2();
    }
    else
    {
        /* assertion due to parameter error */
        DBG_ASSERT(0);
        retVal = ERR_PARAM;
    }
    I2C_LOG("\nI2C-C M=%hhx, A=%hhx, C=%hhx\n", myCfgData.i2cModule, myCfgData.i2cAddressMode, myCfgData.i2cClockMode);
    return retVal;
}

u16 i2cTx (u8 slaveAddr, const u8 * data, u16 numberOfBytesToTx)
{
    u16 txed = i2cTxAllButStopCondition(slaveAddr, data, numberOfBytesToTx);
    if (I2C1_MODULE == myCfgData.i2cModule)
    {
        StopI2C1(); /* send stop condition */
        while (I2C1CONbits.PEN); /* wait until stop sequence is completed */
    }
    else if (I2C2_MODULE == myCfgData.i2cModule)
    {
        StopI2C2(); /* send stop condition */
        while (I2C2CONbits.PEN); /* wait until stop sequence is completed */
    }
    return txed;
}


u16 i2cRx (u8 slaveAddr, const u8 * txData, u16 numberOfBytesToTx, u8 * rxData, u16 numberOfBytesToRx)
{
    u16 rxed = 0; /* failure */
    u16 txed = i2cTxAllButStopCondition(slaveAddr, txData, numberOfBytesToTx);
    if (txed != 1)
    {
        if (I2C1_MODULE == myCfgData.i2cModule)
        {
            StopI2C1(); /* send stop condition */
            while (I2C1CONbits.PEN); /* wait until stop sequence is completed */
        }
        else if (I2C2_MODULE == myCfgData.i2cModule)
        {
            StopI2C2();
            while (I2C2CONbits.PEN);
        }
        return rxed;
    }
    if (I2C1_MODULE == myCfgData.i2cModule)
    {
        RestartI2C1(); /* repeated start */
        while (I2C1CONbits.RSEN); /* wait until start sequence is completed */
        MasterWriteI2C1(slaveAddr << 1 | 1); /* read has LSB = 1 */
        while (I2C1STATbits.TRSTAT); /* wait until address is txed and ack/nak rxed */
        if (I2C1STATbits.ACKSTAT == 0) /* address accepted by slave */
        {
            /* receive data */
            //MI2C1_Clear_Intr_Status_Bit;
            rxed = MastergetsI2C1(numberOfBytesToRx, rxData, 10000);
        }
        IdleI2C2();
        StopI2C2(); /* send stop condition */
        while (I2C2CONbits.PEN); /* wait until stop sequence is completed */
    }
    else if (I2C2_MODULE == myCfgData.i2cModule)
    {
        RestartI2C2(); /* repeated start */
        while (I2C2CONbits.RSEN); /* wait until start sequence is completed */
        MasterWriteI2C2(slaveAddr << 1 | 1); /* read has LSB = 1 */
        while (I2C2STATbits.TRSTAT); /* wait until address is txed and ack/nak rxed */
        if (I2C2STATbits.ACKSTAT == 0) /* address accepted by slave */
        {
            /* receive data */
            //MI2C2_Clear_Intr_Status_Bit;
            rxed = MastergetsI2C2(numberOfBytesToRx, rxData, 10000);
        }
        IdleI2C2();
        StopI2C2(); /* send stop condition */
        while (I2C2CONbits.PEN); /* wait until stop sequence is completed */
    }
    return (rxed == 0 ? numberOfBytesToRx : 0);
}

s8 i2cRxTx ( u16 numberOfBytesToTx, const u8 * txData, u16 numberOfBytesToRx, u8 * rxData, BOOL sendStartCond, BOOL sendStopCond )
{
    u16 txed = 0;
    u16 rxed = 0;
    u16 toTx;

    I2C_LOG("\nI2C-T W=%hhx, R=%hhx, STA=%hhx, STP=%hhx\n", numberOfBytesToTx, numberOfBytesToRx, sendStartCond, sendStopCond);
    I2C_LOGDUMP(txData, numberOfBytesToTx);

    /* 1.) check if wen need to send a start condition. */
    if (sendStartCond)
    {
        /* send a start cond */
        i2cStart();
        if (numberOfBytesToRx)
        {
            /*
             * In case this is a read command, we need to resend the device address.
             * The device address (prepared for reading) is already prepared on the host side.
             * -> Decrement the write count because the last entry in the write buffer is the
             * device address prepared for reading.
             */
            numberOfBytesToTx--;
        }
    }
    
    toTx = numberOfBytesToTx;

    /* 2.) do the write transfer */
    for (; numberOfBytesToTx > 0; numberOfBytesToTx--, txData++)
    {
        if (i2cWriteByte(*txData))
        {
            i2cStop();
            I2C_LOG("I2C-T(-1)\n");
            return ERR_IO; /* this was a NAK */
        }
        txed++;
    }
    i2cIdle();

    /* 3.) do the read transfer if necessary */
    if (numberOfBytesToRx)
    {
        if (sendStartCond)
        {
            i2cRestart();
            /* send the device address (already prepared in buffer on the host side) */
            if (i2cWriteByte(*txData))
            {
                i2cStop();
                I2C_LOG("I2C-T(-2)\n");
                return ERR_IO; /* this was a NAK */
            }
        }
        rxed = i2cReadNBytes(numberOfBytesToRx, rxData);
        I2C_LOG("I2CR\n");
        I2C_LOGDUMP(rxData, numberOfBytesToRx);
    }
    i2cIdle();

    /* 4.) send a top condition if necessary */
    if (sendStopCond)
    {
        i2cStop();
    }
    I2C_LOG("I2C-T rxed=%hhx, nRead=%hhx, txed=%hhx, toTx=%hhx\n", rxed, numberOfBytesToRx, txed, toTx);
    return ((rxed == numberOfBytesToRx && txed == toTx) ? ERR_NONE : ERR_IO);
}

