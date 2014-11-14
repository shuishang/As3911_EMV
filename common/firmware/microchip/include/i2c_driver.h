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
 *      PROJECT:   AS3668 firmware
 *      $Revision: $
 *      LANGUAGE:  ANSI C
 */

/*! \file i2c_driver.h
 *
 *  \author M. Arpa
 *  \author Wolfgang Reichart
 *
 *  \brief abstraction for the i2c driver
 *
 *  This is the definition file for the i2c driver. The i2c driver is an i2c master
 *  driver using the i2c 2 module of the pic.
 *
 */

/*!
 * \defgroup i2cdriver
 *
 */

#ifndef I2C_DRIVER_H
#define I2C_DRIVER_H

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
#define I2C1_MODULE                 0x01    /** I2C Mocule 1 selection. */
#define I2C2_MODULE                 0x02    /** I2C Mocule 2 selection. */

#define I2C_ADDRESS_MODE_7_BIT      0x01    /** I2C 7-bit address mode. */
#define I2C_ADDRESS_MODE_10_BIT     0x02    /** I2C 10-bit address mode. */

#define I2C_SCK_100KHZ              0x01    /** SCK frequency set to 100kHz. */
#define I2C_SCK_400KHZ              0x02    /** SCK frequency set to 400kHz. */
#define I2C_SCK_1000KHZ             0x03    /** SCK frequency set to 1MHz. */
#define I2C_SCK_3400KHZ             0x04    /** SCK frequency set to 3.4MHz. */

/*
******************************************************************************
* STRUCTS, TYPEDEFS
******************************************************************************
*/
struct i2cConfigData
{
    u8  i2cModule;          /** Specifies the module ID. */
    u8  i2cAddressMode;     /** Specifies 7 or 10 bit mode. */
    u8  i2cClockMode;       /** Specifies the SCL frequency between 100kHz and 3.4MHz in 100kHz steps. */
};

typedef struct i2cConfigData i2cConfig_t;

/*
******************************************************************************
* GLOBAL FUNCTION PROTOTYPES
******************************************************************************
*/

/*! \ingroup i2c
 *****************************************************************************
 *  \brief Read configuration in from a byte-stream (the byte stream must
 * have been written with i2cSerialiseConfig.
 *
 *  \param[OUT] config : structure to be filled with the data read in
 *  \param[IN]  data : byte stream containing the data to be converted to the
 *  config struct
 *****************************************************************************
 */
void i2cDeserialiseConfig( i2cConfig_t * config, const u8 * data );

/*! \ingroup i2c
 *****************************************************************************
 *  \brief Write configuration to a byte-stream (the byte stream must
 * be read in with i2cDeserialiseConfig.
 *
 *  \param[IN] config : structure to filled with the data to be written
 *  \param[OUT] data : byte stream to be filled with the data from the
 *  config struct
 *****************************************************************************
 */
void i2cSerialiseConfig( const i2cConfig_t * config, u8 * data );

/*! \ingroup i2cdriver
 *****************************************************************************
 *  \brief  Initializes the i2c driver module.
 *
 *  Initializes the i2c driver by using the microchip pheripheral library.
 *  The old configuration is returned in cfgDataOut structure.
 *
 *  \param *cfgDataIn : pointer to config parameters input structure
 *  \param *cfgDataOut : pointer to config parameters output structure
 *****************************************************************************
 */
extern s8 i2cInitialize ( u32 sysClk, const i2cConfig_t* cfgDataIn, i2cConfig_t *cfgDataOut );

/*! \ingroup i2cdriver
 *****************************************************************************
 *  \brief  Returns the number of transmitted bytes.
 *
 *  Transmits the given byte stream over the i2c interface as an i2c master.
 *
 *  \param slaveAddr : the i2c address of the slave (address is left-shifted
 *         before transmitting it)
 *  \param data : the byte stream to be transmitted
 *  \param numberOfBytesToTx : the number of bytes to be sent
 *
 *  \return xx : the number of bytes transmitted
 *****************************************************************************
 */
extern u16 i2cTx ( u8 slaveAddr, const u8 * data, u16 numberOfBytesToTx );


/*! \ingroup i2cdriver
 *****************************************************************************
 *  \brief  Returns the number of received bytes.
 *
 *  Transmits the given byte stream over the i2c interface as an i2c master and
 *  receives up to numberOfBytesToRx bytes over i2c.
 *
 *  \param slaveAddr : the i2c address of the slave
 *  \param txData : the byte stream to be transmitted
 *  \param numberOfBytesToTx : the number of bytes to be sent
 *  \param rxData : destination of the received bytes
 *  \param numberOfBytesToRx : the number of bytes to be received
 *
 *  \return xx : the number of bytes received
 *****************************************************************************
 */
extern u16 i2cRx ( u8 slaveAddr, const u8 * txData, u16 numberOfBytesToTx, u8 * rxData, u16 numberOfBytesToRx );

/*! \ingroup i2cdriver
 *****************************************************************************
 *  \brief  Performs a raw I2C transfer
 *
 *  Transmits the given byte stream over the i2c interface as an i2c master and
 *  receives numberOfBytesToRx bytes over i2c. Start- and stop-conditions are generated
 *  on demand.
 *
 *  \param numberOfBytesToTx : number of bytes to TX
 *  \param txData  : the byte stream to be transmitted
 *  \param numberOfBytesToRx  : the number of bytes to be received
 *  \param rxData   : the buffer to store the received data
 *  \param sendStartCond  : if true, a start condition is generated
 *  \param sendStopCond   : if true, a stop condition is generated
 *
 *  \return status : ERR_NONE on success, ERR_IO otherwise
 *****************************************************************************
 */
extern s8 i2cRxTx ( u16 numberOfBytesToTx, const u8 * txData, u16 numberOfBytesToRx, u8 * rxData, BOOL sendStartCond, BOOL sendStopCond );

#endif /* I2C_DRIVER_H */



