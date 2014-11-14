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
 *      PROJECT:   AMS Streaming Communication 
 *      $Revision: $
 *      LANGUAGE: C++
 */

/*! \file AmsCom.h
 *
 *  \author M. Arpa
 *
 *  \brief All protocol numbers etc. are here defined.
 */

#ifndef AMS_COM_H
#define AMS_COM_H


/* hid protocol definition for PC: 

Byte   0      1      2       3         4         5        6         (6 + rx-prot)
    +-----+------+------+----------+---------+---------+------- ---+-------------+-----------+-----------+-------
    | RID | TID  | rxed | protocol | rx-prot | tx-prot | data      | protocol B  | rx-prot B | tx-prot B | data B ...
    +-----+------+------+----------+---------+---------+------- ---+-------------+-----------+-----------+-------

*/

/* hid procotol definition for microchip (here the RID = report ID is removed by the usb stack):

Byte   0      1      2          3         4         5          (5 + rx-prot)
    +------+------+----------+---------+---------+------- ---+-------------+-----------+-----------+-------
    | TID  | rxed | protocol | rx-prot | tx-prot | data      | protocol B  | rx-prot B | tx-prot B | data B ...
    +------+------+----------+---------+---------+------- ---+-------------+-----------+-----------+-------
*/

#define AMS_COM_DRIVER_HEADER_SIZE          3 /* currently the max-header size of the driver is 3 */
#define AMS_COM_STREAM_HEADER_SIZE          3 /* the stream class adds a header to each packet of this size */
#define AMS_COM_BUFFER_SIZE                 256

/* all communication protocols that use the AmsComStream for communication
   must define their protocol identifier here */
#define AMS_COM_I2C                         0x01
#define AMS_COM_I2C_CONFIG                  0x81

#define AMS_COM_SPI                         0x02
#define AMS_COM_SPI_CONFIG                  0x82

#define AMS_COM_CTRL_CMD                    0x03

/* a control command has in the first byte its command id */
#define AMS_COM_CTRL_CMD_RESET              0x01
#define AMS_COM_CTRL_CMD_GET_FW_VERSION     0x02
#define AMS_COM_CTRL_CMD_APPL               0x03
/* to be continued ...*/
#define AMS_COM_CTRL_CMD_ENTER_BOOTLOADER   0xEB
/* to be continued ...*/

#define AMS_COM_RESET_MCU                   0x01 /* to reset the MCU use this as the objectToReset parameter */
#define AMS_COM_RESET_PERIPHERAL            0x02 /* to reset all peripherals use this */

/* status values */
#define AMS_COM_STATUS_OK    0
#define AMS_COM_STATUS_ERR   1

#endif /* AMS_COM_H */
