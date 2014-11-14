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
 *      PROJECT:   USB PIC24F HID streaming firmware
 *      $Revision: $
 *      LANGUAGE:  ANSI C
 */

/*! \file usb_hid_stream_driver.h
 *
 *  \author Wolfgang Reichart
 *
 *  \brief USB HID streaming driver declarations.
 *
 */

/*!
 * \defgroup usbstreamdriver
 *
 */
#ifndef _USB_HID_STREAM_DRIVER_H
#define _USB_HID_STREAM_DRIVER_H
/*
 ******************************************************************************
 * INCLUDES
 ******************************************************************************
 */
#include "ams_types.h"
#include "AmsCom.h"
#include "Compiler.h"
#include "i2c_driver.h"
#include "spi_driver.h"

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */
#define USB_HID_REPORT_ID               0
#define USB_HID_REPORT_SIZE             64  /** the USB-HID report size is 0x40 (refer to usb_descriptors.c). */
#define USB_HID_HEADER_SIZE             2   /** ths PIC USB HID stack hides the report ID at position 0 */
#define USB_HID_MAX_PAYLOAD_SIZE        (USB_HID_REPORT_SIZE - USB_HID_HEADER_SIZE)

#define USB_COM_DRIVER_HEADER_SIZE      (AMS_COM_DRIVER_HEADER_SIZE-1) /** report ID not available in out-buffer */
#define USB_COM_STREAM_HEADER_SIZE      AMS_COM_STREAM_HEADER_SIZE
#define USB_COM_BUFFER_SIZE             (AMS_COM_BUFFER_SIZE+USB_COM_STREAM_HEADER_SIZE)


/* HID header macros for the microchip */
#define USB_HID_TID(buf)          (buf[0])
#define USB_HID_PAYLOAD_SIZE(buf) (buf[1])
#define USB_HID_PAYLOAD(buf)      ((buf)+2)

/* protocol header macros for rx and tx - seen from the microchip - the <buf> must always point to the next protocol packet */
#define USB_HID_PROTOCOL(buf)           (buf[0])
#define USB_HID_PROTOCOL_RX_LENGTH(buf) (buf[1])
#define USB_HID_PROTOCOL_STATUS(buf)    (buf[1])
#define USB_HID_PROTOCOL_TX_LENGTH(buf) (buf[2])
#define USB_HID_PROTOCOL_PAYLOAD(buf)   ((buf)+3)


/*
 ******************************************************************************
 * GLOBAL PROTCOL I/O FUNCTIONS
 ******************************************************************************
 */
/*
 * I2C Config and Transfer functions (weak declarations)
 * Implemented in module microchip/common/i2c
 */
extern void WEAK i2cDeserialiseConfig( i2cConfig_t * config, const u8 * data );
extern s8 WEAK i2cInitialize (u32 sysClk, const i2cConfig_t* cfgDataIn, i2cConfig_t *cfgDataOut );
extern s8 WEAK i2cRxTx (u16 numberOfBytesToTx, const u8 * txData, u16 numberOfBytesToRx, u8 * rxData, BOOL sendStartCond, BOOL sendStopCond);
/*
 * SPI Config and Transfer functions (weak declarations)
 * Implemented in module microchip/common/spi
 */
extern void WEAK spiDeserialiseConfig( spiConfig_t * config, const u8 * data );
extern s8 WEAK spiInitialize(u32 sysClk, const spiConfig_t* spiConfigIn, spiConfig_t* spiConfigOut);
extern s8 WEAK spiRxTx(u16 numberOfBytesToTx, const u8 * txData, u16 numberOfBytesToRx, u8 * rxData, BOOL sync);

/*
 ******************************************************************************
 * GLOBAL APPLICATION SPECIFIC FUNCTIONS
 * THESE FUNCTIONS CAN BE IMPLEMENTED IF REQUIRED
 ******************************************************************************
 */
/*! \ingroup usbstreamdriver
 *****************************************************************************
 *  \brief  reset peripherals of board
 *
 *  Weak function which can be implemented by the application to reset
 *  the board peripherals.
 *****************************************************************************
 */
extern void WEAK applPeripheralReset(void);

/*! \ingroup usbstreamdriver
 *****************************************************************************
 *  \brief  get firmware version string
 *
 *  Weak function which can be implemented by the application to return the
 *  application version string (zero-terminated).
 *****************************************************************************
 */
extern const char * WEAK applGetFirmwareVersion(void);

/*! \ingroup usbstreamdriver
 *****************************************************************************
 *  \brief  Command dispatcher for application-specific commands
 *
 *  Weak function which can be implemented by the application process application-
 *  specific commands for I/O. If data should be returned, the txData buffer
 *  can be filled with data which should be sent to the PC. In argument txSize,
 *  the size is returned. 
 *  \param[IN] rxData : pointer to payload for appl commands (in stream protocol buffer).
 *  \param[IN] rxSize : size of rxData
 *  \param[out] txData : pointer to buffer to store returned data (payload only)
 *  \param[out] txSize : size of returned data
 *  \return the status byte to be interpreted by the stream layer on the host
 *****************************************************************************
 */
extern u8 WEAK applProcessCmd ( const u8 * rxData, u8 rxSize, u8 * txData, u8 * txSize);

/*! \ingroup usbstreamdriver
 *****************************************************************************
 *  \brief  Called cyclic (even when no full usb packet was received). Use
 *          this is you need to send several packets (time delayed) in 
 *          response to one usb request.
 *
 *  Weak function which can be implemented by the application process 
 *  If data should be returned, the txData buffer must be filled with the data to
 *  be sent to the PC. In argument txSize, the size is returned. The function
 *  also must fill in the protocol byte (this is the protocol value that is
 *  filled in in the protocol header. 
 *  \param[out] protocol : protocol byte to be used for the packet header
 *  \param[out] txData : pointer to buffer to store returned data (payload only)
 *  \param[out] txSize : size of returned data
 *  \param[in]  remainingSize : how many bytes are free in the buffer txData
 *  \return the status byte to be interpreted by the stream layer on the host
 *****************************************************************************
 */
extern u8 WEAK applProcessCyclic ( u8 * protocol, u8 * txData, u8 * txSize, u8 remainingSize );

/*
 ******************************************************************************
 * GLOBAL FUNCTIONS
 ******************************************************************************
 */


/*! \ingroup usbstreamdriver
 *****************************************************************************
 *  \brief  initializes the USB HID Stream driver variables
 *
 *  This function takes care for proper initialisation of buffers, variables, etc.
 *
 *  \param sysClk : the MCU clock frequency
 *  \return n/a
 *****************************************************************************
 */
void usbStreamInitialize (u32 sysClk);

/*! \ingroup usbstreamdriver
 *****************************************************************************
 *  \brief checks if there is data received on the HID device from the host
 *  and copies the received data into a local buffer
 *
 *  Checks if the usb HID device has data received from the host, copies this
 *  data into a local buffer. The data in the local buffer is than interpreted
 *  as a packet (with header, rx-length and tx-length). As soon as a full 
 *  packet is received the function returns non-null. 
 *  
 *  \param IN usbBuffer : pointer to the usb RX buffer containing the HID report
 *  \return 0 = nothing to process, >0 at least 1 packet to be processed
 *****************************************************************************
 */
u16 usbStreamReceive (u8 * rxData);

/*! \ingroup usbstreamdriver
 *****************************************************************************
 *  \brief checks if there is data to be transmitted from the HID device to
 *  the host.
 *
 *  Checks if there is data waiting to be transmitted to the host. Copies this
 *  data from a local buffer to the usb buffer and transmits this usb buffer.
 *  If more than 1 usb hid report is needed to transmit the data, the function
 *  waits until the first one is sent, and than refills the usb buffer with
 *  the next chunk of data and transmits the usb buffer again. And so on, until
 *  all data is sent.
 * 
 *  \param IN totalTxSize: the size of the data to be transmitted (the HID
 *  header is not included)
 *  \param IN usbBuffer: pointer to the usb buffer to be used for transmission
 *****************************************************************************
 */
void usbStreamTransmit ( u16 totalTxSize, u8 * usbBuffer );

/*! \ingroup usbstreamdriver
 *****************************************************************************
 *  \brief  Processes the protocol specific commands when at least one protocol
 *  packet has been fully received .
 *
 * The protocol byte is interpreted, and the corresponding task is executed
 * (I2C Transfer, I2C Config, Control-Commands, ...)
 *
 *  \return size of byte to be transmitted
 *****************************************************************************
 */
u16 usbStreamProcessReceivedPackets ( );

/*! \ingroup usbstreamdriver
 *****************************************************************************
 *  \brief  Calls the application function applProcessCyclic if it exists
 *
 * checks if the applProcessCyclic function exists and calls this if it does.
 * As long as the function applProcessCyclic wants to transmit some data, 
 * this function builds a packet from the returned data and calls the 
 * applProcessCyclic function again. 
 * Note that all packtes are concatenated in the local buffer (so make sure
 * you do not use more than AMS_COM_BUFFER_SIZE bytes).
 *
 *  \return size of byte to be transmitted
 *****************************************************************************
 */
u16 usbStreamProcessCyclic ( );

#endif // _USB_HID_STREAM_DRIVER_H

