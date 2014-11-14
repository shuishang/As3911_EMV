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
 *          M. Arpa
 *
 *  \brief USB HID streaming driver implementation.
 *
 */

/*
 ******************************************************************************
 * INCLUDES
 ******************************************************************************
 */
#include "ams_types.h"
#include "GenericTypeDefs.h"
#include "errno.h"
#include "usb_hid_stream_driver.h"
#include "usb_appl_handler.h"
#include "usb.h"
#include "usb_function_hid.h"
#include "ams_types.h"
#include "logger.h"
#include <stdio.h>
/* FCY needed by libpic30.h, pessimistically assume the maximum, needed for __delay functions */
#ifndef FCY
    #define FCY 16000000ULL
#endif
#include <libpic30.h>

/*
 ******************************************************************************
 * extern declarations
 ******************************************************************************
 */

extern USB_HANDLE USBOutHandle;
extern USB_HANDLE USBInHandle;

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */

/*
 ******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************
 */
u8 rxBuffer[USB_COM_BUFFER_SIZE]; /*! buffer to store protocol packets received from the USB Host */
u8 txBuffer[USB_COM_BUFFER_SIZE]; /*! buffer to store protocol packets which are transmitted to the USB Host */


/*
 ******************************************************************************
 * LOCAL VARIABLES
 ******************************************************************************
 */
static u8 txTid;
static u8 rxTid;

static u16 rxSize;
static u8 * rxEnd; /* pointer to next position where to copy the received data */

static u32 systemClock;


/*
 ******************************************************************************
 * LOCAL FUNCTIONS
 ******************************************************************************
 */

static s8 haveAnotherPacket( )
{
    return (  rxSize >= USB_COM_STREAM_HEADER_SIZE
	   && rxSize >= ( USB_HID_PROTOCOL_RX_LENGTH( rxBuffer ) + USB_COM_STREAM_HEADER_SIZE )
	   );
}

static void eatUpPacket( u8 rxed )
{
        /*
         * FIXME: knowing that a memmove() is not the most performant variant,
         * it is used here to keep the code simpler.
         * If we run into performance issues on the PIC, this can be replaced
         * by pointer operations.
         */
   rxed += USB_COM_STREAM_HEADER_SIZE;
   /* decrease remaining data length by length of consumed packet */
   rxSize -= rxed;

   memmove( rxBuffer, rxBuffer + rxed, rxSize );
   /* correct end pointer to the new end of the buffer */
   rxEnd = rxBuffer + rxSize;
}

static u8 handleReset( u8 * rxData, u8 rxed )
{
    u8 status = AMS_COM_STATUS_OK;
   
    DBG_ASSERT( rxed == 1); /* a reset command consists of 2 bytes, command byte and flags  */
    /* at position 0 after the reset command we find the device selection flag */
    switch( *rxData )
    {
        case AMS_COM_RESET_MCU:
            /* reset the PIC */
            USB_LOG("MCU RESET\n");
#ifdef __DEBUG
            DBG_ASSERT(0);
#endif
            USBDeviceDetach();
            __delay_ms(100); /* to give host time to recognize detach event */
            Reset();
            break;
        case AMS_COM_RESET_PERIPHERAL:
            /* reset the peripheral(s), in this case the AS1130 */
            USB_LOG("PERI RESET\n");
            /* check if weak function applPeripheralReset() is linked */
            if (applPeripheralReset)
            {
                applPeripheralReset();
            }
            else
            {
                USB_LOG("WARN: applPeripheralReset() called but not implemented\n");
                status = AMS_COM_STATUS_ERR;
            }
            break;
        default:
            USB_LOG("UNKNOWN RESET\n");
            status = AMS_COM_STATUS_ERR;
            break;
        }
    return status;
}

static u8 handleGetFirmwareVersion( u8 * txData, u8 * toTx )
{
    u8 status = AMS_COM_STATUS_ERR;

    if (applGetFirmwareVersion)
    {
        u16 size = strlen(applGetFirmwareVersion());
        USB_LOG("FirmwareVersion\n");
        status = AMS_COM_STATUS_OK;
	/* + 1 for terminating '\0', + 1 for command byte */
	if ( size >= AMS_COM_BUFFER_SIZE - 2 )
	{
	    size = AMS_COM_BUFFER_SIZE - 2;
	}
	*toTx = size + 1; /* for zero terminator */
	memcpy( txData , applGetFirmwareVersion(), size + 1 );
    }
    else
    {
        USB_LOG("WARN: applGetFirmwareVersion() called but not implemented\n");
    }
    return status;
}

static u8 handleEnterBootloader( )
{
    /* write application ID to 0 */
    TBLPAG = 0; //0x2000>>16; -> this causes a compiler warning (right shift count >= width of type)
    __builtin_tblwtl(0x2000, 0x0000);   /* set address and value (0) */
    __builtin_tblwth(0x2000, 0x0000);
    NVMCON = 0x4001;                    /* NVM write address opcode */
    __builtin_write_NVM();
    while(NVMCONbits.WR == 1);
    USBDeviceDetach();
    __delay_ms(100); /* to give host time to recognize detach event */
    Reset();
    return AMS_COM_STATUS_OK;
}

/*
 * Here all application-specific commands can be executed if
 * the weak function applProcessCmd() is implemented.
 */
 static u8 handleApplicationCommand( u8 * rxData, u8 rxed, u8 * txData, u8 * toTx )
{
    u8 status = AMS_COM_STATUS_ERR;

    if (applProcessCmd)
    {
        /* hand in to the application only the payload of the command */
	status = applProcessCmd( rxData, rxed, txData, toTx );
    }
    else
    {
        USB_LOG("WARN: applProcessCmd() called but not implemented\n");
    }
    return status;
}

static u8 handleControlCommand ( u8 rxed, u8 * rxData, u8 * toTx, u8 * txData )
{
    u8 status = AMS_COM_STATUS_ERR;
    u8 command = *rxData;
    u8 * txDataCmd = txData + 1;
    u8 toTxCmd = ( *toTx > 0 ? *toTx - 1 : 0 );

    /* eat up received command */
    ++rxData;
    --rxed;

    USB_LOG("CMD %hhx\n", command );

    /* add your direct commands implementation(s) here */
    switch ( command )
    {
        case AMS_COM_CTRL_CMD_RESET:
  	    status = handleReset( rxData, rxed );
	    break;
        case AMS_COM_CTRL_CMD_GET_FW_VERSION:
    	    status = handleGetFirmwareVersion( txDataCmd, &toTxCmd );
	    break;
        case AMS_COM_CTRL_CMD_ENTER_BOOTLOADER:
            status = handleEnterBootloader( );
	    break;
        case AMS_COM_CTRL_CMD_APPL:
  	    status = handleApplicationCommand( rxData, rxed, txDataCmd, &toTxCmd );
	    break;
        default:
	  break;
    }

    /* if a response is expected, add the command byte and adjust the tx-size */
    if ( * toTx > 0 )
    {
        txData[ 0 ] = command;
	*toTx = toTxCmd + 1; 
    }
    return status;
}

/*
 ******************************************************************************
 * GLOBAL FUNCTIONS
 ******************************************************************************
 */

void usbStreamInitialize (u32 sysClk)
{
  /* so far we have received nothing */
    txTid = 0;
    rxTid = 0;
    rxSize = 0;
    rxEnd = rxBuffer;

    memset(rxBuffer, 0xCC, sizeof(rxBuffer));
    systemClock = sysClk;
}

u16 usbStreamReceive ( u8 * usbBuffer )
{
    if( !HIDRxHandleBusy(USBOutHandle) )				//Check if data was received from the host.
    { // if data was received, it is written to the usbBuffer 
 
        /*
         * Read the usb hid buffer into the local buffer.
         * When the RX-Length within the first streaming packet is 
         * longer than the HID payload, we have to concatenate several
         * packets (up to max. 256 bytes)
         */

        u8 packetSize;
	u8 payload  = USB_HID_PAYLOAD_SIZE( usbBuffer );
	rxTid = USB_HID_TID( usbBuffer );
	/* adjust number of totally received bytes */
	rxSize += payload;

        USB_LOG("HID-rx: tid=%hhx payload=%hhx, rxSize=%u\n", rxTid, payload, rxSize );
        USB_LOGDUMP( usbBuffer, USB_HID_REPORT_SIZE);

	/* add the new data at the end of the data in the rxBuffer (i.e. where rxEnd points to) */
	memcpy( rxEnd, USB_HID_PAYLOAD(usbBuffer), payload );
	rxEnd += payload;
	packetSize = USB_HID_PROTOCOL_RX_LENGTH(rxBuffer) + USB_COM_STREAM_HEADER_SIZE;

#if (USE_USB_LOGGER == LOGGER_ON)
        memset((void*)usbBuffer, 0xab, sizeof(usbBuffer));
#endif

        //Re-arm the OUT endpoint for the next packet
        USBOutHandle = HIDRxPacket(HID_EP,(BYTE*)usbBuffer,USB_HID_REPORT_SIZE);

	if ( packetSize > rxSize )
	{    /* indicate that we must continue receiving */
	     return 0;
	}
	return rxSize;
    }
    /* indicate that we did not receive anything - try next time again */
    return 0;
}

void usbStreamTransmit ( u16 totalTxSize, u8 * usbBuffer )
{
    u16 offset = 0;

    while ( totalTxSize > 0 )
    {
        u8 payload = ( totalTxSize > USB_HID_MAX_PAYLOAD_SIZE ? USB_HID_MAX_PAYLOAD_SIZE : totalTxSize );

        /* wait here (and before copying the IN-buffer) until the USBInHandle is free again */
        while (HIDTxHandleBusy(USBInHandle));

#if (USE_USB_LOGGER == LOGGER_ON)
        memset( usbBuffer, 0xCC, USB_HID_REPORT_SIZE);
#endif

        /* generate a new tid for tx */
	txTid++;
	txTid = ( rxTid << 4 ) | ( txTid & 0xF );

        /* TX-packet setup */
        USB_HID_TID(usbBuffer) = txTid;
	USB_HID_PAYLOAD_SIZE(usbBuffer) = payload;

	/* copy data to usb buffer */
        memcpy( USB_HID_PAYLOAD(usbBuffer), txBuffer + offset, payload);

	USB_LOG("HID-tx: tid=%hhx payload=%hhx, (totalSize=%hhx)\n", txTid, payload, totalTxSize );
        USB_LOGDUMP( usbBuffer, USB_HID_REPORT_SIZE);

        totalTxSize -= payload;
        offset += payload;

	/* initiate transfer now */
        USBInHandle = HIDTxPacket(HID_EP, (BYTE*)usbBuffer, USB_HID_REPORT_SIZE);
    }
}

u16 usbStreamProcessReceivedPackets ( )
{
    /* every time we enter this function, the last txBuffer was already sent.
       So we fill a new transfer buffer */
    u8 * txEnd = txBuffer; /* the txEnd always points to the next position to be filled with data */
    u16 txSize = 0;

    USB_LOG("ProcessReceivedPackets\n");

    while ( haveAnotherPacket( ) )
    {
        /* read out protocol header data */
        u8 protocol = USB_HID_PROTOCOL(rxBuffer);
        u8 rxed     = USB_HID_PROTOCOL_RX_LENGTH(rxBuffer);
        u8 toTx     = USB_HID_PROTOCOL_TX_LENGTH(rxBuffer);
	u8 * rxData = USB_HID_PROTOCOL_PAYLOAD(rxBuffer);
	/* set up tx pointer for any data to be transmitted back to the host */
	u8 * txData = USB_HID_PROTOCOL_PAYLOAD(txEnd);
	u8 status = AMS_COM_STATUS_OK;
	s8 isCallUnhandled = 1; 

        /* decide which protocol to execute */
	USB_LOG("Packet-rx: Protocol=%hhx, rxed=%hhx, toTx=%hhx\n", protocol, rxed, toTx );
        USB_LOGDUMP( rxBuffer, USB_COM_STREAM_HEADER_SIZE + rxed );
        switch (protocol)
        {
        case AMS_COM_I2C:
            if ( i2cRxTx )
            { /* if weak symbol is linked -> we call it */
	        isCallUnhandled = 0;
                status = i2cRxTx( rxed, rxData, toTx, txData, TRUE, TRUE);
            }
            break;
        case AMS_COM_I2C_CONFIG:
            if ( i2cInitialize )
            {
                i2cConfig_t config;
	        isCallUnhandled = 0;
		i2cDeserialiseConfig( &config, rxData );
                i2cInitialize(systemClock, &config, 0 /* oldConfig */ );
            }
            break;
        case AMS_COM_SPI:
            if ( spiRxTx )
            {
	        isCallUnhandled = 0;
                status = spiRxTx( rxed, rxData, toTx, txData, TRUE );
            }
            break;
        case AMS_COM_SPI_CONFIG:
            if (spiInitialize)
            {
                spiConfig_t config;
	        isCallUnhandled = 0;
		spiDeserialiseConfig( &config, rxData );
                spiInitialize( systemClock, &config, 0 );
            }
            break;
        case AMS_COM_CTRL_CMD:
	    isCallUnhandled = 0;
            status = handleControlCommand ( rxed, rxData, &toTx, txData );
            break;
        default:
            USB_LOG("UNKNOWN protocol\n");
            break;
        }

	if ( isCallUnhandled )
	{
	    USB_LOG("WARNING: Call was not handled!!!\n" );
	}

	/* fill out transmit packet header if any data was produced */
        if ( toTx > 0 )
        {
            USB_HID_PROTOCOL( txEnd )           = protocol;
            USB_HID_PROTOCOL_STATUS( txEnd )    = status;
            USB_HID_PROTOCOL_TX_LENGTH( txEnd ) = toTx;
            
            USB_LOG("Packet-tx: protocol=%hhx status=%hhx toTx=%hhx\n", protocol, status, toTx );
            USB_LOGDUMP(txEnd, toTx + USB_COM_STREAM_HEADER_SIZE );

	    /* adjust pointer to the enter next data, and total size */
	    txEnd += ( toTx + USB_COM_STREAM_HEADER_SIZE );
            txSize += (toTx + USB_COM_STREAM_HEADER_SIZE );
        }

	/* remove the handled packet, and move on to next packet */
	eatUpPacket( rxed ); 
    }
    return txSize;
}


u16 usbStreamProcessCyclic ( )
{
    /* every time we enter this function, the last txBuffer was already sent.
       So we fill a new transfer buffer */
    u8 * txEnd = txBuffer; /* the txEnd always points to the next position to be filled with data */
    u16 txSize = 0;
    u8 toTx;
    u8 protocol;
    u8 status = AMS_COM_STATUS_OK;
    
    if ( applProcessCyclic )
    {
        do
	{
	  /* set up tx pointer for any data to be transmitted back to the host */
          u8 * txData = USB_HID_PROTOCOL_PAYLOAD(txEnd);
	  toTx = 0;
	  status = applProcessCyclic( &protocol, txData, &toTx, AMS_COM_BUFFER_SIZE - txSize - USB_COM_STREAM_HEADER_SIZE );
	
	  /* fill out transmit packet header if any data was produced */
	  if ( toTx > 0 )
	  {
	      USB_LOG("ProcessCyclic\n"); 

              USB_HID_PROTOCOL( txEnd )           = protocol;
              USB_HID_PROTOCOL_STATUS( txEnd )    = status;
              USB_HID_PROTOCOL_TX_LENGTH( txEnd ) = toTx;
              
              USB_LOG("Packet-tx: protocol=%u status=%u toTx=%hhx\n", protocol, status, toTx );
              USB_LOGDUMP(txEnd, toTx + USB_COM_STREAM_HEADER_SIZE );
  
  	      /* adjust pointer to the enter next data, and total size */
              txEnd += ( toTx + USB_COM_STREAM_HEADER_SIZE );
              txSize += (toTx + USB_COM_STREAM_HEADER_SIZE );
	  }
	} while ( toTx > 0 );
    }
    return txSize;
}
