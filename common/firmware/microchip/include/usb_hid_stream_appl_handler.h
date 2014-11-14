/********************************************************************
 FileName:     	usb_appl_handler.h
 Dependencies: 	Always: GenericTypeDefs.h, usb_device.h
               	Situational: usb_function_hid.h, usb_function_cdc.h, usb_function_msd.h, etc.
 Processor:		PIC18 or PIC24 USB Microcontrollers
 Hardware:		The code is natively intended to be used on the following
 				hardware platforms: PICDEM� FS USB Demo Board,
 				PIC18F87J50 FS USB Plug-In Module, or
 				Explorer 16 + PIC24 USB PIM.  The firmware may be
 				modified for use on other USB platforms by editing the
 				HardwareProfile.h file.
 Complier:  	Microchip C18 (for PIC18) or C30 (for PIC24)
 Company:		Microchip Technology, Inc.

 Software License Agreement:

 The software supplied herewith by Microchip Technology Incorporated
 (the �Company�) for its PIC� Microcontroller is intended and
 supplied to you, the Company�s customer, for use solely and
 exclusively on Microchip PIC Microcontroller products. The
 software is owned by the Company and/or its supplier, and is
 protected under applicable copyright laws. All rights are reserved.
 Any use in violation of the foregoing restrictions may subject the
 user to criminal sanctions under applicable laws, as well as to
 civil liability for the breach of the terms and conditions of this
 license.

 THIS SOFTWARE IS PROVIDED IN AN �AS IS� CONDITION. NO WARRANTIES,
 WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.

********************************************************************
 File Description:

 Change History:
  Rev   Date         Description
  1.0   11/19/2004   Initial release
  2.1   02/26/2007   Updated for simplicity and to use common
                     coding style
 *******************************************************************/

/*********************************************************************
 * Descriptor specific type definitions are defined in: usbd.h
 ********************************************************************/

#ifndef USB_APPL_HANDLER_H
#define USB_APPL_HANDLER_H

#include "ams_types.h"

void initUsbApplHandler (u32 sysClk);
void ProcessIO (void);
u16 USBTransmit (u8 tid, u8 * txData, u16 txLength);

#endif //USB_APPL_HANDLER_H
