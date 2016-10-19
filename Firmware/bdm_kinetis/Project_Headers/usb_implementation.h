/*
 * usb_implementation.h
 *
 *  Created on: 19Oct.,2016
 *      Author: podonoghue
 */

#ifndef PROJECT_HEADERS_USB_IMPLEMENTATION_H_
#define PROJECT_HEADERS_USB_IMPLEMENTATION_H_

#include "usb.h"

class Usb : public USBDM::UsbBase_T<USBDM::Usb0Info> {

   static void initialise();
};

#endif /* PROJECT_HEADERS_USB_IMPLEMENTATION_H_ */
