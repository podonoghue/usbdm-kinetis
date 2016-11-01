/*
 * usb_implementation.h
 *
 *  Created on: 30Oct.,2016
 *      Author: podonoghue
 *
 *  This file provides the implementation specific code for the USB interface.
 *  It will need to be modified to suit an application.
 */

#ifndef SOURCES_USB_IMPLEMENTATION_H_
#define SOURCES_USB_IMPLEMENTATION_H_

#include "usb.h"

namespace USBDM {

//======================================================================
// Maximum packet sizes for each endpoint
//
static constexpr uint  CONTROL_EP_MAXSIZE      = 64; //!< Control in/out  64

static constexpr uint  CDC_CONTROL_EP_MAXSIZE  = 16; //!< CDC control     16
static constexpr uint  CDC_DATA_OUT_EP_MAXSIZE = 16; //!< CDC data out    16
static constexpr uint  CDC_DATA_IN_EP_MAXSIZE  = 16; //!< CDC data in     16

#ifdef USBDM_USB0_IS_DEFINED
/**
 * Class representing USB0
 */
class Usb0 : public UsbBase_T<Usb0Info, CONTROL_EP_MAXSIZE> {
public:

   /**
    * Endpoint numbers\n
    * Must be consecutive
    */
   enum EndpointNumbers {
      /** USB Control endpoint number - must be zero */
      CONTROL_ENDPOINT  = 0,

      /** CDC Control endpoint number */
      CDC_CONTROL_ENDPOINT,
      /** CDC Data out endpoint number */
      CDC_DATA_OUT_ENDPOINT,
      /** CDC Data in endpoint number */
      CDC_DATA_IN_ENDPOINT,

      /** Total number of end-points */
      NUMBER_OF_ENDPOINTS,
   };

   /**
    * Configuration numbers, consecutive from 1
    */
   enum Configurations {
     CONFIGURATION_NUM = 1,
     /*
      * Assumes single configuration
      */
     /** Total number of configurations */
     NUMBER_OF_CONFIGURATIONS = CONFIGURATION_NUM,
   };

   /**
    * String descriptor table
    */
   static const uint8_t *const stringDescriptors[];

protected:
   static const InEndpoint  <Usb0Info, Usb0::CDC_CONTROL_ENDPOINT,  CDC_CONTROL_EP_MAXSIZE> epCdcControl;
   static const OutEndpoint <Usb0Info, Usb0::CDC_DATA_OUT_ENDPOINT, CDC_CONTROL_EP_MAXSIZE> epCdcDataOut;
   static const InEndpoint  <Usb0Info, Usb0::CDC_DATA_IN_ENDPOINT,  CDC_CONTROL_EP_MAXSIZE> epCdcDataIn;

public:
   /**
    * Initialise the USB interface
    */
   static void initialise();

   /**
    * Initialises all end-points
    */
   static void initialiseEndpoints(void) {
      UsbBase_T::initialiseEndpoints();
      epCdcControl.initialise();
      epCdcDataOut.initialise();
      epCdcDataIn.initialise();
   }

   /**
    * Handler for USB interrupt
    *
    * Determines source and dispatches to appropriate routine.
    */
   static void irqHandler(void);

   /**
    * Handler for Token Complete USB interrupt
    *
    * Handles ep0 [SETUP, IN & OUT] - passed to UsbBase_T::handleTokenCompleteEp0
    * Other end-points as needed
    */
   static void handleTokenComplete(void);

   /**
    * Handle CDC requests
    */
   static void handleCdcEp0(const SetupPacket &setup);

   static void handleSetLineCoding();
   static void handleGetLineCoding();
   static void handleSetControlLineState();
   static void handleSendBreak();

   /**
    * Device Descriptor
    */
   static const DeviceDescriptor deviceDescriptor;

   /**
    * Other descriptors
    */
   struct Descriptors;

   /**
    * Other descriptors
    */
   static const Descriptors otherDescriptors;
};

using UsbImplementation = Usb0;
#endif // USBDM_USB0_IS_DEFINED

} // End namespace USBDM

#endif /* SOURCES_USB_IMPLEMENTATION_H_ */
