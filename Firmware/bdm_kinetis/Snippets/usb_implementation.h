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
static constexpr uint  CONTROL_EP_MAXSIZE      = 64; //!< Control in/out                64
static constexpr uint  BDM_OUT_EP_MAXSIZE      = 64; //!< BDM out                       64
static constexpr uint  BDM_IN_EP_MAXSIZE       = 64; //!< BDM in                        64

#if (HW_CAPABILITY&CAP_CDC)
static constexpr uint  CDC_CONTROL_EP_MAXSIZE  = 16; //!< CDC control                   16
static constexpr uint  CDC_DATA_OUT_EP_MAXSIZE = 16; //!< CDC data out                  16
static constexpr uint  CDC_DATA_IN_EP_MAXSIZE  = 16; //!< CDC data in (ping-pong)       16
#endif

#ifdef USBDM_USB0_IS_DEFINED
/**
 * Class representing USB0
 */
class Usb0 : public UsbBase_T<Usb0Info, CONTROL_EP_MAXSIZE> {
public:

   enum InterfaceNumbers {
      /** Interface number for BDM channel */
      BDM_INTF_ID,
   #if (HW_CAPABILITY&CAP_CDC)
      /** Interface number for CDC Control channel */
      CDC_COMM_INTF_ID,
      /** Interface number for CDC Data channel */
      CDC_DATA_INTF_ID,
   #endif
      /** Total number of interfaces */
      NUMBER_OF_INTERFACES,
   };

   /**
    * Endpoint numbers\n
    * Must be consecutive
    */
   enum EndpointNumbers {
      /** USB Control endpoint number - must be zero */
      CONTROL_ENDPOINT  = 0,

      /** BDM Control and Data out endpoint number */
      BDM_OUT_ENDPOINT,
      /** BDM Data in endpoint number */
      BDM_IN_ENDPOINT,

   #if (HW_CAPABILITY&CAP_CDC)
      /** CDC Control endpoint number */
      CDC_CONTROL_ENDPOINT,
      /** CDC Data out endpoint number */
      CDC_DATA_OUT_ENDPOINT,
      /** CDC Data in endpoint number */
      CDC_DATA_IN_ENDPOINT,
   #endif

      /** Total number of end-points */
      NUMBER_OF_ENDPOINTS,
   };

   /**
    * Endpoint numbers\n
    * Must be consecutive
    */
   enum Configurations {
     CONFIGURATION_NUM = 1,
     /*
      * Assumes single configuration
      */
   };

   /**
    * String descriptor table
    */
   static const uint8_t *const stringDescriptors[];

protected:
   static const OutEndpoint <Usb0Info, Usb0::BDM_OUT_ENDPOINT, BDM_OUT_EP_MAXSIZE> ep1;
   static const InEndpoint  <Usb0Info, Usb0::BDM_IN_ENDPOINT,  BDM_IN_EP_MAXSIZE>  ep2;

#if (HW_CAPABILITY&CAP_CDC)
   static const OutEndpoint  <Usb0Info, Usb0::CDC_CONTROL_ENDPOINT   CDC_CONTROL_EP_MAXSIZE>  ep3;
   static const InEndpoint   <Usb0Info, Usb0::CDC_DATA_OUT_ENDPOINT, CDC_DATA_OUT_EP_MAXSIZE> ep4;
   static const OutEndpoint  <Usb0Info, Usb0::CDC_DATA_IN_ENDPOINT,  CDC_DATA_IN_EP_MAXSIZE>  ep5;
#endif

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
      ep1.initialise();
      ep2.initialise();
#if (HW_CAPABILITY&CAP_CDC)
      ep3.initialise();
      ep4.initialise();
      ep5.initialise();
#endif
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
    * Handles ep0 [SETUP, IN & OUT]
    * Handles ep1 [Out]
    * Handles ep2 [In]
    * Handles ep3 [In]
    * Handles ep4 [Out]
    * Handles ep5 [In]
    */
   static void handleTokenComplete(void);

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
