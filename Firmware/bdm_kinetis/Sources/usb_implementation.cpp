/*
 * usb_implementation.cpp
 *
 *  Created on: 30Oct.,2016
 *      Author: podonoghue
 */
#include <string.h>

#include "configure.h"
#include "usb.h"
#include "usb_cdc.h"

namespace USBDM {

static constexpr int VendorID      = 0x16D0;
static constexpr int ProductID_CDC = 0x8888;
static constexpr int VersionID     = 0x0100;

enum InterfaceNumbers {
   /** Interface number for CDC Control channel */
   CDC_COMM_INTF_ID,
   /** Interface number for CDC Data channel */
   CDC_DATA_INTF_ID,
   /** Total number of interfaces */
   NUMBER_OF_INTERFACES,
};

/**
 * String indexes
 *
 * Must agree with stringDescriptors[] order
 */
enum StringIds {
   /** Language information for string descriptors */
   s_language_index=0,    // Must be zero
   /** Manufacturer */
   s_manufacturer_index,
   /** Product Description */
   s_product_index,
   /** Serial Number */
   s_serial_index,

   /** Name of CDC interface */
   s_cdc_interface_index,
   /** CDC Control Interface */
   s_cdc_control_interface_index,
   /** CDC Data Interface */
   s_cdc_data_Interface_index,

   /** Marks last entry */
   s_last_string_descriptor_index
};

/*
 * String descriptors
 */
static const uint8_t s_language[]        = {4, DT_STRING, 0x09, 0x0C};   //!< Language IDs
static const uint8_t s_manufacturer[]    = "pgo";                        //!< Manufacturer
static const uint8_t s_product[]         = ProductDescription;           //!< Product Description
static const uint8_t s_serial[]          = SERIAL_NO;                    //!< Serial Number

static const uint8_t s_cdc_interface[]    = "USBDM CDC Interface";       //!< Interface Association #2
static const uint8_t s_cdc_control[]      = "CDC Control Interface";     //!< CDC Control Interface
static const uint8_t s_cdc_data[]         = "CDC Data Interface";        //!< CDC Data Interface

/**
 * String descriptor table
 */
const uint8_t *const Usb0::stringDescriptors[] = {
      s_language,
      s_manufacturer,
      s_product,
      s_serial,
      s_cdc_interface,
      s_cdc_control,
      s_cdc_data
};

/**
 * Device Descriptor (Composite)
 */
const DeviceDescriptor Usb0::deviceDescriptor = {
      /* bLength             */ sizeof(DeviceDescriptor),
      /* bDescriptorType     */ DT_DEVICE,
      /* bcdUSB              */ nativeToLe16(0x0200),           // USB specification release No. [BCD = 2.00]
      /* bDeviceClass        */ 0x02,                           // Device Class code [CDC Device Class]
      /* bDeviceSubClass     */ 0x00,                           // Sub Class code    [none]
      /* bDeviceProtocol     */ 0x00,                           // Protocol          [none]
      /* bMaxPacketSize0     */ CONTROL_EP_MAXSIZE,             // EndPt 0 max packet size
      /* idVendor            */ nativeToLe16(VendorID),         // Vendor ID
      /* idProduct           */ nativeToLe16(ProductID_CDC),    // Product ID for Composite device
      /* bcdDevice           */ nativeToLe16(VersionID),        // Device Release    [BCD = 4.10]
      /* iManufacturer       */ s_manufacturer_index,           // String index of Manufacturer name
      /* iProduct            */ s_product_index,                // String index of product description
      /* iSerialNumber       */ s_serial_index,                 // String index of serial number
      /* bNumConfigurations  */ Usb0::NUMBER_OF_CONFIGURATIONS  // Number of configurations
};

/**
 * Other descriptors
 */
struct Usb0::Descriptors {
   ConfigurationDescriptor                  configDescriptor;

   InterfaceDescriptor                      cdc_CCI_Interface;
   CDCHeaderFunctionalDescriptor            cdc_Functional_Header;
   CDCCallManagementFunctionalDescriptor    cdc_CallManagement;
   CDCAbstractControlManagementDescriptor   cdc_Functional_ACM;
   CDCUnionFunctionalDescriptor             cdc_Functional_Union;
   EndpointDescriptor                       cdc_notification_Endpoint;

   InterfaceDescriptor                      cdc_DCI_Interface;
   EndpointDescriptor                       cdc_dataOut_Endpoint;
   EndpointDescriptor                       cdc_dataIn_Endpoint;
};

/**
 * All other descriptors
 */
const Usb0::Descriptors Usb0::otherDescriptors =
{
      { // configDescriptor
            /* bLength                 */ sizeof(ConfigurationDescriptor),
            /* bDescriptorType         */ DT_CONFIGURATION,
            /* wTotalLength            */ nativeToLe16(sizeof(otherDescriptors)),
            /* bNumInterfaces          */ NUMBER_OF_INTERFACES,
            /* bConfigurationValue     */ Usb0::CONFIGURATION_NUM,
            /* iConfiguration          */ 0,
            /* bmAttributes            */ 0x80,     //  = Bus powered, no wakeup (yet?)
            /* bMaxPower               */ USBMilliamps(500)
      },
      /**
       * CDC Control/Communication Interface, 1 end-point
       */
      { // cdc_CCI_Interface
            /* bLength                 */ sizeof(InterfaceDescriptor),
            /* bDescriptorType         */ DT_INTERFACE,
            /* bInterfaceNumber        */ CDC_COMM_INTF_ID,
            /* bAlternateSetting       */ 0,
            /* bNumEndpoints           */ 1,
            /* bInterfaceClass         */ 0x02,      //  CDC Communication
            /* bInterfaceSubClass      */ 0x02,      //  Abstract Control Model
            /* bInterfaceProtocol      */ 0x01,      //  V.25ter, AT Command V.250
            /* iInterface description  */ s_cdc_control_interface_index
      },
      { // cdc_Functional_Header
            /* bFunctionalLength       */ sizeof(CDCHeaderFunctionalDescriptor),
            /* bDescriptorType         */ CS_INTERFACE,
            /* bDescriptorSubtype      */ DST_HEADER,
            /* bcdCDC                  */ nativeToLe16(0x0110),
      },
      { // cdc_CallManagement
            /* bFunctionalLength       */ sizeof(CDCCallManagementFunctionalDescriptor),
            /* bDescriptorType         */ CS_INTERFACE,
            /* bDescriptorSubtype      */ DST_CALL_MANAGEMENT,
            /* bmCapabilities          */ 1,
            /* bDataInterface          */ CDC_DATA_INTF_ID,
      },
      { // cdc_Functional_ACM
            /* bFunctionalLength       */ sizeof(CDCAbstractControlManagementDescriptor),
            /* bDescriptorType         */ CS_INTERFACE,
            /* bDescriptorSubtype      */ DST_ABSTRACT_CONTROL_MANAGEMENT,
            /* bmCapabilities          */ 0x06,
      },
      { // cdc_Functional_Union
            /* bFunctionalLength       */ sizeof(CDCUnionFunctionalDescriptor),
            /* bDescriptorType         */ CS_INTERFACE,
            /* bDescriptorSubtype      */ DST_UNION_MANAGEMENT,
            /* bmControlInterface      */ CDC_COMM_INTF_ID,
            /* bSubordinateInterface0  */ {CDC_DATA_INTF_ID},
      },
      { // cdc_notification_Endpoint - IN,interrupt
            /* bLength                 */ sizeof(EndpointDescriptor),
            /* bDescriptorType         */ DT_ENDPOINT,
            /* bEndpointAddress        */ EP_IN|CDC_CONTROL_ENDPOINT,
            /* bmAttributes            */ ATTR_INTERRUPT,
            /* wMaxPacketSize          */ nativeToLe16(CDC_CONTROL_EP_MAXSIZE),
            /* bInterval               */ USBMilliseconds(255)
      },
      /**
       * CDC Data Interface, 2 end-points
       */
      { // cdc_DCI_Interface
            /* bLength                 */ sizeof(InterfaceDescriptor),
            /* bDescriptorType         */ DT_INTERFACE,
            /* bInterfaceNumber        */ CDC_DATA_INTF_ID,
            /* bAlternateSetting       */ 0,
            /* bNumEndpoints           */ 2,
            /* bInterfaceClass         */ 0x0A,                         //  CDC DATA
            /* bInterfaceSubClass      */ 0x00,                         //  -
            /* bInterfaceProtocol      */ 0x00,                         //  -
            /* iInterface description  */ s_cdc_data_Interface_index
      },
      { // cdc_dataOut_Endpoint - OUT,bulk
            /* bLength                 */ sizeof(EndpointDescriptor),
            /* bDescriptorType         */ DT_ENDPOINT,
            /* bEndpointAddress        */ EP_OUT|CDC_DATA_OUT_ENDPOINT,
            /* bmAttributes            */ ATTR_BULK,
            /* wMaxPacketSize          */ nativeToLe16(CDC_DATA_OUT_EP_MAXSIZE),
            /* bInterval               */ USBMilliseconds(1)
      },
      { // cdc_dataIn_Endpoint - IN,bulk
            /*  bLength                */ sizeof(EndpointDescriptor),
            /*  bDescriptorType        */ DT_ENDPOINT,
            /*  bEndpointAddress       */ EP_IN|CDC_DATA_IN_ENDPOINT,
            /*  bmAttributes           */ ATTR_BULK,
            /*  wMaxPacketSize         */ nativeToLe16(2*CDC_DATA_IN_EP_MAXSIZE), // x2 so all packets are terminating (short))
            /*  bInterval              */ USBMilliseconds(1)
      },
};

/**
 * Handler for Token Complete USB interrupt
 *
 * Handles ep0 [SETUP, IN & OUT] - passed to UsbBase_T::handleTokenCompleteEp0
 * Other end-points as needed
 */
void Usb0::handleTokenComplete(void) {

   // Status from Token
   uint8_t   usbStat  = usb->STAT;

   // Endpoint number
   uint8_t   endPoint = ((uint8_t)usbStat)>>4;

   if (endPoint == CONTROL_ENDPOINT) {
      // EP0
      UsbBase_T::handleTokenCompleteEp0();
      return;
   }
   switch (endPoint) {
#if (HW_CAPABILITY&CAP_CDC)
      case CDC_CONTROL_ENDPOINT: // USBDM CDC Control - Accept IN token
         PRINTF("EP3\n");
         epHardwareState[CDC_CONTROL_ENDPOINT].data0_1 = !epHardwareState[CDC_CONTROL_ENDPOINT].data0_1; // Toggle data0/1
         ep3StartTxTransaction();
         return;
      case CDC_DATA_OUT_ENDPOINT: // USBDM CDC Data - Accept OUT token
         if (cdc_txBufferIsFree()) {
            ep4SaveRxData();
            ep4InitialiseBdtRx();
         }
         else {
            //! Throttle endpoint - send NAKs
            epHardwareState[CDC_DATA_OUT_ENDPOINT].state = EPThrottle;
         }
         return;
      case CDC_DATA_IN_ENDPOINT:  // USBD CDC Data - Accept IN token
         epHardwareState[CDC_DATA_IN_ENDPOINT].state = EPIdle;
         //          ep5StartInTransactionIfIdle();
         return;
#endif
   }
}
/**
 * Initialise the USB0 interface
 *
 *  @note Assumes clock set up for USB operation (48MHz)
 */
void Usb0::initialise() {
   UsbBase_T::initialise();
   // Other initialisation as required
   epCdcControl.initialise();
   epCdcDataOut.initialise();
   epCdcDataIn.initialise();

   // Add extra handling of CDC packets directed to EP0
   setSetupCallback(handleCdcEp0);
}

/**
 * Handler for USB0 interrupt
 *
 * Determines source and dispatches to appropriate routine.
 */
void Usb0::irqHandler() {
   // All active flags
   uint8_t interruptFlags = usb->ISTAT;

   //   if (interruptFlags&~USB_ISTAT_SOFTOK_MASK) {
   //      PRINTF("ISTAT=%2X\n", interruptFlags);
   //   }

   // Get active and enabled interrupt flags
   uint8_t enabledInterruptFlags = interruptFlags & usb->INTEN;

   if ((enabledInterruptFlags&USB_ISTAT_USBRST_MASK) != 0) {
      // Reset signaled on Bus
      handleUSBReset();
      return;
   }
   if ((enabledInterruptFlags&USB_ISTAT_TOKDNE_MASK) != 0) {
      // Token complete interrupt?
      handleTokenComplete();
      usb->ISTAT = USB_ISTAT_TOKDNE_MASK; // Clear source
   }
   else if ((enabledInterruptFlags&USB_ISTAT_RESUME_MASK) != 0) {
      // Resume signaled on Bus?
      handleUSBResume();
      usb->ISTAT = USB_ISTAT_RESUME_MASK; // Clear source
   }
   else if ((enabledInterruptFlags&USB_ISTAT_STALL_MASK) != 0) {
      // Stall sent?
      handleStallComplete();
      usb->ISTAT = USB_ISTAT_STALL_MASK; // Clear source
   }
   else if ((enabledInterruptFlags&USB_ISTAT_SOFTOK_MASK) != 0) {
      // SOF Token?
      handleSOFToken();
      usb->ISTAT = USB_ISTAT_SOFTOK_MASK; // Clear source
   }
   else if ((enabledInterruptFlags&USB_ISTAT_SLEEP_MASK) != 0) {
      // Bus Idle 3ms? => sleep
      //      PUTS("Suspend");
      handleUSBSuspend();
      usb->ISTAT = USB_ISTAT_SLEEP_MASK; // Clear source
   }
   else if ((enabledInterruptFlags&USB_ISTAT_ERROR_MASK) != 0) {
      // Any Error
      PRINTF("Error s=0x%02X\n", usb->ERRSTAT);
      usb->ERRSTAT = 0xFF;
      usb->ISTAT = USB_ISTAT_ERROR_MASK; // Clear source
   }
   else  {
      // Unexpected interrupt
      // Clear & ignore
      PRINTF("Unexpected interrupt, flags=0x%02X\n", interruptFlags);
      usb->ISTAT = interruptFlags; // Clear & ignore
   }
}

void Usb0::handleSetLineCoding() {

   auto callback = []{
         cdc_setLineCoding((LineCodingStructure * const)ep0.getBuffer());
   };
   ep0.setCallback(callback);

   // Don't use buffer - this requires sizeof(LineCodingStructure) < CONTROL_EP_MAXSIZE
   ep0.startRxTransaction(sizeof(LineCodingStructure), nullptr);
}

void Usb0::handleGetLineCoding() {
   // Send packet
   ep0StartTxTransaction( sizeof(LineCodingStructure), (const uint8_t*)cdc_getLineCoding());
}

void Usb0::handleSetControlLineState() {
   cdc_setControlLineState(ep0SetupBuffer.wValue.lo());
   ep0StartTxTransaction( 0, nullptr ); // Tx empty Status packet
}

void Usb0::handleSendBreak() {
   cdc_sendBreak(ep0SetupBuffer.wValue);  // time in milliseconds, 0xFFFF => continuous
   ep0StartTxTransaction( 0, nullptr );   // Tx empty Status packet
}

void Usb0::handleCdcEp0(const SetupPacket &setup) {
   switch(REQ_TYPE(setup.bmRequestType)) {
      case REQ_TYPE_CLASS :
         // Class requests
         switch (setup.bRequest) {
            case SET_LINE_CODING :           handleSetLineCoding();              break;
            case GET_LINE_CODING :           handleGetLineCoding();              break;
            case SET_CONTROL_LINE_STATE:     handleSetControlLineState();        break;
            case SEND_BREAK:                 handleSendBreak();                  break;
            default :                        ep0.stall();                        break;
         }
         break;
      default:
         ep0.stall();
         break;
   }
}

void idleLoop() {
   for(;;) {
      __asm__("nop");
   }
}


#if (HW_CAPABILITY&CAP_CDC)
//======================================================================
// Configure EP3 for an IN transaction [Tx, device -> host, DATA0/1]
//
static void ep3StartTxTransaction() {
   const CDCNotification cdcNotification= {CDC_NOTIFICATION, SERIAL_STATE, 0, RT_INTERFACE, nativeToLe16(2)};
   uint8_t status = cdc_getSerialState();

   if ((status & SERIAL_STATE_CHANGE) == 0) {
      epHardwareState[CDC_CONTROL_ENDPOINT].state = EPIdle; // Not busy
      return;
   }
   // Copy the Tx data to Tx buffer
   (void)memcpy(cdcControlDataBuffer, &cdcNotification, sizeof(cdcNotification));
   cdcControlDataBuffer[sizeof(cdcNotification)+0] = status&~SERIAL_STATE_CHANGE;
   cdcControlDataBuffer[sizeof(cdcNotification)+1] = 0;

   // Set up to Tx packet
   BdtEntry *bdt = epHardwareState[CDC_CONTROL_ENDPOINT].txOdd?&endPointBdts[CDC_CONTROL_ENDPOINT].txOdd:&endPointBdts[CDC_CONTROL_ENDPOINT].txEven;
   bdt->bc = sizeof(cdcNotification)+2;
   if (epHardwareState[CDC_CONTROL_ENDPOINT].data0_1) {
      bdt->u.bits = BDTEntry_OWN_MASK|BDTEntry_DATA1_MASK|BDTEntry_DTS_MASK;
   }
   else {
      bdt->u.bits = BDTEntry_OWN_MASK|BDTEntry_DATA0_MASK|BDTEntry_DTS_MASK;
   }
   epHardwareState[CDC_CONTROL_ENDPOINT].state = EPLastIn;    // Sending one and only packet
}

/**
 *  Configure the BDT for EP4 Out [Rx, device <- host, DATA0/1]
 *  CDC - OUT
 */
static void ep4InitialiseBdtRx() {
   BdtEntry *bdt = epHardwareState[CDC_DATA_OUT_ENDPOINT].rxOdd?&endPointBdts[CDC_DATA_OUT_ENDPOINT].rxOdd:&endPointBdts[CDC_DATA_OUT_ENDPOINT].rxEven;

   // Set up to Rx packet
   bdt->bc = CDC_DATA_OUT_EP_MAXSIZE;
   if (epHardwareState[CDC_DATA_OUT_ENDPOINT].data0_1) {
      bdt->u.bits  = BDTEntry_OWN_MASK|BDTEntry_DATA1_MASK|BDTEntry_DTS_MASK;
   }
   else {
      bdt->u.bits  = BDTEntry_OWN_MASK|BDTEntry_DATA0_MASK|BDTEntry_DTS_MASK;
   }
}

/**
 * Save the data from an EP4 OUT packet
 * CDC - OUT
 */
static void ep4SaveRxData() {
   // Get BDT
   BdtEntry *bdt = (!epHardwareState[CDC_DATA_OUT_ENDPOINT].rxOdd)?&endPointBdts[CDC_DATA_OUT_ENDPOINT].rxOdd:&endPointBdts[CDC_DATA_OUT_ENDPOINT].rxEven;
   uint8_t size = bdt->bc;
   (void)cdc_putTxBuffer((char*)cdcOutDataBuffer, size);

   // Toggle on successful reception
   epHardwareState[CDC_DATA_OUT_ENDPOINT].data0_1 = !epHardwareState[CDC_DATA_OUT_ENDPOINT].data0_1;
}

/**
 * Configure the BDT for EP5 In [Tx, device -> host]
 * CDC - IN
 */
static void ep5InitialiseBdtTx(void) {
   // Set up to Tx packet
   if (epHardwareState[CDC_DATA_IN_ENDPOINT].txOdd) {
      // Set to write to other buffer & get count in current buffer
      endPointBdts[CDC_DATA_IN_ENDPOINT].txOdd.bc     = cdc_setRxBuffer((char*)cdcInDataBuffer0);
      //       cdcInDataBuffer1[0]       = '|';
      endPointBdts[CDC_DATA_IN_ENDPOINT].txOdd.u.bits = BDTEntry_OWN_MASK|BDTEntry_DATA1_MASK;
   }
   else {
      // Set to write to other buffer & get count in current buffer
      endPointBdts[CDC_DATA_IN_ENDPOINT].txEven.bc    = cdc_setRxBuffer((char*)cdcInDataBuffer1);
      //       cdcInDataBuffer0[0]       = '^';
      endPointBdts[CDC_DATA_IN_ENDPOINT].txEven.u.bits = BDTEntry_OWN_MASK|BDTEntry_DATA0_MASK;
   }
   //   epHardwareState[CDC_DATA_IN_ENDPOINT].data0_1 = !epHardwareState[CDC_DATA_IN_ENDPOINT].data0_1; // Toggle data0/1
   epHardwareState[CDC_DATA_IN_ENDPOINT].txOdd     = !epHardwareState[CDC_DATA_IN_ENDPOINT].txOdd;
}
void checkUsbCdcTxData(void) {
   // Check if we need to unThrottle EP4
   if ((epHardwareState[CDC_DATA_OUT_ENDPOINT].state == EPThrottle) && cdc_txBufferIsFree()) {
      ep4SaveRxData();        // Save data from last transfer
      ep4InitialiseBdtRx();   // Set up next transfer
      epHardwareState[CDC_DATA_OUT_ENDPOINT].state = EPDataOut;
   }
}

static uint8_t serialDelayCount = 0;

// This value controls how long the serial interface will wait before
// sending a buffered character. (count of SOFs ~ ms)
constexpr uint SERIAL_THRESHOLD = 0; // ms

/**
 * Configure the BDT for EP5 In [Tx, device -> host]
 * CDC - IN
 */
static void ep5StartTxTransactionIfIdle() {
#if 0
   if ((epHardwareState[CDC_DATA_IN_ENDPOINT].state == EPIdle) && (cdc_rxBufferItemCount()>0)) {
      ep5InitialiseBdtTx();
      epHardwareState[CDC_DATA_IN_ENDPOINT].state = EPLastIn;
      serialDelayCount = 0;
   }
   else if ((epHardwareState[CDC_DATA_IN_ENDPOINT].state == EPLastIn) && (cdc_rxBufferItemCount()==16)) {
      ep5InitialiseBdtTx();
      epHardwareState[CDC_DATA_IN_ENDPOINT].state = EPDataIn;
      serialDelayCount = 0;
   }
#else
   if ((epHardwareState[CDC_DATA_IN_ENDPOINT].state == EPIdle) && (cdc_rxBufferItemCount()>0)) {
      ep5InitialiseBdtTx();
      epHardwareState[CDC_DATA_IN_ENDPOINT].state = EPDataIn;
      serialDelayCount = 0;
   }
#endif
}

void checkUsbCdcRxData(void) {
   ep5StartTxTransactionIfIdle();
}
#endif

} // End namespace USBDM

