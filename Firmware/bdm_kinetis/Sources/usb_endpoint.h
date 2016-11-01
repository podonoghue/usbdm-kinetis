/*
 * usb_endpoint.h
 *
 *  Created on: 31Oct.,2016
 *      Author: podonoghue
 */

#ifndef SOURCES_USB_ENDPOINT_H_
#define SOURCES_USB_ENDPOINT_H_

#include "usb_defs.h"
#include "derivative.h"

namespace USBDM {

/** BDTs organised by endpoint, odd/even, tx/rx */
extern EndpointBdtEntry endPointBdts[];

/** BDTs as simple array */
constexpr BdtEntry * bdts = (BdtEntry *)endPointBdts;

/**
 * Endpoint state values
 */
enum EndpointState {
   EPIdle = 0,  //!< Idle (Tx complete)
   EPDataIn,    //!< Doing a sequence of IN packets (until data count <= EP_MAXSIZE)
   EPDataOut,   //!< Doing a sequence of OUT packets (until data count == 0)
   EPLastIn,    //!< Doing the last IN packet
   EPStatusIn,  //!< Doing an IN packet as a status handshake
   EPStatusOut, //!< Doing an OUT packet as a status handshake
   EPThrottle,  //!< Doing OUT packets but no buffers available (NAKed)
   EPStall,     //!< Endpoint is stalled
   EPComplete,  //!< Used for command protocol - new command available
};

/**
 * Endpoint hardware state
 */
struct EPHardwareState {
   Data0_1        txData1;   //!< Data 0/1 tx state
   Data0_1        rxData1;   //!< Data 0/1 rx state
   EvenOdd        txOdd;     //!< Odd/Even tx buffer
   EvenOdd        rxOdd;     //!< Odd/Even rx buffer
   EndpointState  state;     //!< End-point state
};

/**
 * Class for generic endpoint
 *
 * @tparam ENDPOINT_NUM Endpoint number
 * @tparam EP_MAXSIZE   Maximum size of packet
 */
template<class Info, int ENDPOINT_NUM, int EP_MAXSIZE>
class Endpoint {

public:
   static constexpr int ENDPOINT_NO  = ENDPOINT_NUM;
   static constexpr int BUFFER_SIZE  = EP_MAXSIZE;

protected:
   static bool    busyFlag;

   /** Pointer to hardware */
   static constexpr USB_Type volatile *usb = Info::usb;

   /** Buffer for Tx & Rx data */
   static uint8_t fDataBuffer[EP_MAXSIZE];

   /** State of the endpoint */
   static EPHardwareState fHardwareState;

   /** Callback used on completion of transaction */
   static void (*volatile fCallback)();

   /** Pointer to external data buffer for Rx/Tx */
   static uint8_t* fDataPtr;

   /** Count of remaining bytes in external data buffer to Rx/Tx */
   static uint16_t fDataRemaining;

   /** Indicates that the IN transaction needs to be
    *  terminated with ZLP if modulo endpoint size */
   static bool fNeedZLP;

public:
   /**
    * Constructor
    */
   constexpr Endpoint() {
   }

   /**
    * Gets pointer to USB data buffer
    */
   static uint8_t *getBuffer() {
      return fDataBuffer;
   }
   /**
    * Flip active odd/even buffer state
    *
    * @param usbStat Value from USB->STAT
    */
   static void flipOddEven(uint8_t usbStat) {

      // Direction of transfer 0=>OUT, (!=0)=>IN
      bool isTx  = usbStat&USB_STAT_TX_MASK;

      // Odd/even buffer
      bool isOdd = usbStat&USB_STAT_ODD_MASK;

      if (isTx) {
         // Flip Transmit buffer
         fHardwareState.txOdd = !isOdd;
         if ((endPointBdts[ENDPOINT_NUM].txEven.u.bits&BDTEntry_OWN_MASK) ||
             (endPointBdts[ENDPOINT_NUM].txOdd.u.bits&BDTEntry_OWN_MASK)) {
            printf("Opps-Tx\n");
         }
      }
      else {
         // Flip Receive buffer
         fHardwareState.rxOdd = !isOdd;
      }
   }

   /**
    * Return hardware state
    */
   static EPHardwareState &getHardwareState() {
      return fHardwareState;
   }

   /**
    * Set callback to execute at end of transaction
    */
   static void setCallback(void    (*callback)()) {
      fCallback = callback;
   }

   /**
    *  Indicates that the IN transaction needs to be
    *  terminated with ZLP if modulo endpoint size
    *
    *  @param needZLP True to indicate need for ZLPs.
    */
   static void setNeedZLP(bool needZLP) {
      fNeedZLP = needZLP;
   }

   /**
    * Stall endpoint
    */
   static void stall() {
      fHardwareState.state               = EPStall;
      usb->ENDPOINT[ENDPOINT_NUM].ENDPT |= USB_ENDPT_EPSTALL_MASK;
   }

   /*
    * Clear Stall on endpoint
    */
   static void clearStall() {
      usb->ENDPOINT[ENDPOINT_NUM].ENDPT   &= ~USB_ENDPT_EPSTALL_MASK;
      fHardwareState.state                 = EPIdle;
      fHardwareState.txData1               = DATA0;
      fHardwareState.rxData1               = DATA0;
   }

   /**
    * Initialise endpoint
    *  - Internal state
    *  - BDTs
    */
   static void initialise() {
      static const EPHardwareState initialHardwareState = {DATA0,DATA0,EVEN,EVEN,EPIdle};
      fHardwareState     = initialHardwareState;

      fDataPtr           = nullptr;
      fDataRemaining     = 0;
      fNeedZLP           = false;

      fCallback          = nullptr;

      // Assumes single buffer
      endPointBdts[ENDPOINT_NUM].rxEven.addr = nativeToLe32((uint32_t)fDataBuffer);
      endPointBdts[ENDPOINT_NUM].rxOdd.addr  = nativeToLe32((uint32_t)fDataBuffer);
      endPointBdts[ENDPOINT_NUM].txEven.addr = nativeToLe32((uint32_t)fDataBuffer);
      endPointBdts[ENDPOINT_NUM].txOdd.addr  = nativeToLe32((uint32_t)fDataBuffer);
   }

   /**
    * Start IN transaction [Tx, device -> host, DATA0/1]
    */
   static void startTxTransaction( uint8_t bufSize, const uint8_t *bufPtr ) {

      fDataPtr            = (uint8_t*)bufPtr;    // Pointer to _next_ data
      fDataRemaining      = bufSize;             // Count of remaining bytes
//      pushState('T');
//      pushState('x');

      if ((bufSize == 0) && (ENDPOINT_NUM == 0)) {
         fHardwareState.state = EPStatusIn;   // Assume status handshake for EP0
      }
      else if ((bufSize < EP_MAXSIZE) ||              // Undersize packet OR
            ((bufSize == EP_MAXSIZE) && !fNeedZLP)) { // Even but don't need ZLP
         // Sending one and only packet
         fHardwareState.state = EPLastIn;
      }
      else {
         // Sending first of several packets
         fHardwareState.state = EPDataIn;
      }
      initialiseBdtTx(); // Configure the BDT for transfer
   }

   /**
    * Configure the BDT for next IN [Tx, device -> host]
    */
   static void initialiseBdtTx() {
      // Get BDT to use
      BdtEntry *bdt = fHardwareState.txOdd?&endPointBdts[ENDPOINT_NUM].txOdd:&endPointBdts[ENDPOINT_NUM].txEven;

//      pushState('B');
//      pushState(fHardwareState.txOdd+'0');

      if ((endPointBdts[ENDPOINT_NUM].txEven.u.bits&BDTEntry_OWN_MASK) ||
          (endPointBdts[ENDPOINT_NUM].txOdd.u.bits&BDTEntry_OWN_MASK)) {
         printf("Opps-Tx\n");
      }

      uint16_t size = fDataRemaining;
      if (size > EP_MAXSIZE) {
         size = EP_MAXSIZE;
      }
      // Copy the Tx data to EP buffer
      (void) memcpy(fDataBuffer, fDataPtr, size);

      fDataPtr         += size;  // Ptr to _next_ data
      fDataRemaining   -= size;  // Count of remaining bytes

      // Set up to Tx packet
      bdt->bc     = (uint8_t)size;
      if (fHardwareState.txData1) {
         bdt->u.bits  = BDTEntry_OWN_MASK|BDTEntry_DATA1_MASK|BDTEntry_DTS_MASK;
      }
      else {
         bdt->u.bits  = BDTEntry_OWN_MASK|BDTEntry_DATA0_MASK|BDTEntry_DTS_MASK;
      }
   }
   /**
    *  Start an OUT transaction [Rx, device <- host, DATA0/1]
    *
    *   @param bufSize - Size of data to transfer
    *   @param bufPtr  - Buffer for data
    */
   static void startRxTransaction( uint8_t bufSize, uint8_t *bufPtr ) {
      fDataRemaining  = bufSize; // Total bytes to Rx
      fDataPtr        = bufPtr;  // Where to (eventually) place data

      if ((bufSize == 0) && (ENDPOINT_NUM == 0)) {
         fHardwareState.state = EPStatusOut;  // Assume status handshake
      }
      else {
         fHardwareState.state = EPDataOut;    // Assume first of several data pkts
      }
      initialiseBdtRx(); // Configure the BDT for transfer
   }

   /**
    * Configure the BDT for OUT [Rx, device <- host, DATA0/1]
    */
   static void initialiseBdtRx() {
      // Set up to Rx packet
      BdtEntry *bdt = fHardwareState.rxOdd?&endPointBdts[ENDPOINT_NUM].rxOdd:&endPointBdts[ENDPOINT_NUM].rxEven;

      if (bdt->u.bits&BDTEntry_OWN_MASK) {
         // Already configured
         return;
      }
      if ((endPointBdts[ENDPOINT_NUM].rxEven.u.bits&BDTEntry_OWN_MASK) ||
          (endPointBdts[ENDPOINT_NUM].rxOdd.u.bits&BDTEntry_OWN_MASK)) {
         printf("Opps-Rx\n");
      }
      // Set up to Rx packet
      // Always used maximum size even if expecting less data
      bdt->bc = EP_MAXSIZE;
      if (fHardwareState.rxData1) {
         bdt->u.bits  = BDTEntry_OWN_MASK|BDTEntry_DATA1_MASK|BDTEntry_DTS_MASK;
      }
      else {
         bdt->u.bits  = BDTEntry_OWN_MASK|BDTEntry_DATA0_MASK|BDTEntry_DTS_MASK;
      }
   }
   /**
    *  Save the data from an OUT packet and advance ptrs etc.
    *
    *  @return Number of bytes saved
    */
   static uint8_t saveRxData() {
      // Get BDT
      BdtEntry *bdt = (!fHardwareState.rxOdd)?&endPointBdts[ENDPOINT_NUM].rxOdd:&endPointBdts[ENDPOINT_NUM].rxEven;
      uint8_t size = bdt->bc;

      if (size > 0) {
         // Check if more data than requested - discard excess
         if (size > fDataRemaining)
            size = fDataRemaining;
         // Check if external buffer in use
         if (fDataPtr != nullptr) {
            // Copy the data from the Rx buffer to external buffer
            ( void )memcpy(fDataPtr, fDataBuffer, size);
            fDataPtr    += size;   // Advance buffer ptr
         }
         fDataRemaining -= size;   // Count down bytes to go
      }
      return size;
   }

   /**
    * Handle OUT [Rx, device <- host, DATA0/1]
    */
   static void handleOutToken() {
      uint8_t transferSize;
//      pushState('O');

      switch (fHardwareState.state) {
         case EPDataOut:        // Receiving a sequence of OUT packets
            // Save the data from the Rx buffer
            transferSize = saveRxData();

            // Complete transfer on undersize packet or received expected number of bytes
            if ((transferSize < EP_MAXSIZE) || (fDataRemaining == 0)) {
               fHardwareState.state = EPIdle;
               if (ENDPOINT_NUM == 0) {
                  // Do empty status packet transmission - no response expected
                  startTxTransaction(0, nullptr);
               }
            }
            break;

         case EPStatusOut:       // Done an OUT packet as a status handshake from host
            fHardwareState.state = EPIdle;
            break;

         // We don't expect an OUT token while in the following states
         case EPLastIn:         // Just done the last IN packet
         case EPDataIn:         // Doing a sequence of IN packets (until data count <= EP_MAXSIZE)
         case EPStatusIn:       // Just done an IN packet as a status handshake
         case EPIdle:           // Idle
         case EPComplete:
         case EPStall:          // Not used
         case EPThrottle:       // Not used
            PRINTF("ep0HandleOutToken - unexpected, s = %d\n", fHardwareState.state);
            fHardwareState.state = EPIdle;
            break;
      }
      // Toggle DATA0/1
      fHardwareState.rxData1 = !fHardwareState.rxData1;

      initialiseBdtRx(); // Set up setup/out packet
   }
   /**
    * Handle IN token [Tx, device -> host]
    */
   static void handleInToken() {
      static const uint8_t busyResponse[] = {/*TODO BDM_RC_BUSY*/3,1,2,3};
//      pushState('I');

      fHardwareState.txData1 = !fHardwareState.txData1;   // Toggle DATA0/1 for next packet
      //   PUTS(fHardwareState[BDM_OUT_ENDPOINT].data0_1?"ep2HandleInToken-T-1\n":"ep2HandleInToken-T-0\n");

      switch (fHardwareState.state) {
         case EPDataIn:    // Doing a sequence of IN packets
            // Check if need to send Zero Length Packet
            // Only required if even multiple and fNeedZLP set
            if ((fDataRemaining < EP_MAXSIZE) ||
                  ((fDataRemaining == EP_MAXSIZE) && !fNeedZLP)) {
               // Sending last packet (may be empty)
               // This will be undersized or ZLP or full size if no ZLP required
               fHardwareState.state = EPLastIn;
            }
            else {
               // Sending full packet followed by at least one more packet
               fHardwareState.state = EPDataIn;
            }
            initialiseBdtTx(); // Set up next IN packet
            break;

         case EPLastIn:    // Just done the last IN packet
            if (ENDPOINT_NUM == 0) {
               fHardwareState.state = EPStatusOut;   // Receiving an OUT status packet
            }
            else if (busyFlag) {
               startTxTransaction(sizeof(busyResponse), busyResponse);
            }
            else {
               fHardwareState.state = EPIdle; // Complete
            }
            break;

         case EPStatusIn: // Just done an IN packet as a status handshake for an OUT Data transfer
            fHardwareState.state = EPIdle; // Now Idle
            if (fCallback != nullptr) {
               // Execute callback function to process OUT data
               fCallback();
            }
            fCallback = nullptr;
            break;

            // We don't expect an IN token while in the following states
         case EPIdle:           // Idle (Tx complete)
         case EPDataOut:        // Doing a sequence of OUT packets (until data count <= EP_MAXSIZE)
         case EPStatusOut:      // Doing an OUT packet as a status handshake
         default:
            break;
      }
   }
   /** TODO - Need to think about this
    * Set busy flag
    *
    * @param busy Whether bust or not
    */
   void setBusyFlag(bool busy) {
      busyFlag = busy;
   }

};

/**
 * Class for CONTROL endpoint
 *
 * @tparam ENDPOINT_NUM Endpoint number
 * @tparam EP_MAXSIZE   Maximum size of packet
 */
template<class Info, int EP_MAXSIZE>
class ControlEndpoint : public Endpoint<Info, 0, EP_MAXSIZE> {

public:
   using Endpoint<Info, 0, EP_MAXSIZE>::fHardwareState;
   using Endpoint<Info, 0, EP_MAXSIZE>::usb;
   using Endpoint<Info, 0, EP_MAXSIZE>::startTxTransaction;

   /**
    * Constructor
    */
   constexpr ControlEndpoint() {
   }

   /**
    * Initialise endpoint
    *  - Internal state
    *  - BDTs
    *  - usb->ENDPOINT[].ENDPT
    */
   static void initialise() {
      Endpoint<Info, 0, EP_MAXSIZE>::initialise();
      // Rx/Tx/SETUP
      usb->ENDPOINT[0].ENDPT = USB_ENDPT_EPRXEN_MASK|USB_ENDPT_EPTXEN_MASK|USB_ENDPT_EPHSHK_MASK;
   }

   /**
    * Stall EP0\n
    * This stall is cleared on the next transmission
    */
   static void stall() {
      // Stall Tx only
      PRINTF("stall\n");
      BdtEntry *bdt = fHardwareState.txOdd?&endPointBdts[0].txOdd:&endPointBdts[0].txEven;
      bdt->u.bits = BDTEntry_OWN_MASK|BDTEntry_STALL_MASK|BDTEntry_DTS_MASK;
   }

   /*
    * Clear Stall on endpoint
    */
   static void clearStall() {
      PRINTF("clearStall\n");
      BdtEntry *bdt = fHardwareState.txOdd?&endPointBdts[0].txOdd:&endPointBdts[0].txEven;
      // Release BDT as SIE doesn't
      bdt->u.bits   = 0;
      usb->ENDPOINT[0].ENDPT   &= ~USB_ENDPT_EPSTALL_MASK;
      fHardwareState.state      = EPIdle;
      fHardwareState.txData1    = DATA0;
   }

   /**
    * Start transmission of an empty status packet
    */
   static void startTxStatus() {
      startTxTransaction(0, nullptr);
   }
};

/**
 * Class for IN endpoint
 *
 * @tparam ENDPOINT_NUM Endpoint number
 * @tparam EP_MAXSIZE   Maximum size of packet
 */
template<class Info, int ENDPOINT_NUM, int EP_MAXSIZE>
class InEndpoint : public Endpoint<Info, ENDPOINT_NUM, EP_MAXSIZE> {

   using Endpoint<Info, ENDPOINT_NUM, EP_MAXSIZE>::usb;

public:
   /**
    * Constructor
    */
   constexpr InEndpoint() {
   }

   /**
    * Initialise endpoint
    *  - Internal state
    *  - BDTs
    *  - usb->ENDPOINT[].ENDPT
    */
   static void initialise() {
      Endpoint<Info, ENDPOINT_NUM, EP_MAXSIZE>::initialise();
      // Transmit only
      usb->ENDPOINT[ENDPOINT_NUM].ENDPT = USB_ENDPT_EPTXEN_MASK|USB_ENDPT_EPHSHK_MASK;
   }
};

/**
 * Class for OUT endpoint
 *
 * @tparam ENDPOINT_NUM Endpoint number
 * @tparam EP_MAXSIZE   Maximum size of packet
 */
template<class Info, int ENDPOINT_NUM, int EP_MAXSIZE>
class OutEndpoint : public Endpoint<Info, ENDPOINT_NUM, EP_MAXSIZE> {

   using Endpoint<Info, ENDPOINT_NUM, EP_MAXSIZE>::usb;

public:
   /**
    * Constructor
    */
   constexpr OutEndpoint() {
   }

   /**
    * Initialise endpoint
    *  - Internal state
    *  - BDTs
    *  - usb->ENDPOINT[].ENDPT
    */
   static void initialise() {
      Endpoint<Info, ENDPOINT_NUM, EP_MAXSIZE>::initialise();
      // Receive only
      usb->ENDPOINT[ENDPOINT_NUM].ENDPT = USB_ENDPT_EPRXEN_MASK|USB_ENDPT_EPHSHK_MASK;
   }
};

/** State of the endpoint */
template<class Info, int ENDPOINT_NUM, int EP_MAXSIZE>
EPHardwareState Endpoint<Info, ENDPOINT_NUM, EP_MAXSIZE>::fHardwareState;

/** Buffer for Tx & Rx data */
template<class Info, int ENDPOINT_NUM, int EP_MAXSIZE>
uint8_t Endpoint<Info, ENDPOINT_NUM, EP_MAXSIZE>::fDataBuffer[EP_MAXSIZE];

/** Pointer to data buffer for Rx/Tx */
template<class Info, int ENDPOINT_NUM, int EP_MAXSIZE>
uint8_t* Endpoint<Info, ENDPOINT_NUM, EP_MAXSIZE>::fDataPtr = nullptr;

/** Count of remaining bytes to Rx/Tx */
template<class Info, int ENDPOINT_NUM, int EP_MAXSIZE>
uint16_t Endpoint<Info, ENDPOINT_NUM, EP_MAXSIZE>::fDataRemaining = 0;

/**
 *  Indicates that the IN transaction needs to be
 *  terminated with ZLP if modulo endpoint size
 */
template<class Info, int ENDPOINT_NUM, int EP_MAXSIZE>
bool Endpoint<Info, ENDPOINT_NUM, EP_MAXSIZE>::fNeedZLP = false;

/** USB callback at end of transaction */
template<class Info, int ENDPOINT_NUM, int EP_MAXSIZE>
void (*volatile  Endpoint<Info, ENDPOINT_NUM, EP_MAXSIZE>::fCallback)() = nullptr;

/** USB callback at end of transaction */
template<class Info, int ENDPOINT_NUM, int EP_MAXSIZE>
bool Endpoint<Info, ENDPOINT_NUM, EP_MAXSIZE>::busyFlag;

}; // end namespace

#endif /* SOURCES_USB_ENDPOINT_H_ */
