/*
 * uart.c
 *
 *  Created on: Nov 6, 2012
 *      Author: podonoghue
 */
#include <string.h>
#include "derivative.h" /* include peripheral declarations */
#include "system.h"
#include "CDC.h"
#include "USB.h"
#include "utilities.h"
#include "configure.h"
#include "uart.h"

using UartInfo = USBDM::Uart1Info;

static void uartCallback(uint8_t status);

namespace USBDM {
extern void checkUsbCdcTxData();
};

#if (HW_CAPABILITY&CAP_CDC)

#define CDC_TX_BUFFER_SIZE (16)  // Should equal end-point buffer size
static char txBuffer[CDC_TX_BUFFER_SIZE];
static uint8_t txHead        = 0;
static uint8_t txBufferCount = 0;
static uint8_t breakCount    = 0;

#define CDC_RX_BUFFER_SIZE (16)  // Should less than or equal to end-point buffer size
static char *rxBuffer = NULL;
static uint8_t rxBufferCount = 0;
static uint8_t cdcStatus = SERIAL_STATE_CHANGE;

/*
 * The following routines are assumed to be called from interrupt code - Interrupts masked
 */

/*
 * Simple double-buffering for Rx (in conjunction with USB buffer)
 */

/** 
 * Add a char to the CDC-Rx buffer
 *
 * @param ch - char to add
 *
 * @note Overun flag is set on buffer full
 */
void cdc_putRxBuffer(char ch) {
   if (rxBufferCount >= CDC_RX_BUFFER_SIZE) {
      cdcStatus |= UART_S1_OR_MASK;
      return;
   }
   rxBuffer[rxBufferCount++] = ch;
}

/**
 * Set current Rx Buffer
 *
 * @param buffer Buffer address, new data is written to this buffer
 * 
 * @return Number of characters in existing buffer
 *
 * @note Assumed called while interrupts blocked
 */
uint8_t cdc_setRxBuffer(char *buffer) {
   uint8_t temp;
   rxBuffer = buffer;
   temp = rxBufferCount;
   rxBufferCount = 0;
   return temp;
}

/**
 *  RxBufferEmpty() - Check if Rx buffer is empty
 *
 * @return -  >0 => buffer is not empty
 *            0  => buffer is empty
 */
uint8_t cdc_rxBufferItemCount(void) {
   return rxBufferCount;
}

/*
 * Simple double-buffering for CDC-Tx (in conjunction with USB end-point buffer)
 */
 
/**
 * Add data to Tx Buffer (from USB)
 *
 * @param source Source buffer to copy from
 * @param size   Number of bytes to copy
 *
 *  @return true => OK, false => Buffer is busy (overrun)
 */
bool cdc_putTxBuffer(char *source, uint8_t size) {
   if (txBufferCount > 0) {
      return 1; // Busy
   }
   (void)memcpy(txBuffer, source, size);
   txHead        = 0;
   txBufferCount = size;
   UartInfo::uart->C2 |= UART_C2_TIE_MASK; // Enable UART Tx interrupts
   return 0;
}

/** getTx() -  Gets a character from the CDC-Tx queue.
 *
 * @return
 *  -  -ve => queue is empty \n
 *  -  +ve => char from queue
 */
static int cdc_getTxBuffer(void) {
   uint8_t ch;
   if (txBufferCount == 0) {
      // Check data in USB buffer & restart USB Out if needed
      USBDM::checkUsbCdcTxData();
   }
   // Need to re-check as above may have copied data
   if (txBufferCount == 0) {
      return -1;
   }
   ch = txBuffer[txHead++];
   if (txHead >= txBufferCount)
      txBufferCount = 0;
   return ch;
}

/**
 *  cdcTxSpace - check if CDC-Tx buffer is free
 *
 * @return 0 => buffer is occupied
 *         1 => buffer is free
 */
uint8_t cdc_txBufferIsFree() {
   return (txBufferCount == 0);
}

uint8_t cdc_getSerialState() {
   uint8_t status = 0x3;
   static uint8_t lastSciStatus = 0x00;

   if (cdcStatus&UART_S1_FE_MASK) {
      status |= 1<<4;
   }
   if (cdcStatus&UART_S1_OR_MASK) {
      status |= 1<<6;
   }
   if (cdcStatus&UART_S1_PF_MASK) {
      status |= 1<<5;
   }
   if (lastSciStatus != cdcStatus) {
      lastSciStatus = cdcStatus;
      status |= SERIAL_STATE_CHANGE;
   }
   cdcStatus = 0;
   return status;
}

//! Interrupt handler for CDC-Tx \n
//! Transfers a char from the CDC-Tx queue to UART_D
//!
//! @note Interrupts are disabled on empty queue
//!
void cdc_txHandler(void) {
int ch;

   ch = cdc_getTxBuffer();
   if (ch >= 0) {
      UartInfo::uart->D = (uint8_t)ch; // Send the char
   }
   else if (breakCount > 0) {
      UartInfo::uart->C2 |=  UART_C2_SBK_MASK; // Send another BREAK 'char'
      UartInfo::uart->C2 &= ~UART_C2_SBK_MASK;
      if (breakCount != 0xFF) {
         breakCount--;
       }
   }
   else {
      UartInfo::uart->C2 &= ~UART_C2_TIE_MASK; // Disable further Tx interrupts
   }
}

//! Interrupt handler for CDC Rx \n
//! Transfers a char from the UART_D to USB IN queue
//!
//! @note Overruns are ignored
//!
void cdc_rxHandler(void) {
   cdc_putRxBuffer(UartInfo::uart->D);
}


static void uartCallback(uint8_t status) {
   if (status&UART_S1_RDRF_MASK) {
      cdc_rxHandler();
   }
   else if (status&UART_S1_TDRE_MASK) {
      cdc_txHandler();
   }
   else {
      // Record and clear error status
      cdcStatus |= status;
      (void)UartInfo::uart->D;
   }
}

#define BAUDDIVIDER(x)  (((BUS_FREQ/16))/(x))

static LineCodingStructure lineCoding = {leToNative32(9600UL),0,1,8};

/**
 *  Set CDC communication characteristics\n
 *  Dummy routine
 *
 * @param lineCodingStructure - Structure describing desired settings
 *
 * The CDC is quite limited when compared to the serial interface implied by
 * LineCodingStructure.
 * It does not support many of the combinations available.
 * BAUD > 300
 */
void cdc_setLineCoding(const LineCodingStructure *lineCodingStructure) {
   uint32_t baudrate;
   uint16_t ubd;
   uint8_t  UARTC1Value = 0x00;
   uint8_t  UARTC3Value = 0x00;

   USBDM::UartIrq_T<UartInfo>::setCallback(uartCallback);

   cdcStatus  = SERIAL_STATE_CHANGE;
   breakCount = 0; // Clear any current BREAKs

   (void)memcpy(&lineCoding, lineCodingStructure, sizeof(LineCodingStructure));

   //! Note - for a 48MHz bus speed the useful baud range is ~300 to ~115200 for 0.5% error
   //  230400 & 460800 have a 8.5% error

   // Enable clock to PTA (for UART0 pin multiplexing)
   SIM->SCGC5  |= SIM_SCGC5_PORTA_MASK;

   // Configure shared pins
//   PORTB_PCR16  = PORT_PCR_MUX(3); // Rx (PFE?)
//   PORTB_PCR17  = PORT_PCR_MUX(3) | PORT_PCR_DSE_MASK| PORT_PCR_SRE_MASK; // Tx

   // Enable clock to UART interface
   *UartInfo::clockReg |= UartInfo::clockMask;

   // Configure pins
   UartInfo::initPCRs();
//   USBDM::Uart_T<UartInfo>::setBaudRate(baudrate);

   // Enable clock to UART0
   *UartInfo::clockReg |= UartInfo::clockMask;

   // Disable the transmitter and receiver while changing settings.
   UartInfo::uart->C2 &= ~(UART_C2_TE_MASK | UART_C2_RE_MASK );

   // Determine baud rate divider
   baudrate = leToNative32(lineCoding.dwDTERate);

   // Calculate baud settings
   ubd = (uint16_t)(SystemCoreClock/(baudrate * 16));

   // Set Baud rate register
   UartInfo::uart->BDH = (UartInfo::uart->BDH&~UART_BDH_SBR_MASK) | UART_BDH_SBR((ubd>>8));
   UartInfo::uart->BDL = UART_BDL_SBR(ubd);

#if defined(UART_C4_BRFA_MASK) && 0
   // Determine fractional divider to get closer to the baud rate
   uint16_t brfa;
   brfa     = (uint8_t)(((SystemCoreClock*(32000/16))/baudrate) - (ubd * 32));
   UART1_C4 = (UART1_C4&~UART_C4_BRFA_MASK) | UART_C4_BRFA(brfa);
#endif

// Note: lineCoding.bCharFormat is ignored (always 1 stop bit)
//   switch (lineCoding.bCharFormat) {
//      case 0:  // 1 bits
//      case 1:  // 1.5 bits
//      case 2:  // 2 bits
//   }

   // Available combinations
   //============================================
   // Data bits  Parity   Stop |  M   PE  PT  T8
   //--------------------------------------------
   //     7      Odd       1   |  0   1   1   X
   //     7      Even      1   |  0   1   0   X
   //     8      None      1   |  0   0   X   X
   //     8      Odd       1   |  1   1   1   X
   //     8      Even      1   |  1   1   0   X
   //     8      Mark      1   |  1   0   X   0
   //     8      Space     1   |  1   0   X   1
   //--------------------------------------------
   //   All other values default to 8-None-1

   switch (lineCoding.bDataBits) {
     // 5,6,7,8,16
      case 7  :
         switch (lineCoding.bParityType) {
            case 1:  UARTC1Value = UART_C1_PE_MASK|UART_C1_PT_MASK; break; // Odd
            case 2:  UARTC1Value = UART_C1_PE_MASK;                 break; // Even
         }
           break;
      case 8  :
        UARTC1Value = UART_C1_M_MASK; // 9-data or 8-data+parity
         switch (lineCoding.bParityType) {
            case 0:  UARTC1Value  = 0;                               break; // None
            case 1:  UARTC1Value |= UART_C1_PE_MASK|UART_C1_PT_MASK; break; // Odd
            case 2:  UARTC1Value |= UART_C1_PE_MASK;                 break; // Even
            case 3:  UARTC3Value  = UART_C3_T8_MASK;                 break; // Mark
            case 4:                                                  break; // Space
         }
         break;
     default :
        break;
   }
   UartInfo::uart->C1 = UARTC1Value;
   UartInfo::uart->C3 = UARTC3Value;
   UartInfo::uart->C2 = UART_C2_RIE_MASK|UART_C2_RE_MASK|UART_C2_TE_MASK; // Enable Rx/Tx with interrupts
   UartInfo::uart->C3 = UART_C3_FEIE_MASK|UART_C3_NEIE_MASK|UART_C3_ORIE_MASK|UART_C3_PEIE_MASK;

   // Discard any data in buffers
   rxBufferCount = 0;
   txBufferCount = 0;

   NVIC_EnableIRQ(UartInfo::irqNums[0]);
}

/**
 *  Get CDC communication characteristics\n
 *
 *  @param lineCodingStructure - Structure describing current settings
 */
const LineCodingStructure *cdc_getLineCoding(void) {
   return &lineCoding;
}

/**
 *  Set CDC Line values
 *
 * @param value - Describing desired settings
 */
void cdc_setControlLineState(uint8_t value) {
#define LINE_CONTROL_DTR (1<<0)
#define LINE_CONTROL_RTS (1<<1) // Ignored

   (void) value; // remove warning
   // Temp fix until I can determine why the value is incorrect
//   DTR_ACTIVE();
// if (value & (LINE_CONTROL_DTR|LINE_CONTROL_RTS)) {
//    DTR_ACTIVE();
// }
// else {
//    DTR_INACTIVE();
// }
}

/**
 *  Send CDC break\n
 *
 * @param length - length of break in milliseconds (see note)\n
 *  - 0x0000 => End BREAK
 *  - 0xFFFF => Start indefinite BREAK
 *  - else   => Send a break of 10 chars
 
 * @note - only partially implemented
 *       - breaks are sent after currently queued characters
 */
void cdc_sendBreak(uint16_t length) {
   if (length == 0xFFFF) {
     // Send indefinite BREAKs
     breakCount = 0xFF;
   }
   else if (length == 0x0) {
     // Stop sending BREAKs
     breakCount = 0x00;
   }
   else {
     // Queue a series of BREAKs
      breakCount = 10;
   }
}

#endif // (HW_CAPABILITY&CAP_CDC)
