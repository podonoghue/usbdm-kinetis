/*
 * bdm.h
 *
 *  Created on: 26Nov.,2016
 *      Author: podonoghue
 */

#include <stdint.h>
#include "system.h"
#include "derivative.h"
#include "hardware.h"
#include "delay.h"
#include "USBDM_MK20D5.h"
#include "bdm.h"

namespace Bdm {

/** FTM to use */
using FtmInfo = USBDM::Ftm0Info;
/** FTM channel for BKGD out D6(ch6) */
constexpr int bkgdOutChannel = 6;
/** FTM channel for BKGD enable C3(ch2) */
constexpr int bkgdEnChannel = 2;
/** FTM channel for BKGD in D4(ch4) */
constexpr int bkgdInChannel = 4;

/* Make sure pins have been configured for FTM */
USBDM::CheckSignal<FtmInfo, bkgdOutChannel> x;
USBDM::CheckSignal<FtmInfo, bkgdEnChannel>  y;
USBDM::CheckSignal<FtmInfo, bkgdInChannel>  z;

/** Width of sync pulse (us), 128 target-cycle @ 128kHz (<65536/FTMClock) */
static constexpr unsigned SYNC_WIDTH = 1000;

/** Time to wait for end of target sync pulse response (us), (<65536/FTMClock) */
static constexpr unsigned SYNC_TIMEOUT = 1200;

///** Time to set up Timer - This varies with optimisation! */
//static constexpr unsigned TMR_SETUP_TIME = 40;//15;

/** Length of high drive before 3-state i.e. speed-up pulse (ticks) */
static constexpr unsigned SPEEDUP_PULSE_WIDTH_IN_TICKS = 1;

/** Pointer to hardware */
static constexpr volatile FTM_Type *ftm = reinterpret_cast<volatile FTM_Type*>(FtmInfo::ftm);

/** Pointer to clock register */
static constexpr volatile uint32_t *clockReg  = reinterpret_cast<volatile uint32_t*>(FtmInfo::clockReg);

/**
 * Disable FTM control of BKGD
 */
inline
static void disablePins() {
   ftm->SWOCTRL =
         (0<<(bkgdEnChannel+8)) |(1<<bkgdEnChannel)|  // Force low (disable buffer)
         (1<<(bkgdOutChannel+8))|(1<<bkgdOutChannel); // Force high
}

/**
 * Enable FTM control from BKGD
 */
inline
static void enablePins() {
   ftm->SWOCTRL = 0;
}

///** PCR for BKGD in pin used by timer */
//using BkgdInPcr        = USBDM::PcrTable_T<FtmInfo, bkgdInChannel>;
//
///** PCR for BKGD enable pin used by timer */
//using BkgdEnPcr        = USBDM::PcrTable_T<FtmInfo, bkgdEnChannel>;
//
///** PCR for BKGD in pin used by timer */
//using BkgdOutPcr       = USBDM::PcrTable_T<FtmInfo, bkgdOutChannel>;
//
///** GPIO for BKGD in pin */
//using BkgdIn           = USBDM::GpioTable_T<FtmInfo, bkgdInChannel>;
//
///** GPIO for BKGD enable pin used by timer */
//using BkgdEn = USBDM::GpioTable_T<FtmInfo, bkgdEnChannel>;
//
///** GPIO for BKGD in pin used by timer */
//using BkgdOut = USBDM::GpioTable_T<FtmInfo, bkgdOutChannel>;
//
///** Transceiver for BKGD */
//using Bkgd = Lvc1t45<BkgdEn, BkgdOut>;

/** Measured Target SYNC width (Ticks) */
static unsigned targetSyncWidth;

/** Calculated time for target '1' bit (Ticks) */
static unsigned oneBitTime;

/** Calculated time for target '0' bit (Ticks) */
static unsigned zeroBitTime;

/** Calculated time for to sample target response (Ticks) */
static unsigned sampleBitTime;

/** Calculated time for target minimum bit period (Ticks) */
static unsigned minPeriod;

/** Calculated time for 64 target clock cycles - for ACKN (us) */
static unsigned targetClocks64TimeUS;

/** Calculated time for 150 target clock cycles - for ACKN (us) */
static unsigned targetClocks150TimeUS;

///**
// * Return the maximum of two values
// *
// *  @param a One value
// *  @param b Other value
// *
// *  @return Max value
// */
//static int max(int a, int b) {
//   return (a>b)?a:b;
//}

/**
 * Converts a time in microseconds to number of ticks
 *
 * @param [in] time Time in microseconds
 *
 * @return Time in ticks
 *
 * @note Assumes prescale has been chosen as a appropriate value. No range checking.
 */
static uint32_t convertMicrosecondsToTicks(int time) {
   long t = ((uint64_t)time*FtmInfo::getClockFrequency())/1000000;

   assert((long)(uint32_t)t == t);
   assert(t != 0);

   // Calculate period
   return (uint32_t)t;
}
/**
 * Converts ticks to time in microseconds
 *
 * @param [in] time  Time in ticks
 *
 * @return Time in microseconds
 *
 * @note Assumes prescale has been chosen as a appropriate value. No range checking.
 */
static uint32_t convertTicksToMicroseconds(int time) {
   long t = ((uint64_t)time*1000000)/FtmInfo::getClockFrequency();

   assert((long)(uint32_t)t == t);
   assert(t != 0);

   return t;
}

/**
 * Initialise interface
 */
void initialise() {

   // Enable clock to timer
   *clockReg  |= FtmInfo::clockMask;

   // Extended features
   ftm->MODE     = FTM_MODE_INIT_MASK|FTM_MODE_FTMEN_MASK|FTM_MODE_WPDIS_MASK;

   // Debug mode
   ftm->CONF     = FTM_CONF_BDMMODE(2);

   // Clear s register changes have immediate effect
   ftm->SC = 0;

   // Common registers
   ftm->CNTIN    = 0;
   ftm->CNT      = 0;
   ftm->MOD      = (uint32_t)-1;

   ftm->SC       =
         FTM_SC_CPWMS(0)| // Left-Aligned
         FTM_SC_CLKS(1)|  // Clock source = system
         FTM_SC_TOIE(0)|  // Timer Overflow Interrupt disabled
         FTM_SC_PS(0);    // Prescale = /1

   ftm->OUTINIT =
         (0<<bkgdEnChannel)|  // Initialise low (disable buffer)
         (1<<bkgdOutChannel); // Initialise high

   Reset::initialise();

   disablePins();
   ftm->CONTROLS[bkgdEnChannel].CnSC  = USBDM::ftm_outputCompareClear;
   ftm->CONTROLS[bkgdOutChannel].CnSC = USBDM::ftm_outputCompareSet;

   // Switch pins to FTM
   FtmInfo::initPCRs(PORT_PCR_DSE_MASK|PORT_PCR_PE_MASK); // DS+PDN
}

/**
 * Determine connection speed using sync pulse
 *
 * @param [out] syncLength Sync length in timer ticks
 *
 * @return Error code, BDM_RC_OK indicates success
 */
USBDM_ErrorCode sync(uint16_t &syncLength) {

   /* SYNC pulse width */
   const uint16_t syncPulseWidthInTicks = convertMicrosecondsToTicks(SYNC_WIDTH);

   static_assert((bkgdInChannel&1)==0, "BKGD Channel must be even");

   // Use Dual Edge on BKGD in
   ftm->COMBINE |= FTM_COMBINE_DECAPEN0_MASK<<(bkgdInChannel*4);

   // Enable dual capture on bkgdInChannel, bkgdInChannel+1
   ftm->CONTROLS[bkgdInChannel].CnSC    = USBDM::ftm_dualEdgeCaptureOneShotFallingEdge;
   ftm->CONTROLS[bkgdInChannel+1].CnSC  = USBDM::ftm_inputCaptureRisingEdge;

   // Set up to drive BKGD low (o-low, en=high)
   ftm->CONTROLS[bkgdOutChannel].CnSC   = USBDM::ftm_outputCompareClear;
   ftm->CONTROLS[bkgdEnChannel].CnSC    = USBDM::ftm_outputCompareSet;

   // Schedule event in the near future
   // This must allow enough time to complete configuration before events
   uint16_t eventTime = ftm->CNT+100;
   ftm->CONTROLS[bkgdOutChannel].CnV    = eventTime;
   ftm->CONTROLS[bkgdEnChannel].CnV     = eventTime;

   // Clear channel flags
   ftm->STATUS &= ~((1<<bkgdInChannel)|(1<<(bkgdInChannel+1))|(1<<bkgdOutChannel)|(1<<bkgdEnChannel));

   // Release force pin control
   enablePins();

   // Wait for event
   while ((ftm->CONTROLS[bkgdOutChannel].CnSC&FTM_CnSC_CHF_MASK) == 0) {
      __asm__("nop");
   }

   // Trigger dual-edge capture on BKGD_In
   ftm->COMBINE |= FTM_COMBINE_DECAP0_MASK<<(bkgdInChannel*4);

   // Drive BKGD high then 3-state
   ftm->CONTROLS[bkgdOutChannel].CnSC   = USBDM::ftm_outputCompareSet;
   ftm->CONTROLS[bkgdEnChannel].CnSC    = USBDM::ftm_outputCompareClear;
   eventTime += syncPulseWidthInTicks;
   ftm->CONTROLS[bkgdOutChannel].CnV    = eventTime;
   ftm->CONTROLS[bkgdEnChannel].CnV     = eventTime+SPEEDUP_PULSE_WIDTH_IN_TICKS;

   static auto pollBkgdIn2Timer = [] {
         return ((ftm->CONTROLS[bkgdInChannel+1].CnSC & FTM_CnSC_CHF_MASK) != 0);
   };
   // Wait for dual-edge & capture
   bool success = USBDM::waitUS(SYNC_TIMEOUT, pollBkgdIn2Timer);

   // Clear dual-edge (in case timeout)
   ftm->COMBINE &= ~(FTM_COMBINE_DECAP0_MASK<<(bkgdInChannel*4));

   disablePins();

   if (success) {
      syncLength = ftm->CONTROLS[bkgdInChannel+1].CnV - ftm->CONTROLS[bkgdInChannel].CnV;
      return BDM_RC_OK;
   }
   return BDM_RC_SYNC_TIMEOUT;
}

/**
 *  Set sync length
 *
 *  @param [in] syncLength Sync length in timer ticks to set
 */
void setSyncLength(uint16_t syncLength) {
   constexpr int SYNC_RESPONSE_CYCLES = 128;    // Number of target clock cycles in the SYNC response
   constexpr int HOST_1_CYCLES_LOW    = 4;      // Number of target clock cycles low for a 1
   constexpr int HOST_0_CYCLES_LOW    = 13;     // Number of target clock cycles low for a 0
   constexpr int HOST_MIN_CYCLES      = 16;     // Minimum number of target clock cycles for entire transfer
   constexpr int HOST_SAMPLE_CYCLE    = 10;     // Number of target clock cycles for sample

   targetSyncWidth   = syncLength;

   // Calculate communication parameters
   zeroBitTime       = ((syncLength*HOST_1_CYCLES_LOW)+(SYNC_RESPONSE_CYCLES/2))/SYNC_RESPONSE_CYCLES;  // Ticks (round)
   oneBitTime        = ((syncLength*HOST_0_CYCLES_LOW)+(SYNC_RESPONSE_CYCLES-1))/SYNC_RESPONSE_CYCLES;  // Ticks (round up)
   sampleBitTime     = ((syncLength*HOST_SAMPLE_CYCLE)+(SYNC_RESPONSE_CYCLES/2))/SYNC_RESPONSE_CYCLES;  // Ticks (round)
   minPeriod         = ((syncLength*HOST_MIN_CYCLES)+(SYNC_RESPONSE_CYCLES-1))/SYNC_RESPONSE_CYCLES;    // Ticks (round up)

   targetClocks64TimeUS   = convertTicksToMicroseconds((syncLength*64)/SYNC_RESPONSE_CYCLES);  // us
   targetClocks150TimeUS  = convertTicksToMicroseconds((syncLength*150)/SYNC_RESPONSE_CYCLES); // us
}

/**
 *  Depending on ACKN mode this function:       \n
 *    - Waits for ACKN pulse with timeout.
 *       OR
 *    - Busy waits for 64 target CPU clocks
 *
 *  @return
 *    \ref BDM_RC_OK           => Success \n
 *    \ref BDM_RC_ACK_TIMEOUT  => No ACKN detected [timeout]
 *
 *  @note  Modified to extend timeout for very slow bus clocks e.g. 32kHz
 */
uint8_t acknowledgeOrWait64(void) {
//   if (cable_status.ackn==ACKN) {
//      // Wait for pin capture or timeout
//     enableInterrupts();
//      WAIT_WITH_TIMEOUT_US(ACKN_TIMEOUTus, (BKGD_TPMxCnSC_CHF!=0));
//     if (BKGD_TPMxCnSC_CHF==0) {
//       return BDM_RC_ACK_TIMEOUT;  //   Return timeout error
//     }
//   }
//   else {
      USBDM::waitUS(targetClocks64TimeUS);
//   }
   return BDM_RC_OK;
}

/**
 *  Depending on ACKN mode this function:       \n
 *    - Waits for ACKN pulse with timeout.
 *       OR
 *    - Busy waits for 150 target CPU clocks
 *
 *  @return
 *    \ref BDM_RC_OK           => Success \n
 *    \ref BDM_RC_ACK_TIMEOUT  => No ACKN detected [timeout]
 */
uint8_t acknowledgeOrWait150(void) {
//   if (cable_status.ackn==ACKN) {
//      // Wait for pin capture or timeout
//      enableInterrupts();
//      WAIT_WITH_TIMEOUT_US(ACKN_TIMEOUTus, (BKGD_TPMxCnSC_CHF!=0));
//      if (BKGD_TPMxCnSC_CHF==0) {
//         return BDM_RC_ACK_TIMEOUT;  //   Return timeout error
//      }
//   }
//   else {
      USBDM::waitUS(targetClocks150TimeUS);
//   }
   return BDM_RC_OK;
}

/**
 * Receive an value over BDM interface
 *
 * @param [in]  length Number of bits to receive
 * @param [out] data   Data received
 */
USBDM_ErrorCode rx(int length, unsigned &data) {

   /** Time to set up Timer - This varies with optimisation! */
   static constexpr unsigned TMR_SETUP_TIME = 40;

   ftm->SC = 0;

   ftm->COMBINE |=
         FTM_COMBINE_COMBINE0_MASK<<(bkgdEnChannel*4)|
         FTM_COMBINE_COMBINE0_MASK<<(bkgdOutChannel*4);

   // Positive pulse for buffer enable
   ftm->CONTROLS[bkgdEnChannel].CnSC    = FTM_CnSC_ELS(2);
   ftm->CONTROLS[bkgdEnChannel].CnV     = TMR_SETUP_TIME;
   ftm->CONTROLS[bkgdEnChannel+1].CnV   = TMR_SETUP_TIME+zeroBitTime-SPEEDUP_PULSE_WIDTH_IN_TICKS;

   // Negative pulse for BKGD out
   ftm->CONTROLS[bkgdOutChannel].CnSC   = FTM_CnSC_ELS(1);
   ftm->CONTROLS[bkgdOutChannel].CnV    = TMR_SETUP_TIME;
   ftm->CONTROLS[bkgdOutChannel+1].CnV  = TMR_SETUP_TIME+zeroBitTime;

   // Capture rising edge of BKGD in
   ftm->CONTROLS[bkgdInChannel].CnSC    = USBDM::ftm_inputCaptureRisingEdge;

   ftm->SC =
         FTM_SC_CPWMS(0)| // Left-Aligned
         FTM_SC_CLKS(1)|  // Clock source = system
         FTM_SC_TOIE(0)|  // Timer Overflow Interrupt disabled
         FTM_SC_PS(0);    // Prescale = /1

   bool success = true;
   unsigned value = 0;
   while (length-->0) {
      ftm->CNT = 0;

      // Clear channel flags
      ftm->STATUS &= ~(
            (1<<bkgdEnChannel) |(1<<(bkgdEnChannel+1))|
            (1<<bkgdOutChannel)|(1<<(bkgdOutChannel+1))|
            (1<<bkgdInChannel));

      // Wait until end of bit
      do {
         __asm("nop");
      } while (ftm->CNT < 2*minPeriod);

      // Should have captured a rising edge from target (or float?)
      success = (ftm->CONTROLS[bkgdInChannel].CnSC & FTM_CnSC_CHF_MASK) != 0;

      volatile uint16_t eventTime = ftm->CONTROLS[bkgdInChannel].CnV;
      value = (value<<1)|((eventTime>sampleBitTime)?1:0);
   }
   data = value;
   if (!success) {
      return BDM_RC_BKGD_TIMEOUT;
   }
   return BDM_RC_OK;
}

/**
 * Receive an 8-bit value over BDM interface
 *
 * @param [out] data   Data received
 */
inline
USBDM_ErrorCode rx8(uint8_t *data) {
   unsigned value;
   USBDM_ErrorCode rc = rx(8, value);
   *data = (uint8_t)value;
   return rc;
}

/**
 * Receive an 16-bit value over BDM interface
 *
 * @param [out] data   Data received
 */
inline
USBDM_ErrorCode rx16(uint16_t *data) {
   unsigned value;
   USBDM_ErrorCode rc = rx(16, value);
   *data = (uint16_t)value;
   return rc;
}

/**
 * Receive an 32-bit value over BDM interface
 *
 * @param [out] data   Data received
 */
inline
USBDM_ErrorCode rx32(uint32_t *data) {
   unsigned value;
   USBDM_ErrorCode rc = rx(32, value);
   *data = value;
   return rc;
}

inline
void txInit() {
   ftm->SC = 0;
   ftm->SYNCONF |= FTM_SYNCONF_SYNCMODE(1)|FTM_SYNCONF_SWWRBUF(1);

   disableInterrupts();
   enablePins();
}

inline
void txComplete() {
   ftm->SC = 0;
   enableInterrupts();
   disablePins();
}

/**
 * Transmit an value over BDM interface
 *
 * @param [in]  length Number of bits to transmit
 * @param [in]  data   Data value to transmit
 *
 * @return Error code, BDM_RC_OK indicates success
 */
USBDM_ErrorCode tx(int length, unsigned data) {

   /** Time to set up Timer - This varies with optimisation! */
   static constexpr unsigned TMR_SETUP_TIME = 40;
   uint32_t mask = (1U<<(length-1));

   ftm->COMBINE |=
         FTM_COMBINE_SYNCEN0_MASK<<(bkgdEnChannel*4)|
         FTM_COMBINE_COMBINE0_MASK<<(bkgdEnChannel*4)|
         FTM_COMBINE_SYNCEN0_MASK<<(bkgdOutChannel*4)|
         FTM_COMBINE_COMBINE0_MASK<<(bkgdOutChannel*4);

   // Positive pulse for buffer enable
   ftm->CONTROLS[bkgdEnChannel].CnSC    = FTM_CnSC_ELS(2);
   ftm->CONTROLS[bkgdEnChannel].CnV     = TMR_SETUP_TIME;

   // Negative pulse for BKGD out
   ftm->CONTROLS[bkgdOutChannel].CnSC   = FTM_CnSC_ELS(1);
   ftm->CONTROLS[bkgdOutChannel].CnV    = TMR_SETUP_TIME;

   while (mask>0) {
      int width;
      if (data&mask) {
         width = TMR_SETUP_TIME+oneBitTime;
      }
      else {
         width = TMR_SETUP_TIME+zeroBitTime;
      }
      data >>= 1;
      ftm->CNT = 0;
      ftm->CONTROLS[bkgdOutChannel+1].CnV  = width;
      ftm->CONTROLS[bkgdEnChannel+1].CnV   = width+SPEEDUP_PULSE_WIDTH_IN_TICKS;
      ftm->SYNC     = FTM_SYNC_SWSYNC(1);

      ftm->SC =
            FTM_SC_CPWMS(0)| // Left-Aligned
            FTM_SC_CLKS(1)|  // Clock source = system
            FTM_SC_TOIE(0)|  // Timer Overflow Interrupt disabled
            FTM_SC_PS(0);    // Prescale = /1

      // Clear channel flags
      ftm->STATUS &= ~(
            (1<<bkgdEnChannel) |(1<<(bkgdEnChannel+1))|
            (1<<bkgdOutChannel)|(1<<(bkgdOutChannel+1)));

      // Wait until end of bit
      do {
         __asm("nop");
      } while (ftm->CNT < TMR_SETUP_TIME+minPeriod);
      ftm->SC = 0;
      mask >>= 1;
   }
   return BDM_RC_OK;
}

inline
USBDM_ErrorCode tx8(uint8_t data) {
   return tx(8, data);
}

inline
USBDM_ErrorCode tx16(uint8_t data) {
   return tx(16, data);
}

inline
USBDM_ErrorCode tx24(uint8_t data) {
   return tx(24, data);
}

inline
USBDM_ErrorCode tx32(uint8_t data) {
   return tx(32, data);
}

/**
 * *************************************************************************
 */
//============================================================
// The following commands DO NOT expect an ACK & do not delay
// Interrupts are left disabled!
//

/**
 * Write command byte, truncated sequence
 *
 * @param cmd command byte to write
 *
 * @note Interrupts are left disabled, no ACK is expected
 */
void cmd_0_0_T(uint8_t cmd) {
   txInit();
   tx8(cmd);
//   txComplete();
}

/**
 *  Write command byte + parameter, truncated sequence
 *
 *  @param cmd         command byte to write
 *  @param parameter   byte parameter to write
 *
 * @note Interrupts are left disabled, no ACK is expected
 */
void cmd_1B_0_T(uint8_t cmd, uint8_t parameter) {
   txInit();
   tx8(cmd);
   tx8(parameter);
//   txComplete();
}

/**
 *  Special for Software Reset HCS08, truncated sequence
 *
 *  @param cmd         command byte to write
 *  @param parameter1  word parameter to write
 *  @param parameter2  byte parameter to write
 *
 * @note Interrupts are left disabled, no ACK is expected
 */
void cmd_1W1B_0_T(uint8_t cmd, uint16_t parameter1, uint8_t parameter2) {
   txInit();
   tx8(cmd);
   tx16(parameter1);
   tx8(parameter2);
//   txComplete();
}

//============================================================
// The following commands DO NOT expect an ACK & do not delay
//

/**
 *  Write cmd without ACK (HCS08/RS08/CFV1)
 *
 *  @param cmd command byte to write
 *
 *  @note No ACK is expected
 */
void cmd_0_0_NOACK(uint8_t cmd) {
   txInit();
   tx8(cmd);
   txComplete();
}

/**
 * Write cmd & read byte without ACK (HCS08)
 *
 *  @param cmd command byte to write
 *  @param result word read
 *
 *  @note No ACK is expected
 */
void cmd_0_1B_NOACK(uint8_t cmd, uint8_t *result) {
   txInit();
   tx8(cmd);
   rx8(result);
   txComplete();
}

/**
 *  Write cmd & byte without ACK
 *
 *  @param cmd       command byte to write
 *  @param parameter byte to write
 *
 *  @note No ACK is expected
 */
void cmd_1B_0_NOACK(uint8_t cmd, uint8_t parameter) {
   txInit();
   tx8(cmd);
   tx8(parameter);
   txComplete();
}

/**
 *  Write cmd & read word without ACK (HCS08)
 *
 *  @param cmd    command byte to write
 *  @param result word read
 *
 *  @note No ACK is expected
 */
void cmd_0_1W_NOACK(uint8_t cmd, uint16_t *result) {
   txInit();
   tx8(cmd);
   rx16(result);
   txComplete();
}

/**
 *  Write cmd & word without ACK
 *
 *  @param cmd       command byte to write
 *  @param parameter word to write
 *
 *  @note No ACK is expected
 */
void cmd_1W_0_NOACK(uint8_t cmd, uint16_t parameter) {
   txInit();
   tx8(cmd);
   tx16(parameter);
   txComplete();
}

/**
 *  Write cmd, word & read word without ACK
 *
 *  @param cmd        command byte to write
 *  @param parameter  word to write
 *  @param result     word pointer for read (status+data byte)
 *
 *  @note no ACK is expected
 */
void cmd_1W_1W_NOACK(uint8_t cmd, uint16_t parameter, uint16_t *result) {
   txInit();
   tx8(cmd);
   tx16(parameter);
   rx16(result);
   txComplete();
}

/**
 *  Write cmd, word, byte & read byte without ACK
 *
 *  @param cmd        command byte to write
 *  @param parameter  word to write
 *  @param value      byte to write
 *  @param status     byte pointer for read
 *
 *  @note no ACK is expected
 */
void cmd_1W1B_1B_NOACK(uint8_t cmd, uint16_t parameter, uint8_t value, uint8_t *status) {
   txInit();
   tx8(cmd);
   tx16(parameter);
   tx8(value);
   rx8(status);
   txComplete();
}

//====================================================================
// The following DO expect an ACK or wait at end of the command phase

/**
 *  Write cmd
 *
 *  @param cmd command byte to write
 *
 *  @note ACK is expected
 */
uint8_t cmd_0_0(uint8_t cmd) {
uint8_t rc;
    cmd_0_0_T(cmd);
//    txInit();
//    tx8(cmd);
    rc = acknowledgeOrWait64();
    txComplete();
    return rc;
}

/**
 *  Write cmd, read byte
 *
 *  @param cmd        command byte to write
 *  @param result     byte read
 *
 *  @note ACK is expected
 */
uint8_t cmd_0_1B(uint8_t cmd, uint8_t *result) {
uint8_t rc;
   txInit();
   tx8(cmd);
   rc = acknowledgeOrWait64();
   rx8(result);
   txComplete();
   return rc;
}

/**
 *  Write cmd & read word
 *
 *  @param cmd    command byte to write
 *  @param result word read
 *
 *  @note ACK is expected
 */
uint8_t cmd_0_1W(uint8_t cmd, uint16_t *result) {
uint8_t rc;
   txInit();
   tx8(cmd);
   rc = acknowledgeOrWait64();
   rx16(result);
   txComplete();
   return rc;
}

/**
 *  Write cmd & read longword
 *
 *  @param cmd    command byte to write
 *  @param result longword read
 *
 *  @note ACK is expected
 */
uint8_t cmd_0_1L(uint8_t cmd, uint32_t *result) {
   uint8_t rc;
   txInit();
   tx8(cmd);
   rc = acknowledgeOrWait64();
   rx32(result);
   return rc;
}

/**
 *  Write cmd & byte
 *
 *  @param cmd        command byte to write
 *  @param parameter  byte to write
 *
 *  @note ACK is expected
 */
uint8_t cmd_1B_0(uint8_t cmd, uint8_t parameter) {
   uint8_t rc;
   txInit();
   tx8(cmd);
   tx8(parameter);
   rc = acknowledgeOrWait64();
   txComplete();
   return rc;
}

/**
 *  Write cmd & word
 *
 *  @param cmd       command byte to write
 *  @param parameter word to write
 *
 *  @note ACK is expected
 */
uint8_t cmd_1W_0(uint8_t cmd, uint16_t parameter) {
uint8_t rc;
   txInit();
   tx8(cmd);
   tx16(parameter);
   rc = acknowledgeOrWait64();
   txComplete();
   return rc;
}

/**
 *  Write cmd & longword
 *
 *  @param cmd       command byte to write
 *  @param parameter longword to write
 *
 *  @note ACK is expected
 */
uint8_t cmd_1L_0(uint8_t cmd, uint32_t parameter) {
uint8_t rc;
   txInit();
   tx8(cmd);
   tx32(parameter);
   rc = acknowledgeOrWait64();
   txComplete();
   return rc;
}

/**
 *  Write cmd, word & read byte (read word but return byte - HC/S12(x))
 *
 *  @param cmd       command byte to write
 *  @param parameter word to write
 *  @param result    byte read
 *
 *  @note ACK is expected
 */
uint8_t cmd_1W_1WB(uint8_t cmd, uint16_t parameter, uint8_t *result) {
uint8_t rc;
   txInit();
   tx8(cmd);
   tx16(parameter);
   rc = acknowledgeOrWait150();
   if ((parameter)&0x0001) {
      (void)rx8(result);
      rx8(result);
   } else {
      rx8(result);
      uint8_t dummy;
      rx8(&dummy);
   }
   txComplete();
   return rc;
}

/**
 *  Write cmd & 2 words
 *
 *  @param cmd       command byte to write
 *  @param parameter1 word to write
 *  @param parameter2 word to write
 *
 *  @note ACK is expected
 */
uint8_t cmd_2W_0(uint8_t cmd, uint16_t parameter1, uint16_t parameter2) {
uint8_t rc;
   txInit();
   tx8(cmd);
   tx16(parameter1);
   tx16(parameter2);
   rc = acknowledgeOrWait150();
   txComplete();
   return rc;
}

/**
 *  Write cmd, word & read word
 *
 *  @param cmd        command byte to write
 *  @param parameter  word to write
 *  @param result     word read
 *
 *  @note ACK is expected
 */
uint8_t cmd_1W_1W(uint8_t cmd, uint16_t parameter, uint16_t *result) {
uint8_t rc;
   txInit();
   tx8(cmd);
   tx16(parameter);
   rc = acknowledgeOrWait150();
   rx16(result);
   txComplete();
   return rc;
}

/**
 *  Write cmd, word and a byte
 *  (sends 2 words, the byte in both high and low byte of the 16-bit value)
 *
 *  @param cmd        command byte to write
 *  @param parameter1 word to write
 *  @param parameter2 bye to write
 *
 *  @note ACK is expected
 */
uint8_t cmd_2WB_0(uint8_t cmd, uint16_t parameter1, uint8_t parameter2) {
uint8_t rc;
   txInit();
   tx8(cmd);
   tx16(parameter1);
   tx8(parameter2);
   tx8(parameter2);
   rc = acknowledgeOrWait150();
   txComplete();
   return rc;
}

/**
 *  Write cmd, word & read byte
 *
 *  @param cmd        command byte to write
 *  @param parameter  word to write
 *  @param result     byte read
 *
 *  @note ACK is expected
 */
uint8_t cmd_1W_1B(uint8_t cmd, uint16_t parameter, uint8_t *result) {
uint8_t rc;
   txInit();
   tx8(cmd);
   tx16(parameter);
   rc = acknowledgeOrWait64();
   rx8(result);
   txComplete();
   return rc;
}

/**
 *  Write cmd, word & byte
 *
 *  @param cmd         command byte to write
 *  @param parameter1  word to write
 *  @param parameter2  byte to write
 *
 *  @note ACK is expected
 */
uint8_t cmd_1W1B_0(uint8_t cmd, uint16_t parameter1, uint8_t parameter2) {
uint8_t rc;
   txInit();
   tx8(cmd);
   tx16(parameter1);
   tx8(parameter2);
   rc = acknowledgeOrWait150();
   txComplete();
   return rc;
}

/**
 *  Write cmd, 24-bit value & byte
 *
 *  @param cmd    command byte to write
 *  @param addr   24-bit value to write
 *  @param value  byte to write
 *
 *  @note ACK is expected
 */
uint8_t cmd_1A1B_0(uint8_t cmd, uint32_t addr, uint8_t value) {
uint8_t rc;
   txInit();
   tx8(cmd);
   tx24(addr);
   tx8(value);
   rc = acknowledgeOrWait150();
   txComplete();
   return rc;
}

/**
 *  Write cmd, 24-bit value & word
 *
 *  @param cmd    command byte to write
 *  @param addr   24-bit value to write
 *  @param value  word to write
 *
 *  @note ACK is expected
 */
uint8_t cmd_1A1W_0(uint8_t cmd, uint32_t addr, uint16_t value) {
uint8_t rc;
   txInit();
   tx8(cmd);
   tx24(addr);
   tx16(value);
   rc = acknowledgeOrWait150();
   txComplete();
   return rc;
}

/**
 *  Write cmd, 24-bit value & longword
 *
 *  @param cmd    command byte to write
 *  @param addr   24-bit value to write
 *  @param value  ptr to longword to write
 *
 *  @note ACK is expected
 */
uint8_t cmd_1A1L_0(uint8_t cmd, uint32_t addr, uint32_t value) {
uint8_t rc;
   txInit();
   tx8(cmd);
   tx24(addr);
   tx32(value);
   rc = acknowledgeOrWait150();
   txComplete();
   return rc;
}

/**
 *  Write cmd, 24-bit value & read byte
 *
 *  @param cmd     command byte to write
 *  @param addr    24-bit value to write
 *  @param result  ptr to longword to read
 *
 *  @note ACK is expected
 */
uint8_t cmd_1A_1B(uint8_t cmd, uint32_t addr, uint8_t *result) {
uint8_t rc;
   txInit();
   tx8(cmd);
   tx24(addr);
   rc = acknowledgeOrWait150();
   rx8(result);
   txComplete();
   return rc;
}

}; // End namespace Bdm
