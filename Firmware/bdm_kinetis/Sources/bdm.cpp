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

/** Time to set up Timer - This varies with optimisation! */
static constexpr unsigned TMR_SETUP_TIME = 40;//15;

/** Length of high drive before 3-state i.e. speed-up pulse (ticks) */
static constexpr unsigned SPEEDUP_PULSE_WIDTH_IN_TICKS = 1;

/** Pointer to hardware */
static constexpr volatile FTM_Type *ftm = reinterpret_cast<volatile FTM_Type*>(FtmInfo::ftm);

/** Pointer to clock register */
static constexpr volatile uint32_t *clockReg  = reinterpret_cast<volatile uint32_t*>(FtmInfo::clockReg);

/**
 * Disable FTM control of BKGD
 */
static void disablePins() {
   ftm->SWOCTRL =
         (0<<(bkgdEnChannel+8)) |(1<<bkgdEnChannel)|  // Force low (disable buffer)
         (1<<(bkgdOutChannel+8))|(1<<bkgdOutChannel); // Force high
}

/**
 * Enable FTM control from BKGD
 */
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

/** Calculated time for recovery after a '1' Tx (Ticks) */
static unsigned oneRecoveryTime;

/** Calculated time for recovery after a '0' Tx (Ticks) */
static unsigned zeroRecoveryTime;

/**
 * Return the maximum of two values
 *
 *  @param a One value
 *  @param b Other value
 *
 *  @return Max value
 */
static int max(int a, int b) {
   return (a>b)?a:b;
}

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
///**
// * Converts ticks to time in microseconds
// *
// * @param [in] time  Time in ticks
// *
// * @return Time in microseconds
// *
// * @note Assumes prescale has been chosen as a appropriate value. No range checking.
// */
//static uint32_t convertTicksToMicroseconds(int time) {
//   long t = ((uint64_t)time*1000000)/FtmInfo::getClockFrequency();
//
//   assert((long)(uint32_t)t == t);
//   assert(t != 0);
//
//   return t;
//}

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

   zeroRecoveryTime  = max(TMR_SETUP_TIME, minPeriod-zeroBitTime);   // Ticks
   oneRecoveryTime   = max(TMR_SETUP_TIME, minPeriod-oneBitTime);    // Ticks
}

USBDM_ErrorCode rxDualEdgePulse(uint8_t *p) {

   /** Time to set up Timer - This varies with optimisation! */
   static constexpr unsigned TMR_SETUP_TIME = 40;

   Reset::low();

   ftm->SC = 0;

   enablePins();
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

   bool success;
   for (int bitNum=0; bitNum<8; bitNum++) {
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
      *p = (eventTime>sampleBitTime)?1:0;
   }
   disablePins();
   Reset::high();
   if (!success) {
      return BDM_RC_BKGD_TIMEOUT;
   }

   return BDM_RC_OK;
}

USBDM_ErrorCode txDualEdgePulse(uint8_t data) {

   /** Time to set up Timer - This varies with optimisation! */
   static constexpr unsigned TMR_SETUP_TIME = 40;

   Reset::low();

   ftm->SC = 0;
   ftm->SYNCONF |= FTM_SYNCONF_SYNCMODE(1)|FTM_SYNCONF_SWWRBUF(1);

   enablePins();
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

   for (int bitNum=0; bitNum<8; bitNum++) {
      int width;
      if (data&1) {
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
      __asm("nop");
   }
   disablePins();
   Reset::high();
   return BDM_RC_OK;
}

}; // End namespace Bdm
