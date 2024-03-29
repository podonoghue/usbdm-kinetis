/**
 * @file     sim.h (180.ARM_Peripherals/Project_Headers/sim.h)
 * @brief    System Integration Module
 *
 * @version  V4.12.1.210
 * @date     13 April 2016
 */

#ifndef HEADER_SIM_H
#define HEADER_SIM_H
/*
 * *****************************
 * *** DO NOT EDIT THIS FILE ***
 * *****************************
 *
 * This file is generated automatically.
 * Any manual changes will be lost.
 */
#include "pin_mapping.h"

namespace USBDM {

/**
 * @addtogroup SIM_Group SIM, System Integration Module
 * @brief Abstraction for System Integration Module
 * @{
 */

/**
 * @brief Template class representing the System Integration Module (SIM)
 *
 */
class Sim : public SimInfo {
public:
   /**
    * Default value for Sim::ClockInit
    * This value is created from Configure.usbdmProject settings
    */
   static constexpr ClockInit DefaultSopt2Values[] = {
   { // ClockConfig_FEE_IRC48MHz (McgClockMode_FEE)
      SimPeripheralClockSource_Irc48mClk , // Peripheral Clock - IRC48 MHz clock
      SimUsbFullSpeedClockSource_PeripheralClk , // USB Clock - Peripheral Clock/SIM_CLKDIV2
      SimClkoutSel_FlashClk , // CLKOUT pin clock - Flash clock
      SimFlexbusSecurity_None , // FlexBus off-chip access security level - None - All accesses are disallowed
      SimRtcClkoutSel_32kHz , // RTC clock out source - RTC 32kHz clock
      SimTraceClockoutSel_McgOutClk,  // Debug trace clock select - MCGOUTCLK
      SimLpuartClockSource_PeripheralClk,  // LPUART Clock select - Peripheral Clock
   },
};

   /**
    * Default value for Sim::DefaultInit
    * This value is created from Configure.usbdmProject settings (Peripheral Parameters->SIM)
    */
   static constexpr Init DefaultInitValue {
      SimPeripheralClockSource_Irc48mClk , // Peripheral Clock - IRC48 MHz clock
      SimUsbFullSpeedClockSource_PeripheralClk , // USB Clock - Peripheral Clock/SIM_CLKDIV2
      SimClkoutSel_FlashClk , // CLKOUT pin clock - Flash clock
      SimRtcClkoutSel_32kHz , // RTC clock out source - RTC 32kHz clock
      SimTraceClockoutSel_McgOutClk , // Debug trace clock select - MCGOUTCLK
      SimFlexbusSecurity_None , // FlexBus off-chip access security level - None - All accesses are disallowed
      SimErc32kClkoutPinSelect_None , // ERCLK32K Clock Output - ERCLK32K is not output
      SimErc32kSel_LpoClk , // ERCLK32K clock source - LPO 1kHz clock
      SimUsbPower_EnabledInAll,  // USB voltage regulator power control - Enabled in all modes
      SimLpuartClockSource_PeripheralClk,  // LPUART Clock select - Peripheral Clock
      SimFtm0Flt0_Ftm0Fault0,  // FTM0 Fault 0 Select - FTM0_FLT0 pin
      SimFtm0Trg0Src_Cmp0,  // FTM0 Hardware Trigger 0 Source - CMP0 output
      SimFtm0Flt1_Ftm0Fault1,  // FTM0 Fault 1 Select - FTM0_FLT1 pin
      SimFtm0Trg1Src_PdbTrigger1,  // FTM0 Hardware Trigger 1 Source - PDB output trigger 1
      SimFtm0ClkSel_FtmClkin0,  // FTM0 External Clock Pin - FTM_CLKIN0 pin
      SimFtm1Flt0_Ftm1Fault0,  // FTM1 Fault 0 Select - FTM1_FLT0 pin
      SimFtm1Ch0Src_IcPin,  // FTM 1 channel 0 input capture source - FTM1_CH0 signal
      SimFtm1ClkSel_FtmClkin0,  // FTM1 External Clock Pin - FTM_CLKIN0 pin
      SimFtm2Flt0_Ftm2Fault0,  // FTM2 Fault 0 Select - FTM2_FLT0 pin
      SimFtm2Ch0Src_IcPin,  // FTM2 channel 0 input capture source - FTM2_CH0 signal
      SimFtm2Ch1Src_IcPin,  // FTM2 channel 1 input capture source - FTM2_CH1 signal
      SimFtm2ClkSel_FtmClkin0,  // FTM2 External Clock Pin - FTM_CLKIN0 pin
      SimFtm3Flt0_Ftm3Fault0,  // FTM3 Fault 0 Select - FTM3_FLT0 pin
      SimFtm3Trg0Src_Ftm1,  // FTM3 Hardware Trigger 0 Source - FTM1 channel match
      SimFtm3Trg1Src_Ftm2,  // FTM3 Hardware Trigger 1 Source - FTM2 channel match
      SimFtm3ClkSel_FtmClkin0,  // FTM3 External Clock Pin - FTM_CLKIN0 pin
      SimUart0RxSrc_RxPin,  // UART0 receive data source - Rx pin
      SimUart0TxSrc_Direct,  // UART0 transmit data source - Tx pin
      SimLpuart0RxSrc_RxPin,  // LPUART0 receive data source - Rx pin
      SimUart1RxSrc_RxPin,  // UART1 receive data source - Rx pin
      SimUart1TxSrc_Direct,  // UART1 transmit data source - Tx pin
      SimAdc0TriggerMode_Pdb , // ADC0 trigger mode - Triggered by PDB
      SimAdc0TriggerSrc_External,  // ADC0 trigger source - External trigger pin input (PDB0_EXTRG)
      SimAdc1TriggerMode_Pdb , // ADC1 trigger mode - Triggered by PDB
      SimAdc1TriggerSrc_External,  // ADC1 trigger source - External trigger pin input (PDB0_EXTRG)
      SimFtm0Ch0OutputSrc_Direct,  // FTM0 channel 0 output source - FTM0 ch 0 direct
      SimFtm3Ch0OutputSrc_Direct,  // FTM3 channel 0 output source - FTM3 ch 0 direct
      SimFtm0Ch1OutputSrc_Direct,  // FTM0 channel 1 output source - FTM0 ch 1 direct
      SimFtm3Ch1OutputSrc_Direct,  // FTM3 channel 1 output source - FTM3 ch 1 direct
      SimFtm0Ch2OutputSrc_Direct,  // FTM0 channel 2 output source - FTM0 ch 2 direct
      SimFtm3Ch2OutputSrc_Direct,  // FTM3 channel 2 output source - FTM3 ch 2 direct
      SimFtm0Ch3OutputSrc_Direct,  // FTM0 channel 3 output source - FTM0 ch 3 direct
      SimFtm3Ch3OutputSrc_Direct,  // FTM3 channel 3 output source - FTM3 ch 3 direct
      SimFtm0Ch4OutputSrc_Direct,  // FTM0 channel 4 output source - FTM0 ch 4 direct
      SimFtm3Ch4OutputSrc_Direct,  // FTM3 channel 4 output source - FTM3 ch 4 direct
      SimFtm0Ch5OutputSrc_Direct,  // FTM0 channel 5 output source - FTM0 ch 5 direct
      SimFtm3Ch5OutputSrc_Direct,  // FTM3 channel 5 output source - FTM3 ch 5 direct
      SimFtm0Ch6OutputSrc_Direct,  // FTM0 channel 6 output source - FTM0 ch 6 direct
      SimFtm3Ch6OutputSrc_Direct,  // FTM3 channel 6 output source - FTM3 ch 6 direct
      SimFtm0Ch7OutputSrc_Direct,  // FTM0 channel 7 output source - FTM0 ch 7 direct
      SimFtm3Ch7OutputSrc_Direct,  // FTM3 channel 7 output source - FTM3 ch 7 direct
   };

   static void initRegs() {
   
      DefaultInitValue.configure();
   };



};

/**
 * End SIM_Group
 * @}
 */

} // End namespace USBDM

#endif /* HEADER_SIM_H */
