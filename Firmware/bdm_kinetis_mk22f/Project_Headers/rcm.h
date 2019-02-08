/**
 * @file     rcm.h (180.ARM_Peripherals/Project_Headers/rcm.h)
 * @brief    Reset Control Module interface
 *
 * @version  V4.12.1.210
 * @date     23 Sep 2017
 */

#ifndef HEADER_RCM_H
#define HEADER_RCM_H
 /*
 * *****************************
 * *** DO NOT EDIT THIS FILE ***
 * *****************************
 *
 * This file is generated automatically.
 * Any manual changes will be lost.
 */
#include "hardware.h"
#include "stringFormatter.h"

namespace USBDM {

/**
 * @addtogroup RCM_Group RCM, Reset Control Module
 * @brief Abstraction for Reset Control Module
 * @{
 */

/**
 * Indicates reason for reset
 */
enum RcmSource {
   RcmSource_Wakeup  = (1<<0),   //!  Low Leakage Wake-up Reset
   RcmSource_lvd     = (1<<1),   //!  Low-Voltage Detect Reset
   RcmSource_Loc     = (1<<2),   //!  Loss-of-Clock Reset
   RcmSource_Lol     = (1<<3),   //!  Loss-of-Lock Reset
   RcmSource_4       = (1<<4),   //!  Reserved
   RcmSource_Wdog    = (1<<5),   //!  Watchdog
   RcmSource_Pin     = (1<<6),   //!  External Reset Pin
   RcmSource_Por     = (1<<7),   //!  Power-On Reset
   RcmSource_Jtag    = (1<<8),   //!  JTAG Generated Reset
   RcmSource_Lockup  = (1<<9),   //!  Core Lockup
   RcmSource_Sw      = (1<<10),  //!  Software
   RcmSource_Mdm_Ap  = (1<<11),  //!  MDM-AP System Reset Request
   RcmSource_Ezpt    = (1<<12),  //!  EzPort Reset
   RcmSource_Sackerr = (1<<13),  //!  Stop Mode Acknowledge Error Reset
   RcmSource_14      = (1<<14),  //!  Reserved
   RcmSource_15      = (1<<15),  //!  Reserved
};

/**
 * Reset pin filter select in run and wait modes.
 * Selects how the reset pin filter is enabled in run and wait modes.
 */
enum RcmResetPinRunWaitFilter {
   RcmResetPinRunWaitFilter_Disabled           = RCM_RPFC_RSTFLTSRW(0b00), //!< All Filtering disabled
   RcmResetPinRunWaitFilter_BusCLock           = RCM_RPFC_RSTFLTSRW(0b01), //!< Bus clock filter enabled for normal operation
   RcmResetPinRunWaitFilter_LowPowerOscillator = RCM_RPFC_RSTFLTSRW(0b10), //!< LPO clock filter enabled for normal operation
};

/**
 * Reset pin filter select in stop mode.
 * Selects how the reset pin filter is enabled in STOP and VLPS modes
 */
enum RcmResetPinStopFilter {
   RcmResetPinStopFilter_Disabled           = RCM_RPFC_RSTFLTSS(0), //!< All Filtering disabled
   RcmResetPinStopFilter_LowPowerOscillator = RCM_RPFC_RSTFLTSS(1), //!< LPO clock filter enabled
};

/**
 * Template class providing interface to Reset Control Module.
 *
 * @tparam info      Information class for RCM
 *
 * @code
 * using rcm = RcmBase_T<RcmInfo>;
 *
 *  rcm::configure();
 *
 * @endcode
 */
template <class Info>
class RcmBase_T {

public:
   /** Hardware instance pointer */
   static __attribute__((always_inline)) volatile RCM_Type &rcm() { return Info::rcm(); }

public:
   /**
    * Initialise RCM to default settings as determined by Configure.usbdmProject
    */
   static void defaultConfigure() {
      // Configure RCM
      rcm().RPFC  = Info::rcm_rpfc;
      rcm().RPFW  = Info::rcm_rpfw;
   }

   /**
    * Configure filtering.
    *
    * @param rcmResetPinRunWaitFilter  Reset pin filter select in run and wait modes.
    * @param rcmResetPinStopFilter     Reset pin filter select in stop mode.
    * @param resetWidth                Reset pin filter bus clock filter width [1..32]
    *
    * @note These settings persist through resets other than POR
    */
   static void configure(
         RcmResetPinRunWaitFilter rcmResetPinRunWaitFilter,
         RcmResetPinStopFilter    rcmResetPinStopFilter,
         unsigned                 resetWidth) {

      resetWidth--;
      usbdm_assert(resetWidth<=RCM_RPFW_RSTFLTSEL_MASK, "Reset width out of range");

      rcm().RPFC = rcmResetPinRunWaitFilter|rcmResetPinStopFilter;
      rcm().RPFW = RCM_RPFW_RSTFLTSEL(resetWidth);
   }

   /**
    * Returns a bit mask indicating the source of the last reset.
    * See RcmSource for bit masks to use.
    *
    * @return Bit mask representing sources
    */
   static uint32_t getResetSource() {
      return (rcm().SRS1<<8)|rcm().SRS0;
   }

#ifdef RCM_SSRS0_SWAKEUP_MASK
   /**
    * Returns a bit mask indicating the cumulative reset sources since last cleared.
    *
    * @return Bit mask representing sources
    */
   static uint32_t getStickyResetSource() {
      return (rcm().SSRS1<<8)|rcm().SSRS0;
   }

   /**
    * Returns a bit mask indicating the cumulative reset sources since last cleared.
    * The cumulative value is cleared.
    *
    * @return Bit mask representing sources
    */
   static uint32_t getAndClearStickyResetSource() {
      uint32_t snapShot = (rcm().SSRS1<<8)|rcm().SSRS0;
      rcm().SSRS0 = 0xFF;
      rcm().SSRS1 = 0xFF;
      return snapShot;
   }
#endif

   /**
    * Returns a string indicating the source of the reset indicated by source.
    *
    * @param source Pointer to string in static buffer representing reset sources
    */
   static const char *getResetSourceDescription(uint32_t source) {
      static const char *names[] = {
            "Wakeup,",
            "Lvd,",
            "Loc,",
            "Lol,",
            "4,",
            "Wdog,",
            "Pin,",
            "Por,",
            "Jtag,",
            "Lockup,",
            "Sw,",
            "Mdm_Ap,",
            "Ezpt,",
            "Sackerr,",
            "14,",
            "15,",
      };
      static char buff[20];
      USBDM::StringFormatter stringFormatter(buff, sizeof(buff));
      for (unsigned index=0; index<(sizeof(names)/sizeof(names[0])); index++) {
         if (source&(1<<index)) {
            stringFormatter.write(names[index]);
         }
      }
      return buff;
   }
   /**
    * Returns a string indicating the source of the last reset.
    */
   static const char *getResetSourceDescription() {
      return getResetSourceDescription(getResetSource());
   }
};

#ifdef USBDM_RCM_IS_DEFINED
/**
 * Class providing interface to Reset Control Module
 */
class Rcm : public RcmBase_T<RcmInfo> {};

#endif

/**
 * End RMC_Group
 * @}
 */
} // End namespace USBDM

#endif /* HEADER_RCM_H */