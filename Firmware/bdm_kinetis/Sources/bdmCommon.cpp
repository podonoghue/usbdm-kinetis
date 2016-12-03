/*! \file
    \brief USBDM - Common BDM routines.

   \verbatim
   This software was modified from \e TBLCF software
   This software was modified from \e TBDML software

   USBDM
   Copyright (C) 2007  Peter O'Donoghue

   Turbo BDM Light
   Copyright (C) 2005  Daniel Malik

   Turbo BDM Light ColdFire
   Copyright (C) 2005  Daniel Malik

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
   \endverbatim

   \verbatim
   Change History
+================================================================================================
| 22 Nov 2011 | More thoroughly disabled interfaces when off                       - pgo, ver 4.8 
| 27 Oct 2011 | Modified timer code to avoid TSCR1 changes & TCNT resets           - pgo, ver 4.8 
|  8 Aug 2010 | Re-factored interrupt handling                                     - pgo 
| 10 Apr 2010 | Changed to accommodate changes to Vpp interface                    - pgo 
|  5 Feb 2010 | bdm_cycleTargetVdd() now disables Vdd monitoring                   - pgo
|  4 Feb 2010 | bdm_cycleTargetVdd() parametised for mode                          - pgo
| 19 Oct 2009 | Modified Timer code - Folder together with JS16 code               - pgo
| 20 Sep 2009 | Increased Reset wait                                               - pgo
|    Sep 2009 | Major changes for V2                                               - pgo
+================================================================================================
\endverbatim
*/

#include <string.h>
#include "Configure.h"
#include "Commands.h"
#include "bdmCommon.h"
#include "cmdProcessing.h"
#include "swd.h"
#include "bdm.h"

//=============================================================================================================================================
/** Longest time needed for soft reset of the BDM interface (512 BDM cycles @ 400kHz = 1280us) */
//static constexpr int SOFT_RESET_us = 1280U;

///=========================================================================
// Target monitoring and status routines
//
//=========================================================================

/**
 *  Interrupt function servicing the IC interrupt from Vdd changes
 *  This routine has several purposes:
 *   - Triggers POR into Debug mode on RS08/HCS08/CFV1 targets\n
 *   - Turns off Target power on overload\n
 *   - Updates Target power status\n
 */
//TODO bdm_targetVddSense()
void bdm_targetVddSense(void) {

//#if (HW_CAPABILITY&CAP_VDDSENSE)
//   CLEAR_VDD_SENSE_FLAG(); // Clear Vdd Change Event
//
//   if (VDD_SENSE) {  // Vdd rising
//      // Needs to be done on non-interrupt thread?
//      switch (cable_status.target_type) {
//#if (HW_CAPABILITY&CAP_BDM)
//         case    T_HC12:
//         case    T_HCS08:
//         case    T_RS08:
//         case    T_CFV1:
//            (void)bdmHCS_powerOnReset();
//            break;
//#endif
//#if (HW_CAPABILITY&CAP_CFVx_HW)
//         case    T_CFVx:
//            (void)bdmCF_powerOnReset();
//            break;
//#endif
//         case    T_JTAG:
//         case    T_EZFLASH:
//         case    T_MC56F80xx:
//         case    T_ARM_JTAG:
//         case    T_OFF:
//         default:
//            break;
//      }
//      }
//   else { // Vdd falling
//      VDD_OFF();   // Turn off Vdd in case it's an overload
//      }
//   // Update power status
//   (void)checkTargetVdd();
//#endif // CAP_VDDSENSE
}

#ifdef CLEAR_RESET_SENSE_FLAG
/*
 * Interrupt function servicing the IC interrupt from RESET_IN assertion
 */
//TODO bdm_resetSense()
void bdm_resetSense(void) {
//   CLEAR_RESET_SENSE_FLAG();             // Acknowledge RESET IC Event
   if (RESET_IS_LOW()) {
      cable_status.reset = RESET_DETECTED;  // Record that reset was asserted
   }
}
#endif


//=========================================================================
// Target power control
//
//=========================================================================

#define VDD_2v  (((2*255)/5)*9/10)  // 10% allowance on 2V
#define VDD_3v3 (((3*255)/5)*9/10)  // 10% allowance on 3.3V
#define VDD_5v  (((5*255)/5)*9/10)  // 10% allowance on 5V

/*
 * Checks Target Vdd  - Updates Target Vdd LED & status
 *
 * Updates \ref cable_status
 *
 * @return E_NO_ERROR on success
 */
USBDM_ErrorCode checkTargetVdd(void) {
#if (HW_CAPABILITY&CAP_VDDSENSE)
   if (TargetVdd::vddOK()) {
      PowerLed::on();
      if (bdm_option.targetVdd == BDM_TARGET_VDD_OFF) {
         cable_status.power = BDM_TARGET_VDD_EXT;
      }
      else {
         cable_status.power = BDM_TARGET_VDD_INT;
      }
   }
   else {
      PowerLed::off();
      if (bdm_option.targetVdd == BDM_TARGET_VDD_OFF) {
         cable_status.power = BDM_TARGET_VDD_NONE;
      }
      else {
         cable_status.power = BDM_TARGET_VDD_ERR;
#if (HW_CAPABILITY&CAP_VDDCONTROL)
    	 // Possible overload
//         VDD_OFF();
#endif
      }
   }
#else
   // No target Vdd sensing - assume external Vdd is present
   cable_status.power = BDM_TARGET_VDD_EXT;
#endif // CAP_VDDSENSE

   if ((cable_status.power == BDM_TARGET_VDD_NONE) ||
       (cable_status.power == BDM_TARGET_VDD_ERR)) {
      return BDM_RC_VDD_NOT_PRESENT;
   }
   return BDM_RC_OK;
}

#if (HW_CAPABILITY&CAP_VDDCONTROL)

/**
 *  Turns on Target Vdd if enabled.
 *
 * @return BDM_RC_OK                => Target Vdd confirmed on target \n
 * @return BDM_RC_VDD_NOT_PRESENT   => Target Vdd not present
 */
uint8_t setTargetVdd( void ) {
uint8_t rc = BDM_RC_OK;

#if (HW_CAPABILITY&CAP_VDDSENSE)
   DISABLE_VDD_SENSE_INT();
#endif
   
   switch (bdm_option.targetVdd) {
   case BDM_TARGET_VDD_OFF :
	   VDD_OFF();
	   // Check for externally supplied target Vdd (> 2 V)
	   WAIT_US(VDD_RISE_TIMEus); // Wait for Vdd to rise & stabilise
	   if (targetVddMeasure()<VDD_2v)
		   rc = BDM_RC_VDD_NOT_PRESENT;
	   break;
   case BDM_TARGET_VDD_3V3 :
	   VDD3_ON();
	   // Wait for Vdd to rise to 90% of 3V
	   WAIT_WITH_TIMEOUT_MS( 100 /* ms */, (targetVddMeasure()>VDD_3v3));
	   WAIT_US(VDD_RISE_TIMEus); // Wait for Vdd to rise & stabilise
	   if (targetVddMeasure()<VDD_3v3) {
		   VDD_OFF(); // In case of Vdd overload
		   rc = BDM_RC_VDD_NOT_PRESENT;
	   }
	   break;
   case BDM_TARGET_VDD_5V  :
	   VDD5_ON();
	   // Wait for Vdd to rise to 90% of 5V
	   WAIT_WITH_TIMEOUT_MS( 100 /* ms */, (targetVddMeasure()>VDD_5v));
	   WAIT_US(VDD_RISE_TIMEus); // Wait for Vdd to rise & stabilise
	   if (targetVddMeasure()<VDD_5v) {
		   VDD_OFF(); // In case of Vdd overload
		   rc = BDM_RC_VDD_NOT_PRESENT;
	   }
	   break;
   }
#if (HW_CAPABILITY&CAP_VDDSENSE)
   CLEAR_VDD_SENSE_FLAG(); // Clear Vdd Change Event
   ENABLE_VDD_SENSE_INT();
#endif

   (void)checkTargetVdd(); // Update Target Vdd LED & status
   return (rc);
}

#endif // CAP_VDDCONTROL

/**
 *   Cycle power ON to target
 *
 *  @param mode
 *     - \ref RESET_SPECIAL => Power on in special mode,
 *     - \ref RESET_NORMAL  => Power on in normal mode
 *
 *   BKGD/BKPT is held low when power is re-applied to start
 *   target with BDM active if RESET_SPECIAL
 *
 *   @return
 *    \ref BDM_RC_OK                	=> Target Vdd confirmed on target \n
 *    \ref BDM_RC_VDD_WRONG_MODE    	=> Target Vdd not controlled by BDM interface \n
 *    \ref BDM_RC_VDD_NOT_PRESENT   	=> Target Vdd failed to rise 		\n
 *    \ref BDM_RC_RESET_TIMEOUT_RISE    => RESET signal failed to rise 		\n
 *    \ref BDM_RC_BKGD_TIMEOUT      	=> BKGD signal failed to rise
 */
USBDM_ErrorCode bdm_cycleTargetVddOn(uint8_t mode) {
   USBDM_ErrorCode rc = BDM_RC_OK;

   mode &= RESET_MODE_MASK;

#if (HW_CAPABILITY&CAP_VDDCONTROL)

   switch(cable_status.target_type) {
#if (HW_CAPABILITY&CAP_CFVx_HW)
   case T_CFVx:
      bdmcf_interfaceIdle();  // Make sure BDM interface is idle
      if (mode == RESET_SPECIAL)
         BKPT_LOW();
      break;
#endif
#if (HW_CAPABILITY&CAP_BDM)     
   case T_HC12:
   case T_HCS08:
   case T_RS08:
   case T_CFV1:
      bdmHCS_interfaceIdle();  // Make sure BDM interface is idle
      if (mode == RESET_SPECIAL) {
         BDM_LOW();  // BKGD pin=L
      }
      break;
#endif      
#if (HW_CAPABILITY&CAP_JTAG_HW)     
   case T_JTAG:
   case T_MC56F80xx:
   case T_ARM_JTAG:
      jtag_interfaceIdle();  // Make sure BDM interface is idle
#endif      
      break;
   default:
      swd_interfaceIdle();
      break;
   }
#if (DEBUG&CYCLE_DEBUG)
   DEBUG_PIN     = 0;
   DEBUG_PIN     = 1;
   DEBUG_PIN     = 0;
   DEBUG_PIN     = 1;
#endif //  (DEBUG&CYCLE_DEBUG)

   // Power on with TargetVdd monitoring off
   rc = bdm_setTargetVdd();
   if (rc != BDM_RC_OK) // No target Vdd
      goto cleanUp;

#if (DEBUG&CYCLE_DEBUG)
   DEBUG_PIN     = 1;
   DEBUG_PIN     = 0;
#endif //  (DEBUG&CYCLE_DEBUG)
#if (HW_CAPABILITY&CAP_RST_IN)
   // RESET rise may be delayed by target POR
   if (bdm_option.useResetSignal) {
      WAIT_WITH_TIMEOUT_S( 2 /* s */, resetIsHigh() );
   }
#endif // (HW_CAPABILITY&CAP_RST_IN)
#if (DEBUG&CYCLE_DEBUG)
   DEBUG_PIN   = 0;
   DEBUG_PIN   = 1;
#endif // (DEBUG&CYCLE_DEBUG)

   // Let signals settle & CPU to finish reset (with BKGD held low)
   WAIT_US(BKGD_WAITus);

#if (HW_CAPABILITY&CAP_RST_IN)
   if (bdm_option.useResetSignal && resetIsLow()) {
      // RESET didn't rise
      rc = BDM_RC_RESET_TIMEOUT_RISE;
      goto cleanUp;
      }
#endif // (HW_CAPABILITY&CAP_RST_IN)

#if (DEBUG&CYCLE_DEBUG)
   DEBUG_PIN     = 1;
   DEBUG_PIN     = 0;
#endif // (DEBUG&CYCLE_DEBUG)

#if (HW_CAPABILITY&CAP_CFVx_HW)
   if  (cable_status.target_type == T_CFVx)
      bdmcf_interfaceIdle();  // Release BKPT etc
   else
#endif
#if (HW_CAPABILITY&CAP_BDM)     
      bdmHCS_interfaceIdle();  // Release BKGD
#endif
   // Let processor start up
   WAIT_MS(RESET_RECOVERYms);

#if 0
// Removed - some targets may be holding BKGD low (e.g. used as port pin)
// This situation is handled elsewhere (requires power cycle)
   if (BDM_IN==0) { // BKGD didn't rise!
      rc = BDM_RC_BKGD_TIMEOUT;
      goto cleanUp;
      }
#endif // 0

   cable_status.reset  = RESET_DETECTED; // Cycling the power should have reset it!

cleanUp:
#if (HW_CAPABILITY&CAP_CFVx_HW)
   if  (cable_status.target_type == T_CFVx)
      bdmcf_interfaceIdle();  // Release BKPT etc
   else
#endif
#if (HW_CAPABILITY&CAP_BDM)     
      bdmHCS_interfaceIdle();  // Release BKGD
#endif
   
   WAIT_MS( 250 /* ms */);

//   EnableInterrupts;
#endif // CAP_VDDCONTROL

   (void)checkTargetVdd(); // Update Target Vdd LED & power status

   return(rc);
}

/**
 *   Cycle power OFF to target
 *
 *   @return
 *    \ref BDM_RC_OK                => No error  \n
 *    \ref BDM_RC_VDD_WRONG_MODE    => Target Vdd not controlled by BDM interface \n
 *    \ref BDM_RC_VDD_NOT_REMOVED   => Target Vdd failed to fall \n
 */
USBDM_ErrorCode cycleTargetVddOff(void) {
   USBDM_ErrorCode rc = BDM_RC_OK;

#if (HW_CAPABILITY&CAP_VDDCONTROL)

   (void)checkTargetVdd();

   if (bdm_option.targetVdd == BDM_TARGET_VDD_OFF)
      return BDM_RC_VDD_WRONG_MODE;

#if (HW_CAPABILITY&CAP_CFVx_HW)
   if  (cable_status.target_type == T_CFVx)
      bdmcf_interfaceIdle();  // Make sure BDM interface is idle
   else
#endif 
   {
#if (HW_CAPABILITY&CAP_BDM)    	  
	  bdmHCS_interfaceIdle();  // Make sure BDM interface is idle
#endif
   }
#if (DEBUG&CYCLE_DEBUG)
   DEBUG_PIN     = 0;
   DEBUG_PIN     = 1;
#endif

   // Power off & wait for Vdd to fall to ~5%
   VDD_OFF();
   WAIT_WITH_TIMEOUT_S( 5 /* s */, (targetVddMeasure()<10) );

#if (DEBUG&CYCLE_DEBUG)
   DEBUG_PIN   = 1;
   DEBUG_PIN   = 0;
#endif

   if (targetVddMeasure()>=15) // Vdd didn't turn off!
      rc = BDM_RC_VDD_NOT_REMOVED;

#if (DEBUG&CYCLE_DEBUG)
   DEBUG_PIN     = 0;
   DEBUG_PIN     = 1;
#endif

   (void)checkTargetVdd(); // Update Target Vdd LED

   // Wait a while with power off
   WAIT_US(RESET_SETTLEms);

   // Clear Vdd monitoring interrupt
#if (HW_CAPABILITY&CAP_VDDSENSE)
   CLEAR_VDD_SENSE_FLAG();  // Clear Vdd monitoring flag
#endif
   (void)checkTargetVdd();    // Update Target Vdd LED

#endif // CAP_VDDCONTROL

   return(rc);
}

/**
 *  Cycle power to target
 *
 *  @param mode
 *     - \ref RESET_SPECIAL => Power on in special mode,
 *     - \ref RESET_NORMAL  => Power on in normal mode
 *
 *   BKGD/BKPT is held low when power is re-applied to start
 *   target with BDM active if RESET_SPECIAL
 *
 *   @return
 *    \ref BDM_RC_OK                 => No error \n
 *    \ref BDM_RC_VDD_WRONG_MODE     => Target Vdd not controlled by BDM interface \n
 *    \ref BDM_RC_VDD_NOT_REMOVED    => Target Vdd failed to fall \n
 *    \ref BDM_RC_VDD_NOT_PRESENT    => Target Vdd failed to rise \n
 *    \ref BDM_RC_RESET_TIMEOUT_RISE => RESET signal failed to rise \n
 */
USBDM_ErrorCode cycleTargetVdd(uint8_t mode) {
   USBDM_ErrorCode rc;

   // TODO This may take a while
   //setBDMBusy();

   rc = cycleTargetVddOff();
   if (rc != BDM_RC_OK) {
      return rc;
   }
   USBDM::waitMS(1000);
   rc = bdm_cycleTargetVddOn(mode);
   return rc;
}

 /**
  *   Measures Target Vdd
  *
  *   @return 8-bit value representing the Target Vdd, N ~ (N/255) * 5V \n
  */
uint16_t targetVddMeasure(void) {
#if ((HW_CAPABILITY&CAP_VDDSENSE) == 0)
   // No Target Vdd measurement - Assume external Vdd supplied
   return 255;
#else
   TVddLed::write(TargetVdd::vddOK());
   return TargetVdd::readRawVoltage();
#endif
}

//=========================================================================
// Common BDM routines
//
//=========================================================================

/**   Sets the BDM interface to a suspended state
 *
 *   - All signals idle \n
 *   - All voltages off.
 */
void suspend(void){
#if (HW_CAPABILITY&CAP_FLASH)
   (void)bdmSetVpp(BDM_TARGET_VPP_OFF);
#endif
#if (HW_CAPABILITY&CAP_CFVx_HW)
   bdmCF_suspend();
#endif
#if (HW_CAPABILITY&CAP_BDM)
   Bdm::disable();
#endif   
#if (HW_CAPABILITY&CAP_SWD_HW)
   Swd::disable();
#endif
}

/**
 *
 *  Turns off the BDM interface
 *
 *   Depending upon settings, may leave target power on.
 */
void interfaceOff( void ) {
   Swd::disable();
   Bdm::disable();

   if (!bdm_option.leaveTargetPowered) {
      // TODO VDD_OFF
   }
}

/**
 * Clear Cable status
 */
USBDM_ErrorCode clearStatus(void) {
   memset(&cable_status, 0, sizeof(cable_status));
   cable_status.target_type = T_OFF;
   return BDM_RC_OK;
}

/**
 * Initialises BDM module for the given target type
 *
 * @param target = Target processor (see \ref TargetType_t)
 *
 * @return E_NO_ERROR on success
 */
USBDM_ErrorCode setTarget(TargetType_t target) {
   USBDM_ErrorCode rc = BDM_RC_OK;

   if (target == T_OFF) {
      interfaceOff();
   }   
   clearStatus();

   // Initially assume mode is valid
   cable_status.target_type = target;

   switch (target) {
#if TARGET_CAPABILITY & CAP_S12Z
      case T_HCS12Z  :
#endif
#if (TARGET_CAPABILITY & (CAP_HCS12|CAP_S12Z))   
      case T_HC12:
         bdm_option.useResetSignal = 1; // Must use RESET signal on HC12
         bdmHCS_init();
         break;
#endif         
#if (TARGET_CAPABILITY & CAP_RS08)
      case T_RS08:
#endif
#if (TARGET_CAPABILITY & CAP_HCS08)
      case T_HCS08:
#endif    	  
#if (TARGET_CAPABILITY & CAP_CFV1)
      case T_CFV1:
#endif    	  
#if (TARGET_CAPABILITY & (CAP_RS08|CAP_HCS08|CAP_CFV1))
         Bdm::initialise();
         break;
#endif    	  
#if (TARGET_CAPABILITY&CAP_CFVx)
      case T_CFVx:
         bdm_option.useResetSignal = 1; // Must use RESET signal on CFVx
         bdmcf_init();                  // Initialise the BDM interface
//         (void)bdmcf_resync();          // Synchronise with the target (ignore error?)
         break;
#endif
#if (TARGET_CAPABILITY&CAP_JTAG)
      case T_JTAG:
#endif
#if (TARGET_CAPABILITY&CAP_DSC)
      case T_MC56F80xx:
#endif
#if (TARGET_CAPABILITY&(CAP_JTAG|CAP_DSC))
         bdm_option.useResetSignal = 1; // Must use RESET signal on JTAG etc
         jtag_init();                   // Initialise JTAG
         break;
#endif
#if (TARGET_CAPABILITY&CAP_ARM_JTAG)
      case T_ARM_JTAG:
          jtag_init();                   // Initialise JTAG
          break;
#endif
#if (TARGET_CAPABILITY&CAP_ARM_SWD)
      case T_ARM_SWD:
          Swd::initialise();                   // Initialise JTAG
          break;
#endif
      case T_OFF:
    	  break;
    	  
      default:
         // Turn off the interface
         interfaceOff();
         return BDM_RC_UNKNOWN_TARGET;
   }
   return rc;
}
