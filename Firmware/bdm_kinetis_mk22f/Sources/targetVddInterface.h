/*
 * TargetVddInterface.h
 *
 *  Created on: 21Dec.,2016
 *      Author: podonoghue
 */

#ifndef PROJECT_HEADERS_TARGETVDDINTERFACE_H_
#define PROJECT_HEADERS_TARGETVDDINTERFACE_H_

#include <math.h>
#include "hardware.h"
#include "cmp.h"
#include "console.h"

/**
 * Low-level interface to Target Vdd (Vbdm) control and sensing
 */
class TargetVddInterface {

private:
   /**
    * Represents the 2:1 voltage divider on input
    */
   static constexpr int externalDivider = 2;

   /**
    * Represents the 2:1 voltage divider on input
    */
   static constexpr float vdd = 3.3f;

   /**
    * Minimum working input voltage as an ADC reading \n
    * 1.5 V as ADC reading i.e. 0-255
    */
   static constexpr int onThresholdAdc = (int)((1.5/(externalDivider*vdd))*((1<<8)-1));

   /**
    * Minimum working voltage as an DAC value. \n
    * 1.5V as DAC value i.e. 0-63
    */
   static constexpr int onThresholdDac = (int)((1.5/(externalDivider*vdd))*((1<<6)-1));

   /**
    * Minimum working voltage as an DAC value. \n
    * 0.8V as DAC value i.e. 0-63
    */
   static constexpr int powerOnResetThresholdDac = (int)((0.8/(externalDivider*vdd))*((1<<6)-1));

   /**
    * Minimum POR input voltage as an ADC reading. \n
    * This is the voltage needed to ensure a power-on-reset of the target. \n
    * 0.8 V as ADC reading
    */
   static constexpr int powerOnResetThresholdAdc = (int)((0.8/(externalDivider*vdd))*((1<<8)-1));

   /**
    * GPIO for Target Vdd enable pin
    */
   using Control = USBDM::GpioD<1>;

   /**
    * GPIO for Target Vdd LED
    */
   using Led = USBDM::GpioB<3, USBDM::ActiveHigh>;

   /**
    * ADC channel for Target Vdd measurement
    */
   using VddMeasure = USBDM::Adc0Channel<12>;

   /**
    * Comparator to monitor Vdd level
    */
   using VddMonitor = USBDM::Cmp0;

   /**
    * GPIO used to monitor power switch error indicator
    */
   using VddPowerFaultMonitor = USBDM::GpioD<0>;

   /**
    * Callback for Vdd changes
    */
   static void (*fCallback)();

   /**
    * Dummy routine used if callback is not set
    */
   static void nullCallback() {
#ifdef DEBUG_BUILD
      __BKPT();
#endif
   }

public:
   /**
    * Monitors Target Vdd (Vbdm) level via comparator
    */
   static void vddMonitorCallback(USBDM::CmpEvent status) {
      if ((status & CMP_SCR_CFF_MASK) != 0) {
         // In case Vdd overload
         vddOff();
      }
      // Notify callback
      fCallback();
      Led::write(VddMonitor::cmp->SCR&CMP_SCR_COUT_MASK);
//      isVddOK();
   }

   /**
    * Monitors Target Vdd (Vbdm) power switch overload (IRQ pin)
    */
   static void powerFaultCallback(uint32_t status) {
      if ((VddPowerFaultMonitor::MASK & status) != 0) {
         // In case Vdd overload
         vddOff();

         // Notify callback
         fCallback();
      }
      isVddLow();
   }

   /**
    * Set callback to execute on Target Vdd (Vbdm) changes
    *
    * @param[in] callback Callback to execute
    */
   static void setCallback(void (*callback)()) {
      if (callback == nullptr) {
         callback = nullCallback;
      }
      fCallback = callback;
   }

   /**
    * Initialise Vdd control and measurement interface
    */
   static void initialise() {
      Control::setOutput();
      vddOff();

      Led::setOutput(
            USBDM::PinDriveStrength_High,
            USBDM::PinDriveMode_PushPull,
            USBDM::PinSlewRate_Slow);

      VddMeasure::enable();
      VddMeasure::setResolution(USBDM::AdcResolution_8bit_se);
      VddMeasure::calibrate();

      fCallback = nullCallback;
      VddMonitor::setCallback(vddMonitorCallback);
      VddMonitor::configure(
            USBDM::CmpPower_HighSpeed,
            USBDM::CmpHysteresis_2,
            USBDM::CmpPolarity_Noninverted);
      VddMonitor::configureDac(
            powerOnResetThresholdDac,
            USBDM::CmpDacSource_Vdd);
      VddMonitor::selectInputs(1, 7);
      VddMonitor::enableInterrupts(USBDM::CmpInterrupt_Both);
      VddMonitor::enableNvicInterrupts(true);

      VddPowerFaultMonitor::setCallback(powerFaultCallback);
      VddPowerFaultMonitor::setInput(
            USBDM::PinPull_Up,
            USBDM::PinIrq_Falling,
            USBDM::PinFilter_Passive);
   }

   /**
    * Turn on Target Vdd
    */
   static void vddOn() {
      Control::high();
   }

   /**
    * Turn on Target Vdd @ 3.3V\n
    * Dummy routine as voltage level controlled by physical link
    */
   static void vdd3V3On() {
      vddOn();
   }

   /**
    * Turn on Target Vdd @ 5V\n
    * Dummy routine as voltage level controlled by physical link
    */
   static void vdd5VOn() {
      vddOn();
   }

   /**
    * Turn off Target Vdd
    */
   static void vddOff() {
      Control::low();
   }

   /**
    * Read target Vdd
    *
    * @return Target Vdd as an integer in the range 0-255 => 0-5V
    */
   static int readRawVoltage() {
      return round(VddMeasure::readAnalogue()*externalDivider*5/vdd);
   }

   /**
    * Read target Vdd
    *
    * @return Target Vdd in volts as a float
    */
   static float readVoltage() {
      return VddMeasure::readAnalogue()*vdd*externalDivider/((1<<8)-1);
   }

   /**
    * Check if target Vdd is present \n
    * Also updates Target Vdd LED
    */
   static bool isVddOK() {
      int value = VddMeasure::readAnalogue();
      if (value>onThresholdAdc) {
         Led::on();
         return true;
      }
      else {
         Led::off();
         return false;
      }
   }

   /**
    * Check if target Vdd has reached POR level\n
    * Also updates Target Vdd LED
    */
   static bool isVddLow() {
      if (VddMeasure::readAnalogue()<powerOnResetThresholdAdc) {
         Led::off();
         return true;
      }
      else {
         Led::on();
         return false;
      }
   }

   /**
    * Clear VDD change flag
    */
   static void clearVddChangeFlag() {
      VddMonitor::clearInterruptFlags();
   }

   /**
    * Enable/disable VDD change monitoring
    */
   static void enableVddChangeSense(bool enable) {
      VddMonitor::enableInterrupts(enable?USBDM::CmpInterrupt_Both:USBDM::CmpInterrupt_None);
   }
};

#endif /* PROJECT_HEADERS_TARGETVDDINTERFACE_H_ */
