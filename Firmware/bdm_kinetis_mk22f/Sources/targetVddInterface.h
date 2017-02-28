/*
 * TargetVddInterface.h
 *
 *  Created on: 21Dec.,2016
 *      Author: podonoghue
 */

#ifndef PROJECT_HEADERS_TARGETVDDINTERFACE_H_
#define PROJECT_HEADERS_TARGETVDDINTERFACE_H_

#include "math.h"
#include "hardware.h"
#include "cmp.h"

/**
 * Low-level interface to Vdd control and sensing
 */
class TargetVddInterface {

private:
   /**
    * Represents the 2:1 voltage divider on input
    */
   static constexpr int externalDivider = 2;

   /**
    * Conversion factor for ADC reading to input voltage\n
    * 3.3V range, 8 bit conversion, voltage divider on input
    * V = ADCValue * scaleFactor
    */
   static constexpr float scaleFactor = (externalDivider*3.3)/((1<<8)-1);

   /**
    * Minimum working input voltage as an ADC reading \n
    * 1.5 V as ADC reading
    */
   static constexpr int onThreshold = (int)(1.5/scaleFactor);

   /**
    * Minimum POR input voltage as an ADC reading. \n
    * This is the voltage needed to ensure a power-on-reset of the target. \n
    * 0.8 V as ADC reading
    */
   static constexpr int powerOnResetThresholdAdc = (int)(0.8/scaleFactor);

   /**
    * Minimum POR input voltage as an ADC reading. \n
    * This is the voltage needed to ensure a power-on-reset of the target. \n
    * 0.8 V as ADC reading
    */
   static constexpr int powerOnResetThresholdDac = (int)(0.8*64/3.3);

   /**
    * GPIO for Target Vdd enable pin
    */
   using Control = USBDM::GpioD<1>;

   /**
    * GPIO for Target Vdd LED
    */
   using Led = USBDM::GpioB<3>;

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
   using VddPowerSwitchMonitor = USBDM::GpioD<0>;

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
    * Monitors Vbdm level (comparator)
    */
   static void vddMonitorCallback(int status) {
      if ((status & CMP_SCR_CFF_MASK) != 0) {
         // In case Vdd overload
         vddOff();
      }
      // Notify callback
      fCallback();
   }

   /**
    * Monitors Vbdm power switch (IRQ pin)
    */
   static void powerMonitorCallback(uint32_t status) {
      if ((VddPowerSwitchMonitor::MASK & status) != 0) {
         fCallback();
      }
   }

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

      Led::setOutput();
      ledOff();

      VddMeasure::enable();
      VddMeasure::setResolution(USBDM::resolution_8bit_se);

      VddMonitor::enable();
      VddMonitor::selectInputs(1, 7);
      VddMonitor::setDacLevel(powerOnResetThresholdDac);
      VddMonitor::setCallback(vddMonitorCallback);
      VddMonitor::enableNvicInterrupts(true);
      fCallback = nullCallback;

      VddPowerSwitchMonitor::setInput();
      VddPowerSwitchMonitor::setPullDevice(USBDM::PullUp);
      VddPowerSwitchMonitor::setIrq(USBDM::PinIrqFalling);
      VddPowerSwitchMonitor::setCallback(powerMonitorCallback);
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

   /** Turn on TVdd LED */
   static void ledOn() {
      Led::high();
   }

   /** Turn off TVdd LED */
   static void ledOff() {
      Led::low();
   }

   /**
    * Read target Vdd
    *
    * @return Target Vdd as an integer in the range 0-255 => 0-5V
    */
   static int readRawVoltage() {
      return round(VddMeasure::readAnalogue()*(externalDivider*3.3/5));
   }

   /**
    * Read target Vdd
    *
    * @return Target Vdd in volts as a float
    */
   static float readVoltage() {
      VddMeasure::enable();
      VddMeasure::setResolution(USBDM::resolution_8bit_se);
      return VddMeasure::readAnalogue()*scaleFactor;
   }

   /**
    * Check if target Vdd is present \n
    * Also updates Target Vdd LED
    */
   static bool isVddOK() {
      if (VddMeasure::readAnalogue()>onThreshold) {
         ledOn();
         return true;
      }
      else {
         ledOff();
         return false;
      }
   }

   /**
    * Check if target Vdd has reached POR level\n
    * Also updates Target Vdd LED
    */
   static bool isVddLow() {
      if (VddMeasure::readAnalogue()<powerOnResetThresholdAdc) {
         ledOff();
         return true;
      }
      else {
         ledOn();
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
      VddMonitor::enableFallingEdgeInterrupts(enable);
   }
};

#endif /* PROJECT_HEADERS_TARGETVDDINTERFACE_H_ */
