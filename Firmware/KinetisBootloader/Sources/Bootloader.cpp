/*
 ============================================================================
 * @file     Bootloader.cpp
 *
 * Kinetis bootloader
 *
 *  Created on: 7 Dec 2019
 *      Author: podonoghue
 ============================================================================
 */
#include "hardware.h"
#include "usb.h"
#include "UsbCommandMessage.h"
#include "flash.h"
#include "crc.h"
#include "BootInformation.h"
#include "gpio.h"

using namespace USBDM;

/**********************************
 *  TARGET specific code
 **********************************/

/** ICP button - checked during boot */
using IcpButton = TPa1;

/** Debug pin */
//using DebugPin  = TPa2;

/// What hardware this boot loader is built for
static constexpr HardwareType BOOT_HARDWARE_VERSION  = HW_USBDM_MK22F;

/**********************************/

/// What the version of the bootloader is
static constexpr uint32_t     BOOT_SOFTWARE_VERSION  = BOOTLOADER_V4;

struct FlashImageData {
   uint32_t flash1Start;
   uint32_t flash1Size;
   uint32_t flash2Start;
   uint32_t flash2Size;

   constexpr FlashImageData(const FlashImageData &other) :
      flash1Start(other.flash1Start ),
      flash1Size (other.flash1Size  ),
      flash2Start(other.flash2Start ),
      flash2Size (other.flash2Size  )
      {
   }
   constexpr FlashImageData(
         uint32_t flash1Start,
         uint32_t flash1Size,
         uint32_t flash2Start,
         uint32_t flash2Size
) :
      flash1Start(flash1Start ),
      flash1Size (flash1Size  ),
      flash2Start(flash2Start ),
      flash2Size (flash2Size  )
      {
   }
};

constexpr const FlashImageData getFlashImageData(HardwareType hardwareType) {

constexpr FlashImageData flashImagedata[] = {
      /*                                                 Start 1     Size 1          Start2      Size 2 */
      /* 0 - Unknown",             - MK20DX128VLF5 */  { 0x004000,   0x0,            0x10000000, 0x0        },
      /* 1 - Digital Lab Board V2" - MK20DX128VLF5 */  { 0x004000,   0x20000-0x4000, 0x10000000, 0x0        },
      /* 2 - Digital Lab Board V3" - MK20DX128VLF5 */  { 0x004000,   0x20000-0x4000, 0x10000000, 0x0        },
      /* 3 - Digital Lab Board V4" - MK20DX128VLF5 */  { 0x004000,   0x20000-0x4000, 0x10000000, 0x0        },
      /* 4 - Soldering Station V3" - MK20DX128VLF5 */  { 0x004000,   0x20000-0x4000, 0x10000000, 0x0        },
      /* 5 - Digital Lab Board V4a - MK20DX32VLF5  */  { 0x004000,   0x08000-0x4000, 0x10000000, 0x00008000 }, // Uses Data Flash
      /* 6 - Soldering Station V4" - MK20DX256VLH7 */  { 0x004000,   0x40000-0x4000, 0x10000000, 0x0        }, // Use FlexRAM
      /* 7 - USBDM Interface MK22F - MK22FN512M12  */  { 0x004000,   0x80000-0x4000, 0x10000000, 0x0        },
};

   return flashImagedata[hardwareType];
}

static const FlashImageData flashImageData(getFlashImageData(BOOT_HARDWARE_VERSION));

   /**
    * Checks if the address is within bootloader flash range
    *
    * @param address Address to check
    *
    * @return True if in range
    */
   bool isValidFlashAddress(uint32_t address) {
      return
         ((address>=flashImageData.flash1Start) && (address<(flashImageData.flash1Start + flashImageData.flash1Size))) ||
         ((address>=flashImageData.flash2Start) && (address<(flashImageData.flash2Start + flashImageData.flash2Size)));
   }

/**
 * Report command to console
 *
 * @param message Command message to describe
 */
static void writeCommandMessage(UsbCommandMessage &message) {
   console.WRITE(getCommandName(message.command));
   if (message.byteLength>0) {
      console.
      WRITE(" [0x").WRITE(message.startAddress, Radix_16).
      WRITE("..0x").WRITE(message.startAddress+message.byteLength-1, Radix_16).
      WRITE("]");
      //   console.writeArray(message.data, message.byteLength, message.startAddress);
   }
   console.WRITELN();
}

/**
 * Get Boot information from fixed Flash location in loaded image
 *
 * @return Non-nullptr => Pointer to read-only structure in Flash
 * @return nullptr     => Loaded flash image is not valid
 */
static BootInformation *getBootInformation() {
      BootInformation *bootInfo = ((BootInformation *)(flashImageData.flash1Start+0x3C0));
      if (!bootInfo->isValid()) {
         return nullptr;
      }
      return bootInfo;
}

/**
 * Checks if magic number referenced from boot information is valid.
 *
 * @return true if valid
 */
static bool isMagicNumberValid() {

   auto bootInfo = getBootInformation();

   return (bootInfo != nullptr) &&
          (bootInfo->magicNumber != nullptr) &&
          (*(bootInfo->magicNumber) == ~bootInfo->key);
}

/**
 * Calculate Flash CRC
 *
 * @return CRC done over both flash ranges (excluding last 4 bytes of range1)
 */
uint32_t calcFlashCrc() {
   using Crc = Crc0;

   // Set up CRC32
   Crc::configure_Crc32();

   // Do first range
   Crc::calculateCrc((uint32_t *)flashImageData.flash2Start, flashImageData.flash2Size);

   // Do second range
   Crc::calculateCrc((uint32_t *)flashImageData.flash1Start, flashImageData.flash1Size-4);

   return Crc::getCalculatedCrc();
}

/**
 * Check if flash regions have valid checksum
 *
 * @return true  => CRC  is valid i.e. Flash image is verified
 * @return false => CRC  is invalid i.e. Flash image is not verified
 */
bool isFlashValid() {

   uint32_t calculatedCrc = calcFlashCrc();
   uint32_t expectedCrc   = *(uint32_t *)(flashImageData.flash1Start+flashImageData.flash1Size-4);

   // Should agree with CRC at end of flash region 1
   return calculatedCrc == expectedCrc;
}

/**
 * Execute the flash image from reset vector.
 * Assumed at at start of flash1 image.
 */
void callFlashImage() {

   // Locate the reset vector in image */
   void (*f)() = (void (*)())(((uint32_t *)flashImageData.flash1Start)[1]);

   // Call it
   f();
}

/**
 * Program a region of the flash image
 *
 * @param command Command containing information about region to program
 *
 * @return true  => Successfully programmed
 * @return false => Programming failed
 */
bool programFlash(UsbCommandMessage command) {
   if (!isValidFlashAddress(command.startAddress)) {
      return false;
   }
   if (!isValidFlashAddress(command.startAddress+command.byteLength-1)) {
      return false;
   }
   FlashDriverError_t rc;
   rc = Flash::programRange(command.data, (uint8_t *)(command.startAddress), command.byteLength);
   if (rc != FLASH_ERR_OK) {
      return false;
   }
   return true;
}

#ifndef SCB_AIRCR_VECTKEY
#define SCB_AIRCR_VECTKEY(x) (((x)<<SCB_AIRCR_VECTKEY_Pos)&SCB_AIRCR_VECTKEY_Msk)
#endif

/**
 * Reset system
 */
static void resetSystem() {

   /* Request system reset */
   SCB->AIRCR = SCB_AIRCR_VECTKEY(0x5FA) | SCB_AIRCR_SYSRESETREQ_Msk;

   /* Wait until reset */
   for(;;) {
      __asm__("nop");
   }
}

/**
 * Boot into user program mode if:
 *  - Flash image is valid and
 *  - Magic number is invalid ? and
 *  - ICP button not pressed
 *
 *  @note Does not return if ICP mode is not detected
 *  @note Called from system code before peripherals are initialised
 */
void checkICP() {

   // Enable pull-up
   IcpButton::setInput(PinPull_Up);

   //  Wait a while
   for(unsigned count=0; count++<1000; count++) {
      __asm__("nop");
   }

   bool icpButtonReleased = IcpButton::isHigh();
   bool flashValid        = isFlashValid();
   bool magicNumberValid  = isMagicNumberValid();

   if (!magicNumberValid && icpButtonReleased && flashValid) {
      callFlashImage();
   }
}

/** Buffer for USB command */
static UsbCommandMessage command;

enum UsbState {UsbStartUp, UsbIdle, UsbWaiting};

void pollUsb() {

   //   console.WRITE("Flash image is ").WRITELN(isFlashValid()?"valid":"invalid");
   //   console.WRITELN("Starting USB boot-loader");

   static UsbState usbState = UsbStartUp;

   // Call-back to record USB user events
   static auto cb = [](const Usb0::UserEvent) {
      // Restart USB transfers on reset etc.
      usbState = UsbIdle;
      return E_NO_ERROR;
   };

   if (usbState == UsbStartUp) {
      // Start USB
      console.WRITELN("UsbStartUp");
      UsbImplementation::initialise();
      Usb0::setUserCallback(cb);
      checkError();
      usbState = UsbIdle;
      return;
   }
   // Check for USB connection
   if (!UsbImplementation::isConfigured()) {
#ifdef DEBUG_BUILD
      static unsigned count = 0;
      if (count++==1'000'000) {
         console.WRITELN("Not configured...");
         count = 0;
      }

#endif
      // No connection
      return;
   }

   uint16_t size;

   if (usbState != UsbWaiting) {
      // UsbIdle
      // Set up to receive a message
      Usb0::startReceiveBulkData(sizeof(command), (uint8_t *)&command);
      usbState = UsbWaiting;
      return;
   }

   // UsbWaiting

   // Check if we have received a message
   ErrorCode rc = Usb0::pollReceiveBulkData(size);
   if (size == 0) {
      // No message - USB still ready
      return;
   }

   // *****************************
   // We have a message to process
   // *****************************

   // Response to send
   ResponseMessage response;

   // Default setup for OK using small response
   response.status     = UsbCommandStatus_OK;
   response.byteLength = 0;
   unsigned responseSize = sizeof(ResponseStatus);

   do {
      if (size < (int)sizeof(command.command)) {
         // Incomplete command?
         console.WRITELN("Incomplete command");
         response.status = UsbCommandStatus_Failed;
         continue;
      }
      // Report message on console
      writeCommandMessage(command);

      switch(command.command) {
         default:
            console.WRITE("Unexpected command: ").WRITELN(command.command);
            response.status = UsbCommandStatus_Failed;
            continue;

         case UsbCommand_Nop:
            continue;

         case UsbCommand_Identify:
         {
#if defined(DEBUG_BUILD) && USE_CONSOLE
            BootInformation *bootInfo = ((BootInformation *)(flashImageData.flash1Start+0x3C0));
            USBDM::console.write("isValid() = ").writeln(bootInfo->isValid());
            USBDM::console.write("key       = 0x").writeln(bootInfo->key, Radix_16);

            uint32_t calculatedCrc = calcFlashCrc();
            uint32_t expectedCrc   = *(uint32_t *)(flashImageData.flash1Start+flashImageData.flash1Size-4);
            console.write("Flash    CRC = 0x").writeln(calculatedCrc, Radix_16);
            console.write("Expected CRC = 0x").writeln(expectedCrc,   Radix_16);
#endif

            response.bootHardwareVersion  = BOOT_HARDWARE_VERSION;
            response.bootSoftwareVersion  = BOOT_SOFTWARE_VERSION;
            response.flash1_start         = flashImageData.flash1Start;
            response.flash1_size          = flashImageData.flash1Size;
            response.flash2_start         = flashImageData.flash2Start;
            response.flash2_size          = flashImageData.flash2Size;

            BootInformation *bootInformation = getBootInformation();
            if (bootInformation != nullptr) {
               response.imageHardwareVersion = bootInformation->hardwareVersion;
               response.imageSoftwareVersion = bootInformation->softwareVersion;
            }
            else {
               response.imageHardwareVersion = 0;
               response.imageSoftwareVersion = 0;
            }
            responseSize = sizeof(ResponseIdentify);
            continue;
         }

         case UsbCommand_EraseFlash:
            if (Flash::eraseRange((uint8_t *)flashImageData.flash1Start, flashImageData.flash1Size) != FLASH_ERR_OK) {
               response.status = UsbCommandStatus_Failed;
               continue;
            }
            if (Flash::eraseRange((uint8_t *)flashImageData.flash2Start, flashImageData.flash2Size) != FLASH_ERR_OK) {
               response.status = UsbCommandStatus_Failed;
               continue;
            }
            continue;

         case UsbCommand_ProgramBlock:
            if (!programFlash(command)) {
               console.WRITELN("Flash programming failed");
               response.status = UsbCommandStatus_Failed;
            }
            continue;

         case UsbCommand_ReadBlock:
            if (command.startAddress < (BOOTLOADER_ORIGIN+BOOTLOADER_SIZE)) {
               // Invalid flash address
               console.WRITELN("Flash address is in bootloader");
               response.status = UsbCommandStatus_Failed;
               continue;
            }
            if (response.byteLength > sizeof(command.data)) {
               // Illegal block size
               console.WRITELN("Read block too large");
               response.status = UsbCommandStatus_Failed;
               continue;
            }
            response.byteLength  = command.byteLength;
            responseSize         = command.byteLength + sizeof(ResponseStatus);
            memcpy(response.data, (uint8_t *)(command.startAddress), command.byteLength);
            continue;

         case UsbCommand_Reset:
            resetSystem();
            continue;
      }
   } while (false);

   // Send response
   rc = Usb0::sendBulkData(responseSize, (uint8_t *)&response, 1000);
   if (rc != 0) {
      console.WRITE("sendBulkData() failed, reason = ").WRITELN(getErrorMessage(rc));
   }
   usbState = UsbIdle;
}

int main() {

#ifdef DEBUG_BUILD
   console.writeln();
   console.writeln("icpButtonReleased = ", IcpButton::isHigh());
   console.writeln("flashValid        = ", isFlashValid());
   console.writeln("magicNumberValid  = ", isMagicNumberValid());
#endif

   // Check for error in start-up
   checkError();

   for(;;) {
      pollUsb();
   }
}
