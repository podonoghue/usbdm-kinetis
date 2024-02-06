/*
 * BootInformation.h
 *
 *  Created on: 4 Jan. 2022
 *      Author: peter
 */

#ifndef SOURCES_BOOTINFORMATION_H_
#define SOURCES_BOOTINFORMATION_H_

#include <stdint.h>

#if defined(USBDM_ASSERT)
#include "crc.h"
#else
#include "crc32b.h"
#endif

// USB messages are packed data in LE (native) format
#pragma pack(push, 1)

/// Magic number used to reboot into ICP mode
static constexpr uint32_t MAGIC_NUMBER = 0xA55A1234;

/**
 * Structure of Boot information in Image Flash memory
 */
struct BootInformation {
   static constexpr uint32_t KEY_VALUE = 0xAA551234;

   const uint32_t *magicNumber;        ///< Pointer to magic number location used to force ICP
   const uint32_t softwareVersion;     ///< Version of this software image
   const uint32_t hardwareVersion;     ///< Identifies the hardware this image is intended for
   const uint32_t reserved[3]{0};      ///<
   const uint32_t key;                 ///< Key indicating valid Information

   bool isValid() const {
      return key == KEY_VALUE;
   }

   constexpr BootInformation(
         const uint32_t *magicNumber,
         const uint32_t softwareVersion,
         const uint32_t hardwareVersion ) :
            magicNumber(magicNumber),
            softwareVersion(softwareVersion),
            hardwareVersion(hardwareVersion),
            key(KEY_VALUE) {
   }
};

enum HardwareType : uint32_t {

   // Each unique hardware should define a new number here
   HW_UNKNOWN            = 0,
   HW_LOGIC_BOARD_V2     = 1, // MK20DX128VLF5
   HW_LOGIC_BOARD_V3     = 2, // MK20DX128VLF5
   HW_LOGIC_BOARD_V4     = 3, // MK20DX128VLF5
   HW_SOLDER_STATION_V3  = 4, // MK20DX128VLF5
   HW_LOGIC_BOARD_V4a    = 5, // As for V4 but smaller flash MK20DX32VLF5
   HW_SOLDER_STATION_V4  = 6, // MK20DX256VLH7
   HW_USBDM_MK22F           = 7  // MK22FN512M12
};

#if defined(USBDM_ASSERT)
/**
 * Get hardware type as string
 *
 * @param hardwareVersion Version being queried
 *
 * @return Name as string
 *
 * @note return value is a pointer to a STATIC object - do not free
 */
static inline const char *getHardwareType(HardwareType hardwareVersion) {
   static const char *names[] = {
         "Unknown",
         "LOGIC_BOARD_V2",
         "LOGIC_BOARD_V3",
         "LOGIC_BOARD_V4",
         "SOLDER_STATION_V3",
         "LOGIC_BOARD_V4a",
         "SOLDER_STATION_V4",
         "USBDM_MK"
   };
   const char *name = "Unknown";
   if (hardwareVersion < (sizeof(names)/sizeof(names[0]))) {
      name = names[hardwareVersion];
   }
   return name;
}
#else
/**
 * Get hardware type as string
 *
 * @param hardwareVersion Version being queried
 *
 * @return Name as string
 *
 * @note return value is a pointer to a STATIC object - do not free
 */
static inline const char *getHardwareType(HardwareType hardwareVersion) {
   static const char *names[] = {
         "Unknown",
         "Digital Lab Board V2",
         "Digital Lab Board V3",
         "Digital Lab Board V4",
         "Soldering Station V3",
         "Digital Lab Board V4a",
         "Soldering Station V4",
         "USBDM MK22F"
   };
   const char *name = "Unknown";
   if (hardwareVersion < (sizeof(names)/sizeof(names[0]))) {
      name = names[hardwareVersion];
   }
   return name;
}
#endif

enum BootloaderVersion : uint32_t  {
   BOOTLOADER_UNKNOWN = 0,
   BOOTLOADER_V1      = 1,
   BOOTLOADER_V2      = 2,
   BOOTLOADER_V3      = 3,
   BOOTLOADER_V4      = 4,
};

template<int version>
constexpr const char *getHardwareVersion() {
   if constexpr(version == HW_LOGIC_BOARD_V2) {
      return "Dig-Logic 2";
   }
   if constexpr(version == HW_LOGIC_BOARD_V3) {
      return "Dig-Logic 3";
   }
   if constexpr(version == HW_LOGIC_BOARD_V4) {
      return "Dig-Logic 4";
   }
   if constexpr(version == HW_LOGIC_BOARD_V4a) {
      return "Dig-Logic 4a";
   }
   if constexpr(version == HW_SOLDER_STATION_V3) {
      return "Soldering station V3";
   }
   if constexpr(version == HW_SOLDER_STATION_V4) {
      return "Soldering station V4";
   }
   if constexpr(version == HW_USBDM_MK22F) {
      return "USBDM MK22F";
   }
   return "Unknown";
}

static constexpr uint32_t BOOTLOADER_ORIGIN = 0;
static constexpr uint32_t BOOTLOADER_SIZE   = 0x4000;

#pragma pack(pop)

#endif /* SOURCES_BOOTINFORMATION_H_ */
