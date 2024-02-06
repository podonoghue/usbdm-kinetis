/*
 * UsbCommandMessage.h
 *
 *  Created on: 9 Dec 2019
 *      Author: podonoghue
 */

#ifndef SOURCES_USBCOMMANDMESSAGE_H_
#define SOURCES_USBCOMMANDMESSAGE_H_

#include <stdint.h>
#include "BootInformation.h"

// USB messages are packed data in LE (native) format
#pragma pack(push, 1)

/**
 * Commands available
 */
enum UsbCommand : uint32_t {
   UsbCommand_Nop,               ///< No operation
   UsbCommand_Identify,          ///< Identify boot-loader and hardware versions etc
   UsbCommand_EraseFlash,        ///< Erase range of flash image
   UsbCommand_ReadBlock,         ///< Read block from flash
   UsbCommand_ProgramBlock,      ///< Program block to flash
   UsbCommand_Reset,             ///< Reset device
};

/**
 * Result of command
 */
enum UsbCommandStatus : uint32_t {
   UsbCommandStatus_OK,          ///< OK result
   UsbCommandStatus_Failed,      ///< Failed
};

/** Maximum size of data in message e.g. flash data block */
static constexpr unsigned MAX_MESSAGE_DATA = 1024;

/**
 * Get command name as string
 *
 * @param command
 *
 * @return Name as string
 *
 * @note return value is a pointer to a STATIC object - do not free
 */
static inline const char *getCommandName(UsbCommand command) {
   static const char *names[] = {
         "UsbCommand_Nop",
         "UsbCommand_Identify",
         "UsbCommand_EraseFlashRange",
         "UsbCommand_ReadBlock",
         "UsbCommand_ProgramBlock",
         "UsbCommand_Reset",
   };
   const char *name = "Unknown";
   if (command < (sizeof(names)/sizeof(names[0]))) {
      name = names[command];
   }
   return name;
}

/**
 * General USB command message
 */
struct UsbCommandMessage {
   UsbCommand  command;                 ///< Command to execute
   uint32_t    startAddress;            ///< Target memory address
   uint32_t    byteLength;              ///< Size of data
   uint8_t     data[MAX_MESSAGE_DATA];  ///< Data (up to 1 flash block)
};

/**
 * Simple USB command message (no data)
 */
struct SimpleCommandMessage {
   UsbCommand  command;       ///< Command to execute
   uint32_t    startAddress;  ///< Target memory address
   uint32_t    byteLength;    ///< Size of data
};

struct __BootInformation {
   uint32_t bootHardwareVersion;  ///< Hardware version
   uint32_t bootSoftwareVersion;  ///< Boot-loader software version
   uint32_t imageHardwareVersion; ///< Hardware version from loaded image
   uint32_t imageSoftwareVersion; ///< Software version from loaded image
   uint32_t flash1_start;         ///< Flash 1 start address from loaded image
   uint32_t flash1_size;          ///< Flash 1 size address from loaded image
   uint32_t flash2_start;         ///< Flash 2 start address from loaded image
   uint32_t flash2_size;          ///< Flash 2 size address from loaded image
};

/**
 * General USB response message
 */
struct ResponseMessage {
   UsbCommandStatus   status;        ///< Status
   uint32_t           byteLength;    ///< Size of data
   union {
      struct {
         // ResponseIdentify
         uint32_t bootHardwareVersion;  ///< Hardware version
         uint32_t bootSoftwareVersion;  ///< Boot-loader software version
         uint32_t imageHardwareVersion; ///< Hardware version from loaded image
         uint32_t imageSoftwareVersion; ///< Software version from loaded image
         uint32_t flash1_start;         ///< Flash 1 start address from loaded image
         uint32_t flash1_size;          ///< Flash 1 size address from loaded image
         uint32_t flash2_start;         ///< Flash 2 start address from loaded image
         uint32_t flash2_size;          ///< Flash 2 size address from loaded image
      };
      uint8_t data[MAX_MESSAGE_DATA];    ///< Data
   };
};

/**
 * USB status response message
 */
struct ResponseStatus {
   UsbCommandStatus   status;        ///< Status
   uint32_t           byteLength;    ///< Size of data (0)
};

/**
 * USB identify response message
 */
struct ResponseIdentify {
   UsbCommandStatus   status;               ///< Status
   uint32_t           byteLength;           ///< Size of data (not used)
   HardwareType       bootHardwareVersion;  ///< Hardware version
   BootloaderVersion  bootSoftwareVersion;  ///< Boot-loader software version
   HardwareType       imageHardwareVersion; ///< Hardware version from loaded image
   uint32_t           imageSoftwareVersion; ///< Software version from loaded image
   uint32_t           flash1_start;         ///< Flash 1 start address from loaded image
   uint32_t           flash1_size;          ///< Flash 1 size address from loaded image
   uint32_t           flash2_start;         ///< Flash 2 start address from loaded image
   uint32_t           flash2_size;          ///< Flash 2 size address from loaded image
};
#pragma pack(pop)

#endif /* SOURCES_USBCOMMANDMESSAGE_H_ */
