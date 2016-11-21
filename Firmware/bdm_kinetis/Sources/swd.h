/** \file
    \brief ARM-SWD low-level interface

   \verbatim

   USBDM
   Copyright (C) 2016  Peter O'Donoghue

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
   +=========================================================================================================
   | 27 Jul 2016 | Kinetis version                                                          V4.12.1.120 - pgo
   +=========================================================================================================
   \endverbatim
 */

#ifndef INCLUDE_SWD_H_
#define INCLUDE_SWD_H_

#include <stdint.h>
#include "commands.h"

namespace Swd {

//==========================================================================
// Command bytes for SWD transfers - Park,Stop,Parity,A[3:2],R/W,AP/DP,Start
//
// Read registers
static constexpr uint8_t  SWD_RD_DP_IDCODE  = 0xA5; // 10100101
static constexpr uint8_t  SWD_RD_DP_STATUS  = 0x8D; // 10001101
static constexpr uint8_t  SWD_RD_DP_RESEND  = 0x95; // 10010101
static constexpr uint8_t  SWD_RD_DP_RDBUFF  = 0xBD; // 10111101
//
// Write registers
static constexpr uint8_t  SWD_WR_DP_ABORT   = 0x81; // 10000001
static constexpr uint8_t  SWD_WR_DP_CONTROL = 0xA9; // 10101001
static constexpr uint8_t  SWD_WR_DP_SELECT  = 0xB1; // 10110001
//
// Read AP register
static constexpr uint8_t  SWD_RD_AP_REG0    = 0x87; // 10000111
static constexpr uint8_t  SWD_RD_AP_REG1    = 0xAF; // 10101111
static constexpr uint8_t  SWD_RD_AP_REG2    = 0xB7; // 10110111
static constexpr uint8_t  SWD_RD_AP_REG3    = 0x9F; // 10011111
//
// Write AP register
static constexpr uint8_t  SWD_WR_AP_REG0    = 0xA3; // 10100011
static constexpr uint8_t  SWD_WR_AP_REG1    = 0x8B; // 10001011
static constexpr uint8_t  SWD_WR_AP_REG2    = 0x93; // 10010011
static constexpr uint8_t  SWD_WR_AP_REG3    = 0xBB; // 10111011

// Memory addresses of debug/core registers
static constexpr uint32_t  DHCSR_ADDR              = 0xE000EDF0U; // RW Debug Halting Control and Status Register
static constexpr uint32_t  DCRSR_ADDR              = 0xE000EDF4U; // WO Debug Core Selector Register
static constexpr uint32_t  DCRDR_ADDR              = 0xE000EDF8U; // RW Debug Core Data Register

static constexpr uint32_t  DCRSR_WRITE             = (1<<16);
static constexpr uint32_t  DCRSR_READ              = (0);
static constexpr uint32_t  DCRSR_REGMASK           = (0x7F);

static constexpr uint32_t  DHCSR_DBGKEY            = (0xA05F<<16);
static constexpr uint32_t  DHCSR_S_RESET_ST        = (1<<25);
static constexpr uint32_t  DHCSR_S_RETIRE_ST       = (1<<24);
static constexpr uint32_t  DHCSR_S_LOCKUP          = (1<<19);
static constexpr uint32_t  DHCSR_S_SLEEP           = (1<<18);
static constexpr uint32_t  DHCSR_S_HALT            = (1<<17);
static constexpr uint32_t  DHCSR_S_REGRDY          = (1<<16);
static constexpr uint32_t  DHCSR_C_SNAPSTALL       = (1<<5);
static constexpr uint32_t  DHCSR_C_MASKINTS        = (1<<3);
static constexpr uint32_t  DHCSR_C_STEP            = (1<<2);
static constexpr uint32_t  DHCSR_C_HALT            = (1<<1);
static constexpr uint32_t  DHCSR_C_DEBUGEN         = (1<<0);

/**
 * Pack 4 bytes into a 32-bit value in LITTLE-ENDIAN order order
 *
 * @param  data Data value in LITTLE-ENDIAN order order
 *
 * @return Value
 */
static inline constexpr uint32_t pack32LE(const uint8_t data[4]) {
   return data[0]+(data[1]<<8)+(data[2]<<16)+(data[3]<<24);
}

/**
 * Pack 4 bytes into a 32-bit value in BIG-ENDIAN order order
 *
 * @param  data Data value in BIG_ENDIAN order
 *
 * @return Value
 */
static inline
constexpr uint32_t pack32BE(const uint8_t data[4]) {
   return (data[0]<<24)+(data[1]<<16)+(data[2]<<8)+data[3];
}

/**
 * Pack 2 bytes into a 32-bit value for ARM addresses
 *
 * @param  data   Data value in BIG-ENDIAN order order
 *
 * @return Value
 */
static inline
constexpr uint32_t pack16AddressBE(const uint8_t data[2]) {
   return (data[0]<<24)+data[1];
}

/**
 * Unpack a 32-bit value into 4 bytes in LE order
 *
 * @param  data    Value to unpack
 * @param  ar   Buffer for data value in LITTLE-ENDIAN order order
 *
 * @return Value
 */
static inline
void unpack32LE(uint32_t data, uint8_t ar[4]) {
   ar[3] = data>>24;
   ar[2] = data>>16;
   ar[1] = data>>8;
   ar[0] = data;
}

/**
 * Unpack a 32-bit value into 4 bytes in BE order
 *
 * @param  data    Value to unpack
 * @param  ar      Buffer for data value in BIG-ENDIAN order
 *
 * @return Value
 */
static inline void unpack32BE(uint32_t data, uint8_t ar[4]) {
   ar[0] = data>>24;
   ar[1] = data>>16;
   ar[2] = data>>8;
   ar[3] = data;
}

/**
 * Sets Communication speed for SWD
 *
 * @param frequency Frequency in Hz
 *
 * Note: Chooses the highest speed that is not greater than frequency.
 * Note: This will only have effect the next time a CTAR is changed
 */
USBDM_ErrorCode setSpeed(uint32_t frequency);

/**
 * Gets Communication speed of SWD
 *
 * @return frequency Frequency in Hz
 *
 * @note This may differ from set speed due to limited range of speeds available
 */
uint32_t getSpeed();

/**
 * Initialise interface\n
 * Does not communicate with target
 */
void initialise();

/**
 *  Switches interface to SWD and confirm connection to target
 *
 *  Reference ARM Debug Interface v5 Architecture Specification
 *            ADIv5.1 Supplement - 6.2.1 JTAG to Serial Wire switching
 *
 *  Sequence as follows:
 *   - >=50-bit sequence of 1's
 *   - 16-bit magic number 0xE79E
 *   - >=50-bit sequence of 1's
 *   - 8-bit idle
 *   - Read IDCODE
 *
 *  @return \n
 *     == \ref BDM_RC_OK => Success
 */
USBDM_ErrorCode connect(void);

/**
 * Power up debug interface and system\n
 * Sets CSYSPWRUPREQ and CDBGPWRUPREQ\n
 * Confirms CSYSPWRUPACK and CDBGPWRUPACK
 *
 *  @return \n
 *     == \ref BDM_RC_OK => Success
 */
USBDM_ErrorCode powerUp();


/**
 * Resets SWD interface
 *
 *  Sequence as follows:
 *   - >=50-bit sequence of 1's (55 0's)
 *   - >=8-bit sequence of 0's  (9 1's)
 *   - Read IDCODE
 *
 *  @return \n
 *     == \ref BDM_RC_OK => Success
 */
USBDM_ErrorCode lineReset(void);

/**
 *  Read ARM-SWD DP & AP register
 *
 *  @param command - SWD command byte to select register etc.
 *  @param data    - 32-bit value read
 *
 *  @return \n
 *     == \ref BDM_RC_OK               => Success        \n
 *     == \ref BDM_RC_ARM_FAULT_ERROR  => FAULT response from target \n
 *     == \ref BDM_RC_ACK_TIMEOUT      => Excessive number of WAIT responses from target \n
 *     == \ref BDM_RC_NO_CONNECTION    => Unexpected/no response from target \n
 *     == \ref BDM_RC_ARM_PARITY_ERROR => Parity error on data read
 *
 *  @note Action and Data returned depends on register (some responses are pipelined)\n
 *    SWD_RD_DP_IDCODE - Value from IDCODE reg \n
 *    SWD_RD_DP_STATUS - Value from STATUS reg \n
 *    SWD_RD_DP_RESEND - LAST value read (AP read or DP-RDBUFF), FAULT on sticky error    \n
 *    SWD_RD_DP_RDBUFF - Value from last AP read and clear READOK flag in STRL/STAT, FAULT on sticky error \n
 *    SWD_RD_AP_REGx   - Value from last AP read, clear READOK flag in STRL/STAT and INITIATE next AP read, FAULT on sticky error
 */
USBDM_ErrorCode readReg(uint8_t command, uint32_t &data);

/**
 *  Read ARM-SWD DP & AP register
 *
 *  @param command - SWD command byte to select register etc.
 *  @param data    - Buffer for 32-bit value read in BIG-ENDIAN order order
 *
 *  @return \n
 *     == \ref BDM_RC_OK               => Success        \n
 *     == \ref BDM_RC_ARM_FAULT_ERROR  => FAULT response from target \n
 *     == \ref BDM_RC_ACK_TIMEOUT      => Excessive number of WAIT responses from target \n
 *     == \ref BDM_RC_NO_CONNECTION    => Unexpected/no response from target \n
 *     == \ref BDM_RC_ARM_PARITY_ERROR => Parity error on data read
 *
 *  @note Action and Data returned depends on register (some responses are pipelined)\n
 *    SWD_RD_DP_IDCODE - Value from IDCODE reg \n
 *    SWD_RD_DP_STATUS - Value from STATUS reg \n
 *    SWD_RD_DP_RESEND - LAST value read (AP read or DP-RDBUFF), FAULT on sticky error    \n
 *    SWD_RD_DP_RDBUFF - Value from last AP read and clear READOK flag in STRL/STAT, FAULT on sticky error \n
 *    SWD_RD_AP_REGx   - Value from last AP read, clear READOK flag in STRL/STAT and INITIATE next AP read, FAULT on sticky error
 */
static inline
USBDM_ErrorCode readReg(uint8_t command, uint8_t data[4]) {
   uint32_t temp = 0;
   USBDM_ErrorCode rc = readReg(command, temp);
   unpack32BE(temp, data);
   return rc;
}

/**
 *  Write ARM-SWD DP & AP register
 *
 *  @param command - SWD command byte to select register etc.
 *  @param data    - 32-bit value to write
 *
 *  @return \n
 *     == \ref BDM_RC_OK               => Success        \n
 *     == \ref BDM_RC_ARM_FAULT_ERROR  => FAULT response from target \n
 *     == \ref BDM_RC_ACK_TIMEOUT      => Excessive number of WAIT responses from target \n
 *     == \ref BDM_RC_NO_CONNECTION    => Unexpected/no response from target
 *
 *  @note Action depends on register (some responses are pipelined)\n
 *    SWD_WR_DP_ABORT   - Write value to ABORT register (accepted) \n
 *    SWD_WR_DP_CONTROL - Write value to CONTROL register (may be pending), FAULT on sticky error. \n
 *    SWD_WR_DP_SELECT  - Write value to SELECT register (may be pending), FAULT on sticky error. \n
 *    SWD_WR_AP_REGx    - Write to AP register.  May initiate action e.g. memory access.  Result is pending, FAULT on sticky error.
 */
USBDM_ErrorCode writeReg(uint8_t command, const uint32_t data);

/**
 *  Write ARM-SWD DP & AP register
 *
 *  @param command - SWD command byte to select register etc.
 *  @param data    - Buffer containing 32-bit value to write in BIG-ENDIAN format
 *
 *  @return \n
 *     == \ref BDM_RC_OK               => Success        \n
 *     == \ref BDM_RC_ARM_FAULT_ERROR  => FAULT response from target \n
 *     == \ref BDM_RC_ACK_TIMEOUT      => Excessive number of WAIT responses from target \n
 *     == \ref BDM_RC_NO_CONNECTION    => Unexpected/no response from target
 *
 *  @note Action depends on register (some responses are pipelined)\n
 *    SWD_WR_DP_ABORT   - Write value to ABORT register (accepted) \n
 *    SWD_WR_DP_CONTROL - Write value to CONTROL register (may be pending), FAULT on sticky error. \n
 *    SWD_WR_DP_SELECT  - Write value to SELECT register (may be pending), FAULT on sticky error. \n
 *    SWD_WR_AP_REGx    - Write to AP register.  May initiate action e.g. memory access.  Result is pending, FAULT on sticky error.
 */
static inline
USBDM_ErrorCode writeReg(uint8_t command, const uint8_t data[4]) {
   return writeReg(command, pack32BE(data));
}

/**
 *  Read register of Access Port
 *
 *  @param address 32-bit address \n
 *     A[31:24] => DP-AP-SELECT[31:24] (AP # Select) \n
 *     A[7:4]   => DP-AP-SELECT[7:4]   (Bank select within AP) \n
 *     A[3:2]   => APACC[3:2]          (Register select within bank)
 *  @param buff 32-bit register value
 *
 *  @return
 *   == \ref BDM_RC_OK => success
 *
 *  @note - Access is completed before return
 */
USBDM_ErrorCode readAPReg(const uint32_t address, uint32_t &buff);

/**
 *  Read register of Access Port
 *
 *  @param address 32-bit address in BIG-ENDIAN order\n
 *     A[31:24] => DP-AP-SELECT[31:24] (AP # Select) \n
 *     A[7:4]   => DP-AP-SELECT[7:4]   (Bank select within AP) \n
 *     A[3:2]   => APACC[3:2]          (Register select within bank)
 *  @param data Buffer for 32-bit register value in BIG-ENDIAN order
 *
 *  @return
 *   == \ref BDM_RC_OK => success
 *
 *  @note - Access is completed before return
 */
static inline
USBDM_ErrorCode readAPReg(uint8_t address[2], uint8_t data[4]) {
   uint32_t temp = 0;
   USBDM_ErrorCode rc = readAPReg(pack16AddressBE(address), temp);
   unpack32BE(temp, data);
   return rc;
}
/**
 *  Write Access Port register
 *
 *  @param address 16-bit address \n
 *     A[15:8]  => DP-AP-SELECT[31:24] (AP # Select) \n
 *     A[7:4]   => DP-AP-SELECT[7:4]   (Bank select within AP) \n
 *     A[3:2]   => APACC[3:2]          (Register select within bank)
 *  @param data 32-bit register value
 *
 *  @return
 *   == \ref BDM_RC_OK => success
 *
 *  @note - Access is completed before return
 */
USBDM_ErrorCode writeAPReg(const uint32_t address, const uint32_t data);

/**
 *  Write Access Port register
 *
 *  @param address 16-bit address in BIG-ENDIAN order \n
 *     A[15:8]  => DP-AP-SELECT[31:24] (AP # Select) \n
 *     A[7:4]   => DP-AP-SELECT[7:4]   (Bank select within AP) \n
 *     A[3:2]   => APACC[3:2]          (Register select within bank)
 *  @param buff \n
 *    - [1..4]  => 32-bit register value in BIG-ENDIAN order
 *
 *  @return
 *   == \ref BDM_RC_OK => success
 *
 *  @note - Access is completed before return
 */
static inline
USBDM_ErrorCode writeAPReg(const uint8_t address[2], const uint8_t buff[4]) {
   return writeAPReg(pack16AddressBE(address), pack32BE(buff));
}
/**
 *  Clear all sticky bits in status register
 *
 *  @return error code
 */
USBDM_ErrorCode clearStickyBits(void);

/**
 * Clear sticky bits and abort AP transactions
 *
 *  @return error code
 */
USBDM_ErrorCode abortAP(void);

/**
 * Mass erase target
 *
 * @return BDM_RC_OK if successful
 */
USBDM_ErrorCode kinetisMassErase(void);

/** Write 32-bit value to ARM-SWD Memory
 *
 *  @param address 32-bit memory address
 *  @param data    32-bit data value
 *
 *  @return
 *   == \ref BDM_RC_OK => success         \n
 *   != \ref BDM_RC_OK => various errors
 */
USBDM_ErrorCode writeMemoryWord(const uint32_t address, const uint32_t data);

/**  Write 32-bit value to ARM-SWD Memory
 *
 *  @param address 32-bit memory address
 *  @param data    32-bit data value in BIG-ENDIAN order
 *
 *  @return
 *   == \ref BDM_RC_OK => success         \n
 *   != \ref BDM_RC_OK => various errors
 */
static inline
USBDM_ErrorCode writeMemoryWord(const uint32_t address, const uint8_t data[4]) {
   return writeMemoryWord(address, pack32BE(data));
}

/**  Write ARM-SWD Memory
 *
 *  @note
 *   commandBuffer\n
 *    - [2]     =>  size of data elements
 *    - [3]     =>  # of bytes
 *    - [4..7]  =>  Memory address in BIG-ENDIAN order
 *    - [8..N]  =>  Data to write
 *
 *  @return \n
 *   == \ref BDM_RC_OK => success         \n
 *   != \ref BDM_RC_OK => various errors
 */
USBDM_ErrorCode writeMemory(
      uint32_t  elementSize,  // Size of the data writes
      uint32_t  count,        // # of bytes
      uint32_t  addr,         // Address in target memory
      uint8_t   *data_ptr     // Where the data is
);

/** Read 32-bit value from ARM-SWD Memory
 *
 *  @param address 32-bit memory address
 *  @param data    32-bit data value from _last_ read in BIG-ENDIAN order!
 *
 *  @return
 *   == \ref BDM_RC_OK => success         \n
 *   != \ref BDM_RC_OK => various errors
 */
USBDM_ErrorCode readMemoryWord(const uint32_t address, uint32_t &data);

/**  Read 32-bit value from ARM-SWD Memory
 *
 *  @param address 32-bit memory address
 *  @param data    32-bit data value from _last_ read in BIG-ENDIAN order!
 *
 *  @return
 *   == \ref BDM_RC_OK => success         \n
 *   != \ref BDM_RC_OK => various errors
 */
static inline
USBDM_ErrorCode readMemoryWord(const uint32_t address, uint8_t data[4]) {
   uint32_t temp = 0;
   USBDM_ErrorCode rc = readMemoryWord(address, temp);
   unpack32BE(temp, data);
   return rc;
}

/**  Read ARM-SWD Memory
 *
 *  @note
 *   commandBuffer\n
 *    - [2]     =>  size of data elements
 *    - [3]     =>  # of bytes
 *    - [4..7]  =>  Memory address in BIG-ENDIAN order
 *
 *  @return
 *   == \ref BDM_RC_OK => success         \n
 *   != \ref BDM_RC_OK => various errors  \n
 *                                        \n
 *   commandBuffer                        \n
 *    - [1..N]  =>  Data read
 */
USBDM_ErrorCode readMemory(
      uint32_t   elementSize,  // Size of the data writes
      int        count,        // # of data bytes
      uint32_t   addr,         // LSB of Address in target memory
      uint8_t   *data_ptr      // Where in buffer to write the data
);

/**  Write ARM-SWD Memory
 *
 *  @note
 *   commandBuffer\n
 *    - [2]     =>  size of data elements
 *    - [3]     =>  # of bytes
 *    - [4..7]  =>  Memory address in BIG-ENDIAN order
 *    - [8..N]  =>  Data to write
 *
 *  @return \n
 *   == \ref BDM_RC_OK => success         \n
 *   != \ref BDM_RC_OK => various errors
 */
USBDM_ErrorCode writeMemory(
      uint32_t        elementSize, // Size of the data writes
      uint32_t        count,       // # of bytes
      uint32_t        addr,     // Address in target memory
      const uint8_t  *data_ptr     // Where the data is
);

/**
 *  Read target register
 *
 *  @param regNo    Number of register to read
 *  @param data     Where to place data read (in big-endian order)
 *
 *  @return error code
 */
USBDM_ErrorCode readCoreRegister(uint8_t regNo, uint8_t *data);

/**
 *  Write ARM-SWD core register
 *
 *  @note
 *   commandBuffer\n
 *    - [2..3]  =>  16-bit register number [MSB ignored]
 *    - [4..7]  =>  32-bit register value
 *
 *  @return
 *   == \ref BDM_RC_OK => success
 */
USBDM_ErrorCode writeCoreReg(uint32_t regNo, uint8_t *data);

/**
 *  ARM-SWD -  Modifies value in LSB of DHCSR
 *  DHCSR.lsb = (DHCSR&preserveBits)|setBits
 *
 *  @param preserveBits - Bits to preserve (done first)
 *  @param setBits      - Bits to set (done last)
 *
 *  @return
 *     == \ref BDM_RC_OK => success       \n
 *     != \ref BDM_RC_OK => error         \n
 */
USBDM_ErrorCode modifyDHCSR(uint8_t preserveBits, uint8_t setBits);

}; // End namespace Swd

#endif /* INCLUDE_SWD_H_ */
