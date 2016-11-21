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

#include <stdio.h>
#include "spi.h"
#include "commands.h"
#include "targetDefines.h"
#include "configure.h"

namespace Swd {

/** Select SPI to use */
using SpiInfo = USBDM::Spi0Info;

//===========================================================================

// Masks for SWD_WR_DP_ABORT clear sticky
static constexpr uint32_t  SWD_DP_ABORT_CLEAR_STICKY_ERRORS = 0x0000001E;

// Masks for SWD_WR_DP_ABORT abort AP
static constexpr uint32_t  SWD_DP_ABORT_ABORT_AP            = 0x00000001;

// Masks for SWD_RD_DP_STATUS
//static constexpr uint32_t  SWD_RD_DP_STATUS_ANYERROR        = 0x000000B2;

// Masks for SWD_WR_DP_CONTROL
static constexpr uint32_t  SWD_WR_DP_CONTROL_POWER_REQ = (1<<30)|(1<<28);
static constexpr uint32_t  SWD_WR_DP_CONTROL_POWER_ACK = (1<<31)|(1<<29);

// AP number for AHB-AP (MEM-AP implementation)
static constexpr uint32_t  AHB_AP_NUM        = (0x0);

// DP_SELECT register value to access AHB_AP Bank #0 for memory read/write
static constexpr uint32_t ARM_AHB_AP_BANK0 = Swd::AHB_AP_NUM;

//   static constexpr uint32_t  AHB_CSW_REGNUM    = (0x0);  // CSW register bank+register number
//   static constexpr uint32_t  AHB_TAR_REGNUM    = (0x4);  // TAR register bank+register number
//   static constexpr uint32_t  AHB_DRW_REGNUM    = (0xC);  // DRW register bank+register number

static constexpr uint32_t  SWD_RD_AHB_CSW = SWD_RD_AP_REG0; // SWD command for reading AHB-CSW
//static constexpr uint32_t  SWD_RD_AHB_TAR = SWD_RD_AP_REG1; // SWD command for reading AHB-TAR
static constexpr uint32_t  SWD_RD_AHB_DRW = SWD_RD_AP_REG3; // SWD command for reading AHB-DRW

static constexpr uint32_t  SWD_WR_AHB_CSW = SWD_WR_AP_REG0; // SWD command for writing AHB-CSW
static constexpr uint32_t  SWD_WR_AHB_TAR = SWD_WR_AP_REG1; // SWD command for writing AHB-TAR
static constexpr uint32_t  SWD_WR_AHB_DRW = SWD_WR_AP_REG3; // SWD command for writing AHB-DRW

// AHB-AP (MEM-AP) CSW Register masks
static constexpr uint32_t  AHB_AP_CSW_INC_SINGLE    = (1<<4);
//static constexpr uint32_t  AHB_AP_CSW_INC_PACKED    = (2<<4);
//static constexpr uint32_t  AHB_AP_CSW_INC_MASK      = (3<<4);
static constexpr uint32_t  AHB_AP_CSW_SIZE_BYTE     = (0<<0);
static constexpr uint32_t  AHB_AP_CSW_SIZE_HALFWORD = (1<<0);
static constexpr uint32_t  AHB_AP_CSW_SIZE_WORD     = (2<<0);
//static constexpr uint32_t  AHB_AP_CSW_SIZE_MASK     = (7<<0);

/**
 * Get AHB.CSW value based on size
 *
 * @param size Transfer size in bytes (one of 1,2 or 4)
 *
 * @return AHB.CSW mask.  This will include size and increment.
 */
static uint32_t getcswValue(int size) {
   static const uint32_t cswValues[5] = {
         0,
         Swd::AHB_AP_CSW_SIZE_BYTE    |Swd::AHB_AP_CSW_INC_SINGLE,
         Swd::AHB_AP_CSW_SIZE_HALFWORD|Swd::AHB_AP_CSW_INC_SINGLE,
         0,
         Swd::AHB_AP_CSW_SIZE_WORD    |Swd::AHB_AP_CSW_INC_SINGLE,
   };
   return cswValues[size];
};

enum SwdAck {
   /** OK ACK value. Indicates SWD transfer was successful */
   SWD_ACK_OK    = 0x1,
   /** WAIT ACK value. Indicates SWD transfer cannot complete yet. Should be retried */
   SWD_ACK_WAIT  = 0x2,
   /** FAULT ACK value. Indicates SWD transfer failed. */
   SWD_ACK_FAULT = 0x4,
};

/** Select for BKGD/SWD_DIR pin direction - Transmit */
static constexpr uint32_t TX_MASK = SPI_PUSHR_PCS(1<<1);

/** Select for BKGD/SWD_DIR pin direction - Receive */
static constexpr uint32_t RX_MASK = SPI_PUSHR_PCS(0);

/** Base communication settings (CTAR value) */
static constexpr uint32_t  CTAR_TX =
      SPI_CTAR_CPOL(1)   | // Clock idle=1
      SPI_CTAR_CPHA(1)   | // Data change falling edge of clock
      SPI_CTAR_LSBFE(1)  | // LSB First
      SPI_CTAR_PCSSCK(0) | // PCS to SCK  Delay Prescaler
      SPI_CTAR_PASC(0)   | // SCK to PCSn Delay Prescaler
      SPI_CTAR_ASC(0)    | // SCK to PCSn Delay
      SPI_CTAR_PDT(0)    | // PCSn to PCS Delay Prescaler
      SPI_CTAR_DT(0);      // PCSn to PCS Delay

/** Base communication settings (CTAR value) */
static constexpr uint32_t  CTAR_RX =
      SPI_CTAR_CPOL(1)   | // Clock idle=1
      SPI_CTAR_CPHA(0)   | // Data capture rising edge of clock
      SPI_CTAR_LSBFE(1)  | // LSB First
      SPI_CTAR_PCSSCK(0) | // PCS to SCK  Delay Prescaler
      SPI_CTAR_PASC(0)   | // SCK to PCSn Delay Prescaler
      SPI_CTAR_ASC(0)    | // SCK to PCSn Delay
      SPI_CTAR_PDT(0)    | // PCSn to PCS Delay Prescaler
      SPI_CTAR_DT(0);      // PCSn to PCS Delay

static constexpr uint32_t CTAR_MASK = ~(SPI_CTAR_BR_MASK|SPI_CTAR_PBR_MASK|SPI_CTAR_DBR_MASK);

/** SPI Object */
static constexpr SPI_Type volatile *spi = SpiInfo::spi;

/** Current Baud Rate */
static uint32_t spiBaudValue;

/** Initial value of AHB_SP_CSW register read from target */
static uint32_t ahb_ap_csw_defaultValue;

/**
 * Set SPI.CTAR0 value\n
 * Value will be combined with the current frequency divider
 *
 * @param ctar 32-bit CTAR value (excluding baud related settings)\n
 *     e.g. setCTAR0Value(SPI_CTAR_SLAVE_FMSZ(8-1)|SPI_CTAR_CPOL_MASK|SPI_CTAR_CPHA_MASK);
 */
static void setCTAR0Value(uint32_t ctar) {
   spi->CTAR[0] = spiBaudValue|(ctar&CTAR_MASK);
}

/**
 * Set SPI.CTAR1 value\n
 * Value will be combined with the current frequency divider
 *
 * @param ctar 32-bit CTAR value (excluding baud related settings)\n
 *     e.g. setCTAR1Value(SPI_CTAR_SLAVE_FMSZ(8-1)|SPI_CTAR_CPOL_MASK|SPI_CTAR_CPHA_MASK);
 */
static void setCTAR1Value(uint32_t ctar) {
   spi->CTAR[1] = spiBaudValue|(ctar&CTAR_MASK);
}

/**
 * Calculate parity of a 32-bit value
 *
 * @param  data Data value
 *
 * @return parity value (0/1)
 */
__attribute__((naked))
static uint8_t calcParity(const uint32_t data) {
   (void)data;
   __asm__ volatile (
         "eor.w r0, r0, r0, lsr #16  \n\t"
         "eor.w r0, r0, r0, lsr #8   \n\t"
         "eor.w r0, r0, r0, lsr #4   \n\t"
         "eor.w r0, r0, r0, lsr #2   \n\t"
         "eor.w r0, r0, r0, lsr #1   \n\t"
         "and.w r0, #1               \n\t"
         "bx lr                      \n\t"
   );
   return 0; // stop warning
}

/**
 * Transmits 8-bits of idle (SWDIO=0)
 */
static void txIdle8() {
   setCTAR0Value(CTAR_TX|SPI_CTAR_FMSZ(8-1)); // 8-bit Transmit

   // Write data
   spi->PUSHR = SPI_PUSHR_CTAS(0)|SPI_PUSHR_EOQ_MASK|TX_MASK|SPI_PUSHR_TXDATA(0);
   // Wait until complete
   while ((spi->SR & SPI_SR_EOQF_MASK) == 0) {
   }
   // Discard read data
   (void)spi->POPR;
   // Clear flags
   spi->SR = SPI_SR_RFDF_MASK|SPI_SR_EOQF_MASK;
}

/**
 *  Transmit [mark, 8-bit word], receive [ACK]
 *
 *  @param data Data to send
 *
 *  @return ACK value received
 */
static SwdAck txMark_8_rxAck(uint32_t data) {
   setCTAR0Value(CTAR_TX|SPI_CTAR_FMSZ(9-1)); // 9-bit Transmit
   setCTAR1Value(CTAR_RX|SPI_CTAR_FMSZ(3-1)); // 3-bit Receive

   // Write data
   spi->PUSHR = SPI_PUSHR_CTAS(0)|TX_MASK|SPI_PUSHR_TXDATA((data<<1)|1);
   // Read ACK
   spi->PUSHR = SPI_PUSHR_CTAS(1)|RX_MASK|SPI_PUSHR_TXDATA(0)|SPI_PUSHR_EOQ_MASK;
   while ((spi->SR & SPI_SR_EOQF_MASK) == 0) {
   }
   // Discard read data
   (void)spi->POPR;
   // Clear flags
   spi->SR = SPI_SR_RFDF_MASK|SPI_SR_EOQF_MASK;
   // Return ACK
   return (SwdAck)(spi->POPR);
}

/**
 *  Transmit [mark, 8-bit word], receive [ACK, TURN]
 *
 *  @param data Data to send
 *
 *  @return ACK value received
 */
static SwdAck txMark_8_rxAck_Trn(uint32_t data) {
   setCTAR0Value(CTAR_TX|SPI_CTAR_FMSZ(9-1)); // 9-bit Transmit
   setCTAR1Value(CTAR_RX|SPI_CTAR_FMSZ(3-1)); // 5-bit Receive

   // Write data
   spi->PUSHR = SPI_PUSHR_CTAS(0)|TX_MASK|SPI_PUSHR_TXDATA((data<<1)|1);
   // Read ACK & TURN
   spi->PUSHR = SPI_PUSHR_CTAS(1)|RX_MASK|SPI_PUSHR_TXDATA(0)|SPI_PUSHR_EOQ_MASK;
   while ((spi->SR & SPI_SR_EOQF_MASK) == 0) {
   }
   // Discard read data
   (void)spi->POPR;
   // Clear flags
   spi->SR = SPI_SR_RFDF_MASK|SPI_SR_EOQF_MASK;
   // Return ACK
   return (SwdAck)(spi->POPR&0x3);
}

/**
 * Transmit [command], Receive [ACK]
 *
 *  @param command Command to send
 *
 *  @return ACK value received
 */
static SwdAck txCommand_rxAck(uint32_t command) {

   setCTAR0Value(CTAR_TX|SPI_CTAR_FMSZ(8-1)); // 8-bit Transmit
   setCTAR1Value(CTAR_RX|SPI_CTAR_FMSZ(4-1)); // 4-bit Receive

   // Write data
   spi->PUSHR = SPI_PUSHR_CTAS(0)|TX_MASK|SPI_PUSHR_TXDATA(command);
   // Read ACK
   spi->PUSHR = SPI_PUSHR_CTAS(1)|RX_MASK|SPI_PUSHR_TXDATA(0)|SPI_PUSHR_EOQ_MASK;
   // Wait until complete
   while ((spi->SR & SPI_SR_EOQF_MASK) == 0) {
   }
   // Discard 1st byte read data
   (void)spi->POPR;
   // Clear flags
   spi->SR = SPI_SR_RFDF_MASK|SPI_SR_EOQF_MASK;
   // Return ACK value
   return (SwdAck)(spi->POPR>>1);
}

/**
 * Transmit [command], Receive [ACK, TURN]
 *
 *  @param command Command to send
 *
 *  @return ACK value received
 */
static SwdAck txCommand_rxAck_Trn(uint32_t command) {

   setCTAR0Value(CTAR_TX|SPI_CTAR_FMSZ(8-1)); // 8-bit Transmit
   setCTAR1Value(CTAR_RX|SPI_CTAR_FMSZ(5-1)); // 5-bit Receive = [TURN,ACK,TURN]

   // Write data
   spi->PUSHR = SPI_PUSHR_CTAS(0)|TX_MASK|SPI_PUSHR_TXDATA(command);
   // Read ACK
   spi->PUSHR = SPI_PUSHR_CTAS(1)|RX_MASK|SPI_PUSHR_TXDATA(0)|SPI_PUSHR_EOQ_MASK;
   // Wait until complete
   while ((spi->SR & SPI_SR_EOQF_MASK) == 0) {
   }
   // Discard 1st byte read data
   (void)spi->POPR;
   // Clear flags
   spi->SR = SPI_SR_RFDF_MASK|SPI_SR_EOQF_MASK;
   // Return 1st 3 bits (1st TURN not captured, 3xACK, 2nd TURN)
   return (SwdAck)((spi->POPR>>1)&0x7);
}

/**
 *  Transmit [32-bit word, parity]
 *
 *  @param data Data to send
 */
static void tx32_parity(const uint32_t data) {
   uint8_t parity = calcParity(data);
   setCTAR0Value(CTAR_TX|SPI_CTAR_FMSZ(8-1)); // 8-bit Transmit
   setCTAR1Value(CTAR_TX|SPI_CTAR_FMSZ(9-1)); // 9-bit Transmit

   // Write data with parity
   spi->PUSHR = SPI_PUSHR_CTAS(0)|TX_MASK|SPI_PUSHR_CONT_MASK|SPI_PUSHR_TXDATA(data);
   spi->PUSHR = SPI_PUSHR_CTAS(0)|TX_MASK|SPI_PUSHR_CONT_MASK|SPI_PUSHR_TXDATA(data>>8);
   spi->PUSHR = SPI_PUSHR_CTAS(0)|TX_MASK|SPI_PUSHR_CONT_MASK|SPI_PUSHR_TXDATA(data>>16);
   spi->PUSHR = SPI_PUSHR_CTAS(1)|TX_MASK|                    SPI_PUSHR_TXDATA((data>>24)|(parity<<8))|SPI_PUSHR_EOQ_MASK;
   while ((spi->SR & SPI_SR_EOQF_MASK) == 0) {
   }
   (void)spi->POPR; // Discard read data
   (void)spi->POPR;
   (void)spi->POPR;
   (void)spi->POPR;
   // Clear flags
   spi->SR = SPI_SR_RFDF_MASK|SPI_SR_EOQF_MASK;
}

/**
 *  Transmit [32-bit word]
 *
 *  @param data Data to send
 */
static void tx32(const uint32_t data) {
   setCTAR0Value(CTAR_TX|SPI_CTAR_FMSZ(8-1)); // 8-bit Transmit

   // Write data
   spi->PUSHR = SPI_PUSHR_CTAS(0)|TX_MASK|SPI_PUSHR_CONT_MASK|SPI_PUSHR_TXDATA(data);
   spi->PUSHR = SPI_PUSHR_CTAS(0)|TX_MASK|SPI_PUSHR_CONT_MASK|SPI_PUSHR_TXDATA(data>>8);
   spi->PUSHR = SPI_PUSHR_CTAS(0)|TX_MASK|SPI_PUSHR_CONT_MASK|SPI_PUSHR_TXDATA(data>>16);
   spi->PUSHR = SPI_PUSHR_CTAS(0)|TX_MASK|                    SPI_PUSHR_TXDATA(data>>24)|SPI_PUSHR_EOQ_MASK;
   while ((spi->SR & SPI_SR_EOQF_MASK) == 0) {
   }
   (void)spi->POPR;  // Discard read data
   (void)spi->POPR;
   (void)spi->POPR;
   (void)spi->POPR;
   // Clear flags
   spi->SR = SPI_SR_RFDF_MASK|SPI_SR_EOQF_MASK;
}

/**
 *  Receive [32-bit word, parity] from the target
 *
 *  @param receive Data received
 *
 *  @return BDM_RC_OK               => Success
 *  @return BDM_RC_ARM_PARITY_ERROR => Parity error on reception
 */
static USBDM_ErrorCode rx32_parity(uint32_t &receive) {
   uint16_t byte_plus_parity;
   setCTAR0Value(CTAR_RX|SPI_CTAR_FMSZ(8-1)); // 8-bit Receive
   setCTAR1Value(CTAR_RX|SPI_CTAR_FMSZ(9-1)); // 9-bit Receive

   // Read data & parity
   spi->PUSHR = SPI_PUSHR_CTAS(0)|RX_MASK|SPI_PUSHR_CONT_MASK|SPI_PUSHR_TXDATA(0);
   spi->PUSHR = SPI_PUSHR_CTAS(0)|RX_MASK|SPI_PUSHR_CONT_MASK|SPI_PUSHR_TXDATA(0);
   spi->PUSHR = SPI_PUSHR_CTAS(0)|RX_MASK|SPI_PUSHR_CONT_MASK|SPI_PUSHR_TXDATA(0);
   spi->PUSHR = SPI_PUSHR_CTAS(1)|RX_MASK|                    SPI_PUSHR_TXDATA(0)|SPI_PUSHR_EOQ_MASK;
   while ((spi->SR & SPI_SR_EOQF_MASK) == 0) {
   }
   spi->SR = SPI_SR_EOQF_MASK;
   receive           = spi->POPR;
   receive          |= (spi->POPR<<8);
   receive          |= (spi->POPR<<16);
   byte_plus_parity  = spi->POPR;
   receive          |= (byte_plus_parity<<24);
   return ((byte_plus_parity>>8)!=calcParity(receive))?BDM_RC_ARM_PARITY_ERROR:BDM_RC_OK;
}

/**
 * Sets Communication speed for SPI
 *
 * @param frequency Frequency in Hz
 *
 * @return BDM_RC_OK success
 *
 * Note: Chooses the highest speed that is not greater than frequency.
 */
USBDM_ErrorCode setSpeed(uint32_t frequency) {
   spiBaudValue = USBDM::Spi::calculateDividers(SpiInfo::getClockFrequency(), frequency);
   return BDM_RC_OK;
}

/**
 * Gets Communication speed of SWD
 *
 * @return frequency Frequency in Hz
 *
 * @note This may differ from set speed due to limited range of speeds available
 */
uint32_t getSpeed() {
   return USBDM::Spi::calculateSpeed(SpiInfo::getClockFrequency(), spiBaudValue);
}

/**
 * Initialise interface\n
 * Does not communicate with target
 */
void initialise() {

   PRINTF("Swd::initialise()\n");

   // Configure SPI pins
   SpiInfo::initPCRs();

   *SpiInfo::clockReg |= SpiInfo::clockMask;

   setSpeed(15000000);

   Reset::initialise();
   Reset::highZ();

   // Enable SWD interface
   Swd_enable::setOutput();
   Swd_enable::high();

   // Set mode
   spi->MCR = SPI_MCR_CLR_RXF_MASK|SPI_MCR_CLR_TXF_MASK|SPI_MCR_ROOE_MASK|
         SPI_MCR_MSTR_MASK|SPI_MCR_DCONF(0)|SPI_MCR_SMPL_PT(0)|SPI_MCR_PCSIS(0);
}

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
 *  @return BDM_RC_OK => Success
 */
USBDM_ErrorCode connect(void) {
   ahb_ap_csw_defaultValue = 0;

   tx32(0xFFFFFFFF);  // 32 1's
   tx32(0x79EFFFFF);  // 20 1's + 0x79E
   tx32(0xFFFFFFFE);  // 0xE + 28 1's
   tx32(0x00FFFFFF);  // 24 1's + 8 0's

   // Target must respond to read IDCODE immediately
   uint32_t buff;
   return readReg(SWD_RD_DP_IDCODE, buff);
}

/**
 * Power up debug interface and system\n
 * Sets CSYSPWRUPREQ and CDBGPWRUPREQ\n
 * Confirms CSYSPWRUPACK and CDBGPWRUPACK
 *
 *  @return \n
 *  @return BDM_RC_OK => Success
 */
USBDM_ErrorCode powerUp() {
   USBDM_ErrorCode rc;
   rc = writeReg(SWD_WR_DP_CONTROL, SWD_WR_DP_CONTROL_POWER_REQ);
   if (rc != BDM_RC_OK) {
      return rc;
   }
   uint32_t status;
   rc = readReg(SWD_RD_DP_STATUS, status);
   if (rc != BDM_RC_OK) {
      return rc;
   }
   return ((status&SWD_WR_DP_CONTROL_POWER_ACK) == SWD_WR_DP_CONTROL_POWER_ACK)?BDM_RC_OK:BDM_RC_ARM_PWR_UP_FAIL;
}

/**
 * Resets SWD interface
 *
 *  Sequence as follows:
 *   - >=50-bit sequence of 1's (55 0's)
 *   - >=8-bit sequence of 0's  (9 1's)
 *   - Read IDCODE
 *
 *  @return BDM_RC_OK => Success, error otherwise
 */
USBDM_ErrorCode lineReset(void) {
   tx32(0xFFFFFFFF);  // 32 1's
   tx32(0x007FFFFF);  // 23 1's, 9 0's

   // Target must respond to read IDCODE immediately
   uint32_t buff;
   return readReg(SWD_RD_DP_IDCODE, buff);
}

/**
 *  Read ARM-SWD DP & AP register
 *
 *  @param command - SWD command byte to select register etc.
 *  @param data    - 32-bit value read
 *
 *  @return BDM_RC_OK               => Success        \n
 *  @return BDM_RC_ARM_FAULT_ERROR  => FAULT response from target \n
 *  @return BDM_RC_ACK_TIMEOUT      => Excessive number of WAIT responses from target \n
 *  @return BDM_RC_NO_CONNECTION    => Unexpected/no response from target \n
 *  @return BDM_RC_ARM_PARITY_ERROR => Parity error on data read
 *
 *  @note Action and Data returned depends on register (some responses are pipelined)\n
 *    SWD_RD_DP_IDCODE - Value from IDCODE reg \n
 *    SWD_RD_DP_STATUS - Value from STATUS reg \n
 *    SWD_RD_DP_RESEND - LAST value read (AP read or DP-RDBUFF), FAULT on sticky error    \n
 *    SWD_RD_DP_RDBUFF - Value from last AP read and clear READOK flag in STRL/STAT, FAULT on sticky error \n
 *    SWD_RD_AP_REGx   - Value from last AP read, clear READOK flag in STRL/STAT and INITIATE next AP read, FAULT on sticky error
 */
USBDM_ErrorCode readReg(uint8_t command, uint32_t &data) {
   int retry  = 2000;      // Set up retry count
   USBDM_ErrorCode rc;

   // Transmit command + Receive ACK (1st attempt)
   SwdAck ack = txCommand_rxAck(command);
   do {
      if (ack == SWD_ACK_OK) {
         rc = rx32_parity(data);
      }
      else if (ack == SWD_ACK_WAIT) {
         if (retry-- > 0) {
            // 1 clock turn-around on WAIT + retry
            // Turn-around + Transmit command (retry) + Receive ACK
            ack = txMark_8_rxAck(command);
            continue;
         }
         rc = BDM_RC_ACK_TIMEOUT;
      }
      else if (ack == SWD_ACK_FAULT) {
         rc = BDM_RC_ARM_FAULT_ERROR;
      }
      else {
         rc = BDM_RC_NO_CONNECTION;
      }
      break;
   } while (true);
   txIdle8();
   return rc;
}

/**
 *  Write ARM-SWD DP & AP register
 *
 *  @param command - SWD command byte to select register etc.
 *  @param data    - 32-bit value to write
 *
 *  @return BDM_RC_OK               => Success        \n
 *  @return BDM_RC_ARM_FAULT_ERROR  => FAULT response from target \n
 *  @return BDM_RC_ACK_TIMEOUT      => Excessive number of WAIT responses from target \n
 *  @return BDM_RC_NO_CONNECTION    => Unexpected/no response from target
 *
 *  @note Action depends on register (some responses are pipelined)\n
 *    SWD_WR_DP_ABORT   - Write value to ABORT register (accepted) \n
 *    SWD_WR_DP_CONTROL - Write value to CONTROL register (may be pending), FAULT on sticky error. \n
 *    SWD_WR_DP_SELECT  - Write value to SELECT register (may be pending), FAULT on sticky error. \n
 *    SWD_WR_AP_REGx    - Write to AP register.  May initiate action e.g. memory access.  Result is pending, FAULT on sticky error.
 */
USBDM_ErrorCode writeReg(uint8_t command, const uint32_t data) {
   int retry = 2000;            // Set up retry count
   SwdAck ack = txCommand_rxAck_Trn(command); // Transmit command & get ACK (1st attempt)
   USBDM_ErrorCode rc;
   do {
      if (ack == SWD_ACK_OK) {
         tx32_parity(data);
         rc = BDM_RC_OK;
      }
      else if (ack == SWD_ACK_WAIT) {
         if (retry-- > 0) {
            // 1 clock turn-around on WAIT + retry
            // Turn-around + Transmit command (retry) + rx ACK
            ack = txMark_8_rxAck_Trn(command);
            continue;
         }
         rc = BDM_RC_ACK_TIMEOUT;
      }
      else if (ack == SWD_ACK_FAULT) {
         rc = BDM_RC_ARM_FAULT_ERROR;
      }
      else {
         rc = BDM_RC_NO_CONNECTION;
      }
      break;
   } while (true);
   txIdle8();
   return rc;
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
 *  @return BDM_RC_OK => success
 *
 *  @note - Access is completed before return
 */
USBDM_ErrorCode readAPReg(const uint32_t address, uint32_t &buff) {
   static const uint8_t readAP[]  = {SWD_RD_AP_REG0,   SWD_RD_AP_REG1,    SWD_RD_AP_REG2,   SWD_RD_AP_REG3};
   USBDM_ErrorCode rc;
   uint8_t  regNo      = readAP[(address>>2)&0x3];
   uint32_t selectData = address&0xFF0000F0;

   // Set up SELECT register for AP access
   rc = writeReg(SWD_WR_DP_SELECT, selectData);
   if (rc != BDM_RC_OK) {
      return rc;
   }
   // Initiate read from AP register (dummy data)
   rc = readReg(regNo, buff);
   if (rc != BDM_RC_OK) {
      return rc;
   }
   // Read from READBUFF register
   return readReg(SWD_RD_DP_RDBUFF, buff);
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
 *  @return BDM_RC_OK => success
 *
 *  @note - Access is completed before return
 */
USBDM_ErrorCode writeAPReg(const uint32_t address, const uint32_t data) {
   static const uint8_t writeAP[] = {SWD_WR_AP_REG0,   SWD_WR_AP_REG1,    SWD_WR_AP_REG2,   SWD_WR_AP_REG3};
   USBDM_ErrorCode rc;
   uint8_t  regNo      = writeAP[(address>>2)&0x3];
   uint32_t selectData = address&0xFF0000F0;

   // Set up SELECT register for AP access
   rc = writeReg(SWD_WR_DP_SELECT, selectData);
   if (rc != BDM_RC_OK) {
      return rc;
   }
   // Initiate write to AP register
   rc = writeReg(regNo, data);
   if (rc != BDM_RC_OK) {
      return rc;
   }
   // Read from READBUFF register to allow stall/status response
   return readReg(SWD_RD_DP_RDBUFF, selectData);
}

/**
 *  Clear all sticky bits in status register
 *
 *  @return error code
 */
USBDM_ErrorCode clearStickyBits(void) {
   return writeReg(SWD_WR_DP_ABORT, SWD_DP_ABORT_CLEAR_STICKY_ERRORS);
}

/**
 * Clear sticky bits and abort AP transactions
 *
 *  @return error code
 */
USBDM_ErrorCode abortAP(void) {
   return writeReg(SWD_WR_DP_ABORT, SWD_DP_ABORT_CLEAR_STICKY_ERRORS|SWD_DP_ABORT_ABORT_AP);
}

static constexpr uint32_t  MDM_AP_STATUS                     = 0x01000000;
static constexpr uint32_t  MDM_AP_CONTROL                    = 0x01000004;
//static constexpr uint32_t  MDM_AP_IDR                        = 0x010000FC;

static constexpr uint32_t  MDM_AP_CONTROL_MASS_ERASE_REQUEST = (1<<0);
//static constexpr uint32_t  MDM_AP_CONTROL_DEBUG_REQUEST      = (1<<2);
static constexpr uint32_t  MDM_AP_CONTROL_RESET_REQUEST      = (1<<3);
//static constexpr uint32_t  MDM_AP_CONTROL_VLLDBGREQ          = (1<<5);
//static constexpr uint32_t  MDM_AP_CONTROL_VLLDBGACK          = (1<<6);
//static constexpr uint32_t  MDM_AP_CONTROL_LLS_VLLSx_ACK      = (1<<7);

//static constexpr uint32_t  MDM_AP_STATUS_FLASH_READY         = (1<<1);
static constexpr uint32_t  MDM_AP_STATUS_SECURE              = (1<<2);
static constexpr uint32_t  MDM_AP_STATUS_MASS_ERASE_ENABLE   = (1<<5);

static constexpr uint32_t  ATTEMPT_MULTIPLE                  = 100;  // How many times to attemp mass erase
static constexpr uint32_t  ERASE_MULTIPLE                    = 2;    // How many times to mass erase

/**
 * Mass erase target
 *
 * @return BDM_RC_OK if successful
 */
USBDM_ErrorCode kinetisMassErase(void) {
   unsigned successCount = 0;
   unsigned attemptCount = 0;
   uint8_t rc;
   uint32_t valueRead;

   Reset::low();
   for (;;) {
      UsbLed::on();
      if (attemptCount++>ATTEMPT_MULTIPLE) {
         break;
      }
      // Do connect sequence
      rc = connect();
      if (rc != BDM_RC_OK) {
         continue;
      }
      rc = clearStickyBits();
      if (rc != BDM_RC_OK) {
         continue;
      }
      // Power up Debug interface
      rc = powerUp();
      if (rc != BDM_RC_OK) {
         continue;
      }
      // Do mass erase
      rc = writeAPReg(MDM_AP_CONTROL, MDM_AP_CONTROL_RESET_REQUEST|MDM_AP_CONTROL_MASS_ERASE_REQUEST);
      if (rc != BDM_RC_OK) {
         continue;
      }
      // Check if mass erase commenced
      rc = readAPReg(MDM_AP_CONTROL, valueRead);
      if (rc != BDM_RC_OK) {
         continue;
      }
      if ((valueRead&MDM_AP_CONTROL_MASS_ERASE_REQUEST) == 0) {
         continue;
      }
      UsbLed::off();
      // Wait until complete
      for (int eraseWait=0; eraseWait<20; eraseWait++) {
         USBDM::waitMS(100);
         rc = readAPReg(MDM_AP_CONTROL, valueRead);
         if (rc != BDM_RC_OK) {
            continue;
         }
         if ((valueRead&MDM_AP_CONTROL_MASS_ERASE_REQUEST) == 0) {
            break;
         }
      }
      rc = readAPReg(MDM_AP_STATUS, valueRead);
      if (rc != BDM_RC_OK) {
         continue;
      }
      // Check if mass erase is disabled
      if ((valueRead&MDM_AP_STATUS_MASS_ERASE_ENABLE) == 0) {
         return BDM_RC_MASS_ERASE_DISABLED;
      }
      rc = ((valueRead&MDM_AP_STATUS_SECURE) == 0)?BDM_RC_OK:BDM_RC_FAIL;
      if (rc == BDM_RC_OK) {
         successCount++;
      }
      // Only consider successful if done ERASE_MULTIPLE times in a row
      // This prevents mass-erase attempts during power-on bounces etc.
      if (successCount>=ERASE_MULTIPLE) {
         break;
      }
   }
   return (successCount>=ERASE_MULTIPLE)?BDM_RC_OK:BDM_RC_FAIL;
}

/** Write 32-bit value to ARM-SWD Memory
 *
 *  @param address 32-bit memory address
 *  @param data    32-bit data value
 *
 *  @return BDM_RC_OK => success, error otherwise
 */
USBDM_ErrorCode writeMemoryWord(const uint32_t address, const uint32_t data) {
   USBDM_ErrorCode  rc;
   /* Steps
    *  - Set up to access AHB-AP register bank 0 (CSW,TAR,DRW)
    *  - Write AP-CSW value (auto-increment etc)
    *  - Write AP-TAR value (target memory address)
    *  - Write value to DRW (data value to target memory)
    */
   // Select AHB-AP memory bank - subsequent AHB-AP register accesses are all in the same bank
   rc = writeReg(Swd::SWD_WR_DP_SELECT, ARM_AHB_AP_BANK0);
   if (rc != BDM_RC_OK) {
      return rc;
   }
   // Write CSW (word access etc)
   rc = writeReg(Swd::SWD_WR_AHB_CSW, ahb_ap_csw_defaultValue|Swd::AHB_AP_CSW_SIZE_WORD);
   if (rc != BDM_RC_OK) {
      return rc;
   }
   // Write TAR (target address)
   rc = writeReg(Swd::SWD_WR_AHB_TAR, address);
   if (rc != BDM_RC_OK) {
      return rc;
   }
   // Write data value
   rc = writeReg(Swd::SWD_WR_AHB_DRW, data);
   if (rc != BDM_RC_OK) {
      return rc;
   }
   // Dummy read to get status
   uint32_t tt;
   return readReg(Swd::SWD_RD_DP_RDBUFF, tt);
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
 *  @return BDM_RC_OK => success, error otherwise
 */
USBDM_ErrorCode writeMemory(
      uint32_t  elementSize,  // Size of the data writes
      uint32_t  count,        // # of bytes
      uint32_t  addr,         // Address in target memory
      uint8_t   *data_ptr     // Where the data is
) {
   USBDM_ErrorCode  rc;
   uint8_t  temp[4] = {0xAA,0xAA,0xAA,0xAA,};

   /* Steps
    *  - Set up to access AHB-AP register bank 0 (CSW,TAR,DRW)
    *  - Write AP-CSW value (auto-increment etc)
    *  - Write AP-TAR value (target memory address)
    *  - Loop
    *    - Pack data
    *    - Write value to DRW (data value to target memory)
    */
   // Select AHB-AP memory bank - subsequent AHB-AP register accesses are all in the same bank
   rc = writeReg(Swd::SWD_WR_DP_SELECT, ARM_AHB_AP_BANK0);
   if (rc != BDM_RC_OK) {
      return rc;
   }
   if (ahb_ap_csw_defaultValue == 0) {
      // Read initial AHB-AP.csw register value as device dependent
      // Do posted read - dummy data returned
      rc = readReg(Swd::SWD_RD_AHB_CSW, ahb_ap_csw_defaultValue);
      if (rc != BDM_RC_OK) {
         ahb_ap_csw_defaultValue = 0;
         return rc;
      }
      // Get actual data
      rc = readReg(Swd::SWD_RD_DP_RDBUFF, ahb_ap_csw_defaultValue);
      if (rc != BDM_RC_OK) {
         ahb_ap_csw_defaultValue = 0;
         return rc;
      }
      ahb_ap_csw_defaultValue &= 0xFF000000;
      ahb_ap_csw_defaultValue |= 0x00000040;
   }
   // Write CSW (auto-increment etc)
   rc = writeReg(Swd::SWD_WR_AHB_CSW, ahb_ap_csw_defaultValue|getcswValue(elementSize));
   if (rc != BDM_RC_OK) {
      return rc;
   }
   // Write TAR (target address)
   rc = writeReg(Swd::SWD_WR_AHB_TAR, addr);
   if (rc != BDM_RC_OK) {
      return rc;
   }
   switch (elementSize) {
   case MS_Byte:
      while (count > 0) {
         switch (addr&0x3) {
         case 0: temp[3] = *data_ptr++; break;
         case 1: temp[2] = *data_ptr++; break;
         case 2: temp[1] = *data_ptr++; break;
         case 3: temp[0] = *data_ptr++; break;
         }
         rc = writeReg(Swd::SWD_WR_AHB_DRW, temp);
         if (rc != BDM_RC_OK) {
            return rc;
         }
         addr++;
         count--;
      }
      break;
   case MS_Word:
      count >>= 1;
      while (count > 0) {
         switch (addr&0x2) {
         case 0:  temp[3] = *data_ptr++;
         temp[2] = *data_ptr++; break;
         case 2:  temp[1] = *data_ptr++;
         temp[0] = *data_ptr++; break;
         }
         rc = writeReg(Swd::SWD_WR_AHB_DRW, temp);
         if (rc != BDM_RC_OK) {
            return rc;
         }
         addr  += 2;
         count--;
      }
      break;
   case MS_Long:
      count >>= 2;
      while (count-- > 0) {
         temp[3] = *data_ptr++;
         temp[2] = *data_ptr++;
         temp[1] = *data_ptr++;
         temp[0] = *data_ptr++;
         rc = writeReg(Swd::SWD_WR_AHB_DRW, temp);
         if (rc != BDM_RC_OK) {
            return rc;
         }
      }
      break;
   }
   // Dummy read to obtain status from last write
   return readReg(Swd::SWD_RD_DP_RDBUFF, temp);
}

/** Read 32-bit value from ARM-SWD Memory
 *
 *  @param address 32-bit memory address
 *  @param data    32-bit data value from _last_ read in BIG-ENDIAN order!
 *
 *  @return BDM_RC_OK => success, error otherwise
 */
USBDM_ErrorCode readMemoryWord(const uint32_t address, uint32_t &data) {
   USBDM_ErrorCode  rc;

   /* Steps
    *  - Set up to DP_SELECT to access AHB-AP register bank 0 (CSW,TAR,DRW)
    *  - Write AP-CSW value (auto-increment etc)
    *  - Write AP-TAR value (starting target memory address)
    *  - Initiate read by reading from DRW (dummy value)
    *  - Read data value from DP-READBUFF
    */
   // Select AHB-AP memory bank - subsequent AHB-AP register accesses are all in the same bank
   rc = writeReg(Swd::SWD_WR_DP_SELECT, ARM_AHB_AP_BANK0);
   if (rc != BDM_RC_OK) {
      return rc;
   }
   // Write memory access control to CSW
   rc = writeReg(Swd::SWD_WR_AHB_CSW, ahb_ap_csw_defaultValue|Swd::AHB_AP_CSW_SIZE_WORD);
   if (rc != BDM_RC_OK) {
      return rc;
   }
   // Write TAR (target address)
   rc = writeReg(Swd::SWD_WR_AHB_TAR, address);
   if (rc != BDM_RC_OK) {
      return rc;
   }
   uint32_t tt;
   // Initial read of DRW (dummy data)
   rc = readReg(Swd::SWD_RD_AHB_DRW, tt);
   if (rc != BDM_RC_OK) {
      return rc;
   }
   // Read memory data
   return readReg(Swd::SWD_RD_DP_RDBUFF, data);
}

/**  Read ARM-SWD Memory
 *
 *  @param elementSize  Size of the data elements
 *  @param count        Number of data bytes
 *  @param addr         LSB of Address in target memory
 *  @param data_ptr     Where in buffer to write the data
 *
 *  @return BDM_RC_OK => success, error otherwise
 */
USBDM_ErrorCode readMemory(uint32_t elementSize, int count, uint32_t addr, uint8_t *data_ptr) {
   USBDM_ErrorCode  rc;
   uint8_t  temp[4];

   /* Steps
    *  - Set up to DP_SELECT to access AHB-AP register bank 0 (CSW,TAR,DRW)
    *  - Write AP-CSW value (auto-increment etc)
    *  - Write AP-TAR value (starting target memory address)
    *  - Loop
    *    - Read value from DRW (data value from target memory)
    *      Note: 1st value read from DRW is discarded
    *      Note: Last value is read from DP-READBUFF
    *    - Copy to buffer adjusting byte order
    */
   if (count>MAX_COMMAND_SIZE-1) {
      return BDM_RC_ILLEGAL_PARAMS;  // requested block+status is too long to fit into the buffer
   }
#ifdef HACK
   {
      uint32_t address = (commandBuffer[4]<<24)+(commandBuffer[5]<<16)+(commandBuffer[6]<<8)+commandBuffer[7];
      memcpy(data_ptr, (void*)address, count);
      returnSize = count+1;
      return BDM_RC_OK;
   }
#else
   // Select AHB-AP memory bank - subsequent AHB-AP register accesses are all in the same bank
   rc = writeReg(Swd::SWD_WR_DP_SELECT, ARM_AHB_AP_BANK0);
   if (rc != BDM_RC_OK) {
      return rc;
   }
   if (ahb_ap_csw_defaultValue == 0) {
      // Read initial AHB-AP.csw register value as device dependent
      // Do posted read - dummy data returned
      rc = readReg(Swd::SWD_RD_AHB_CSW, ahb_ap_csw_defaultValue);
      if (rc != BDM_RC_OK) {
         ahb_ap_csw_defaultValue = 0;
         return rc;
      }
      // Get actual data
      rc = readReg(Swd::SWD_RD_DP_RDBUFF, ahb_ap_csw_defaultValue);
      if (rc != BDM_RC_OK) {
         ahb_ap_csw_defaultValue = 0;
         return rc;
      }
      ahb_ap_csw_defaultValue &= 0xFF000000;
      ahb_ap_csw_defaultValue |= 0x00000040;
   }
   // Write CSW (auto-increment etc)
   rc = writeReg(Swd::SWD_WR_AHB_CSW, ahb_ap_csw_defaultValue|getcswValue(elementSize));
   if (rc != BDM_RC_OK) {
      return rc;
   }
   // Write TAR (target address)
   rc = writeReg(Swd::SWD_WR_AHB_TAR, addr);
   if (rc != BDM_RC_OK) {
      return rc;
   }

   // Initial read of DRW (dummy data)
   rc = readReg(Swd::SWD_RD_AHB_DRW, temp);
   if (rc != BDM_RC_OK) {
      return rc;
   }
   switch (elementSize) {
   case MS_Byte:
      do {
         count--;
         if (count == 0) {
            // Read data from RDBUFF for final read
            rc = readReg(Swd::SWD_RD_DP_RDBUFF, temp);
         }
         else {
            // Start next read and collect data from last read
            rc = readReg(Swd::SWD_RD_AHB_DRW, temp);
         }
         if (rc != BDM_RC_OK) {
            return rc;
         }
         // Save data
         switch (addr&0x3) {
         case 0: *data_ptr++ = temp[3];  break;
         case 1: *data_ptr++ = temp[2];  break;
         case 2: *data_ptr++ = temp[1];  break;
         case 3: *data_ptr++ = temp[0];  break;
         }
         addr++;
      } while (count > 0);
      break;
   case MS_Word:
      count >>= 1;
      do {
         count--;
         if (count == 0) {
            // Read data from RDBUFF for final read
            rc = readReg(Swd::SWD_RD_DP_RDBUFF, temp);
         }
         else {
            // Start next read and collect data from last read
            rc = readReg(Swd::SWD_RD_AHB_DRW, temp);
         }
         if (rc != BDM_RC_OK) {
            return rc;
         }
         // Save data
         switch (addr&0x2) {
         case 0:
            *data_ptr++ = temp[3];
            *data_ptr++ = temp[2];  break;
         case 2:
            *data_ptr++ = temp[1];
            *data_ptr++ = temp[0];  break;
         }
         addr+=2;
      } while (count > 0);
      break;
   case MS_Long:
      count >>= 2;
      do {
         count--;
         if (count == 0) {
            // Read data from RDBUFF for final read
            rc = readReg(Swd::SWD_RD_DP_RDBUFF, temp);
         }
         else {
            // Start next read and collect data from last read
            rc = readReg(Swd::SWD_RD_AHB_DRW, temp);
         }
         if (rc != BDM_RC_OK) {
            return rc;
         }
         // Save data
         *data_ptr++ = temp[3];
         *data_ptr++ = temp[2];
         *data_ptr++ = temp[1];
         *data_ptr++ = temp[0];
         //      addrLSB+=4;
      } while (count > 0);
      break;
   }
   return rc;
#endif
}

/**  Write ARM-SWD Memory
 *
 *  @param elementSize  Size of the data elements
 *  @param count        Number of data bytes
 *  @param addr         LSB of Address in target memory
 *  @param data_ptr     Where in buffer to write the data
 *
 *  @return BDM_RC_OK => success, error otherwise
 */
USBDM_ErrorCode writeMemory(uint32_t elementSize, int count, uint32_t addr, uint8_t *data_ptr) {
   USBDM_ErrorCode  rc;
   uint8_t  temp[4] ={0};

   /* Steps
    *  - Set up to access AHB-AP register bank 0 (CSW,TAR,DRW)
    *  - Write AP-CSW value (auto-increment etc)
    *  - Write AP-TAR value (target memory address)
    *  - Loop
    *    - Pack data
    *    - Write value to DRW (data value to target memory)
    */
   // Select AHB-AP memory bank - subsequent AHB-AP register accesses are all in the same bank
   rc = writeReg(Swd::SWD_WR_DP_SELECT, ARM_AHB_AP_BANK0);
   if (rc != BDM_RC_OK) {
      return rc;
   }
   if (ahb_ap_csw_defaultValue == 0) {
      // Read initial AHB-AP.csw register value as device dependent
      // Do posted read - dummy data returned
      rc = readReg(Swd::SWD_RD_AHB_CSW, ahb_ap_csw_defaultValue);
      if (rc != BDM_RC_OK) {
         ahb_ap_csw_defaultValue = 0;
         return rc;
      }
      // Get actual data
      rc = readReg(Swd::SWD_RD_DP_RDBUFF, ahb_ap_csw_defaultValue);
      if (rc != BDM_RC_OK) {
         ahb_ap_csw_defaultValue = 0;
         return rc;
      }
      ahb_ap_csw_defaultValue &= 0xFF000000;
      ahb_ap_csw_defaultValue |= 0x00000040;
   }
   // Write CSW (auto-increment etc)
   rc = writeReg(Swd::SWD_WR_AHB_CSW, ahb_ap_csw_defaultValue|getcswValue(elementSize));
   if (rc != BDM_RC_OK) {
      return rc;
   }
   // Write TAR (target address)
   rc = writeReg(Swd::SWD_WR_AHB_TAR, addr);
   if (rc != BDM_RC_OK) {
      return rc;
   }
   switch (elementSize) {
   case MS_Byte:
      while (count > 0) {
         switch (addr&0x3) {
         case 0:
            temp[3] = *data_ptr++;
            break;
         case 1:
            temp[2] = *data_ptr++;
            break;
         case 2:
            temp[1] = *data_ptr++;
            break;
         case 3:
            temp[0] = *data_ptr++;
            break;
         }
         rc = writeReg(Swd::SWD_WR_AHB_DRW, temp);
         if (rc != BDM_RC_OK) {
            return rc;
         }
         addr++;
         count--;
      }
      break;
   case MS_Word:
      count >>= 1;
      while (count > 0) {
         switch (addr&0x2) {
         case 0:
            temp[3] = *data_ptr++;
            temp[2] = *data_ptr++;
            break;
         case 2:
            temp[1] = *data_ptr++;
            temp[0] = *data_ptr++;
            break;
         }
         rc = writeReg(Swd::SWD_WR_AHB_DRW, temp);
         if (rc != BDM_RC_OK) {
            return rc;
         }
         addr  += 2;
         count--;
      }
      break;
   case MS_Long:
      count >>= 2;
      while (count-- > 0) {
         temp[3] = *data_ptr++;
         temp[2] = *data_ptr++;
         temp[1] = *data_ptr++;
         temp[0] = *data_ptr++;
         rc = writeReg(Swd::SWD_WR_AHB_DRW, temp);
         if (rc != BDM_RC_OK) {
            return rc;
         }
      }
      break;
   }
   // Dummy read to obtain status from last write
   return readReg(Swd::SWD_RD_DP_RDBUFF, temp);
}

/**
 *  Initiates core register operation (read/write) and waits for completion
 *
 *  @param dcrsrValue - value to write to DCSRD register to control operation
 *
 *  @return BDM_RC_OK               Success
 *  @return BDM_RC_TARGET_BUSY      Register is inaccessible as processor is not in debug mode
 *  @return BDM_RC_ARM_ACCESS_ERROR Failed access
 */
static USBDM_ErrorCode coreRegisterOperation(uint32_t dcrsrValue) {
   int retryCount = 40;
   USBDM_ErrorCode rc;

   // Write operation+regNo
   rc = writeMemoryWord(DCRSR_ADDR, dcrsrValue);
   if (rc != BDM_RC_OK) {
      return rc;
   }
   // Wait for transfer complete
   uint32_t dhcsrValue;
   do {
      if (retryCount-- == 0) {
         // Assume target busy
         return BDM_RC_ARM_ACCESS_ERROR;
      }
      // Check complete (use dcrsrValue as scratch)
      rc = readMemoryWord(DHCSR_ADDR, dhcsrValue);
      if (rc != BDM_RC_OK) {
         return rc;
      }
      if ((dhcsrValue & DHCSR_C_HALT) == 0) {
         // Target must be in DEBUG mode
         return BDM_RC_TARGET_BUSY;
      }
   } while ((dhcsrValue & DHCSR_S_REGRDY) == 0);
   return BDM_RC_OK;
}

/**
 *  Read target register
 *
 *  @param regNo    Number of register to read
 *  @param data     Register value as 32-bit data value in BIG-ENDIAN order
 *
 *  @return BDM_RC_OK               Success
 *  @return BDM_RC_TARGET_BUSY      Register is inaccessible as processor is not in debug mode
 *  @return BDM_RC_ARM_ACCESS_ERROR Failed access
 */
USBDM_ErrorCode readCoreRegister(uint8_t regNo, uint8_t data[4]) {
   USBDM_ErrorCode rc;
   // Execute register transfer command
   rc = coreRegisterOperation(DCRSR_READ|regNo);
   if (rc != BDM_RC_OK) {
      return rc;
   }
   // Read register value from DCRDR holding register (Big-endian)
   return readMemoryWord(DCRDR_ADDR, data);
}

/**
 *  Write ARM-SWD core register
 *
 *  @param regNo  Register number
 *  @param data   Register value as 32-bit data value in BIG-ENDIAN order
 *
 *  @return BDM_RC_OK               Success
 *  @return BDM_RC_TARGET_BUSY      Register is inaccessible as processor is not in debug mode
 *  @return BDM_RC_ARM_ACCESS_ERROR Failed access
 */
USBDM_ErrorCode writeCoreReg(uint32_t regNo, uint8_t data[4]) {
   USBDM_ErrorCode rc;

   // Write data value to DCRDR holding register
   rc = writeMemoryWord(DCRDR_ADDR, data);
   if (rc != BDM_RC_OK) {
      return rc;
   }
   // Execute register transfer
   return coreRegisterOperation(DCRSR_WRITE|regNo);
}

/**
 *  Modifies value in DHCSR
 *  DHCSR.lsb = (DHCSR&preserveBits)|setBits
 *
 *  @param preserveBits Bits to preserve (done first)
 *  @param setBits      Bits to set (done last)
 *
 *  @return BDM_RC_OK => success, error otherwise
 */
USBDM_ErrorCode modifyDHCSR(uint8_t preserveBits, uint8_t setBits) {
   uint32_t debugStepValue;
   USBDM_ErrorCode rc = readMemoryWord(DHCSR_ADDR, debugStepValue);
   if (rc != BDM_RC_OK) {
      return rc;
   }
   debugStepValue = (debugStepValue&preserveBits) | DHCSR_DBGKEY | setBits;
   return writeMemoryWord(DHCSR_ADDR, debugStepValue);
}

}; // End namespace Swd
