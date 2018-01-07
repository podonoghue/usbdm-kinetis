/*! \file
    \brief Header file for BDM MACROS

   Change History
+================================================================================================
| 18 Jul 2014 | Added HCS12ZVM support                                             - pgo V4.10.6.170
+================================================================================================
\endverbatim
 */
#ifndef _BDMMACROS_H_
#define _BDMMACROS_H_

namespace Bdm {

// Hardware commands
constexpr uint8_t _BDM_BACKGROUND           = (0x90);
constexpr uint8_t _BDM_ACK_ENABLE           = (0xD5);
constexpr uint8_t _BDM_ACK_DISABLE          = (0xD6);
constexpr uint8_t _BDM_READ_BYTE            = (0xE0);
constexpr uint8_t _BDM_WRITE_BYTE           = (0xC0);
//constexpr uint8_t _BDM_WRITE_BLOCK          = (0x88);

// HC/S12= (x); hardware commands
constexpr uint8_t _BDM12_READ_BD_BYTE       = (0xE4);
constexpr uint8_t _BDM12_READ_BD_WORD       = (0xEC);
constexpr uint8_t _BDM12_READ_WORD          = (0xE8);
constexpr uint8_t _BDM12_WRITE_BD_BYTE      = (0xC4);
constexpr uint8_t _BDM12_WRITE_BD_WORD      = (0xCC);
constexpr uint8_t _BDM12_WRITE_WORD         = (0xC8);

// HCS08 'hardware' non-intrusive commands
constexpr uint8_t _BDM08_READ_STATUS        = (0xE4);
constexpr uint8_t _BDM08_WRITE_CONTROL      = (0xC4);
constexpr uint8_t _BDM08_READ_BYTE_WS       = (0xE1);
constexpr uint8_t _BDM08_READ_LAST          = (0xE8);
constexpr uint8_t _BDM08_WRITE_BYTE_WS      = (0xC1);
constexpr uint8_t _BDM08_READ_BKPT          = (0xE2);
constexpr uint8_t _BDM08_WRITE_BKPT         = (0xC2);

// Firmware commands
constexpr uint8_t _BDM_GO                   = (0x08);
constexpr uint8_t _BDM_TRACE1               = (0x10);
constexpr uint8_t _BDM_TAGGO                = (0x18);

// HCS08 'firmware' = (active background mode commands);
constexpr uint8_t _BDM08_READ_REG           = (0x60); // Add reg #
constexpr uint8_t _BDM08_READ_A             = (0x68);
constexpr uint8_t _BDM08_READ_CCR           = (0x69);
constexpr uint8_t _BDM08_READ_PC            = (0x6B);
constexpr uint8_t _BDM08_READ_HX            = (0x6C);
constexpr uint8_t _BDM08_READ_SP            = (0x6F);
constexpr uint8_t _BDM08_READ_NEXT          = (0x70);
constexpr uint8_t _BDM08_READ_NEXT_WS       = (0x71);
constexpr uint8_t _BDM08_WRITE_REG          = (0x40); // Add reg #
constexpr uint8_t _BDM08_WRITE_A            = (0x48);
constexpr uint8_t _BDM08_WRITE_CCR          = (0x49);
constexpr uint8_t _BDM08_WRITE_PC           = (0x4B);
constexpr uint8_t _BDM08_WRITE_HX           = (0x4C);
constexpr uint8_t _BDM08_WRITE_SP           = (0x4F);
constexpr uint8_t _BDM08_WRITE_NEXT         = (0x50);
constexpr uint8_t _BDM08_NEXT_WS            = (0x51);

constexpr uint8_t _BDMRS08_WRITE_SPC        = (0x4F);

// HC/S12= (x); firmware commands
constexpr uint8_t _BDM12_READ_NEXT          = (0x62);
constexpr uint8_t _BDM12_READ_REG           = (0x60); // Add reg #
constexpr uint8_t _BDM12_READ_PC            = (0x63);
constexpr uint8_t _BDM12_READ_D             = (0x64);
constexpr uint8_t _BDM12_READ_X             = (0x65);
constexpr uint8_t _BDM12_READ_Y             = (0x66);
constexpr uint8_t _BDM12_READ_SP            = (0x67);
constexpr uint8_t _BDM12_WRITE_NEXT         = (0x42);
constexpr uint8_t _BDM12_WRITE_REG          = (0x40); // Add reg #
constexpr uint8_t _BDM12_WRITE_PC           = (0x43);
constexpr uint8_t _BDM12_WRITE_D            = (0x44);
constexpr uint8_t _BDM12_WRITE_X            = (0x45);
constexpr uint8_t _BDM12_WRITE_Y            = (0x46);
constexpr uint8_t _BDM12_WRITE_SP           = (0x47);
constexpr uint8_t _BDM12_GO_UNTIL           = (0x0C);

// RS08 BDM Commands
//=======================================
constexpr uint8_t RS_BDC_RESET = (0x18);

// 9S12ZV  Commands
//=======================================
constexpr uint8_t _BDMZ12_ACK_DISABLE        = (0x03);
constexpr uint8_t _BDMZ12_ACK_ENABLE         = (0x02);
constexpr uint8_t _BDMZ12_BACKGROUND         = (0x04);
constexpr uint8_t _BDMZ12_DUMP_MEM           = (0x32);
constexpr uint8_t _BDMZ12_DUMP_MEM_WS        = (0x33);
constexpr uint8_t _BDMZ12_FILL_MEM           = (0x12);
constexpr uint8_t _BDMZ12_FILL_MEM_WS        = (0x13);
constexpr uint8_t _BDMZ12_GO                 = (0x08);
constexpr uint8_t _BDMZ12_GO_UNTIL           = (0x0C);
constexpr uint8_t _BDMZ12_NOP                = (0x00);
constexpr uint8_t _BDMZ12_READ_Rn            = (0x60);
constexpr uint8_t _BDMZ12_READ_MEM           = (0x30);
constexpr uint8_t _BDMZ12_READ_MEM_WS        = (0x31);
constexpr uint8_t _BDMZ12_READ_DBGTB         = (0x07);
constexpr uint8_t _BDMZ12_READ_SAME          = (0x54);
constexpr uint8_t _BDMZ12_READ_SAME_WS       = (0x55);
constexpr uint8_t _BDMZ12_READ_BDCCSR        = (0x2D);
constexpr uint8_t _BDMZ12_SYNC_PC            = (0x01);
constexpr uint8_t _BDMZ12_WRITE_MEM          = (0x10);
constexpr uint8_t _BDMZ12_WRITE_MEM_WS       = (0x11);
constexpr uint8_t _BDMZ12_WRITE_Rn           = (0x40);
constexpr uint8_t _BDMZ12_WRITE_BDCCSR       = (0x0D);
constexpr uint8_t _BDMZ12_ERASE_FLASH        = (0x95);
constexpr uint8_t _BDMZ12_TRACE1             = (0x09);

constexpr uint8_t _BDMZ12_SZ_BYTE            = (0x0<<2);
constexpr uint8_t _BDMZ12_SZ_WORD            = (0x1<<2);
constexpr uint8_t _BDMZ12_SZ_LONG            = (0x2<<2);

// Coldfire V1  Commands
//=======================================
constexpr uint8_t _BDMCF_ACK_DISABLE        = (0x03);
constexpr uint8_t _BDMCF_ACK_ENABLE         = (0x02);
constexpr uint8_t _BDMCF_BACKGROUND         = (0x04);
constexpr uint8_t _BDMCF_DUMP_MEM           = (0x32);
constexpr uint8_t _BDMCF_DUMP_MEM_WS        = (0x33);
constexpr uint8_t _BDMCF_FILL_MEM           = (0x12);
constexpr uint8_t _BDMCF_FILL_MEM_WS        = (0x13);
constexpr uint8_t _BDMCF_GO                 = (0x08);
constexpr uint8_t _BDMCF_NOP                = (0x00);
constexpr uint8_t _BDMCF_READ_CREG          = (0xE0);
constexpr uint8_t _BDMCF_READ_DREG          = (0xA0);
constexpr uint8_t _BDMCF_READ_MEM           = (0x30);
constexpr uint8_t _BDMCF_READ_MEM_WS        = (0x31);
constexpr uint8_t _BDMCF_READ_PSTB          = (0x50);
constexpr uint8_t _BDMCF_READ_Rn            = (0x60);
constexpr uint8_t _BDMCF_READ_XCSR_BYTE     = (0x2D);
constexpr uint8_t _BDMCF_READ_CSR2_BYTE     = (0x2E);
constexpr uint8_t _BDMCF_READ_CSR3_BYTE     = (0x2F);
constexpr uint8_t _BDMCF_SYNC_PC            = (0x01);
constexpr uint8_t _BDMCF_WRITE_CREG         = (0xC0);
constexpr uint8_t _BDMCF_WRITE_DREG         = (0x80);
constexpr uint8_t _BDMCF_WRITE_MEM          = (0x10);
constexpr uint8_t _BDMCF_WRITE_MEM_WS       = (0x11);
constexpr uint8_t _BDMCF_WRITE_Rn           = (0x40);
constexpr uint8_t _BDMCF_WRITE_XCSR_BYTE    = (0x0D);
constexpr uint8_t _BDMCF_WRITE_CSR2_BYTE    = (0x0E);
constexpr uint8_t _BDMCF_WRITE_CSR3_BYTE    = (0x0F);

constexpr uint8_t _BDMCF_SZ_BYTE            = (0x0<<2);
constexpr uint8_t _BDMCF_SZ_WORD            = (0x1<<2);
constexpr uint8_t _BDMCF_SZ_LONG            = (0x2<<2);

/* Definitions of the actual commands - these should be used in the C-code
 *
 * All BDM commands need to be called with interrupts disabled (either by hand or, for example, from within an ISR);
 *    value   is 8 or 16 bit value,
 *    addr    is 16 bit address,
 *    addr24  is 24-bit address
 *    value_p is pointer to 8 or 16 bit variable
 */

// FIRMWARE Commands
//! Target Go (HC12,HCS08,RS08)
inline USBDM_ErrorCode BDM_CMD_GO()                                          { return cmd_0_0(_BDM_GO);         }
//! Target Go until (HC12)
inline USBDM_ErrorCode BDM12_CMD_GO_UNTIL()                                  { return cmd_0_0(_BDM12_GO_UNTIL); }
//! Target Trace a single instruction (HC12,HCS08,RS08)
inline USBDM_ErrorCode  BDM_CMD_TRACE1()                                     { return cmd_0_0(_BDM_TRACE1);     }
//!  Target Go with tagging (HC12?)
inline USBDM_ErrorCode  BDM_CMD_TAGGO()                                      { return cmd_0_0(_BDM_TAGGO);      }

// Write memory using X as a pointer with automatic pre-increment (HC12)
inline USBDM_ErrorCode  BDM12_CMD_WRITE_NEXT(uint8_t value)                  { return cmd_1W_0(_BDM12_WRITE_NEXT,value); }

// Write register commands
//! Write REG (HC12)
inline USBDM_ErrorCode  BDM12_CMD_WRITE_REG(uint8_t reg, uint16_t value)     { return cmd_1W_0(_BDM12_WRITE_REG+reg, value); }
//! Write PC (HC12)
inline USBDM_ErrorCode  BDM12_CMD_WRITE_PC(uint16_t value)                   { return cmd_1W_0(_BDM12_WRITE_PC, value);      }
//! Write D(HC12)
inline USBDM_ErrorCode  BDM12_CMD_WRITE_D(uint16_t value)                    { return cmd_1W_0(_BDM12_WRITE_D, value);       }
//! Write X (HC12)
inline USBDM_ErrorCode  BDM12_CMD_WRITE_X(uint16_t value)                    { return cmd_1W_0(_BDM12_WRITE_X, value);       }
//! Write Y (HC12)
inline uint8_t BDM12_CMD_WRITE_Y(uint16_t value)                             { return cmd_1W_0(_BDM12_WRITE_Y, value);       }
//! Write SP (HC12)
inline USBDM_ErrorCode  BDM12_CMD_WRITE_SP(uint16_t value)                   { return cmd_1W_0(_BDM12_WRITE_SP, value);      }

//! Read memory using X as a pointer with automatic pre-increment (HC12)
inline USBDM_ErrorCode  BDM12_CMD_READ_NEXT(uint8_t *value_p)                { return cmd_0_1W(_BDM12_READ_NEXT, value_p);   }

// Read register commands
//! Read REG (HC12)
inline USBDM_ErrorCode  BDM12_CMD_READ_REG(uint8_t reg, uint8_t *value_p)   { return cmd_0_1W(_BDM12_READ_REG+reg, value_p); }
//! Read PC (HC12)
inline USBDM_ErrorCode  BDM12_CMD_READ_PC(uint8_t *value_p)                 { return cmd_0_1W(_BDM12_READ_PC, value_p);      }
//! Read D (HC12)
inline USBDM_ErrorCode  BDM12_CMD_READ_D(uint8_t *value_p)                  { return cmd_0_1W(_BDM12_READ_D, value_p);       }
//! Read X (HC12)
inline USBDM_ErrorCode  BDM12_CMD_READ_X(uint8_t *value_p)                  { return cmd_0_1W(_BDM12_READ_X, value_p);       }
//! Read Y (HC12)
inline USBDM_ErrorCode  BDM12_CMD_READ_Y(uint8_t *value_p)                  { return cmd_0_1W(_BDM12_READ_Y, value_p);       }
//! Read SP (HC12)
inline USBDM_ErrorCode  BDM12_CMD_READ_SP(uint8_t *value_p)                 { return cmd_0_1W(_BDM12_READ_SP, value_p);      }

// Hardware commands
//! Enable ACKN (HC12,HCS08,RS08)
inline USBDM_ErrorCode  BDM_CMD_ACK_ENABLE()                                 { return cmd_0_0(_BDM_ACK_ENABLE);  }
//! Disable ACKN  (HC12,HCS08) - Does timeout on ACKN
inline USBDM_ErrorCode  BDM_CMD_ACK_DISABLE()                                { return cmd_0_0(_BDM_ACK_DISABLE); }
//! Halt Target (HC12,HCS08,RS08)
inline USBDM_ErrorCode  BDM_CMD_BACKGROUND()                                 { return cmd_0_0(_BDM_BACKGROUND);  }

// Read and write commands
//! Write 16-bit value (HC12)
inline USBDM_ErrorCode  BDM12_CMD_WRITEW(uint16_t addr, uint16_t value)      { return cmd_2W_0(_BDM12_WRITE_WORD, addr, value);   }
//! Write 8-bit value (HC12)
inline USBDM_ErrorCode  BDM12_CMD_WRITEB(uint16_t addr, uint8_t value)       { return cmd_2WB_0(_BDM_WRITE_BYTE, addr, value);    }
//! Read 16-bit value (HC12)
inline USBDM_ErrorCode  BDM12_CMD_READW(uint16_t addr,  uint8_t *value_p)    { return cmd_1W_1W(_BDM12_READ_WORD, addr, value_p); }
//! Read 8-bit value (HC12)
inline USBDM_ErrorCode  BDM12_CMD_READB(uint16_t addr,  uint8_t *value_p)    { return cmd_1W_1WB(_BDM_READ_BYTE, addr, value_p);  }

// Read and writes from/to the BDM memory space
//! Write 16-bit value  BDM address space (HC12)
inline USBDM_ErrorCode  BDM12_CMD_BDWRITEW(uint16_t addr, uint16_t value)    { return cmd_2W_0(_BDM12_WRITE_BD_WORD, addr, value);    }
//! Write 8-bit value  BDM address space (HC12)
inline USBDM_ErrorCode  BDM12_CMD_BDWRITEB(uint16_t addr, uint8_t value)     { return cmd_2WB_0(_BDM12_WRITE_BD_BYTE, addr, value);   }
//! Read 16-bit value  BDM address space (HC12)
inline USBDM_ErrorCode  BDM12_CMD_BDREADW(uint16_t addr,  uint8_t *value_p)  { return cmd_1W_1W(_BDM12_READ_BD_WORD, addr, value_p);  }
//! Read 8-bit value  BDM address space(HC12)
inline USBDM_ErrorCode  BDM12_CMD_BDREADB(uint16_t addr,  uint8_t *value_p)  { return cmd_1W_1WB(_BDM12_READ_BD_BYTE, addr, value_p); }

// Read register commands
//! Read Status Register(HCS08,RS08)
inline void BDM08_CMD_READSTATUS(uint8_t *value_p)               { cmd_0_1B_NOACK(_BDM08_READ_STATUS, value_p); }
//! Write Control Register (HCS08,RS08)
inline void BDM08_CMD_WRITECONTROL(uint8_t value)                { cmd_1B_0_NOACK(_BDM08_WRITE_CONTROL, value); }

// Memory read/write
//! Read 8-bit memory value (HCS08,RS08)
inline USBDM_ErrorCode  BDM08_CMD_READB(uint16_t addr, uint8_t *value_p)                  { return cmd_1W_1B(_BDM_READ_BYTE, addr, value_p);            }
//! Write 8-bit memory value (HCS08,RS08)
inline USBDM_ErrorCode  BDM08_CMD_WRITEB(uint16_t addr, uint8_t value)                    { return cmd_1W1B_0(_BDM_WRITE_BYTE, addr, value);            }
//! Write memory using ++H:X as a pointer
inline USBDM_ErrorCode  BDM08_CMD_WRITE_NEXT(uint8_t value)                               { return cmd_1B_0(_BDM08_WRITE_NEXT, value);                  }
//! Read memory using ++H:X as a pointer
inline USBDM_ErrorCode  BDM08_CMD_READ_NEXT(uint8_t *value_p)                             { return cmd_0_1B(_BDM08_READ_NEXT, value_p);                 }

//! Read 8-bit memory value with status (HCS08)
inline void BDM08_CMD_READB_WS(uint16_t addr, uint8_t *val_stat_p)               { cmd_1W_1W_NOACK(_BDM08_READ_BYTE_WS, addr, val_stat_p);     }
//! Write 8-bit memory value with status (HCS08)
inline void BDM08_CMD_WRITEB_WS(uint16_t addr, uint8_t val, uint8_t *stat_p)     { cmd_1W1B_1B_NOACK(_BDM08_WRITE_BYTE_WS, addr, val, stat_p); }
//! Read last 8-bit memory location accessed with status (HCS08)
inline void BDM08_CMD_READ_LAST(uint8_t *val_stat_p)                             { cmd_0_1W_NOACK(_BDM08_READ_LAST, val_stat_p); }

//! Write Block (HCS08)
//inline uint8_t BDM08_CMD_WRITEBLOCK(uint16_t addr, uint8_t value)              { return cmd_1W1B_0(_BDM_WRITE_BLOCK,addr,value); }

//! Reset Target (HCS08)
inline void BDM08_CMD_RESET(uint16_t addr, uint8_t val)              { cmd_1W1B_0_T(_BDM_WRITE_BYTE, addr, val); }
//! Reset Target (RS08)
inline void BDMRS08_CMD_RESET()                                      { cmd_0_0_T(RS_BDC_RESET); }

// Read register commands
//! Read PC (HC08)
inline USBDM_ErrorCode  BDM08_CMD_READ_PC(uint8_t *value_p)          { return cmd_0_1W(_BDM08_READ_PC, value_p);  }
//! Read SP (HC08)
inline USBDM_ErrorCode  BDM08_CMD_READ_SP(uint8_t *value_p)          { return cmd_0_1W(_BDM08_READ_SP, value_p);  }
//! Read HX (HC08)
inline USBDM_ErrorCode  BDM08_CMD_READ_HX(uint8_t *value_p)          { return cmd_0_1W(_BDM08_READ_HX, value_p);  }
//! Read A (HC08)
inline USBDM_ErrorCode  BDM08_CMD_READ_A(uint8_t *value_p)           { return cmd_0_1B(_BDM08_READ_A, value_p);   }
//! Read CCR (HC08)
inline USBDM_ErrorCode  BDM08_CMD_READ_CCR(uint8_t *value_p)         { return cmd_0_1B(_BDM08_READ_CCR, value_p); }
//! Read BKPT (HC08) - No ACK fix - pgo
inline void BDM08_CMD_READ_BKPT(uint8_t *value_p)                    { cmd_0_1W_NOACK(_BDM08_READ_BKPT, value_p); }

// Write register commands
//! Write PC (HC08)
inline USBDM_ErrorCode BDM08_CMD_WRITE_PC(uint16_t value)           { return cmd_1W_0(_BDM08_WRITE_PC, value);    }
//! Write SP (HC08)
inline USBDM_ErrorCode BDM08_CMD_WRITE_SP(uint16_t value)           { return cmd_1W_0(_BDM08_WRITE_SP, value);    }
//! Write HX (HC08)
inline USBDM_ErrorCode BDM08_CMD_WRITE_HX(uint16_t value)           { return cmd_1W_0(_BDM08_WRITE_HX, value);    }
//! Write A (HC08)
inline USBDM_ErrorCode BDM08_CMD_WRITE_A(uint8_t value)             { return cmd_1B_0(_BDM08_WRITE_A, value);     }
//! Write CCR (HC08)
inline USBDM_ErrorCode BDM08_CMD_WRITE_CCR(uint8_t value)           { return cmd_1B_0(_BDM08_WRITE_CCR, value);   }
//! Write Breakpoint (HC08) - No ACK, despite what the manual implies!
inline void BDM08_CMD_WRITE_BKPT(uint16_t value)                    { cmd_1W_0_NOACK(_BDM08_WRITE_BKPT, value);   }
//! Write Shadow PC (RS08)
inline USBDM_ErrorCode BDMRS08_CMD_WRITE_SPC(uint16_t value)        { return cmd_1W_0(_BDMRS08_WRITE_SPC, value); }

/*
 * 
 */
//! Read BDCCSR - No ACK
inline void BDMZ12_CMD_READ_BDCCSR(uint8_t *value_p)                { cmd_0_1W_NOACK(_BDMZ12_READ_BDCCSR, value_p);  }
//! Read BDCCSR - No ACK
inline void BDMZ12_CMD_WRITE_BDCCSR(uint16_t value_p)               { cmd_1W_0_NOACK(_BDMZ12_WRITE_BDCCSR, value_p); }

//! Trace a single instruction
inline USBDM_ErrorCode BDMZ12_CMD_TRACE1()                          { return cmd_0_0(_BDMZ12_TRACE1); }

// Coldfire V1 Commands
//! Enable ACKN (CFv1)
inline USBDM_ErrorCode BDMCF_CMD_ACK_ENABLE()                       { return cmd_0_0(_BDMCF_ACK_ENABLE);  }
//! Disable ACKN (Expects ACK but times out) (CFv1)
inline USBDM_ErrorCode BDMCF_CMD_ACK_DISABLE()                      { return cmd_0_0(_BDMCF_ACK_DISABLE); }
//! Halt Target (CFv1)
inline USBDM_ErrorCode BDMCF_CMD_BACKGROUND()                       { return cmd_0_0(_BDMCF_BACKGROUND);  }
//! Target Go (CFv1)
inline USBDM_ErrorCode BDMCF_CMD_GO()                               { return cmd_0_0(_BDMCF_GO);          }
//! No Operation (CFv1)
inline USBDM_ErrorCode BDMCF_CMD_NOP()                              { return cmd_0_0(_BDMCF_NOP);         }
//! Sync PC (CFv1)
inline USBDM_ErrorCode BDMCF_CMD_SYNC_PC()                          { return cmd_0_0(_BDMCF_SYNC_PC);     }

//! Read Register (CFv1)
inline USBDM_ErrorCode BDMCF_CMD_READ_REG(uint8_t regNo, uint8_t value_p[4])              { return cmd_0_1L(_BDMCF_READ_Rn|regNo,value_p);   }
//! Read Control Register (CFv1)
inline USBDM_ErrorCode BDMCF_CMD_READ_CREG(uint8_t regNo, uint8_t  value_p[4])            { return cmd_0_1L(_BDMCF_READ_CREG|regNo,value_p); }
//! Read Debug Register (CFv1)
inline USBDM_ErrorCode BDMCF_CMD_READ_DREG(uint8_t regNo, uint8_t  value_p[4])            { return cmd_0_1L(_BDMCF_READ_DREG|regNo,value_p); }
//! Read Debug Register (CFv1)
inline USBDM_ErrorCode BDMCF_CMD_READ_DREG(uint8_t regNo, uint32_t &value) {
   uint8_t temp[4];
   USBDM_ErrorCode rc = cmd_0_1L(_BDMCF_READ_DREG|regNo, temp);
   value = pack32BE(temp);
   return rc;
}
//! Read Trace buffer (CFv1)
inline USBDM_ErrorCode BDMCF_CMD_READ_PSTBe(USBDM_ErrorCode regNo, uint8_t  value_p[4])   { return cmd_0_1L(_BDMCF_READ_PSTB|regNo,value_p); }

//! Read XCSR.msb (CFv1) - No ACK
inline void BDMCF_CMD_READ_XCSR(uint8_t *value_p)                       { cmd_0_1B_NOACK(_BDMCF_READ_XCSR_BYTE,value_p); }
//! Read CSR2.msb (CFv1) - No ACK
inline void BDMCF_CMD_READ_CSR2(uint8_t *value_p)                       { cmd_0_1B_NOACK(_BDMCF_READ_CSR2_BYTE,value_p); }
//! Read CSR3.msb (CFv1) - No ACK
inline void BDMCF_CMD_READ_CSR3(uint8_t *value_p)                       { cmd_0_1B_NOACK(_BDMCF_READ_CSR3_BYTE,value_p); }

//! Write Register (CFv1)
inline USBDM_ErrorCode BDMCF_CMD_WRITE_REG(uint8_t regNo, uint32_t value)       { return cmd_1L_0(_BDMCF_WRITE_Rn|regNo,value);   }
//! Write Control Register (CFv1)
inline USBDM_ErrorCode BDMCF_CMD_WRITE_CREG(uint8_t regNo, uint32_t value)      { return cmd_1L_0(_BDMCF_WRITE_CREG|regNo,value); }
//! Write Debug Register (CFv1)
inline USBDM_ErrorCode BDMCF_CMD_WRITE_DREG(uint8_t regNo, uint32_t value)      { return cmd_1L_0(_BDMCF_WRITE_DREG|regNo,value); }

//! Write XCSR.msb (CFv1) - No ACK
inline void BDMCF_CMD_WRITE_XCSR(uint8_t value)                         { cmd_1B_0_NOACK(_BDMCF_WRITE_XCSR_BYTE,value); }
//! Write CSR2.msb (CFv1) - No ACK
inline void BDMCF_CMD_WRITE_CSR2(uint8_t value)                         { cmd_1B_0_NOACK(_BDMCF_WRITE_CSR2_BYTE,value); }
//! Write CSR3.msb (CFv1) - No ACK
inline void BDMCF_CMD_WRITE_CSR3(uint8_t value)                         { cmd_1B_0_NOACK(_BDMCF_WRITE_CSR3_BYTE,value); }

////! Read 8-bit memory value with return status (CFv1)
//inline USBDM_ErrorCode BDMCF_CMD_READ_MEM_S(uint32_t addr24, uint8_t *value_p) { return cmd_1A_CS_1B(_BDMCF_READ_MEM_WS|_BDMCF_SZ_BYTE,addr24,value_p); }

//! Read 8-bit memory value (CFv1)
inline USBDM_ErrorCode BDMCF_CMD_READ_MEM_B(uint32_t addr24, uint8_t *value_p)  { return cmd_1A_1B(_BDMCF_READ_MEM|_BDMCF_SZ_BYTE,addr24,value_p); }
////! Read 16-bit memory value (CFv1)
inline USBDM_ErrorCode BDMCF_CMD_READ_MEM_W(uint32_t addr24, uint8_t *value_p)  { return cmd_1A_1W(_BDMCF_READ_MEM|_BDMCF_SZ_WORD,addr24,value_p); }
////! Read 32-bit memory value (CFv1)
inline USBDM_ErrorCode BDMCF_CMD_READ_MEM_L(uint32_t addr24, uint8_t *value_p)  { return cmd_1A_1L(_BDMCF_READ_MEM|_BDMCF_SZ_LONG,addr24,value_p); }

//! Read consecutive 8-bit memory value (CFv1)
inline USBDM_ErrorCode BDMCF_CMD_DUMP_MEM_B(uint8_t value_p[1])                  { return cmd_0_1B(_BDMCF_DUMP_MEM|_BDMCF_SZ_BYTE, value_p); }
//! Read consecutive 16-bit memory value (CFv1)
inline USBDM_ErrorCode BDMCF_CMD_DUMP_MEM_W(uint8_t value_p[2])                  { return cmd_0_1W(_BDMCF_DUMP_MEM|_BDMCF_SZ_WORD, value_p); }
//! Read consecutive 32-bit memory value (CFv1)
inline USBDM_ErrorCode BDMCF_CMD_DUMP_MEM_L(uint8_t value_p[4])                  { return cmd_0_1L(_BDMCF_DUMP_MEM|_BDMCF_SZ_LONG, value_p); }

//! Write 8-bit memory value (CFv1)
inline USBDM_ErrorCode BDMCF_CMD_WRITE_MEM_B(uint32_t addr24, uint8_t value)    { return cmd_1A1B_0(_BDMCF_WRITE_MEM|_BDMCF_SZ_BYTE, addr24, value); }
//! Write 16-bit memory value (CFv1)
inline USBDM_ErrorCode BDMCF_CMD_WRITE_MEM_W(uint32_t addr24, uint16_t value)   { return cmd_1A1W_0(_BDMCF_WRITE_MEM|_BDMCF_SZ_WORD, addr24, value); }
//! Write 32-bit memory value (CFv1)
inline USBDM_ErrorCode BDMCF_CMD_WRITE_MEM_L(uint32_t addr24, uint32_t value)   { return cmd_1A1L_0(_BDMCF_WRITE_MEM|_BDMCF_SZ_LONG, addr24, value); }

//! Write consecutive 8-bit memory value (CFv1)
inline USBDM_ErrorCode BDMCF_CMD_FILL_MEM_B(uint8_t value)                      { return cmd_1B_0(_BDMCF_FILL_MEM|_BDMCF_SZ_BYTE, value); }
//! Write consecutive 16-bit memory value (CFv1)
inline USBDM_ErrorCode BDMCF_CMD_FILL_MEM_W(uint16_t value)                     { return cmd_1W_0(_BDMCF_FILL_MEM|_BDMCF_SZ_WORD, value); }
//! Write consecutive 32-bit memory value (CFv1)
inline USBDM_ErrorCode BDMCF_CMD_FILL_MEM_L(uint32_t value)                     { return cmd_1L_0(_BDMCF_FILL_MEM|_BDMCF_SZ_LONG, value); }

} // end namespace Bdm

#endif // _BDMMACROS_H_
