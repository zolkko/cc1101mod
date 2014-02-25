//  CC1101 SPI address space definitions
//
//  Copyright (c) 2010 Hans Klunder <hans.klunder (at) bigfoot.com>
//  Author: Hans Klunder, based on the original Rfbee v1.0 firmware by Seeedstudio
//  Version: June 28, 2010
//
//  This library is free software; you can redistribute it and/or
//  modify it under the terms of the GNU Lesser General Public
//  License as published by the Free Software Foundation; either
//  version 2.1 of the License, or (at your option) any later version.
//
//  This library is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//  Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public
//  License along with this library; if not, write to the Free Software
//  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

#ifndef CC1101_H_
#define CC1101_H_

// header bytes
#define CC1101_RW_BIT_bm    0x80
#define CC1101_BURST_BIT_bm 0x40

#define CCx_REG_BEGIN    0x00
#define CCx_IOCFG2       0x00        // GDO2 output pin configuration
#define CCx_IOCFG1       0x01        // GDO1 output pin configuration
#define CCx_IOCFG0       0x02        // GDO0 output pin configuration
#define CCx_FIFOTHR      0x03        // RX FIFO and TX FIFO thresholds
#define CCx_SYNC1        0x04        // Sync word, high unsigned char
#define CCx_SYNC0        0x05        // Sync word, low unsigned char
#define CCx_PKTLEN       0x06        // Packet length
#define CCx_PKTCTRL1     0x07        // Packet automation control
#define CCx_PKTCTRL0     0x08        // Packet automation control
#define CCx_ADDR         0x09        // Device address
#define CCx_CHANNR       0x0A        // Channel number
#define CCx_FSCTRL1      0x0B        // Frequency synthesizer control
#define CCx_FSCTRL0      0x0C        // Frequency synthesizer control
#define CCx_FREQ2        0x0D        // Frequency control word, high unsigned char
#define CCx_FREQ1        0x0E        // Frequency control word, middle unsigned char
#define CCx_FREQ0        0x0F        // Frequency control word, low unsigned char
#define CCx_MDMCFG4      0x10        // Modem configuration
#define CCx_MDMCFG3      0x11        // Modem configuration
#define CCx_MDMCFG2      0x12        // Modem configuration
#define CCx_MDMCFG1      0x13        // Modem configuration
#define CCx_MDMCFG0      0x14        // Modem configuration
#define CCx_DEVIATN      0x15        // Modem deviation setting
#define CCx_MCSM2        0x16        // Main Radio Control State Machine configuration
#define CCx_MCSM1        0x17        // Main Radio Control State Machine configuration
#define CCx_MCSM0        0x18        // Main Radio Control State Machine configuration
#define CCx_FOCCFG       0x19        // Frequency Offset Compensation configuration
#define CCx_BSCFG        0x1A        // Bit Synchronization configuration
#define CCx_AGCCTRL2     0x1B        // AGC control
#define CCx_AGCCTRL1     0x1C        // AGC control
#define CCx_AGCCTRL0     0x1D        // AGC control
#define CCx_WOREVT1      0x1E        // High unsigned char Event 0 timeout
#define CCx_WOREVT0      0x1F        // Low unsigned char Event 0 timeout
#define CCx_WORCTRL      0x20        // Wake On Radio control
#define CCx_FREND1       0x21        // Front end RX configuration
#define CCx_FREND0       0x22        // Front end TX configuration
#define CCx_FSCAL3       0x23        // Frequency synthesizer calibration
#define CCx_FSCAL2       0x24        // Frequency synthesizer calibration
#define CCx_FSCAL1       0x25        // Frequency synthesizer calibration
#define CCx_FSCAL0       0x26        // Frequency synthesizer calibration
#define CCx_RCCTRL1      0x27        // RC oscillator configuration
#define CCx_RCCTRL0      0x28        // RC oscillator configuration
#define CCx_FSTEST       0x29        // Frequency synthesizer calibration control
#define CCx_PTEST        0x2A        // Production test
#define CCx_AGCTEST      0x2B        // AGC test
#define CCx_TEST2        0x2C        // Various test settings
#define CCx_TEST1        0x2D        // Various test settings
#define CCx_TEST0        0x2E        // Various test settings

// Strobe commands
#define CCx_SRES         0x30        // Reset chip.
#define CCx_SFSTXON      0x31        // Enable and calibrate frequency synthesizer (if MCSM0.FS_AUTOCAL=1).
                                     // If in RX/TX: Go to a wait state where only the synthesizer is
                                     // running (for quick RX / TX turnaround).
#define CCx_SXOFF        0x32        // Turn off crystal oscillator.
#define CCx_SCAL         0x33        // Calibrate frequency synthesizer and turn it off
                                     // (enables quick start).
#define CCx_SRX          0x34        // Enable RX. Perform calibration first if coming from IDLE and
                                     // MCSM0.FS_AUTOCAL=1.
#define CCx_STX          0x35        // In IDLE state: Enable TX. Perform calibration first if
                                     // MCSM0.FS_AUTOCAL=1. If in RX state and CCA is enabled:
                                     // Only go to TX if channel is clear.
#define CCx_SIDLE        0x36        // Exit RX / TX, turn off frequency synthesizer and exit
                                     // Wake-On-Radio mode if applicable.
#define CCx_SAFC         0x37        // Perform AFC adjustment of the frequency synthesizer
#define CCx_SWOR         0x38        // Start automatic RX polling sequence (Wake-on-Radio)
#define CCx_SPWD         0x39        // Enter power down mode when CSn goes high.
#define CCx_SFRX         0x3A        // Flush the RX FIFO buffer.
#define CCx_SFTX         0x3B        // Flush the TX FIFO buffer.
#define CCx_SWORRST      0x3C        // Reset real time clock.
#define CCx_SNOP         0x3D        // No operation. May be used to pad strobe commands to two
                                     // bytes for simpler software.
                                     
// Status registers (read & burst)
#define CCx_PARTNUM      (0x30 | 0xc0)
#define CCx_VERSION      (0x31 | 0xc0)
#define CCx_FREQEST      (0x32 | 0xc0)
#define CCx_LQI          (0x33 | 0xc0)
#define CCx_RSSI         (0x34 | 0xc0)
#define CCx_MARCSTATE    (0x35 | 0xc0)
#define CCx_WORTIME1     (0x36 | 0xc0)
#define CCx_WORTIME0     (0x37 | 0xc0)
#define CCx_PKTSTATUS    (0x38 | 0xc0)
#define CCx_VCO_VC_DAC   (0x39 | 0xc0)
#define CCx_TXBYTES      (0x3A | 0xc0)
#define CCx_RXBYTES      (0x3B | 0xc0)

#define CCx_PATABLE      0x3E
#define CCx_TXFIFO       0x3F
#define CCx_RXFIFO       0x3F

#define CCx_FIFO_SIZE    0x40 // 64 bytes

#define CCx_PACKT_LEN   (CCx_FIFO_SIZE - 3)

/**
 * Packet Control 0 register values
 */
#define CCx_PKTCTRL0_LENGTH_FIXED_bm     0x00
#define CCx_PKTCTRL0_LENGTH_VARIABLE_bm  0x01
#define CCx_PKTCTRL0_LENGTH_INFINITE_bm  0x02

#define CCx_PKTCTRL0_CRC_EN_bm           0x04

#define CCx_PKTCTRL0_FMT_NORMAL_bm       0x00
#define CCx_PKTCTRL0_FMT_SYNC_SERIAL_bm  0x10
#define CCx_PKTCTRL0_FMT_RANDOM_bm       0x20
#define CCx_PKTCTRL0_FMT_ASYNC_SERIAL_bm 0x30

#define CCx_PKTCTRL0_WHITE_DATA_bm       0x40


/**
 * GDOx configuration registers
 */
#define GDOx_CFG_RX_THR_RX_THR_gc   0x00
#define GDOx_CFG_RX_THR_RX_EMPTY_gc 0x01

// Associated to the TX FIFO: asserts when TX FIFO is filled above TXFIFO_THR.
// De-asserts when TX FIFO is drained below TX_FIFOR_THR
#define GDOx_CFG_TX_THR_TX_THR_gc   0x02

// Assets when sync word has been sent / received, and de-asserts at the end of the packet. In Rx the pin will
// also de-assert when a packet is discarded due to address or maximum length filtering or when
// the radio enters RXFIFO_OVERFLOW state.
#define GDOx_CFG_SYNC_WORD_gc        0x06
#define GDOx_CFG_PKT_RECEIVED_gc     0x07
#define GDOx_CFG_PREAMBLE_QUALITY_gc 0x08
#define GDOx_CFG_CCA_gc              0x09
// ...
#define GDOx_CFG_CHIP_RDYn       0x29
#define GDOx_CFG_XOSC_STABLE     0x2b
#define GDOx_CFG_HI_Z            0x2e
#define GDOx_CFG_HW_0            0x2f

#define GDOx_CFG_CLK_XOSC1_gc    0x30
#define GDOx_CFG_CLK_XOSC1_5_gc  0x31
#define GDOx_CFG_CLK_XOSC2_gc    0x32
#define GDOx_CFG_CLK_XOSC3_gc    0x33
#define GDOx_CFG_CLK_XOSC4_gc    0x34
#define GDOx_CFG_CLK_XOSC6_gc    0x35
#define GDOx_CFG_CLK_XOSC8_gc    0x36
#define GDOx_CFG_CLK_XOSC12_gc   0x37
#define GDOx_CFG_CLK_XOSC16_gc   0x38
#define GDOx_CFG_CLK_XOSC24_gc   0x39
#define GDOx_CFG_CLK_XOSC32_gc   0x3a
#define GDOx_CFG_CLK_XOSC48_gc   0x3b
#define GDOx_CFG_CLK_XOSC64_gc   0x3c
#define GDOx_CFG_CLK_XOSC96_gc   0x3d
#define GDOx_CFG_CLK_XOSC128_gc  0x3e
#define GDOx_CFG_CLK_XOSC192_gc  0x3f

#define GDOx_CFG_INV_bm          0x40


/**
 * Status byte bit-masks
 */

#define CC1101_STATUS_STATE_bm           0x70
#define CC1101_STATUS_FIFO_BYTES_bm      0x0f
#define CC1101_STATUS_CHIP_RDYn_bm       0x80

#define CC1101_STATUS_STATE_IDLE_bm      0x00
#define CC1101_STATUS_STATE_RX_bm        0x10
#define CC1101_STATUS_STATE_TX_bm        0x20
#define CC1101_STATUS_STATE_FSTXON_bm    0x30
#define CC1101_STATUS_STATE_CALIBRATE_bm 0x40
#define CC1101_STATUS_STATE_SETTLING_bm  0x50
#define CC1101_STATUS_STATE_OVERFLOW_bm  0x60
#define CC1101_STATUS_STATE_UNDERFLOW_bm 0x70


/*
 * Main Radio Control State Machine
 */
#define CC1101_MARC_bm                   0x1f
#define CC1101_MARC_SLEEP_gc             0x00
#define CC1101_MARC_IDLE_gc              0x01
#define CC1101_MARC_XOFF_gc              0x02
#define CC1101_MARC_VCOON_MC_gc          0x03
#define CC1101_MARC_REGON_MC_gc          0x04
#define CC1101_MARC_MANCAL_gc            0x05
#define CC1101_MARC_VCOON_gc             0x06
#define CC1101_MARC_REGON_gc             0x07
#define CC1101_MARC_STARTCAL_gc          0x08
#define CC1101_MARC_BWBOOST_gc           0x09
#define CC1101_MARC_FS_LOCK_gc           0x0a
#define CC1101_MARC_IFADCON_gc           0x0b
#define CC1101_MARC_ENDCAL_gc            0x0c
#define CC1101_MARC_RX_gc                0x0d
#define CC1101_MARC_RX_END_gc            0x0e
#define CC1101_MARC_RX_RST_gc            0x0f
#define CC1101_MARC_TXRX_SWITCH_gc       0x10
#define CC1101_MARC_RXFIFO_OVERFLOW_gc   0x11
#define CC1101_MARC_FSTXON_gc            0x12
#define CC1101_MARC_TX_gc                0x13
#define CC1101_MARC_TX_END_gc            0x14
#define CC1101_MARC_RXTX_SWITCH_gc       0x15
#define CC1101_MARC_TXFIFO_UNDERFLOW_gc  0x16


#endif /* CC1101_H_ */
