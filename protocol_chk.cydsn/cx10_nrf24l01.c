/*
 This project is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 Deviation is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 see <http://www.gnu.org/licenses/>.
 */


#include <project.h>
#include "nrf24l01.h"
#include "protocols.h"
#include "protocol_chk.h"
#include <stdio.h>


#define CX10_PACKET_SIZE 15
#define CX10A_PACKET_SIZE 19       // CX10 blue board packets have 19-byte payload
#define CX10_PACKET_PERIOD   1316  // Timeout for callback in uSec
#define CX10A_PACKET_PERIOD  6000
#define BIND_COUNT 4360   // 6 seconds
#define INITIAL_WAIT     500

// flags 
#define FLAG_FLIP       0x1000 // goes to rudder channel
#define FLAG_MODE_MASK  0x0003
#define FLAG_HEADLESS   0x0004
// flags2
#define FLAG_VIDEO      0x0002
#define FLAG_SNAPSHOT   0x0004


enum {
    PROTOOPTS_FORMAT = 0,
    LAST_PROTO_OPT,
};

enum {
    FORMAT_CX10_GREEN = 0,
    FORMAT_CX10_BLUE,
    FORMAT_DM007,
} protoopts_format = FORMAT_CX10_BLUE;

// For code readability
enum {
    CHANNEL1 = 0,   // Aileron
    CHANNEL2,       // Elevator
    CHANNEL3,       // Throttle
    CHANNEL4,       // Rudder
    CHANNEL5,       // Rate/Mode (+ Headless on CX-10A)
    CHANNEL6,       // Flip
    CHANNEL7,       // Still Camera (DM007)
    CHANNEL8,       // Video Camera (DM007)
    CHANNEL9,       // Headless (DM007)
    CHANNEL10
};

static uint8 packet[CX10A_PACKET_SIZE]; // CX10A (blue board) has larger packet size
static uint8 packet_size;
static uint16 packet_period;
static uint8 phase;
static uint8 bind_phase;
static uint16 bind_counter;
//static uint8 tx_power;
static uint16 throttle, rudder, elevator, aileron, flags, flags2;
static const uint8 rx_tx_addr[] = {0xcc, 0xcc, 0xcc, 0xcc, 0xcc};
static int32 Channels[8];   // simulate Deviation channels

// frequency channel management
#define RF_BIND_CHANNEL 0x02
#define NUM_RF_CHANNELS    4
static uint8 current_chan = 0;
static uint8 txid[4];
static uint8 rf_chans[4];

enum {
    CX10_INIT1 = 0,
    CX10_BIND1,
    CX10_BIND2,
    CX10_DATA
};

// Bit vector from bit position
#define BV(bit) (1 << bit)

// Channel values are servo time in ms, 1500ms is the middle,
// 1000 and 2000 are min and max values
static uint16 convert_channel(uint8 num)
{
    int32 ch = Channels[num];
    if (ch < CHAN_MIN_VALUE) {
        ch = CHAN_MIN_VALUE;
    } else if (ch > CHAN_MAX_VALUE) {
        ch = CHAN_MAX_VALUE;
    }
    return (uint16) ((ch * 500 / CHAN_MAX_VALUE) + 1500);
}

static void read_controls(uint16* throttle, uint16* rudder, uint16* elevator, uint16* aileron, uint16* flags, uint16* flags2)
{
    // Protocol is registered AETRF, that is
    // Aileron is channel 1, Elevator - 2, Throttle - 3, Rudder - 4

    *aileron  = convert_channel(CHANNEL1);
    // Correct direction so the model file would be straightforward
    *elevator = 3000 - convert_channel(CHANNEL2);
    *throttle = convert_channel(CHANNEL3);
    // Same for rudder
    *rudder   = 3000 - convert_channel(CHANNEL4);

    *flags &= ~FLAG_MODE_MASK;
    // Channel 5 - mode
    if (Channels[CHANNEL5] > 0) {
        if (Channels[CHANNEL5] < CHAN_MAX_VALUE / 2)
            *flags |= 1;
        else
            *flags |= 2; // headless on CX-10A
    }

    // Channel 6 - flip flag
    if (Channels[CHANNEL6] <= 0)
        *flags &= ~FLAG_FLIP;
    else
        *flags |= FLAG_FLIP;

    if(protoopts_format == FORMAT_DM007) {
        // invert aileron direction
        *aileron = 3000 - *aileron;
        
        // Channel 7 - snapshot
        if(Channels[CHANNEL7] <= 0)
            *flags2 &= ~FLAG_SNAPSHOT;
        else
            *flags2 |= FLAG_SNAPSHOT;

        // Channel 8 - video
        if(Channels[CHANNEL8] <=0)
            *flags2 &= ~FLAG_VIDEO;
        else
            *flags2 |= FLAG_VIDEO;

        // Channel 9 - headless
        if (Channels[CHANNEL9] <= 0)
            *flags &= ~FLAG_HEADLESS;
        else
            *flags |= FLAG_HEADLESS;
    }

}

static void send_packet(uint8 bind)
{
    uint8 offset=0;
    if(protoopts_format == FORMAT_CX10_BLUE)
        offset = 4;
    packet[0] = bind ? 0xAA : 0x55;
    packet[1] = txid[0];
    packet[2] = txid[1];
    packet[3] = txid[2];
    packet[4] = txid[3];
    // for CX-10A [5]-[8] is aircraft id received during bind 
    read_controls(&throttle, &rudder, &elevator, &aileron, &flags, &flags2);
    packet[5+offset] = aileron & 0xff;
    packet[6+offset] = (aileron >> 8) & 0xff;
    packet[7+offset] = elevator & 0xff;
    packet[8+offset] = (elevator >> 8) & 0xff;
    packet[9+offset] = throttle & 0xff;
    packet[10+offset] = (throttle >> 8) & 0xff;
    packet[11+offset] = rudder & 0xff;
    packet[12+offset] = ((rudder >> 8) & 0xff) | ((flags & FLAG_FLIP) >> 8);  // 0x10 here is a flip flag 
    packet[13+offset] = flags & 0xff;
    packet[14+offset] = flags2 & 0xff;

    // Power on, TX mode, 2byte CRC
    // Why CRC0? xn297 does not interpret it - either 16-bit CRC or nothing
    XN297_Configure(BV(NRF24L01_00_EN_CRC) | BV(NRF24L01_00_CRCO) | BV(NRF24L01_00_PWR_UP));
    if (bind) {
        NRF24L01_WriteReg(NRF24L01_05_RF_CH, RF_BIND_CHANNEL);
    } else {
        NRF24L01_WriteReg(NRF24L01_05_RF_CH, rf_chans[current_chan++]);
        current_chan %= NUM_RF_CHANNELS;
    }
    // clear packet status bits and TX FIFO
    NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);
    NRF24L01_FlushTx();

    XN297_WritePayload(packet, packet_size);

}

void cx10_initialize()
{
    NRF24L01_Initialize();
    NRF24L01_SetTxRxMode(TX_EN);

    // SPI trace of stock TX has these writes to registers that don't appear in
    // nRF24L01 or Beken 2421 datasheets.  Uncomment if you have an XN297 chip?
    // NRF24L01_WriteRegisterMulti(0x3f, "\x4c\x84\x67,\x9c,\x20", 5); 
    // NRF24L01_WriteRegisterMulti(0x3e, "\xc9\x9a\xb0,\x61,\xbb,\xab,\x9c", 7); 
    // NRF24L01_WriteRegisterMulti(0x39, "\x0b\xdf\xc4,\xa7,\x03,\xab,\x9c", 7); 

    XN297_SetTXAddr(rx_tx_addr, 5);
    XN297_SetRXAddr(rx_tx_addr, 5);
    NRF24L01_FlushTx();
    NRF24L01_FlushRx();
    NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);     // Clear data ready, data sent, and retransmit
    NRF24L01_WriteReg(NRF24L01_01_EN_AA, 0x00);      // No Auto Acknowldgement on all data pipes
    NRF24L01_WriteReg(NRF24L01_02_EN_RXADDR, 0x01);  // Enable data pipe 0 only
    NRF24L01_WriteReg(NRF24L01_11_RX_PW_P0, packet_size); // bytes of data payload for rx pipe 1 
    NRF24L01_WriteReg(NRF24L01_05_RF_CH, RF_BIND_CHANNEL);
    NRF24L01_SetBitrate(NRF24L01_BR_1M);             // 1Mbps
    NRF24L01_WriteReg(NRF24L01_06_RF_SETUP, 0x07);
    
    // this sequence necessary for module from stock tx
    NRF24L01_ReadReg(NRF24L01_1D_FEATURE);
    NRF24L01_Activate(0x73);                          // Activate feature register
    NRF24L01_ReadReg(NRF24L01_1D_FEATURE);

    NRF24L01_WriteReg(NRF24L01_1C_DYNPD, 0x00);       // Disable dynamic payload length on all pipes
    NRF24L01_WriteReg(NRF24L01_1D_FEATURE, 0x00);     // Set feature bits on

    // Check for Beken BK2421/BK2423 chip
    // It is done by using Beken specific activate code, 0x53
    // and checking that status register changed appropriately
    // There is no harm to run it on nRF24L01 because following
    // closing activate command changes state back even if it
    // does something on nRF24L01
    NRF24L01_Activate(0x53); // magic for BK2421 bank switch
//  USB_serial_UartPutString("Trying to switch banks\r\n");
  if (NRF24L01_ReadReg(NRF24L01_07_STATUS) & 0x80) {
//    USB_serial_UartPutString("BK2421 detected\r\n");
    // Beken registers don't have such nice names, so we just mention
    // them by their numbers
    // It's all magic, eavesdropped from real transfer and not even from the
    // data sheet - it has slightly different values
    NRF24L01_WriteRegisterMulti(0x00, (uint8 *) "\x40\x4B\x01\xE2", 4);
    NRF24L01_WriteRegisterMulti(0x01, (uint8 *) "\xC0\x4B\x00\x00", 4);
    NRF24L01_WriteRegisterMulti(0x02, (uint8 *) "\xD0\xFC\x8C\x02", 4);
    NRF24L01_WriteRegisterMulti(0x03, (uint8 *) "\x99\x00\x39\x21", 4);
    NRF24L01_WriteRegisterMulti(0x04, (uint8 *) "\xF9\x96\x82\x1B", 4);
    NRF24L01_WriteRegisterMulti(0x05, (uint8 *) "\x24\x06\x7F\xA6", 4);
    NRF24L01_WriteRegisterMulti(0x06, (uint8 *) "\x00\x00\x00\x00", 4);
    NRF24L01_WriteRegisterMulti(0x07, (uint8 *) "\x00\x00\x00\x00", 4);
    NRF24L01_WriteRegisterMulti(0x08, (uint8 *) "\x00\x00\x00\x00", 4);
    NRF24L01_WriteRegisterMulti(0x09, (uint8 *) "\x00\x00\x00\x00", 4);
    NRF24L01_WriteRegisterMulti(0x0A, (uint8 *) "\x00\x00\x00\x00", 4);
    NRF24L01_WriteRegisterMulti(0x0B, (uint8 *) "\x00\x00\x00\x00", 4);
    NRF24L01_WriteRegisterMulti(0x0C, (uint8 *) "\x00\x12\x73\x00", 4);
    NRF24L01_WriteRegisterMulti(0x0D, (uint8 *) "\x46\xB4\x80\x00", 4);
    NRF24L01_WriteRegisterMulti(0x0E, (uint8 *) "\x41\x10\x04\x82\x20\x08\x08\xF2\x7D\xEF\xFF", 11);
    NRF24L01_WriteRegisterMulti(0x04, (uint8 *) "\xFF\x96\x82\x1B", 4);
    NRF24L01_WriteRegisterMulti(0x04, (uint8 *) "\xF9\x96\x82\x1B", 4);
  } else {
    USB_serial_UartPutString("nRF24L01 detected\r\n");
  }
  NRF24L01_Activate(0x53); // switch bank back

  NRF24L01_FlushTx();
  NRF24L01_ReadReg(NRF24L01_07_STATUS);
  NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x0e);
  NRF24L01_ReadReg(NRF24L01_00_CONFIG); 
  NRF24L01_WriteReg(NRF24L01_00_CONFIG, 0x0c); 
  NRF24L01_WriteReg(NRF24L01_00_CONFIG, 0x0e);  // power on 
  
  
}


uint16 cx10_callback(volatile int32 channels[])
{
    memcpy(Channels, (const void *)channels, sizeof(Channels));
    
    switch (phase) {
    case CX10_INIT1:
        phase = bind_phase;
        break;

    case CX10_BIND1:
        if (bind_counter == 0) {
            phase = CX10_DATA;
        } else {
            send_packet(1);
            bind_counter -= 1;
        }
        break;
        
    case CX10_BIND2:
//        printd("00_config = 0x%02x\r\n", NRF24L01_ReadReg(NRF24L01_00_CONFIG));
        if( NRF24L01_ReadReg(NRF24L01_07_STATUS) & BV(NRF24L01_07_RX_DR)) { // RX fifo data ready
            XN297_ReadPayload(packet, packet_size);
            USB_serial_UartPutString("RX Packet\r\n");
            NRF24L01_SetTxRxMode(TXRX_OFF);
            NRF24L01_SetTxRxMode(TX_EN);
            bind_counter = 10;
            phase = CX10_BIND1; 
        } else {
            NRF24L01_SetTxRxMode(TXRX_OFF);
            NRF24L01_SetTxRxMode(TX_EN);
            send_packet(1);
            CyDelayUs(15);  // usleep(15);
            NRF24L01_FlushRx();
            NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70); // Clear data ready, data sent, and retransmit
            // switch to RX mode
            XN297_Configure(BV(NRF24L01_00_EN_CRC) | BV(NRF24L01_00_CRCO) 
                          | BV(NRF24L01_00_PWR_UP) | BV(NRF24L01_00_PRIM_RX)); 
        }
        break;

    case CX10_DATA:
        send_packet(0);
        break;
    }
    return packet_period;
}

// Generate address to use from TX id and manufacturer id (STM32 unique id)
static void initialize_txid()
{
    uint32 lfsr = 0xb2c54a2ful;

    // Pump zero bytes for LFSR to diverge more
     // tx id
    txid[0] = (lfsr >> 24) & 0xFF;
    txid[1] = ((lfsr >> 16) & 0xFF) % 0x30;
    txid[2] = (lfsr >> 8) & 0xFF;
    txid[3] = lfsr & 0xFF;
    // rf channels
    rf_chans[0] = 0x03 + (txid[0] & 0x0F);
    rf_chans[1] = 0x16 + (txid[0] >> 4);
    rf_chans[2] = 0x2D + (txid[1] & 0x0F);
    rf_chans[3] = 0x40 + (txid[1] >> 4);
}

void cx10_init(uint8 *unused)
{
    switch( protoopts_format) {
        case FORMAT_CX10_GREEN:
        case FORMAT_DM007:
            packet_size = CX10_PACKET_SIZE;
            packet_period = CX10_PACKET_PERIOD;
            bind_phase = CX10_BIND1;
            bind_counter = BIND_COUNT;
            break;
        
        case FORMAT_CX10_BLUE:
            packet_size = CX10A_PACKET_SIZE;
            packet_period = CX10A_PACKET_PERIOD;
            bind_phase = CX10_BIND2;
            uint8 i;
            for(i=0; i<4; i++)
                packet[5+i] = 0xFF; // clear aircraft id
            packet[9] = 0;
            break;
    }
    initialize_txid();
    flags = 0;
    flags2 = 0;
    cx10_initialize();
    phase = CX10_INIT1;
}




