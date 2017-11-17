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
#include <stdio.h>

#define PROTOOPTS_X5C 0

#define BIND_COUNT 10
// #define BIND_COUNT 345   // 1.5 seconds
#define FIRST_PACKET_DELAY  12000

#define PACKET_PERIOD        4000     // Timeout for callback in uSec
#define INITIAL_WAIT          500

#define FLAG_FLIP      0x01
#define FLAG_RATES     0x02
#define FLAG_VIDEO     0x04
#define FLAG_PICTURE   0x08



enum {
    SYMAX_INIT1 = 0,
    SYMAX_BIND2,
    SYMAX_BIND3,
    SYMAX_DATA
};
uint8 symax_phase = SYMAX_INIT1;

    
// For code readability
enum {
    CHANNEL1 = 0,
    CHANNEL2,
    CHANNEL3,
    CHANNEL4,
    CHANNEL5,
    CHANNEL6,
    CHANNEL7,
    CHANNEL8,
    CHANNEL9,
    CHANNEL10
};

#define PAYLOADSIZE 10       // receive data pipes set to this size, but unused
#define MAX_PACKET_SIZE 16   // X11,X12,X5C-1 10-byte, X5C 16-byte

static uint8 packet[MAX_PACKET_SIZE];
static uint8 packet_size;
static uint32 packet_counter;
static uint16 counter;
static uint8 throttle, rudder, elevator, aileron, flags;
static uint8 rx_tx_addr[5];
static int32 Channels[8];   // simulate Deviation channels

// frequency channel management
#define MAX_RF_CHANNELS    17
#define NUM_X11_CHANNELS   4

static uint8 current_chan;
static uint8 chans[MAX_RF_CHANNELS];
static uint8 num_rf_channels;

// Bit vector from bit position
#define BV(bit) (1 << bit)


static uint8 checksum(uint8 *data) {
    uint8 sum = data[0];
    uint8 i;

    for (i=1; i < packet_size-1; i++)
        if (PROTOOPTS_X5C)
            sum += data[i];
        else
            sum ^= data[i];
    
    return sum + (PROTOOPTS_X5C ? 0 : 0x55);
}


#define BABS(X) (((X) < 0) ? -(uint8)(X) : (X))
// Channel values are sign + magnitude 8bit values
static uint8 convert_channel(uint8 num)
{
    int32 ch = Channels[num];
    if (ch < CHAN_MIN_VALUE) {
        ch = CHAN_MIN_VALUE;
    } else if (ch > CHAN_MAX_VALUE) {
        ch = CHAN_MAX_VALUE;
    }
    return (uint8) ((ch < 0 ? 0x80 : 0) | BABS(ch * 127 / CHAN_MAX_VALUE));
}


static void read_controls(uint8* throttle, uint8* rudder, uint8* elevator, uint8* aileron, uint8* flags)
{
    *aileron  = convert_channel(CHANNEL1);
    *elevator = convert_channel(CHANNEL2);
    *throttle = convert_channel(CHANNEL3);
    *throttle = *throttle & 0x80 ? 0xff - *throttle : 0x80 + *throttle;
    *rudder   = convert_channel(CHANNEL4);

    // Channel 5
    if (Channels[CHANNEL5] <= 0)
        *flags &= ~FLAG_FLIP;
    else
        *flags |= FLAG_FLIP;

    // Channel 6
    if (Channels[CHANNEL6] <= 0)
        *flags &= ~FLAG_RATES;
    else
        *flags |= FLAG_RATES;

    // Channel 7
    if (Channels[CHANNEL7] <= 0)
        *flags &= ~FLAG_PICTURE;
    else
        *flags |= FLAG_PICTURE;

    // Channel 8
    if (Channels[CHANNEL8] <= 0)
        *flags &= ~FLAG_VIDEO;
    else
        *flags |= FLAG_VIDEO;
}


#define X5C_CHAN2TRIM(X) ((((X) & 0x80 ? 0xff - (X) : 0x80 + (X)) >> 2) + 0x20)

static void build_packet_x5c(uint8 bind)
{
    if (bind) {
        memset(packet, 0, packet_size);
        packet[7] = 0xae;
        packet[8] = 0xa9;
        packet[14] = 0xc0;
        packet[15] = 0x17;
    } else {
        read_controls(&throttle, &rudder, &elevator, &aileron, &flags);

        packet[0] = throttle;
        packet[1] = rudder;
        packet[2] = elevator ^ 0x80;  // reversed from default
        packet[3] = aileron;
        packet[4] = X5C_CHAN2TRIM(rudder ^ 0x80);     // drive trims for extra control range
        packet[5] = X5C_CHAN2TRIM(elevator);
        packet[6] = X5C_CHAN2TRIM(aileron ^ 0x80);
        packet[7] = 0xae;
        packet[8] = 0xa9;
        packet[9] = 0x00;
        packet[10] = 0x00;
        packet[11] = 0x00;
        packet[12] = 0x00;
        packet[13] = 0x00;
        packet[14] = (flags & FLAG_VIDEO   ? 0x10 : 0x00) 
                   | (flags & FLAG_PICTURE ? 0x08 : 0x00)
                   | (flags & FLAG_FLIP    ? 0x01 : 0x00)
                   | (flags & FLAG_RATES   ? 0x04 : 0x00);
        packet[15] = checksum(packet);
    }
}


static void build_packet(uint8 bind) {
    if (bind) {
        packet[0] = rx_tx_addr[4];
        packet[1] = rx_tx_addr[3];
        packet[2] = rx_tx_addr[2];
        packet[3] = rx_tx_addr[1];
        packet[4] = rx_tx_addr[0];
        packet[5] = 0xaa;
        packet[6] = 0xaa;
        packet[7] = 0xaa;
        packet[8] = 0x00;
    } else {
        read_controls(&throttle, &rudder, &elevator, &aileron, &flags);

        packet[0] = throttle;
        packet[1] = elevator;
        packet[2] = rudder;
        packet[3] = aileron;
        packet[4] = 0x00;
        packet[4] = (flags & FLAG_VIDEO   ? 0x80 : 0x00) 
                  | (flags & FLAG_PICTURE ? 0x40 : 0x00);
        packet[5] = (elevator >> 2) | (flags & FLAG_RATES ? 0x80 : 0x00) | 0x40;  // use trims to extend controls
        packet[6] = (rudder >> 2) | (flags & FLAG_FLIP ? 0x40 : 0x00);
        packet[7] = aileron >> 2;
        packet[8] = 0x00;

    }
    packet[9] = checksum(packet);
}


void symax_send_packet(uint8 bind)
{
    if (PROTOOPTS_X5C)
        build_packet_x5c(bind);
    else
        build_packet(bind);

    // clear packet status bits and TX FIFO
    NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);
    NRF24L01_WriteReg(NRF24L01_00_CONFIG, 0x2e);
    NRF24L01_WriteReg(NRF24L01_05_RF_CH, chans[current_chan]);
    NRF24L01_FlushTx();

    NRF24L01_WritePayload(packet, packet_size);

    if (packet_counter++ % 2) {   // use each channel twice
        current_chan = (current_chan + 1) % num_rf_channels;
    }
}

static void symax_init1()
{
    // write a strange first packet to RF channel 8 ...
    uint8 first_packet[] = {0xf9, 0x96, 0x82, 0x1b, 0x20, 0x08, 0x08, 0xf2, 0x7d, 0xef, 0xff, 0x00, 0x00, 0x00, 0x00};
    uint8 chans_bind[] = {0x4b, 0x30, 0x40, 0x2e};
    uint8 chans_bind_x5c[] = {0x27, 0x1b, 0x39, 0x28, 0x24, 0x22, 0x2e, 0x36,
                           0x19, 0x21, 0x29, 0x14, 0x1e, 0x12, 0x2d, 0x18};

//    uint8 data_rx_tx_addr[] = {0x3b,0xb6,0x00,0x00,0xa2};

    NRF24L01_FlushTx();
    NRF24L01_WriteReg(NRF24L01_05_RF_CH, 0x08);
    NRF24L01_WritePayload(first_packet, 15);

    if (PROTOOPTS_X5C) {
      num_rf_channels = sizeof(chans_bind_x5c);
      memcpy(chans, chans_bind_x5c, num_rf_channels);
    } else {
//      memcpy(rx_tx_addr, data_rx_tx_addr, 5);   // make info available for bind packets
      num_rf_channels = sizeof(chans_bind);
      memcpy(chans, chans_bind, num_rf_channels);
    }
    current_chan = 0;
    packet_counter = 0;
}

void symax_set_channels(uint8 address) {
  static const uint8 start_chans_1[] = {0x0a, 0x1a, 0x2a, 0x3a};
  static const uint8 start_chans_2[] = {0x2a, 0x0a, 0x42, 0x22};
  static const uint8 start_chans_3[] = {0x1a, 0x3a, 0x12, 0x32};

  uint8 laddress = address & 0x1f;
  uint8 i;

  num_rf_channels = sizeof(start_chans_1);
  if (laddress < 0x10) {
    if (laddress == 6) laddress = 7;
    for(i=0; i < sizeof(start_chans_1); i++) {
      chans[i] = start_chans_1[i] + laddress;
    }
  } else if (laddress < 0x18) {
    for(i=0; i < sizeof(start_chans_2); i++) {
      chans[i] = start_chans_2[i] + (laddress & 0x07);
    }
    if (laddress == 0x16) {
      chans[0] += 1;
      chans[1] += 1;
    }
  } else if (laddress < 0x1e) {
    for(i=0; i < sizeof(start_chans_3); i++) {
      chans[i] = start_chans_3[i] + (laddress & 0x07);
    }
  } else if (laddress == 0x1e)
      *(uint32*)chans = 0x38184121;
    else
      *(uint32*)chans = 0x39194121;

char outbuf[64];
snprintf(outbuf, sizeof(outbuf), "lb:%02X, ch:%02X%02X%02X%02X\r\n", address, chans[0], chans[1], chans[2], chans[3]);
USB_serial_UartPutString(outbuf);
}

static void symax_init2()
{
//    uint8 chans_data[] = {0x1d, 0x3d, 0x15, 0x35};
    uint8 chans_data_x5c[] = {0x1d, 0x2f, 0x26, 0x3d, 0x15, 0x2b, 0x25, 0x24,
                           0x27, 0x2c, 0x1c, 0x3e, 0x39, 0x2d, 0x22};

    if (PROTOOPTS_X5C) {
      num_rf_channels = sizeof(chans_data_x5c);
      memcpy(chans, chans_data_x5c, num_rf_channels);
    } else {
//      num_rf_channels = sizeof(chans_data);
//      memcpy(chans, chans_data, num_rf_channels);
      symax_set_channels(rx_tx_addr[0]);
      
      NRF24L01_WriteRegisterMulti(NRF24L01_10_TX_ADDR, rx_tx_addr, 5);
    }
    current_chan = 0;
    packet_counter = 0;
}

void symax_init(uint8 tx_addr[]) {
  const uint8 bind_rx_tx_addr[] = {0xab,0xac,0xad,0xae,0xaf};
  const uint8 bind_rx_tx_addr_x5c[] = {0x6d,0x6a,0x73,0x73,0x73};
  
  symax_phase = SYMAX_INIT1;
  packet_counter = 0;
  flags = 0;
  memcpy(rx_tx_addr, tx_addr, sizeof(rx_tx_addr));  

  NRF24L01_ReadReg(NRF24L01_07_STATUS);
  NRF24L01_WriteReg(NRF24L01_00_CONFIG, BV(NRF24L01_00_EN_CRC) | BV(NRF24L01_00_CRCO)); 
  NRF24L01_WriteReg(NRF24L01_01_EN_AA, 0x00);      // No Auto Acknoledgement
  NRF24L01_WriteReg(NRF24L01_02_EN_RXADDR, 0x3F);  // Enable all data pipes (even though not used?)
  NRF24L01_WriteReg(NRF24L01_03_SETUP_AW, 0x03);   // 5-byte RX/TX address
  NRF24L01_WriteReg(NRF24L01_04_SETUP_RETR, 0xff); // 4mS retransmit t/o, 15 tries (retries w/o AA?)
  NRF24L01_WriteReg(NRF24L01_05_RF_CH, 0x08);

  if (PROTOOPTS_X5C) {
    NRF24L01_SetBitrate(NRF24L01_BR_1M);
    packet_size = 16;
  } else {
    NRF24L01_SetBitrate(NRF24L01_BR_250K);
    packet_size = 10;
  }

  NRF24L01_SetPower(TXPOWER_150mW);
  NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);     // Clear data ready, data sent, and retransmit
  NRF24L01_WriteReg(NRF24L01_08_OBSERVE_TX, 0x00);
  NRF24L01_WriteReg(NRF24L01_09_CD, 0x00);
  NRF24L01_WriteReg(NRF24L01_0C_RX_ADDR_P2, 0xC3); // LSB byte of pipe 2 receive address
  NRF24L01_WriteReg(NRF24L01_0D_RX_ADDR_P3, 0xC4);
  NRF24L01_WriteReg(NRF24L01_0E_RX_ADDR_P4, 0xC5);
  NRF24L01_WriteReg(NRF24L01_0F_RX_ADDR_P5, 0xC6);
  NRF24L01_WriteReg(NRF24L01_11_RX_PW_P0, PAYLOADSIZE);   // bytes of data payload for pipe 1
  NRF24L01_WriteReg(NRF24L01_12_RX_PW_P1, PAYLOADSIZE);
  NRF24L01_WriteReg(NRF24L01_13_RX_PW_P2, PAYLOADSIZE);
  NRF24L01_WriteReg(NRF24L01_14_RX_PW_P3, PAYLOADSIZE);
  NRF24L01_WriteReg(NRF24L01_15_RX_PW_P4, PAYLOADSIZE);
  NRF24L01_WriteReg(NRF24L01_16_RX_PW_P5, PAYLOADSIZE);
  NRF24L01_WriteReg(NRF24L01_17_FIFO_STATUS, 0x00); // Just in case, no real bits to write here

   NRF24L01_WriteRegisterMulti(NRF24L01_10_TX_ADDR,
                              PROTOOPTS_X5C ? bind_rx_tx_addr_x5c : bind_rx_tx_addr,
                              5);

  NRF24L01_ReadReg(NRF24L01_07_STATUS);

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


uint16 symax_callback(volatile int32 channels[])
{
    memcpy(Channels, (const void *)channels, sizeof(Channels));
    
    switch (symax_phase) {
    case SYMAX_INIT1:
        symax_init1();
        symax_phase = SYMAX_BIND2;
        return FIRST_PACKET_DELAY;
        break;

    case SYMAX_BIND2:
        counter = BIND_COUNT;
        symax_phase = SYMAX_BIND3;
        symax_send_packet(1);
        break;

    case SYMAX_BIND3:
        if (counter == 0) {
            symax_init2();
            symax_phase = SYMAX_DATA;
        } else {
            symax_send_packet(1);
            counter -= 1;
        }
        break;

    case SYMAX_DATA:
        symax_send_packet(0);
        break;
    }
    return PACKET_PERIOD;
}

