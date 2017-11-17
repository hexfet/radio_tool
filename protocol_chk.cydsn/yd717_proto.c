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

/* Uncomment define below to enable packet loss telemetry. Also add
   YD717 protocol to TELEMETRY_SetTypeByProtocol to
   set type to DSM.
   */
//#define YD717_TELEMETRY


#include <project.h>
#include "nrf24l01.h"
#include "protocols.h"



#ifdef EMULATOR
#define USE_FIXED_MFGID
#define BIND_COUNT 5
#define dbgprintf printf
#else
#define BIND_COUNT 60
#endif

#ifndef EMULATOR
#define PACKET_PERIOD   8000     // Timeout for callback in uSec, 8ms=8000us for YD717
#define INITIAL_WAIT   50000     // Initial wait before starting callbacks
#else
#define PACKET_PERIOD   1000     // Adjust timeouts for reasonable emulator printouts
#define INITIAL_WAIT     500
#endif
#define PACKET_CHKTIME   500     // Time to wait if packet not yet acknowledged or timed out    

// Stock tx fixed frequency is 0x3C. Receiver only binds on this freq.
#define RF_CHANNEL 0x3C

#define FLAG_FLIP   0x0F
#define FLAG_LIGHT  0x10

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

#define PAYLOADSIZE 8       // receive data pipes set to this size, but unused
#define MAX_PACKET_SIZE 9   // YD717 packets have 8-byte payload, Syma X4 is 9

static uint8 packet[MAX_PACKET_SIZE];
static uint16 counter;
static uint32 packet_counter;
static uint8 tx_power = TXPOWER_1mW;
static uint8 throttle, rudder, elevator, aileron, flags;
static uint8 rudder_trim, elevator_trim, aileron_trim;
static uint8 rx_tx_addr[5];
static int32 Channels[8];   // simulate Deviation channels


enum {
    YD717_INIT1 = 0,
    YD717_BIND2,
    YD717_BIND3,
    YD717_DATA
};
static uint8 phase = YD717_INIT1;

#define FORMAT_YD717   0
#define FORMAT_SKYWLKR 1
#define FORMAT_XINXUN  2
#define FORMAT_NI_HUI  3
#define FORMAT_SYMAX2  4
#define PROTOOPTS_FORMAT  FORMAT_YD717


#ifdef YD717_TELEMETRY
#define TELEM_OFF 0
#define TELEM_ON 1
#endif

// Packet ack status values
enum {
    PKT_PENDING = 0,
    PKT_ACKED,
    PKT_TIMEOUT
};

static uint8 packet_ack()
{
    switch (NRF24L01_ReadReg(NRF24L01_07_STATUS) & (BV(NRF24L01_07_TX_DS) | BV(NRF24L01_07_MAX_RT))) {
    case BV(NRF24L01_07_TX_DS):
        return PKT_ACKED;
    case BV(NRF24L01_07_MAX_RT):
        return PKT_TIMEOUT;
    }
#ifndef EMULATOR
    return PKT_PENDING;
#else
    return PKT_ACKED;
#endif
}


static uint8 convert_channel(uint8 num)
{
    int32 ch = Channels[num];
    if (ch < CHAN_MIN_VALUE) {
        ch = CHAN_MIN_VALUE;
    } else if (ch > CHAN_MAX_VALUE) {
        ch = CHAN_MAX_VALUE;
    }

    return (uint8) (((ch * 0xFF / CHAN_MAX_VALUE) + 0x100) >> 1);
}



static void read_controls(uint8* throttle, uint8* rudder, uint8* elevator, uint8* aileron,
                          uint8* flags, uint8* rudder_trim, uint8* elevator_trim, uint8* aileron_trim)
{
    // Protocol is registered AETRF, that is
    // Aileron is channel 1, Elevator - 2, Throttle - 3, Rudder - 4, Flip control - 5

    // Channel 3
    *throttle = convert_channel(CHANNEL3);

    // Channel 4
    if(PROTOOPTS_FORMAT == FORMAT_XINXUN) {
      *rudder = convert_channel(CHANNEL4);
      *rudder_trim = (0xff - *rudder) >> 1;
    } else {
      *rudder = 0xff - convert_channel(CHANNEL4);
      *rudder_trim = *rudder >> 1;
    }

    // Channel 2
    *elevator = convert_channel(CHANNEL2);
    *elevator_trim = *elevator >> 1;

    // Channel 1
    *aileron = 0xff - convert_channel(CHANNEL1);
    *aileron_trim = *aileron >> 1;

    // Channel 5
    if (Channels[CHANNEL5] <= 0)
      *flags &= ~FLAG_FLIP;
    else
      *flags |= FLAG_FLIP;

    // Channel 6
    if (Channels[CHANNEL6] <= 0)
      *flags &= ~FLAG_LIGHT;
    else
      *flags |= FLAG_LIGHT;
}


static void send_packet(uint8 bind)
{
    if (bind) {
        packet[0]= rx_tx_addr[0]; // send data phase address in first 4 bytes
        packet[1]= rx_tx_addr[1];
        packet[2]= rx_tx_addr[2];
        packet[3]= rx_tx_addr[3];
        packet[4] = 0x56;
        packet[5] = 0xAA;
        packet[6] = PROTOOPTS_FORMAT == FORMAT_NI_HUI ? 0x00 : 0x32;
        packet[7] = 0x00;
    } else {
        read_controls(&throttle, &rudder, &elevator, &aileron, &flags, &rudder_trim, &elevator_trim, &aileron_trim);
        packet[0] = throttle;
        packet[1] = rudder;
        packet[3] = elevator;
        packet[4] = aileron;
        if(PROTOOPTS_FORMAT == FORMAT_YD717) {
            packet[2] = elevator_trim;
            packet[5] = aileron_trim;
            packet[6] = rudder_trim;
        } else {
            packet[2] = rudder_trim;
            packet[5] = elevator_trim;
            packet[6] = aileron_trim;
        }
        packet[7] = flags;
    }


    // clear packet status bits and TX FIFO
    NRF24L01_WriteReg(NRF24L01_07_STATUS, (BV(NRF24L01_07_TX_DS) | BV(NRF24L01_07_MAX_RT)));
    NRF24L01_FlushTx();

    if(PROTOOPTS_FORMAT == FORMAT_YD717) {
        NRF24L01_WritePayload(packet, 8);
    } else {
        packet[8] = packet[0];  // checksum
        uint8 i;
        for(i=1; i < 8; i++) packet[8] += packet[i];
        packet[8] = ~packet[8];

        NRF24L01_WritePayload(packet, 9);
    }

    ++packet_counter;

//    radio.ce(HIGH);
//    delayMicroseconds(15);
    // It saves power to turn off radio after the transmission,
    // so as long as we have pins to do so, it is wise to turn
    // it back.
//    radio.ce(LOW);

    // Check and adjust transmission power. We do this after
    // transmission to not bother with timeout after power
    // settings change -  we have plenty of time until next
    // packet.
}

static void set_rx_tx_addr(uint32 id)
{
    rx_tx_addr[0] = (id >> 24) & 0xFF;
    rx_tx_addr[1] = (id >> 16) & 0xFF;
    rx_tx_addr[2] = (id >>  8) & 0xFF;
    rx_tx_addr[3] = (id >>  0) & 0xFF;
    rx_tx_addr[4] = 0xC1; // always uses first data port
}

// Generate address to use from TX id and manufacturer id (STM32 unique id)
static void initialize_rx_tx_addr()
{
    uint32 lfsr = 0xb2c54a2ful;
  
    set_rx_tx_addr(lfsr);
}


static void YD717_init1()
{
    // for bind packets set address to prearranged value known to receiver
    uint8 bind_rx_tx_addr[] = {0x65, 0x65, 0x65, 0x65, 0x65};
    uint8 i;
    if (PROTOOPTS_FORMAT == FORMAT_SYMAX2)
        for(i=0; i < 5; i++) bind_rx_tx_addr[i]  = 0x60;
    else if (PROTOOPTS_FORMAT == FORMAT_NI_HUI)
        for(i=0; i < 5; i++) bind_rx_tx_addr[i]  = 0x64;

    NRF24L01_WriteRegisterMulti(NRF24L01_0A_RX_ADDR_P0, bind_rx_tx_addr, 5);
    NRF24L01_WriteRegisterMulti(NRF24L01_10_TX_ADDR, bind_rx_tx_addr, 5);
}


static void YD717_init2()
{
    // set rx/tx address for data phase
    NRF24L01_WriteRegisterMulti(NRF24L01_0A_RX_ADDR_P0, rx_tx_addr, 5);
    NRF24L01_WriteRegisterMulti(NRF24L01_10_TX_ADDR, rx_tx_addr, 5);
}

void yd717_init(uint8 unused[])
{
    (void)unused;
    packet_counter = 0;
    flags = 0;
    initialize_rx_tx_addr();
    
#ifdef YD717_TELEMETRY
    memset(&Telemetry, 0, sizeof(Telemetry));
    TELEMETRY_SetType(TELEM_DSM);
#endif
  
    NRF24L01_Initialize();
    while (!NRF24L01_Reset())
      USB_serial_UartPutString("nRF24L01 not found!\r\n");

    // CRC, radio on
    NRF24L01_SetTxRxMode(TX_EN);
    NRF24L01_WriteReg(NRF24L01_00_CONFIG, BV(NRF24L01_00_EN_CRC) | BV(NRF24L01_00_PWR_UP)); 
    NRF24L01_WriteReg(NRF24L01_01_EN_AA, 0x3F);      // Auto Acknoledgement on all data pipes
    NRF24L01_WriteReg(NRF24L01_02_EN_RXADDR, 0x3F);  // Enable all data pipes
    NRF24L01_WriteReg(NRF24L01_03_SETUP_AW, 0x03);   // 5-byte RX/TX address
    NRF24L01_WriteReg(NRF24L01_04_SETUP_RETR, 0x1A); // 500uS retransmit t/o, 10 tries
    NRF24L01_WriteReg(NRF24L01_05_RF_CH, RF_CHANNEL);      // Channel 3C
    NRF24L01_SetBitrate(NRF24L01_BR_1M);             // 1Mbps
    NRF24L01_SetPower(tx_power);
    NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);     // Clear data ready, data sent, and retransmit
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
    NRF24L01_WriteReg(NRF24L01_1C_DYNPD, 0x3F);       // Enable dynamic payload length on all pipes

    // this sequence necessary for module from stock tx
    NRF24L01_ReadReg(NRF24L01_1D_FEATURE);
    NRF24L01_Activate(0x73);                          // Activate feature register
    NRF24L01_ReadReg(NRF24L01_1D_FEATURE);
    NRF24L01_WriteReg(NRF24L01_1C_DYNPD, 0x3F);       // Enable dynamic payload length on all pipes
    NRF24L01_WriteReg(NRF24L01_1D_FEATURE, 0x07);     // Set feature bits on


    NRF24L01_WriteRegisterMulti(NRF24L01_0A_RX_ADDR_P0, rx_tx_addr, 5);
    NRF24L01_WriteRegisterMulti(NRF24L01_10_TX_ADDR, rx_tx_addr, 5);

    // Check for Beken BK2421/BK2423 chip
    // It is done by using Beken specific activate code, 0x53
    // and checking that status register changed appropriately
    // There is no harm to run it on nRF24L01 because following
    // closing activate command changes state back even if it
    // does something on nRF24L01
    NRF24L01_Activate(0x53); // magic for BK2421 bank switch
    USB_serial_UartPutString("Trying to switch banks\r\n");
    if (NRF24L01_ReadReg(NRF24L01_07_STATUS) & 0x80) {
        USB_serial_UartPutString("BK2421 detected\r\n");
        // Beken registers don't have such nice names, so we just mention
        // them by their numbers
        // It's all magic, eavesdropped from real transfer and not even from the
        // data sheet - it has slightly different values
        NRF24L01_WriteRegisterMulti(0x00, (uint8 *) "\x40\x4B\x01\xE2", 4);
        NRF24L01_WriteRegisterMulti(0x01, (uint8 *) "\xC0\x4B\x00\x00", 4);
        NRF24L01_WriteRegisterMulti(0x02, (uint8 *) "\xD0\xFC\x8C\x02", 4);
        NRF24L01_WriteRegisterMulti(0x03, (uint8 *) "\x99\x00\x39\x21", 4);
        NRF24L01_WriteRegisterMulti(0x04, (uint8 *) "\xD9\x96\x82\x1B", 4);
        NRF24L01_WriteRegisterMulti(0x05, (uint8 *) "\x24\x06\x7F\xA6", 4);
        NRF24L01_WriteRegisterMulti(0x0C, (uint8 *) "\x00\x12\x73\x00", 4);
        NRF24L01_WriteRegisterMulti(0x0D, (uint8 *) "\x46\xB4\x80\x00", 4);
        NRF24L01_WriteRegisterMulti(0x04, (uint8 *) "\xDF\x96\x82\x1B", 4);
        NRF24L01_WriteRegisterMulti(0x04, (uint8 *) "\xD9\x96\x82\x1B", 4);
    } else {
        USB_serial_UartPutString("nRF24L01 detected\r\n");
    }
    NRF24L01_Activate(0x53); // switch bank back
}




#ifdef YD717_TELEMETRY
static void update_telemetry() {
  static uint8 frameloss = 0;

  frameloss += NRF24L01_ReadReg(NRF24L01_08_OBSERVE_TX) >> 4;
  NRF24L01_WriteReg(NRF24L01_05_RF_CH, RF_CHANNEL);   // reset packet loss counter

  Telemetry.p.dsm.flog.frameloss = frameloss;
  TELEMETRY_SetUpdated(TELEM_DSM_FLOG_FRAMELOSS);
}
#endif



uint16 yd717_callback(volatile int32 channels[])
{
  memcpy(Channels, (const void *)channels, sizeof(Channels));
        
    switch (phase) {
    case YD717_INIT1:
        send_packet(0);      // receiver doesn't re-enter bind mode if connection lost...check if already bound
        phase = YD717_BIND3;
//        USB_serial_UartPutString("YD717_INIT1\r\n");
        break;

    case YD717_BIND2:
        if (counter == 0) {
            if (packet_ack() == PKT_PENDING)
                return PACKET_CHKTIME;             // packet send not yet complete
//            USB_serial_UartPutString("YD717_BIND2=0\r\n");
            YD717_init2();                         // change to data phase rx/tx address
            send_packet(0);
            phase = YD717_BIND3;
        } else {
            if (packet_ack() == PKT_PENDING)
                return PACKET_CHKTIME;             // packet send not yet complete
            send_packet(1);
            counter -= 1;
        }
        break;

    case YD717_BIND3:
        switch (packet_ack()) {
        case PKT_PENDING:
//            USB_serial_UartPutString("YD717_BIND3_P\r\n");
            return PACKET_CHKTIME;                 // packet send not yet complete
        case PKT_ACKED:
//            USB_serial_UartPutString("YD717_BIND3\r\n");
            phase = YD717_DATA;
            break;
        case PKT_TIMEOUT:
//            USB_serial_UartPutString("YD717_BIND3_T\r\n");
            YD717_init1();                         // change to bind rx/tx address
            counter = BIND_COUNT;
            phase = YD717_BIND2;
            send_packet(1);
        }
        break;

    case YD717_DATA:
#ifdef YD717_TELEMETRY
        update_telemetry();
#endif
        if (packet_ack() == PKT_PENDING)
            return PACKET_CHKTIME;                 // packet send not yet complete
#if 0  // unimplemented channel hopping for Ni Hui quad
        else if (packet_ack() == PKT_TIMEOUT && Model.proto_opts[PROTOOPTS_FORMAT] == FORMAT_NI_HUI) {
            // Sequence (after default channel 0x3C) is channels 0x02, 0x21 (at least for TX Addr is 87 04 14 00)
        }
#endif

        send_packet(0);
        break;
    }
    return PACKET_PERIOD;                          // Packet every 8ms
}






