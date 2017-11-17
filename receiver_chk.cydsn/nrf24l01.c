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



/* Instruction Mnemonics */
#define R_REGISTER    0x00
#define W_REGISTER    0x20
#define REGISTER_MASK 0x1F
#define ACTIVATE      0x50
#define R_RX_PL_WID   0x60
#define R_RX_PAYLOAD  0x61
#define W_TX_PAYLOAD  0xA0
#define W_ACK_PAYLOAD 0xA8
#define FLUSH_TX      0xE1
#define FLUSH_RX      0xE2
#define REUSE_TX_PL   0xE3
#define NOP           0xFF


static uint8 rf_setup = 0x0f;

#define BUFLEN  32
static uint8 outbuf[BUFLEN];


// possible infinite loops here...
void SPI_wait_done() {
  while(0u == (nRF_SPI_GetMasterInterruptSource() & nRF_SPI_INTR_MASTER_SPI_DONE))
    ;  /* Wait while Master completes transfer */
    /* Clear interrupt source after transfer completion */
  nRF_SPI_ClearMasterInterruptSource(nRF_SPI_INTR_MASTER_SPI_DONE);
}
uint8 SPI_wait_data() {
  while (!nRF_SPI_SpiUartGetRxBufferSize())
    ;
  return nRF_SPI_SpiUartGetRxBufferSize();
}
  
  
uint8 NRF24L01_WriteReg(uint8 reg, uint8 data)
{
  nRF_SPI_SpiUartClearRxBuffer();
  nRF_SPI_SpiUartWriteTxData(W_REGISTER | (REGISTER_MASK & reg));
  nRF_SPI_SpiUartWriteTxData(data);
  SPI_wait_done();
  SPI_wait_data();
  return nRF_SPI_SpiUartReadRxData();
}

uint8 NRF24L01_WriteRegisterMulti(uint8 reg, const uint8 data[], uint8 length)
{
  uint8 outbuf[32];
  
  outbuf[0] = W_REGISTER | ( REGISTER_MASK & reg);
  memcpy(&outbuf[1], data, length > (BUFLEN-1) ? (BUFLEN-1) : length);

  nRF_SPI_SpiUartClearRxBuffer();
  nRF_SPI_SpiUartPutArray(outbuf, length > (BUFLEN-1) ? BUFLEN : length + 1);
  SPI_wait_done();
  SPI_wait_data();
  return nRF_SPI_SpiUartReadRxData();
}

uint8 NRF24L01_WritePayload(uint8 *data, uint8 length)
{
  nRF_SPI_SpiUartClearRxBuffer();
  outbuf[0] = W_TX_PAYLOAD;
  memcpy(&outbuf[1], data, length > (BUFLEN-1) ? (BUFLEN-1) : length);
  nRF_SPI_SpiUartPutArray(outbuf, length > (BUFLEN-1) ? BUFLEN : length + 1);
  SPI_wait_done();
  SPI_wait_data();
  return nRF_SPI_SpiUartReadRxData();
}

uint8 NRF24L01_ReadReg(uint8 reg)
{
  nRF_SPI_SpiUartClearRxBuffer();
  outbuf[0] = R_REGISTER | (REGISTER_MASK & reg);
  nRF_SPI_SpiUartPutArray(outbuf, 2);
  SPI_wait_done();
  nRF_SPI_SpiUartReadRxData();
  
  SPI_wait_data();
  return nRF_SPI_SpiUartReadRxData();
}

uint8 NRF24L01_ReadRegisterMulti(uint8 reg, uint8 data[], uint8 length)
{
  uint8 res, i;

  nRF_SPI_SpiUartClearRxBuffer();
  outbuf[0] = R_REGISTER | (REGISTER_MASK & reg);
  nRF_SPI_SpiUartPutArray(outbuf, length+1);
  SPI_wait_done();
  while (SPI_wait_data() != (length+1))
    ;
  res = nRF_SPI_SpiUartReadRxData();
  for(i = 0; i < length; i++) {
    data[i] =   nRF_SPI_SpiUartReadRxData();
  }
  
  return res;
}

uint8 NRF24L01_ReadPayload(uint8 *data, uint8 length)
{
  uint8 res, i;

  nRF_SPI_SpiUartClearRxBuffer();
  outbuf[0] = R_RX_PAYLOAD;
  nRF_SPI_SpiUartPutArray(outbuf, length+1);
  SPI_wait_done();
  while (SPI_wait_data() != (length+1))
    ;
  res = nRF_SPI_SpiUartReadRxData();
  for(i = 0; i < length; i++) {
    data[i] =   nRF_SPI_SpiUartReadRxData();
  }
 
  return res;
}

static uint8 Strobe(uint8 state)
{
  nRF_SPI_SpiUartClearRxBuffer();
  nRF_SPI_SpiUartWriteTxData(state);
  SPI_wait_done();

  SPI_wait_data();
  return nRF_SPI_SpiUartReadRxData();
}

uint8 NRF24L01_FlushTx()
{
    return Strobe(FLUSH_TX);
}

uint8 NRF24L01_FlushRx()
{
    return Strobe(FLUSH_RX);
}

uint8 NRF24L01_Activate(uint8 code)
{
  nRF_SPI_SpiUartClearRxBuffer();
  outbuf[0] = ACTIVATE;
  outbuf[1] = code;
  nRF_SPI_SpiUartPutArray(outbuf, 2);
  SPI_wait_done();
  SPI_wait_data();
  return nRF_SPI_SpiUartReadRxData();
}

uint8 NRF24L01_SetBitrate(uint8 bitrate)
{
    // Note that bitrate 250kbps (and bit RF_DR_LOW) is valid only
    // for nRF24L01+. There is no way to programmatically tell it from
    // older version, nRF24L01, but the older is practically phased out
    // by Nordic, so we assume that we deal with with modern version.

    // Bit 0 goes to RF_DR_HIGH, bit 1 - to RF_DR_LOW
    rf_setup = (rf_setup & 0xD7) | ((bitrate & 0x02) << 4) | ((bitrate & 0x01) << 3);
    return NRF24L01_WriteReg(NRF24L01_06_RF_SETUP, rf_setup);
}

// Power setting is 0..3 for nRF24L01
// Claimed power amp for nRF24L01 from eBay is 20dBm. 
//      Raw            w 20dBm PA
// 0 : -18dBm  (16uW)   2dBm (1.6mW)
// 1 : -12dBm  (60uW)   8dBm   (6mW)
// 2 :  -6dBm (250uW)  14dBm  (25mW)
// 3 :   0dBm   (1mW)  20dBm (100mW)
// So it maps to Deviation as follows
/*
TXPOWER_100uW  = -10dBm
TXPOWER_300uW  = -5dBm
TXPOWER_1mW    = 0dBm
TXPOWER_3mW    = 5dBm
TXPOWER_10mW   = 10dBm
TXPOWER_30mW   = 15dBm
TXPOWER_100mW  = 20dBm
TXPOWER_150mW  = 22dBm
*/
uint8 NRF24L01_SetPower(uint8 power)
{
    uint8 nrf_power = 3;
    switch(power) {
        case TXPOWER_100uW: nrf_power = 0; break;
        case TXPOWER_300uW: nrf_power = 0; break;
        case TXPOWER_1mW:   nrf_power = 0; break;
        case TXPOWER_3mW:   nrf_power = 1; break;
        case TXPOWER_10mW:  nrf_power = 1; break;
        case TXPOWER_30mW:  nrf_power = 2; break;
        case TXPOWER_100mW: nrf_power = 3; break;
        case TXPOWER_150mW: nrf_power = 3; break;
        default:            nrf_power = 0; break;
    };
    // Power is in range 0..3 for nRF24L01
    rf_setup = (rf_setup & 0xF9) | ((nrf_power & 0x03) << 1);
    return NRF24L01_WriteReg(NRF24L01_06_RF_SETUP, rf_setup);
}


void NRF24L01_SetTxRxMode(enum TxRxState mode)
{
    if(mode == TX_EN) {
//        CE_lo();
        NRF24L01_WriteReg(NRF24L01_07_STATUS, (1 << NRF24L01_07_RX_DR)    //reset the flag(s)
                                            | (1 << NRF24L01_07_TX_DS)
                                            | (1 << NRF24L01_07_MAX_RT));
        NRF24L01_WriteReg(NRF24L01_00_CONFIG, (1 << NRF24L01_00_EN_CRC)   // switch to TX mode
                                            | (1 << NRF24L01_00_CRCO)
                                            | (1 << NRF24L01_00_PWR_UP));
        CyDelayUs(130);
//        CE_hi();
    } else if (mode == RX_EN) {
//        CE_lo();
        NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);        // reset the flag(s)
        NRF24L01_WriteReg(NRF24L01_00_CONFIG, 0x0F);        // switch to RX mode
        NRF24L01_WriteReg(NRF24L01_07_STATUS, (1 << NRF24L01_07_RX_DR)    //reset the flag(s)
                                            | (1 << NRF24L01_07_TX_DS)
                                            | (1 << NRF24L01_07_MAX_RT));
        NRF24L01_WriteReg(NRF24L01_00_CONFIG, (1 << NRF24L01_00_EN_CRC)   // switch to RX mode
                                            | (1 << NRF24L01_00_CRCO)
                                            | (1 << NRF24L01_00_PWR_UP)
                                            | (1 << NRF24L01_00_PRIM_RX));
        CyDelayUs(130);
//        CE_hi();
    } else {
        NRF24L01_WriteReg(NRF24L01_00_CONFIG, (1 << NRF24L01_00_EN_CRC)); //PowerDown
//        CE_lo();
    }
}



int NRF24L01_Reset()
{

    nRF_SPI_SpiUartClearTxBuffer();
    NRF24L01_FlushTx();
    NRF24L01_FlushRx();
    uint8 status1 = Strobe(NOP);
    uint8 status2 = NRF24L01_ReadReg(0x07);
    NRF24L01_SetTxRxMode(TXRX_OFF);
#ifdef EMULATOR
    return 1;
#endif
    return (status1 == status2 && (status1 & 0x0f) == 0x0e);

}

void NRF24L01_Initialize() {
    rf_setup = 0x0F;
}    



// XN297 emulation layer

static int xn297_addr_len;
static uint8  xn297_tx_addr[5];
static uint8  xn297_rx_addr[5];
static uint8  xn297_crc = 0;
static uint8  is_xn297 = 0;
static const uint8_t xn297_scramble[] = {
  0xe3, 0xb1, 0x4b, 0xea, 0x85, 0xbc, 0xe5, 0x66,
  0x0d, 0xae, 0x8c, 0x88, 0x12, 0x69, 0xee, 0x1f,
  0xc7, 0x62, 0x97, 0xd5, 0x0b, 0x79, 0xca, 0xcc,
  0x1b, 0x5d, 0x19, 0x10, 0x24, 0xd3, 0xdc, 0x3f,
  0x8e, 0xc5, 0x2f};

 static const uint16 xn297_crc_xorout[] = {
    0x0000, 0x3448, 0x9BA7, 0x8BBB, 0x85E1, 0x3E8C, // 1st entry is missing, probably never needed
    0x451E, 0x18E6, 0x6B24, 0xE7AB, 0x3828, 0x8148, // it's used for 3-byte address w/ 0 byte payload only
    0xD461, 0xF494, 0x2503, 0x691D, 0xFE8B, 0x9BA7,
    0x8B17, 0x2920, 0x8B5F, 0x61B1, 0xD391, 0x7401, 
    0x2138, 0x129F, 0xB3A0, 0x2988};
  
static uint8_t bit_reverse(uint8_t b_in)
{
    uint8_t b_out = 0;
    int i;
    for (i = 0; i < 8; ++i) {
        b_out = (b_out << 1) | (b_in & 1);
        b_in >>= 1;
    }
    return b_out;
}


static const uint16_t polynomial = 0x1021;
static const uint16_t initial    = 0xb5d2;

static uint16_t crc16_update(uint16_t crc, unsigned char a)
{
    crc ^= a << 8;
    int i;
    for (i = 0; i < 8; ++i) {
        if (crc & 0x8000) {
            crc = (crc << 1) ^ polynomial;
        } else {
            crc = crc << 1;
        }
    }
    return crc;
}


void XN297_SetTXAddr(const uint8* addr, int len)
{
    if (len > 5) len = 5;
    if (len < 3) len = 3;
    if (is_xn297) {
        uint8 buf[] = { 0, 0, 0, 0, 0 };
        memcpy(buf, addr, len);
        NRF24L01_WriteReg(NRF24L01_03_SETUP_AW, len-2);
        NRF24L01_WriteRegisterMulti(NRF24L01_0A_RX_ADDR_P0, buf, 5);
    } else {
        uint8 buf[] = { 0x55, 0x0F, 0x71, 0x0C, 0x00 }; // bytes for XN297 preamble 0xC710F55 (28 bit)
        xn297_addr_len = len;
        if (xn297_addr_len < 4) {
            int i;
            for (i = 0; i < 4; ++i) {
                buf[i] = buf[i+1];
            }
        }
        NRF24L01_WriteReg(NRF24L01_03_SETUP_AW, len-2);
        NRF24L01_WriteRegisterMulti(NRF24L01_10_TX_ADDR, buf, 5);
        // Receive address is complicated. We need to use scrambled actual address as a receive address
        // but the TX code now assumes fixed 4-byte transmit address for preamble. We need to adjust it
        // first. Also, if the scrambled address begings with 1 nRF24 will look for preamble byte 0xAA
        // instead of 0x55 to ensure enough 0-1 transitions to tune the receiver. Still need to experiment
        // with receiving signals.
        memcpy(xn297_tx_addr, addr, len);
    }
}


void XN297_SetRXAddr(const uint8* addr, int len)
{
    if (len > 5) len = 5;
    if (len < 3) len = 3;
    uint8 buf[] = { 0, 0, 0, 0, 0 };
    memcpy(buf, addr, len);
    if (is_xn297) {
        NRF24L01_WriteReg(NRF24L01_03_SETUP_AW, len-2);
        NRF24L01_WriteRegisterMulti(NRF24L01_0A_RX_ADDR_P0, buf, 5);
    } else {
        memcpy(xn297_rx_addr, addr, len);
        int i;
        for (i = 0; i < xn297_addr_len; ++i) {
            buf[i] = xn297_rx_addr[i] ^ xn297_scramble[xn297_addr_len-i-1];
        }
        NRF24L01_WriteReg(NRF24L01_03_SETUP_AW, len-2);
        NRF24L01_WriteRegisterMulti(NRF24L01_0A_RX_ADDR_P0, buf, 5);
    }
}


void XN297_Configure(uint8 flags)
{
    if (!is_xn297) {
        xn297_crc = !!(flags & BV(NRF24L01_00_EN_CRC));
        flags &= ~(BV(NRF24L01_00_EN_CRC) | BV(NRF24L01_00_CRCO));
    }
    NRF24L01_WriteReg(NRF24L01_00_CONFIG, flags);      
}


uint8 XN297_WritePayload(uint8* msg, int len)
{
    uint8 packet[32];
    uint8 res;
    if (is_xn297) {
        res = NRF24L01_WritePayload(msg, len);
    } else {
        int last = 0;
        if (xn297_addr_len < 4) {
            // If address length (which is defined by receive address length)
            // is less than 4 the TX address can't fit the preamble, so the last
            // byte goes here
            packet[last++] = 0x55;
        }
        int i;
        for (i = 0; i < xn297_addr_len; ++i) {
            packet[last++] = xn297_tx_addr[xn297_addr_len-i-1] ^ xn297_scramble[i];
        }

        for (i = 0; i < len; ++i) {
            // bit-reverse bytes in packet
            uint8 b_out = bit_reverse(msg[i]);
            packet[last++] = b_out ^ xn297_scramble[xn297_addr_len+i];
        }
        if (xn297_crc) {
            int offset = xn297_addr_len < 4 ? 1 : 0;
            uint16 crc = initial;
            for (i = offset; i < last; ++i) {
                crc = crc16_update(crc, packet[i]);
            }
            crc ^= xn297_crc_xorout[xn297_addr_len - 3 + len];
            packet[last++] = crc >> 8;
            packet[last++] = crc & 0xff;
        }
        res = NRF24L01_WritePayload(packet, last);
    }
    return res;
}


uint8 XN297_ReadPayload(uint8* msg, int len)
{
    // TODO: if xn297_crc==1, check CRC before filling *msg 
    uint8 res = NRF24L01_ReadPayload(msg, len);
    uint8 i;
    for(i=0; i<len; i++)
      msg[i] = bit_reverse(msg[i])^bit_reverse(xn297_scramble[i+xn297_addr_len]);
    return res;
}


// End of XN297 emulation


