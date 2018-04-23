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
#include <stdlib.h>
#include <stdio.h>
#include "nrf24l01.h"
#include "protocols.h"


void printd(char *str, uint32 data) {
  char outbuf[50];
  snprintf(outbuf, sizeof(outbuf), str, data);
  USB_serial_UartPutString(outbuf);
}

void DUT_reset() {
  DUT_rst_Write(0);
  CyDelay(2);
  DUT_rst_Write(1);
  CyDelay(20);
}


// Scan through received data looking for pattern match
void DUT_getfreq(uint8 *freqs) {
  static const uint8 match_data[] = {0x0F};
  uint8 match_idx = 0;
  uint8 freq_count = 0;
  uint8 spi_data;
  
  DUT_SPI_ClearRxBuffer();
 
// look for 16 unique frequencies
  while (freq_count < 16) {
    spi_data = DUT_SPI_ReadRxData();
// snprintf(outbuf, sizeof(outbuf), "%02X", spi_data);
// USB_serial_UartPutString(outbuf);
    if (match_idx >= sizeof(match_data)) {
      if (!memchr(freqs, spi_data, freq_count)) {
        *freqs++ = spi_data;
        freq_count += 1;
      }
      match_idx = 0;
    } else if (spi_data == match_data[match_idx]) {
      match_idx += 1;
    } else {
      match_idx = 0;
    }
  }
}

void rc_send(const uint8 *bytes, const int len) {
  UART_RC_SpiUartPutArray((const uint8 *)"B", 1);
  UART_RC_SpiUartPutArray(bytes, len);
}

uint8 rc_recv(uint8 *bytes, const int len, const uint32 seconds) {
  uint8 *p = bytes;
  uint8 count = 0;
  uint32 timeout = 10 * seconds;

  while (timeout--) {
    if (UART_RC_SpiUartGetRxBufferSize() >= 1) {
      *p = UART_RC_SpiUartReadRxData();
      count++;
      if (count == len) break;
    } else {
      CyDelayUs(100);
    }
  }
  return count;
}

void bugs3_capture() {
  uint8 tx_id[3];
  uint8 rcbuf[16], result;
  char outbuf[256], *outp, *endbuf;
  uint8 frequencies[16];
  
  tx_id[0] = 0;
  tx_id[1] = 0;
  tx_id[2] = 0;
  
  for(;;) {
    proto_timer_int_Disable();
    
    DUT_reset();
    rc_send(tx_id, sizeof(tx_id));
    
    // wait for bound message
    result = rc_recv(rcbuf, 1, 10);
    if (!result) continue;
    
    DUT_getfreq(frequencies);
    
    outp = outbuf;
    endbuf = outbuf + sizeof outbuf;
    outp += snprintf(outp, endbuf-outp, "%02X%02X%02X,", tx_id[2], tx_id[1], tx_id[0]);
    for (int i=0; i < 16; i++)
      outp += snprintf(outp, endbuf-outp, "%02X", frequencies[i]);
    snprintf(outp, endbuf-outp, "\r\n");
    USB_serial_UartPutString(outbuf);

    tx_id[0] += 1;
    if (tx_id[0] == 0) tx_id[1] += 1;
    if (tx_id[1] == 0) tx_id[2] += 1;
  }
}




int main() {
  uint8 led = 0;
  uint8 ch;
  
  CyGlobalIntEnable; /* enable global interrupts. */

  DUT_reset();
  USB_serial_Start();
  DUT_SPI_Start();
  UART_RC_Start();
//  proto_timer_int_StartEx(proto_timer_interrupt_service);
//  proto_timer_Start();

  for(;;) {
    /* Get received character or zero if nothing has been received yet */
    ch = USB_serial_UartGetChar(); 
    if (ch != 0u) {
      USB_serial_UartPutChar(ch);
      USB_serial_UartPutString("\r\n");
    }
  
    switch(ch) {
    case '1':
      USB_serial_UartPutString("bugs3 capture start\r\n");
      bugs3_capture();
      break;

    case 'l':
      P1_6_Write(led ^= 1);
      break;
    case 'r':
      CySoftwareReset();
    case '?':
    case 'h':
      USB_serial_UartPutString("1 - bugs3 capture\r\n");
      USB_serial_UartPutString("r - reset\r\n");
      USB_serial_UartPutString("l - led\r\n");
      break;
    }
  }
}

