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
uint32 DUT_wait_radioid() {
  static const uint8 match_data[] = {0x00, 0x06};
  uint8 match_idx = 0;
  uint8 bytes_found = 0;
  uint8 spi_data;
  uint32 radio_id = 0;
//char outbuf[10];
  
  DUT_SPI_ClearRxBuffer();
 
// look for 06 command then capture following 4 bytes
  for (;;) {
    if (!DUT_SPI_GetRxBufferSize()) {
      CyDelayUs(30);
      continue;
    }
    spi_data = DUT_SPI_ReadRxData();
//snprintf(outbuf, sizeof(outbuf), "%02X", spi_data);
//USB_serial_UartPutString(outbuf);
    if (match_idx >= sizeof(match_data)) {
      radio_id = (radio_id << 8) + spi_data;
      bytes_found += 1;
      if (bytes_found == sizeof radio_id) {
        if (radio_id != 0xac59a453) break;   // ignore known bind value as it appears sometimes after bind
        bytes_found = 0;
        match_idx = 0;
      }
    } else if (spi_data == match_data[match_idx]) {
      match_idx += 1;
    } else if (match_idx > 0 && match_data[match_idx-1] != spi_data) {   // this logic only works for match_data size 2
      bytes_found = 0;
      match_idx = 0;
    }
  }
  return radio_id;
}

void rx_bind(const uint16 rxid) {
  char buffer[80];
  
  snprintf(buffer, sizeof buffer, "C%0x04X\r", rxid);
  UART_RC_SpiUartPutArray((const uint8 *)buffer, strlen(buffer));
}


void bugs3_capture() {
  uint32 radio_id;
  uint16 rxid;
  uint8 loop = 1;
  char c;
  char *outp, *endbuf, outbuf[80];
      
  for(rxid=0; loop && (rxid < 0xffff); rxid++) {
    if (USB_serial_SpiUartGetRxBufferSize()) {
      switch (c=USB_serial_UartGetChar()) {
      case 'q':
        loop = 0;
        continue;
      }
    }
    
    DUT_reset();                     // reset bugs3 TX with bind button depressed
    rx_bind(rxid);                   // tell deviation to bind with this receiver id
    radio_id = DUT_wait_radioid();   // capture the radio id set by stock tx
  
    outp = outbuf;
    endbuf = outbuf + sizeof outbuf;
    outp += snprintf(outp, endbuf-outp, "%04X,%08lX", rxid, radio_id);
    snprintf(outp, endbuf-outp, "\r\n");
    USB_serial_UartPutString(outbuf);

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
    case 'b':
      Bootloadable_1_Load();
    case '?':
    case 'h':
      USB_serial_UartPutString("1 - bugs3 capture\r\n");
      USB_serial_UartPutString("r - reset\r\n");
      USB_serial_UartPutString("l - led\r\n");
      USB_serial_UartPutString("b - enter bootloader\r\n");
      break;
    }
  }
}

