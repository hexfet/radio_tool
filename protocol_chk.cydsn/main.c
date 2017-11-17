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
void DUT_getchan(uint8 channels[]) {
  static const uint8 match_data[] = {0x25};
  uint8 match_idx = 0;
  uint8 got_chans = 0;
  uint8 spi_data;
//  static char outbuf[16];
  
  DUT_SPI_ClearRxBuffer();
  
  while (got_chans < 4) {
    spi_data = DUT_SPI_ReadRxData();
// snprintf(outbuf, sizeof(outbuf), "%02X", spi_data);
// USB_serial_UartPutString(outbuf);
    if (match_idx >= sizeof(match_data)) {
      *channels++ = spi_data;
      got_chans += 1;
      match_idx = 0;
    } else if (spi_data == match_data[match_idx]) {
      match_idx += 1;
    } else {
      match_idx = 0;
    }
  }
}


#define MAX_CHANS 16
static volatile int32 Channels[MAX_CHANS] = {0, 0, CHAN_MIN_VALUE, 0, 1000, 1000, 1000, 1000};
static volatile uint8 number_of_channels;

static uint16 (*proto_callback)(volatile int32[]) = NULL;
CY_ISR(proto_timer_interrupt_service) {
  if (proto_callback) proto_timer_WritePeriod( proto_callback(Channels) );
  proto_timer_ClearInterrupt(proto_timer_INTR_MASK_TC);
}

static uint32 max_width;
CY_ISR(ppm_timer_interrupt_service) {
  static uint16 prev_capture;
  static uint32 curr_channel;
  uint32 curr_capture = ppm_timer_ReadCapture();
  uint32 width = ((curr_capture - prev_capture) % 0xffff) / 12;   // microseconds - divisor from C/T clock
  prev_capture = curr_capture;
  
  if (width > max_width) max_width = width;
  if (width > 3000) {
    number_of_channels = curr_channel;
    curr_channel = 0;
  } else {
//    Channels[curr_channel++] = ((int)width - 1500) * 20;
    Channels[curr_channel++] = (int)width;
    curr_channel %= MAX_CHANS;
  }
  
  ppm_timer_ClearInterrupt(ppm_timer_INTR_MASK_CC_MATCH);
}

CY_ISR(pwm_timer_interrupt_service) {
  static uint32 int_mode = ppm_timer_TRIG_RISING;  // start mode as set in main
  static uint32 prev_capture;
  uint32 curr_capture = ppm_timer_ReadCapture();
  uint32 width = ((curr_capture - prev_capture) % 0xffff) / 12;   // microseconds - divisor from C/T clock
  prev_capture = curr_capture;
  
  if (int_mode == ppm_timer_TRIG_RISING) {
    ppm_timer_SetCaptureMode(ppm_timer_TRIG_FALLING);
    int_mode = ppm_timer_TRIG_FALLING;
  } else {
    ppm_timer_SetCaptureMode(ppm_timer_TRIG_RISING);
    int_mode = ppm_timer_TRIG_RISING;
    number_of_channels = 1;
    Channels[0] = width;
  }

  ppm_timer_ClearInterrupt(ppm_timer_INTR_MASK_CC_MATCH);
}

// AETR1234
#define AILERON   0
#define ELEVATOR  1
#define THROTTLE  2
#define RUDDER    3
#define AUX1      4
#define CTL_STEP  500

void channel_step(uint8 channel, int32 step) {
  Channels[channel] += step;
  if (Channels[channel] > CHAN_MAX_VALUE) Channels[channel] = CHAN_MAX_VALUE;
  if (Channels[channel] < CHAN_MIN_VALUE) Channels[channel] = CHAN_MIN_VALUE;
}

void proto_run(void (*init)(uint8 tx_addr[]), uint16 (*callback)(volatile int32 channels[])) {
  uint8 def_addr[] = {0x3b,0xb6,0x00,0x00,0xa2};
  uint8 ch, loop=1;
  
  if (init)
    init(def_addr);

  // run the protocol callback with timer interrupt
  proto_timer_int_Disable();  // just in case
  proto_timer_int_ClearPending();
  proto_callback = callback;
  proto_timer_int_Enable();
  
  while(loop) {
    if (USB_serial_SpiUartGetRxBufferSize()) {
      switch (ch=USB_serial_UartGetChar()) {
#if 0
      case '1':
      case '2':
      case '3':
      case '4':
        Channels[AUX1+(ch-'1')] *= -1;
        break;
      case 'w': channel_step(THROTTLE,  CTL_STEP); break;
      case 's': channel_step(THROTTLE, -CTL_STEP); break;
      case 'a': channel_step(RUDDER,    CTL_STEP); break;
      case 'd': channel_step(RUDDER,   -CTL_STEP); break;
      case 'i': channel_step(ELEVATOR,  CTL_STEP); break;
      case 'k': channel_step(ELEVATOR, -CTL_STEP); break;
      case 'j': channel_step(AILERON,   CTL_STEP); break;
      case 'l': channel_step(AILERON,  -CTL_STEP); break;
#endif     
      case ' ':
        Channels[THROTTLE] = CHAN_MIN_VALUE;
        Channels[RUDDER] = 0;
        Channels[ELEVATOR] = 0;
        Channels[AILERON] = 0;
        break;
        
      case 'q':
        proto_timer_int_Disable();
        loop = 0;
      }
    }
  }
}

void symax_capture() {
  uint8 tx_addr[5], channels[4];
//  uint32 number = 0x0000b63b;
  uint32 number = 0x00001946;
  char outbuf[64];
  uint32 loopcount;
  
  tx_addr[4] = 0xa2;
  
  
  for(;;) {
    proto_timer_int_Disable();
    
    DUT_reset();
    
    memcpy(tx_addr, &number, sizeof(uint32));
    symax_init(tx_addr);

    // run the protocol callback with timer interrupt
    proto_timer_int_ClearPending();
    proto_callback = symax_callback;
    proto_timer_int_Enable();
    
    // wait for data phase
    loopcount = 0;
    while (symax_phase != 3) {
      if (loopcount++ > 5000) break;
      CyDelay(10);
    }
    if (loopcount > 5000) continue;
    
    DUT_getchan(channels);
    
    snprintf(outbuf, sizeof(outbuf),
             "%02X%02X%02X%02X%02X %02X%02X%02X%02X\r\n", tx_addr[4], tx_addr[3],
             tx_addr[2], tx_addr[1], tx_addr[0], channels[0], channels[1], channels[2], channels[3]);
    USB_serial_UartPutString(outbuf);
    
    number += 1;
  }
}

void symax_bind32() {
  uint8 tx_addr[5];
  uint32 number = 0xffffffe0;
  char outbuf[64];
  uint32 loopcount;
  uint8 ch;
  uint8 loop;
  
  tx_addr[4] = 0xa2;
  
  for(loopcount = 0; loopcount < 32; loopcount++) {
    proto_timer_int_Disable();
    
    DUT_reset();
    
    memcpy(tx_addr, &number, sizeof(uint32));
    symax_init(tx_addr);

    // run the protocol callback with timer interrupt
    proto_timer_int_ClearPending();
    proto_callback = symax_callback;
    proto_timer_int_Enable();

    loop = 1;
    while(loop) {
          // wait for data phase
      if (symax_phase == 3) {
        snprintf(outbuf, sizeof(outbuf),
             "%02X%02X%02X%02X%02X bound\r\n", tx_addr[4], tx_addr[3],
             tx_addr[2], tx_addr[1], tx_addr[0]);
        USB_serial_UartPutString(outbuf);
      }
      CyDelay(10);

      if (USB_serial_SpiUartGetRxBufferSize()) {
        switch (ch=USB_serial_UartGetChar()) {
        case ' ':
          loop = 0;
          break;
        case 'q':
          proto_timer_int_Disable();
          return;
        }
      }
    }
  number += 1;
  }
}


void read_xn297() {
  char outbuf[64];
  uint8 data[16];
  
  NRF24L01_ReadRegisterMulti(0x1f, data, 5);
  snprintf(outbuf, sizeof(outbuf),
           "0x1F: %02X%02X%02X%02X%02X\r\n", data[0], data[1], data[2], data[3], data[4]);
  USB_serial_UartPutString(outbuf);
  
  NRF24L01_ReadRegisterMulti(0x1e, data, 7);
  snprintf(outbuf, sizeof(outbuf),
           "0x1E: %02X%02X%02X%02X%02X%02X%02X\r\n", data[0], data[1], data[2], data[3], data[4], data[5], data[6]);
  USB_serial_UartPutString(outbuf);
  
  NRF24L01_ReadRegisterMulti(0x19, data, 5);
  snprintf(outbuf, sizeof(outbuf),
           "0x19: %02X%02X%02X%02X%02X\r\n", data[0], data[1], data[2], data[3], data[4]);
  USB_serial_UartPutString(outbuf);
  
  NRF24L01_ReadRegisterMulti(0x07, data, 1);
  snprintf(outbuf, sizeof(outbuf),
           "0x07: %02X\r\n", data[0]);
  USB_serial_UartPutString(outbuf);

}

uint16 ppm_monitor(volatile int32 channels[])
{
  CyDelay(300);
  static char outbuf[127];
  char *pc = outbuf;
  int8 nc;
  int32 chars_out;
  int size = sizeof outbuf;

  nc = number_of_channels;
  for (int i=0; i < nc; i++) {
    chars_out = snprintf(pc, size, "%6ld ", (int32)channels[i]);
    pc += chars_out;
    size -= chars_out;
  }
  if (nc) snprintf(pc, size, "   max_width %ld\r\n", (int32)max_width);
  USB_serial_UartPutString(outbuf);   
  return 10000;
}

int main() {
  uint8 led = 0;
  uint8 ch;
  
  CyGlobalIntEnable; /* enable global interrupts. */

  DUT_reset();
  nRF_SPI_Start();
  USB_serial_Start();
  DUT_SPI_Start();
  proto_timer_int_StartEx(proto_timer_interrupt_service);
  proto_timer_Start();


 
  for(;;) {

    /* Get received character or zero if nothing has been received yet */
    ch = USB_serial_UartGetChar(); 
    if (ch != 0u) {
      USB_serial_UartPutChar(ch);
      USB_serial_UartPutString("\r\n");
    }
  
    switch(ch) {
    case '1':
      USB_serial_UartPutString("Running YD717\r\n");
      proto_run(yd717_init, yd717_callback);
      break;
    case '2':
      USB_serial_UartPutString("Running SymaX\r\n");
      proto_run(symax_init, symax_callback);
      break;
    case '3':
      USB_serial_UartPutString("symax_capture start\r\n");
      symax_capture();
      break;
    case '4':
      USB_serial_UartPutString("symax_bind32 start\r\n");
      symax_bind32();
      break;      
    case '5':
      USB_serial_UartPutString("Running CX10A with capture\r\n");
      proto_run(cx10_init, cx10_callback);
      read_xn297();
      break;
    case '6':
      USB_serial_UartPutString("PPM monitor - pulse width in microseconds\r\n");
      ppm_timer_Stop();
      ppm_timer_SetCaptureMode(ppm_timer_TRIG_FALLING);
      ppm_timer_int_StartEx(ppm_timer_interrupt_service);
      number_of_channels = 0;
      ppm_timer_Start();
      proto_run(NULL, ppm_monitor);
      break;    
    case '7':
      USB_serial_UartPutString("PWM monitor - pulse width in microseconds\r\n");
      ppm_timer_Stop();
      ppm_timer_SetCaptureMode(ppm_timer_TRIG_RISING);
      ppm_timer_int_StartEx(pwm_timer_interrupt_service);
      number_of_channels = 0;
      ppm_timer_Start();
      proto_run(NULL, ppm_monitor);
      break;    
    case 'l':
      P1_6_Write(led ^= 1);
      break;
    case 'r':
      CySoftwareReset();
    case '?':
    case 'h':
      USB_serial_UartPutString("1 - bind yd717\r\n");
      USB_serial_UartPutString("2 - bind symax\r\n");
      USB_serial_UartPutString("3 - symax capture\r\n");
      USB_serial_UartPutString("4 - symax bind 32\r\n");
      USB_serial_UartPutString("5 - bind CX10A\r\n");
      USB_serial_UartPutString("6 - PPM monitor\r\n"); 
      USB_serial_UartPutString("7 - PWM monitor\r\n");      
      USB_serial_UartPutString("r - reset\r\n");
      USB_serial_UartPutString("l - led\r\n");
      break;
    }
  }
}

