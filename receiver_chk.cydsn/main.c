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
#include "protocols.h"


void printd(char *str, uint32 data) {
  char outbuf[50];
  snprintf(outbuf, sizeof(outbuf), str, data);
  USB_serial_UartPutString(outbuf);
}



#define MAX_CHANS 18
static volatile int32 Channels[MAX_CHANS];
static volatile uint8 number_of_channels;

static uint16 (*proto_callback)(volatile int32[]) = NULL;
CY_ISR(proto_timer_interrupt_service) {
  if (proto_callback) proto_timer_WritePeriod( proto_callback(Channels) );
  proto_timer_ClearInterrupt(proto_timer_INTR_MASK_TC);
}

static uint32 max_width;
CY_ISR(ppm_timer_interrupt_service) {
  Pin_sigout_Write(1);
  ppm_timer_ClearInterrupt(ppm_timer_INTR_MASK_CC_MATCH);
  
  static uint32 prev_capture;
  static uint32 curr_channel;
  uint32 curr_capture = ppm_timer_ReadCapture();
  uint32 width = ((curr_capture - prev_capture) % 0xffff) / 3;   // microseconds - divisor from C/T clock
  prev_capture = curr_capture;
  
  if (width > max_width) max_width = width;
  if (width > 4500) {
    number_of_channels = curr_channel;
    curr_channel = 0;
  } else {
//    Channels[curr_channel++] = ((int)width - 1500) * 20;
    Channels[curr_channel++] = (int)width;
    curr_channel %= MAX_CHANS;
  }
  Pin_sigout_Write(0);
}

CY_ISR(pwm_timer_interrupt_service) {
  static uint32 int_mode = ppm_timer_TRIG_RISING;  // start mode as set in main
  static uint32 prev_capture;
  uint32 curr_capture = ppm_timer_ReadCapture();
  uint32 width = ((curr_capture - prev_capture) % 0xffff) / 3;   // microseconds - divisor from C/T clock
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
      case 'q':
        proto_timer_int_Disable();
        loop = 0;
      }
    }
  }
}


uint16 ppm_monitor(volatile int32 channels[])
{
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
  if (nc) snprintf(pc, size, "   max_width %ld, channels %d\r\n", (int32)max_width, nc);
  USB_serial_UartPutString(outbuf);   
  return 30000;
}


uint16 sbus_monitor() {
  static char outbuf[256];
  char *pc = outbuf;
  int32 chars_out;
  int size = sizeof outbuf;

  static uint8 inbuf[25];
  static uint8 inbuf_idx = 0;
  uint32 c;
  uint8 frame_loss, failsafe, loop=1;
  while(loop) {
    if (USB_serial_SpiUartGetRxBufferSize()) {
      switch (c=USB_serial_UartGetChar()) {
      case 'q':
        loop = 0;
        continue;
      }
    }
    
    while (UART_in_SpiUartGetRxBufferSize()) {
      c = UART_in_UartGetByte();
// USB_serial_UartPutChar(c);
      switch (inbuf_idx) {
      case 24:
    		if (c == 0x00) {
    			inbuf[inbuf_idx] = c;

          uint8 byte_in_sbus = 1;
    			uint8 bit_in_sbus = 0;
    			uint8 ch = 0;
    			uint8 bit_in_channel = 0;
          memset((void *)Channels, 0, sizeof Channels);
          
    			// channel data is interleaved and bit-reversed
          for (int i=0; i < 176; i++) {
    				if (inbuf[byte_in_sbus] & (1<<bit_in_sbus)) {
    						Channels[ch] |= (1<<bit_in_channel);
    				}
    				bit_in_sbus = (bit_in_sbus + 1) % 8;
            if (bit_in_sbus == 0) byte_in_sbus++;
    				bit_in_channel = (bit_in_channel + 1) % 11;
            if (bit_in_channel == 0) ch++;            
    			}
          Channels[16] = (inbuf[23] & 1) ? 2000 : 1000;
          Channels[17] = (inbuf[23] & 2) ? 2000 : 1000;
    			frame_loss   = (inbuf[23] & 4) ? 1 : 0;
    			failsafe     = (inbuf[23] & 8) ? 1 : 0;
          
          for (int i=0; i < MAX_CHANS; i++) {
            chars_out = snprintf(pc, size, "%4ld ", (int32)Channels[i]);
            pc += chars_out;
            size -= chars_out;
          }
          chars_out = snprintf(pc, size, "fl %d, fs %d\n\r", frame_loss, failsafe);
          Pin_sigout_Write(1);
          USB_serial_UartPutString(outbuf);
          while (USB_serial_SpiUartGetTxBufferSize()) CyDelayUs(10);
          Pin_sigout_Write(0);
        }
        inbuf_idx = 0;
        pc = outbuf;
        size = sizeof outbuf;
        UART_in_SpiUartClearRxBuffer();
        break;
              
      case 0:
        if (c != 0x0f) break;
        // intentional fall-through
      default:
        inbuf[inbuf_idx++] = c;
      }
    }

  }
  return 6000;
}


int main() {
  uint8 led = 0;
  uint8 ch;
  
  CyGlobalIntEnable; /* enable global interrupts. */

  USB_serial_Start();
  proto_timer_int_StartEx(proto_timer_interrupt_service);
  proto_timer_Start();
  UART_in_Start();


 
  for(;;) {

    /* Get received character or zero if nothing has been received yet */
    ch = USB_serial_UartGetChar(); 
    if (ch != 0u) {
      USB_serial_UartPutChar(ch);
      USB_serial_UartPutString("\r\n");
    }
  
    switch(ch) {
    case '1':
      USB_serial_UartPutString("PPM monitor - pulse width in microseconds\r\n");
      ppm_timer_Stop();
      ppm_timer_SetCaptureMode(ppm_timer_TRIG_RISING);
      ppm_timer_int_StartEx(ppm_timer_interrupt_service);
      number_of_channels = 0;
      ppm_timer_Start();
      proto_run(NULL, ppm_monitor);
      break;    
    case '2':
      USB_serial_UartPutString("PWM monitor - pulse width in microseconds\r\n");
      ppm_timer_Stop();
      ppm_timer_SetCaptureMode(ppm_timer_TRIG_RISING);
      ppm_timer_int_StartEx(pwm_timer_interrupt_service);
      number_of_channels = 0;
      ppm_timer_Start();
      proto_run(NULL, ppm_monitor);
      break;
    case '3':
      USB_serial_UartPutString("SBUS monitor\r\n");
      sbus_monitor();
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
      USB_serial_UartPutString("1 - PPM monitor\r\n"); 
      USB_serial_UartPutString("2 - PWM monitor\r\n");      
      USB_serial_UartPutString("3 - SBUS monitor\r\n");      
      USB_serial_UartPutString("r - reset\r\n");
      USB_serial_UartPutString("l - led\r\n");
      USB_serial_UartPutString("b - enter bootloader\r\n");
      break;
    }
  }
}
