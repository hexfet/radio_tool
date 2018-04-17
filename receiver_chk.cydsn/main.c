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
static volatile struct {
  int32 minimum;    // min pulse width
  int32 maximum;    // max pulse width
} channel_info[MAX_CHANS];

void reset_channel_info() {
  for (int i=0; i < MAX_CHANS; i++) {
    channel_info[i].minimum = 50000;
    channel_info[i].maximum = 0;
  }
}


static uint16 (*proto_callback)(volatile int32[]) = NULL;
CY_ISR(proto_timer_interrupt_service) {
  if (proto_callback) proto_timer_WritePeriod( proto_callback(Channels) );
  proto_timer_ClearInterrupt(proto_timer_INTR_MASK_TC);
}

static uint32 max_width;
static volatile int32 interrupting;
static volatile uint32 ppm_sync;
CY_ISR(ppm_timer_interrupt_service) {
  ppm_timer_ClearInterrupt(ppm_timer_INTR_MASK_CC_MATCH);
  interrupting = 1;
  
  static uint32 prev_capture, prev_num_channels, sync_count;
  static uint32 curr_channel;
  uint32 curr_capture = ppm_timer_ReadCapture();
  uint32 width = ((curr_capture - prev_capture - 12) % 0xffff) / 3;   // microseconds - divisor from C/T clock
  prev_capture = curr_capture;

  if (width > max_width) max_width = width;
  if (width > 4500) {
    prev_num_channels = number_of_channels;
    number_of_channels = curr_channel;
    if (number_of_channels == prev_num_channels) {
      if (sync_count < 50) sync_count++;
    } else {
      sync_count = 0;
    }
    ppm_sync = sync_count >= 3;
    curr_channel = 0;
  } else {
//    Channels[curr_channel++] = ((int)width - 1500) * 20;
    if ((int)width < channel_info[curr_channel].minimum)
      channel_info[curr_channel].minimum = (int)width;
    if ((int)width > channel_info[curr_channel].maximum)
      channel_info[curr_channel].maximum = (int)width;
    Channels[curr_channel++] = (int)width;
    curr_channel %= MAX_CHANS;
  }
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
        break;
      case 'z':
        reset_channel_info();
      }
    }
  }
}


uint16 ppm_monitor(volatile int32 channels[])
{
  static char outbuf[127];
  char *pc = outbuf;
  int32 chars_out;
  int size = sizeof outbuf;

  if (!interrupting) {
    snprintf(pc, size, "Pulses not being received\r");
  } else if (!ppm_sync) {
      snprintf(pc, size, "Synchronizing: nc %ld\r\n", (int32)number_of_channels);
  } else {
    for (int i=0; i < number_of_channels; i++) {
      chars_out = snprintf(pc, size, "%6ld ", (int32)channels[i]);
      pc += chars_out;
      size -= chars_out;
    }
    snprintf(pc, size, "   max_width %ld, channels %d\r\n", (int32)max_width, number_of_channels);
  }
  USB_serial_UartPutString(outbuf);   
  interrupting = 0;
  return 30000;
}


uint16 ppm_jitter_monitor(volatile int32 channels[])
{
  (void)channels;
  static char outbuf[127];
  char *pc = outbuf;
  int32 chars_out;
  int size = sizeof outbuf;

  if (!interrupting) {
    snprintf(pc, size, "Pulses not being received\r");
  } else if (!ppm_sync) {
      snprintf(pc, size, "Synchronizing: nc %ld\r\n", (int32)number_of_channels);
      reset_channel_info();
  } else {
    for (int i=0; i < number_of_channels; i++) {
      chars_out = snprintf(pc, size, "%ld/%ld/%ld ", channel_info[i].minimum,
                  channel_info[i].maximum,channel_info[i].maximum-channel_info[i].minimum);
      pc += chars_out;
      size -= chars_out;
    }
    snprintf(pc, size, "\r\n");
  }
  USB_serial_UartPutString(outbuf);   
  interrupting = 0;
  return 30000;
}

#define SBUS_NORMAL 0
#define SBUS_INVERT 1
uint16 sbus_monitor(uint8 polarity) {
  static char outbuf[256];
  char *pc = outbuf;
  int32 chars_out;
  int size = sizeof outbuf;

  if (polarity)
    Invert_input_Write(1);
  else
    Invert_input_Write(0);
    
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
      USB_serial_UartPutString("PPM jitter monitor - min/max/diff in microseconds\r\n");
      ppm_timer_Stop();
      ppm_timer_SetCaptureMode(ppm_timer_TRIG_RISING);
      ppm_timer_int_StartEx(ppm_timer_interrupt_service);
      number_of_channels = 0;
      ppm_timer_Start();
      proto_run(NULL, ppm_jitter_monitor);
      break;       
    case '3':
      USB_serial_UartPutString("PWM monitor - pulse width in microseconds\r\n");
      ppm_timer_Stop();
      ppm_timer_SetCaptureMode(ppm_timer_TRIG_RISING);
      ppm_timer_int_StartEx(pwm_timer_interrupt_service);
      for (int i=0; i < MAX_CHANS; i++) {
        channel_info[i].minimum = INT_MAX;
        channel_info[i].maximum = 0;
      }
      number_of_channels = 0;
      ppm_timer_Start();
      proto_run(NULL, ppm_monitor);
      break;
    case '4':
      USB_serial_UartPutString("SBUS monitor - normal\r\n");
      sbus_monitor(SBUS_NORMAL);
      break;    
    case '5':
      USB_serial_UartPutString("SBUS monitor - inverted\r\n");
      sbus_monitor(SBUS_INVERT);
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
      USB_serial_UartPutString("2 - PPM jitter monitor\r\n");
      USB_serial_UartPutString("3 - PWM monitor\r\n");      
      USB_serial_UartPutString("4 - SBUS monitor - normal polarity\r\n");      
      USB_serial_UartPutString("5 - SBUS monitor - inverted polarity\r\n");
      USB_serial_UartPutString("r - reset\r\n");
      USB_serial_UartPutString("l - led\r\n");
      USB_serial_UartPutString("b - enter bootloader\r\n");
      break;
    }
  }
}

