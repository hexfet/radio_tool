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
    Control1_Write(1);
  else
    Control1_Write(0);
  
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
//USB_serial_UartPutChar(c);
//USB_serial_UartPutChar(inbuf_idx);
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


// crc implementation from CRSF protocol document rev7
static uint8 crc8tab[256] = {
    0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54, 0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D,
    0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06, 0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F,
    0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0, 0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
    0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2, 0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B,
    0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9, 0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0,
    0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B, 0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
    0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D, 0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44,
    0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F, 0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16,
    0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E, 0xBB, 0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
    0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9, 0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0,
    0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F, 0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36,
    0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D, 0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
    0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26, 0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F,
    0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74, 0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D,
    0xD6, 0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 0x82, 0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
    0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0, 0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9};

static uint8 crc8(const uint8 *ptr, uint8 len)
{
    uint8 crc = 0;
    for (uint8 i=0; i < len; i++) {
        crc = crc8tab[crc ^ *ptr++];
    }
    return crc;
}


#define CRSF_PACKET_SIZE          26
#define CRSF_SYNC_BYTE            0xc8
#define CRSF_TYPE_RCDATA          0x16
static uint8 testrxframe[] = { 0x00, 0x0C, 0x14, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x01, 0x03, 0x00, 0x00, 0x00, 0xF4 };
uint16 crsf_monitor(uint8 polarity) {
  static char outbuf[256];
  char *pc = outbuf;
  int32 chars_out;
  int size = sizeof outbuf;

  if (polarity)
    Control1_Write(1);
  else
    Control1_Write(0);
  
  static uint8 inbuf[26];
  static uint8 inbuf_idx = 0;
  uint32 c;
  uint8 loop=1;
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
//USB_serial_UartPutChar(c);
//USB_serial_UartPutChar(inbuf_idx);
      switch (inbuf_idx) {
      case 26:
    		if (c == crc8(&inbuf[2], inbuf[1]-1)) {
    			inbuf[inbuf_idx] = c;

          uint8 byte_in_crsf = 1;
    			uint8 bit_in_crsf = 0;
    			uint8 ch = 0;
    			uint8 bit_in_channel = 0;
          memset((void *)Channels, 0, sizeof Channels);
          
    			// channel data is interleaved and bit-reversed
          for (int i=0; i < 176; i++) {
    				if (inbuf[byte_in_crsf] & (1<<bit_in_crsf)) {
    						Channels[ch] |= (1<<bit_in_channel);
    				}
    				bit_in_crsf = (bit_in_crsf + 1) % 8;
            if (bit_in_crsf == 0) byte_in_crsf++;
    				bit_in_channel = (bit_in_channel + 1) % 11;
            if (bit_in_channel == 0) ch++;            
    			}
          
          for (int i=0; i < MAX_CHANS; i++) {
            chars_out = snprintf(pc, size, "%4ld ", (int32)Channels[i]);
            pc += chars_out;
            size -= chars_out;
          }
          Pin_sigout_Write(1);
          USB_serial_UartPutString(outbuf);
          while (USB_serial_SpiUartGetTxBufferSize()) CyDelayUs(10);
          Pin_sigout_Write(0);
        }
        inbuf_idx = 0;
        pc = outbuf;
        size = sizeof outbuf;
        UART_in_SpiUartClearRxBuffer();
        
        // send test telemetry frame
        UART_in_SpiUartPutArray(testrxframe, sizeof testrxframe);
        break;

      case 2:
        if (c != CRSF_TYPE_RCDATA) inbuf_idx = 0;
        break;
        
      case 1:
        if (c != 24) inbuf_idx = 0;
        break;
              
      case 0:
        if (c != CRSF_SYNC_BYTE) break;
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
    case 0u:
      break;
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
    case '6':
      USB_serial_UartPutString("CRSF monitor & telemetry\r\n");
      crsf_monitor(SBUS_NORMAL);
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
    default:
      USB_serial_UartPutString("1 - PPM monitor\r\n"); 
      USB_serial_UartPutString("2 - PPM jitter monitor\r\n");
      USB_serial_UartPutString("3 - PWM monitor\r\n");      
      USB_serial_UartPutString("4 - SBUS monitor - normal polarity\r\n");      
      USB_serial_UartPutString("5 - SBUS monitor - inverted polarity\r\n");
      USB_serial_UartPutString("6 - CRSF monitor and telemetry\r\n");
      USB_serial_UartPutString("r - reset\r\n");
      USB_serial_UartPutString("l - led\r\n");
      USB_serial_UartPutString("b - enter bootloader\r\n");
      break;
    }
  }
}

