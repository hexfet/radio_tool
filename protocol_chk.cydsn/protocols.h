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
#ifndef _PROTOCOLS_H_
#define _PROTOCOLS_H_
  
#define CHAN_MIN_VALUE -10000
#define CHAN_MAX_VALUE  10000
  
extern uint8 symax_phase;
void symax_init(uint8 tx_addr[]);
uint16 symax_callback(volatile int32 channels[]);
void symax_send_packet(uint8 bind);
void symax_set_channels(uint8);

void yd717_init(uint8 tx_addr[]);
uint16 yd717_callback(volatile int32 channels[]);

void cx10_init(uint8 tx_addr[]);
uint16 cx10_callback(volatile int32 channels[]);

#endif
