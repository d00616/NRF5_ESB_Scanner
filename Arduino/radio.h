/*
  MIT License

  Copyright (c) 2017 Frank Holtz

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
*/

#ifndef radio_h
#define radio_h
#include <Arduino.h>
#include <CircularBuffer.h>

// Parameters
#define RX_BUFFER_SIZE (128)
#define MAX_MESSAGE_LENGTH (32)
#define MAX_PACKET_TIME (2000)
#define TIMER_RESET_BITS (31)

// Define Timer. Because of predefined PPI, Timer0 is required.
#define NRF5_RADIO_TIMER NRF_TIMER0
#define NRF5_RADIO_TIMER_IRQ_HANDLER TIMER0_IRQHandler
#define NRF5_RADIO_TIMER_IRQN TIMER0_IRQn

/** Structure of radio rackets
 */
typedef struct nrf5_radio_packet_s {
  // structure written by radio unit
  struct {
    uint8_t address;
    uint8_t len;
    union {
      uint8_t s1;
      struct {
        uint8_t noack : 1;
        uint8_t pid : 2;
      };
    };
    uint8_t data[MAX_MESSAGE_LENGTH];
    uint64_t timestamp;
    uint32_t pkglen_us;
    uint16_t crc;
    uint8_t crcstatus;
    int8_t rssi;
  }
#ifndef DOXYGEN
  __attribute__((packed));
#endif
} NRF5_ESB_Packet;

// Functions
void radio_init(uint8_t datarate, uint8_t channel, uint8_t addrlen, uint8_t *address);
bool radio_available();
NRF5_ESB_Packet radio_getpkg();

/**
 * Reset events and read back on nRF52
 * http://infocenter.nordicsemi.com/pdf/nRF52_Series_Migration_v1.0.pdf
 */
#if __CORTEX_M == 0x04
#define NRF_RESET_EVENT(event)                                                 \
  event = 0;                                                                   \
  (void)event
#else
#define NRF_RESET_EVENT(event) event = 0
#endif

#endif
