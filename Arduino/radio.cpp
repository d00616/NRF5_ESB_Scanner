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

#include "radio.h"

// Circular buffer for RX_BUFFER_SIZE packages
static CircularBuffer<NRF5_ESB_Packet,RX_BUFFER_SIZE> rx_circular_buffer;

/* Define an RX buffer */

// RX Buffer
static NRF5_ESB_Packet rx_buffer;
// Pcap base seconds
uint32_t ts_overflow=0;

// Private
static uint8_t reverse_byte(uint8_t address);

// Return buffer status
bool radio_available() {
  return !rx_circular_buffer.isEmpty();
}

// Return package
NRF5_ESB_Packet radio_getpkg() {
  return rx_circular_buffer.shift();
}


// Initialize radio unit
void radio_init(uint8_t datarate, uint8_t channel, uint8_t addrlen, uint8_t *address)
{
  // Check parameters
  if (addrlen>5) return;

  // Clock is manged by sleep modes. Radio depends on HFCLK.
  // Force to start HFCLK
  NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
  NRF_CLOCK->TASKS_HFCLKSTART = 1;
  while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0)
    ;

  // Enable low latency sleep mode
  NRF_POWER->TASKS_CONSTLAT = 1;

  // Enable cache on >= NRF52
#ifndef NRF51
  NRF_NVMC->ICACHECNF = NVMC_ICACHECNF_CACHEEN_Msk;
#endif

  // Power on radio unit
  NRF_RADIO->POWER = 1;

  // Disable shorts
  NRF_RADIO->SHORTS = 0;

  // Disable radio
  NRF_RADIO->TASKS_DISABLE = 1;

  // Enable radio interrupt
  NVIC_SetPriority(RADIO_IRQn, 1);
  NVIC_ClearPendingIRQ(RADIO_IRQn);
  NVIC_EnableIRQ(RADIO_IRQn);

  // Enable timer interrupt
  NVIC_SetPriority(NRF5_RADIO_TIMER_IRQN, 2);
  NVIC_ClearPendingIRQ(NRF5_RADIO_TIMER_IRQN);
  NVIC_EnableIRQ(NRF5_RADIO_TIMER_IRQN);

  // Configure radio parameters: radio channel
  NRF_RADIO->FREQUENCY = channel;

  // Configure radio parameters: data rate
  NRF_RADIO->MODE = datarate;

  // Configure radio parameters: CRC16
  NRF_RADIO->CRCCNF = (RADIO_CRCCNF_LEN_Two << RADIO_CRCCNF_LEN_Pos);
  NRF_RADIO->CRCINIT = 0xFFFFUL;
  NRF_RADIO->CRCPOLY = 0x11021UL;

  // Configure addresses
  NRF_RADIO->PREFIX0 = reverse_byte(address[1]);
  NRF_RADIO->BASE0 = reverse_byte(address[2]) << 24 |
                     reverse_byte(address[3]) << 16 |
                     reverse_byte(address[4]) << 8 | reverse_byte(address[5]);

  // Packet configuration for NRF24 compatibility
  NRF_RADIO->PCNF0 = (6 << RADIO_PCNF0_LFLEN_Pos) | // 6 Bits length field
                     (1 << RADIO_PCNF0_S0LEN_Pos) | // S0 used for Prefix field
#ifdef RADIO_PCNF0_S1INCL_Pos
                     (1 << RADIO_PCNF0_S1INCL_Pos) | // Force include S1 in RAM
#endif
                     (3 << RADIO_PCNF0_S1LEN_Pos); // 3 Bits S1 (NOACK and PID)

  // Packet configuration
  NRF_RADIO->PCNF1 =
      (MAX_MESSAGE_LENGTH << RADIO_PCNF1_MAXLEN_Pos) | // maximum length
      (0 << RADIO_PCNF1_STATLEN_Pos) |                 // minimum message length
      ((addrlen - 2) << RADIO_PCNF1_BALEN_Pos) | // Set base address length
      (RADIO_PCNF1_ENDIAN_Big << RADIO_PCNF1_ENDIAN_Pos) | // Big endian
      (RADIO_PCNF1_WHITEEN_Disabled << RADIO_PCNF1_WHITEEN_Pos); // Disable whitening

  // Enable fast ramp up for controllers supporting this feature >NRF51
#ifdef NRF5_ESB_FAST_RU
  NRF_RADIO->MODECNF0 = (NRF5_ESB_FAST_RU << RADIO_MODECNF0_RU_Pos) |
                        (RADIO_MODECNF0_DTX_B1 << RADIO_MODECNF0_DTX_Pos);
#endif

  // Radio buffer storing packages
  NRF_RADIO->PACKETPTR = (uint32_t)&rx_buffer;

#ifdef NRF51
  // Enable timer
  NRF5_RADIO_TIMER->POWER = 1;
#endif
  // Stop timer, if running
  NRF5_RADIO_TIMER->TASKS_STOP = 1;
  // Prepare timer running at 1 MHz/1us
  NRF5_RADIO_TIMER->PRESCALER = 4;
  // Timer mode
  NRF5_RADIO_TIMER->MODE = TIMER_MODE_MODE_Timer;
  // in 16 Bit mode
  NRF5_RADIO_TIMER->BITMODE = TIMER_BITMODE_BITMODE_32Bit << TIMER_BITMODE_BITMODE_Pos;
  // Disable shorts
  NRF5_RADIO_TIMER->SHORTS = (TIMER_SHORTS_COMPARE3_CLEAR_Enabled << TIMER_SHORTS_COMPARE3_CLEAR_Pos);

  // Disable timer interrupts
  NRF5_RADIO_TIMER->INTENCLR = (uint32_t)~0;

  // Configure CC[3]
  NRF5_RADIO_TIMER->CC[3] = 1<<TIMER_RESET_BITS;

  // Configure timer interrupts
  NRF5_RADIO_TIMER->INTENSET = (TIMER_INTENSET_COMPARE3_Msk);

  // Configure radio interrupts
  NRF_RADIO->INTENCLR = (uint32_t)~0;
  NRF_RADIO->INTENSET = (RADIO_INTENSET_ADDRESS_Msk | RADIO_INTENSET_END_Msk);

  // Configure Shorts
  NRF_RADIO->SHORTS = (RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_START_Msk | RADIO_SHORTS_ADDRESS_RSSISTART_Msk | RADIO_SHORTS_DISABLED_RXEN_Msk);

  // Configure listen address(es)
  NRF_RADIO->RXADDRESSES = 0x01;

  // Clear all radio events
  NRF_RADIO->EVENTS_ADDRESS = 0;
  NRF_RADIO->EVENTS_DEVMATCH = 0;
  NRF_RADIO->EVENTS_DEVMISS = 0;
  NRF_RADIO->EVENTS_DISABLED = 0;
  NRF_RADIO->EVENTS_END = 0;
  NRF_RADIO->EVENTS_PAYLOAD = 0;
  NRF_RADIO->EVENTS_READY = 0;
  NRF_RADIO->EVENTS_RSSIEND = 0;

  // Configure Shorts
  NRF5_RADIO_TIMER->SHORTS = 0;

  // Reset compare events and registers
#ifdef NRF51
  for (uint8_t i=0; i<4; i++) {
#else
  for (uint8_t i=0; i<6; i++) {
#endif
    NRF5_RADIO_TIMER->EVENTS_COMPARE[i] = 0;
    NRF5_RADIO_TIMER->CC[i] = 0;
  }

  // Enable PPI

  // Predefined PPI 22: TIMER0->EVENTS_COMPARE[1] -> RADIO->TASKS_DISABLE (PAN#102)
  NRF_PPI->CHENSET = (PPI_CHENSET_CH22_Enabled << PPI_CHENSET_CH22_Pos);

  // Predefined PPI 26: RADIO->EVENTS_ADDRESS  TIMER0->TASKS_CAPTURE[1]
  NRF_PPI->CHENSET = (PPI_CHENSET_CH26_Enabled << PPI_CHENSET_CH26_Pos);

  // Predefined PPI 27: RADIO->EVENTS_END ->  TIMER0->TASKS_CAPTURE[2]
  NRF_PPI->CHENSET = (PPI_CHENSET_CH27_Enabled << PPI_CHENSET_CH27_Pos);

  // Enable timer
  NRF5_RADIO_TIMER->TASKS_START = 1;
  
  // Enable RX
  NRF_RADIO->TASKS_RXEN = 1;
}



// Reverse a byte for address
static uint8_t reverse_byte(uint8_t address)
{
#if __CORTEX_M >= (0x01U)
  return __REV(__RBIT(address));
#else
  address = ((address * 0x0802LU & 0x22110LU) | (address * 0x8020LU & 0x88440LU)) * 0x10101LU >> 16;
#endif
  return address;
}

#ifdef __cplusplus
extern "C" {
#endif  /** Radio Interrupt handler */
  void RADIO_IRQHandler()
  {
    digitalWrite(LED_BUILTIN, HIGH);
    if (NRF_RADIO->EVENTS_ADDRESS == 1) {
      NRF_RESET_EVENT(NRF_RADIO->EVENTS_ADDRESS);
      NRF5_RADIO_TIMER->TASKS_CAPTURE[0] = 1;
      // Disable radio by PPI22 after timeout (PAN#102)
      NRF5_RADIO_TIMER->CC[1]= NRF5_RADIO_TIMER->CC[0] + MAX_PACKET_TIME;
    }
    if (NRF_RADIO->EVENTS_END == 1) {
      NRF_RESET_EVENT(NRF_RADIO->EVENTS_END);
      // STOP RSSI
      NRF_RADIO->TASKS_RSSISTOP = 1;
      // Deactiveate timer
      NRF5_RADIO_TIMER->CC[1] = 0;
      // Store RSSI
      rx_buffer.rssi = 0-NRF_RADIO->RSSISAMPLE;
      // Store CRC
      rx_buffer.crc = NRF_RADIO->RXCRC;
      rx_buffer.crcstatus = NRF_RADIO->CRCSTATUS;
      // Store package TX time in µS
      rx_buffer.pkglen_us = NRF5_RADIO_TIMER->CC[2]-NRF5_RADIO_TIMER->CC[0];
      // Calculate timestamp
      rx_buffer.timestamp = (ts_overflow<<TIMER_RESET_BITS)+NRF5_RADIO_TIMER->CC[2];
      // Store package
      rx_circular_buffer.push(rx_buffer);
    }
    digitalWrite(LED_BUILTIN, LOW);
  }

  void NRF5_RADIO_TIMER_IRQ_HANDLER() {
    if (NRF5_RADIO_TIMER->EVENTS_COMPARE[3]) {
      ts_overflow++;
      NRF_RESET_EVENT(NRF5_RADIO_TIMER->EVENTS_COMPARE[3]);
    }
  }
#ifdef __cplusplus
}
#endif
