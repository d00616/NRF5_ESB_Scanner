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

#include <nrf.h>
#include "radio.h"

// Datarate: RADIO_MODE_MODE_Ble_1Mbit, RADIO_MODE_MODE_Nrf_1Mbit, RADIO_MODE_MODE_Nrf_250Kbit, RADIO_MODE_MODE_Nrf_2Mbit
#define DATARATE (RADIO_MODE_MODE_Nrf_1Mbit)
// Radio channel
#define CHANNEL (76)
// Length of full address (max 5)
#define ADDRLEN (5)
// Base address (first byte is the variable address part)
#define ADDR 0x00, 0xFC, 0xE1, 0xA8, 0xA8

void print_header() {
  Serial.println("\n");
  Serial.println("TIMESTAMP,ADDR,PID,NOACK,CRC,CRCSTATUS,RSSI,PKG_US,LEN,DATA");  
}

void print_hex(uint8_t h) {
  if (h<=0x0f) {
    Serial.print("0");
  }
  Serial.print(h, HEX);
}

void setup() {
  Serial.begin(115200);
  uint8_t addr[ADDRLEN] = {ADDR};
  radio_init(DATARATE, CHANNEL, ADDRLEN, addr);
  delay(1500); // Allow reconnecting serial port
  print_header();
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  if (Serial.available()) {
    print_header();
    while (Serial.available()) {
      Serial.read();
    }
  }
  if (radio_available()) {
    NRF5_ESB_Packet pkg = radio_getpkg();

    char buffer[1024];
    sprintf(buffer,"%lu,%d,%d,%d,%d,%d,%d,%d,%d,", (long unsigned int)pkg.timestamp,pkg.address,pkg.pid,pkg.noack,pkg.crc, pkg.crcstatus, pkg.rssi, pkg.pkglen_us,pkg.len);
    Serial.print(buffer);
    for (int i=0;i<pkg.len;i++) {
      print_hex(pkg.data[i]);
    }
    Serial.println();
  }
}
