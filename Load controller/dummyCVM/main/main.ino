#include <SPI.h>
#include "RTClib.h"
#include "SerialTransfer.h"
RTC_PCF8523 rtc;


SerialTransfer intf_stf;


// flags
bool USB_flag    = false;




// interface packet
struct PKT {
  char cmd;
  bool flag;
  int  value;
} pkt;




// Pin specificaton
#define TX_1_pin          8
#define RX_1_pin          9
#define TX_0_pin         12
#define RX_0_pin         13


// boot core 0
void setup() {
  Serial.begin(115200);

  USB_flag = true;
  Serial.println("Load controller connected");

  // Init communication to interface
  Serial1.setRX(RX_0_pin);
  Serial1.setTX(TX_0_pin);
  Serial1.setPollingMode(true);
  Serial1.begin(115200);
  intf_stf.begin(Serial1);


  // Pinmodes
  pinMode(LED_BUILTIN, OUTPUT);

  // running
  digitalWrite(LED_BUILTIN, HIGH);
}



void loop() {
  msgRead();
  msgWrite(intf_stf, 'K', true, 0);
  delay(1000);
}
