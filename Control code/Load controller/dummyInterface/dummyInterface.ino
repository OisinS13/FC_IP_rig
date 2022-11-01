//#include <SPI.h>
#include "RTClib.h"
#include "SerialTransfer.h"

SerialTransfer stf;

struct STRUCT {
  char cmd;
  bool flag;
  uint32_t value;
} pkt;


#define TX_0_pin 12
#define RX_0_pin 13


void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);

  // Init communication to interface
  Serial1.setRX(RX_0_pin);
  Serial1.setTX(TX_0_pin);
  Serial1.setPollingMode(true);
  Serial1.begin(115200);
  stf.begin(Serial1);
}

void loop() {

  // Send message for load on
  msgWrite('L', true, 0);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);

  digitalWrite(LED_BUILTIN, LOW);
  delay(500);

  // Send message for current set to 100 amp
  msgWrite('I', true, 100000);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);

  digitalWrite(LED_BUILTIN, LOW);
  delay(500);

  // Send message for current set to 200 amp
  msgWrite('I', true, 200000);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);

  
  digitalWrite(LED_BUILTIN, LOW);
  delay(500);

  // Send message to trigger a short-circuit for 1000 microseconds
  msgWrite('S', true, 1000);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);

  digitalWrite(LED_BUILTIN, LOW);
  delay(500);


  // Send message to set current reference to zero
  msgWrite('I', true, 0);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);

  digitalWrite(LED_BUILTIN, LOW);
  delay(500);

  // Send message to turn load off
  msgWrite('L', false, 0);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);

  digitalWrite(LED_BUILTIN, LOW);
  delay(500);

}


void msgWrite(char cmd, bool flag, uint32_t value) {
  // bytes we're stuffing in the transmit buffer
  uint16_t n_byt = 0;

  // Set struct values
  pkt.cmd   = cmd;
  pkt.flag  = flag;
  pkt.value = value;

  // Stuff buffer with struct
  n_byt = stf.txObj(pkt, n_byt);

  // Send buffer
  stf.sendData(n_byt);
}
