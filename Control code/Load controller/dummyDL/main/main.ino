#include <SPI.h>
#include "RTClib.h"
#include "SerialTransfer.h"
RTC_PCF8523 rtc;

SerialTransfer LC;

// flags
bool USB_flag    = false;

int ping_timer = 0;

// LC msg packet
struct PKT {
  char cmd;
  bool flag;
  uint32_t value;
} rx_msg, tx_msg;


// Data packet
struct DATA {
  char cmd;
  uint32_t I_ref;
  uint32_t V_ref;
  uint32_t P_ref;
  uint32_t V_fc;
  uint32_t V_fc_spike;
  uint32_t I_monitor;
  uint32_t I_clamp;
  uint32_t time_stamp;
  uint8_t  op_mode;
  bool alarm_1;
  bool alarm_2;
  bool alarm_ext;
  bool load_status;
} data;


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
  LC.begin(Serial1);


  // Pinmodes
  pinMode(LED_BUILTIN, OUTPUT);

  // running
  digitalWrite(LED_BUILTIN, HIGH);
}


void loop() {
  // continuously listen for data & messages
  msgRead(LC);  

  // Send ping every 2s
  if ( (millis() - ping_timer) > 2000) {
    
    // reset timer
    ping_timer = millis();

    // ping
    msgWrite(LC, 0x67, 'P', true, 0);  
  }
}
