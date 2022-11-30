#include <SPI.h>
#include "RTClib.h"
#include "SerialTransfer.h"
RTC_PCF8523 rtc;


SerialTransfer DL;
SerialTransfer CVM;


// Program states:
int ref_time_ms   = 0;
int ref_time_us   = 0;
int data_time     = 0;
int data_interval = 500;


// Set maximum values
// Units: mA, mV, W, uS
const int max_current    = 240000;
const int max_voltage    = 150000;
const int max_power      = 3000;
const int max_short_time = 100000;


// Reference variables:
// refs: Current, voltage, power
// units: mA, mV,  W
int V_ref = 24;
int I_ref = 0;
int P_ref = 0;


// split uf reference tracking to cc, cv cp


// Operating mode
// Available:
// 1 Reference tracking           (REF)
// 2 Maximum power point tracking (MPPT)
// 3 Maximum efficiency tracking  (MET)
int op_mode = 1;


// Short circuit parameters
int  SCD = 0; // short_circuit_duration
int  SCT = 0; // short_circuit_timer
int  igbt_selected  = 1;


// PWM Settings
int PWM_range = 24000;
int PWM_freq  = 10000;


// flags
bool alarm_1     = false;
bool alarm_2     = false;
bool alarm_ext   = false;
bool load_flag   = false;
bool short_flag  = false;
bool USB_flag    = false;


// ADC channels
uint16_t voltage_spike = 0;
uint16_t voltage       = 0;
uint16_t bank_current  = 0;



// CVM packet
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
#define SDA_0_pin         0
#define SCL_0_pin         1
#define CLK_pin           2
#define MOSI_pin          3 // (TX)
#define MISO_pin          4 // (RX)
#define in_flag_pin       5
#define out_flag_pin      6
#define igbt_1_pin        7
#define TX_1_pin          8
#define RX_1_pin          9
#define igbt_2_pin       10
#define COUT_1_pin       11
#define TX_0_pin         12
#define RX_0_pin         13
#define CS_pin           14
#define COUT_2_pin       15
#define CV_pin           16
#define CC_pin           17
#define set_alarm_pin    18
#define load_pin         19
#define clear_pin        20
#define alarm_2_pin      21
#define alarm_1_pin      22
#define SDA_1_pin        26
#define SCL_1_pin        27



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
  CVM.begin(Serial1);

  //Set pins for I2C
  //  Wire.setSDA(SDA_0_pin);
  //  Wire.setSCL(SCL_0_pin);



  // Pinmodes
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(load_pin,          OUTPUT);
  pinMode(CC_pin,            OUTPUT);
  pinMode(CV_pin,            OUTPUT);
  pinMode(clear_pin,         OUTPUT);
  pinMode(alarm_1_pin,        INPUT);
  pinMode(alarm_2_pin,        INPUT);
  pinMode(load_pin,          OUTPUT);
  pinMode(CS_pin,       OUTPUT_12MA);
  pinMode(in_flag_pin,        INPUT);
  pinMode(out_flag_pin, OUTPUT_12MA);
  pinMode(set_alarm_pin,     OUTPUT);
  pinMode(igbt_1_pin,        OUTPUT);
  pinMode(igbt_2_pin,        OUTPUT);


  // PWM frequency
  analogWriteFreq(PWM_freq);
  analogWriteRange(PWM_range);

  // SPI setup
  //Set the correct SPI pins for the SPI
  SPI.setSCK(CLK_pin);
  SPI.setTX(MOSI_pin);
  SPI.setRX(MISO_pin);
  SPI.setCS(1);
  digitalWrite(CS_pin, HIGH);

  SPI.begin(0);
  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
  SPI.endTransaction();


  // initial outputs
  digitalWrite(set_alarm_pin, HIGH); // Alarm is activated by pulling low
  digitalWrite(clear_pin, HIGH);
  digitalWrite(CS_pin, HIGH);        // Set CS pins high while ADC's not being read

  // Init communication to data logger
  Serial2.setRX(RX_1_pin);
  Serial2.setTX(TX_1_pin);
  Serial2.setPollingMode(true);
  Serial2.begin(115200);
  DL.begin(Serial2);

  // running
  digitalWrite(LED_BUILTIN, HIGH);
}



// boot core 1
void setup1() {
  pinMode(igbt_1_pin, OUTPUT_12MA);
  pinMode(igbt_2_pin, OUTPUT_12MA);
  pinMode(in_flag_pin, INPUT);
  digitalWrite(igbt_1_pin, LOW);     // Close igbt 1
  digitalWrite(igbt_2_pin, LOW);     // Close igbt 2
}



void loop() {
  // check alarms: sets alarm_1 and alarm_2
  // flags to true if alarms on the kikusui have been triggered
  statusAlarm1();
  statusAlarm2();

  // read & execute instructions from CVM and DL
  msgRead(CVM);
  msgRead(DL);

  // read & send data with 10 ms sampling
  if ( (millis() - data_time) > data_interval) {
    
    // reset timer
    data_time = millis();

    // collect data
    aggData();

    // send data
    dataWrite();
  }
}


void loop1() {

  

  
}
