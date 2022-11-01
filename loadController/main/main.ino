#include <SPI.h>
#include "RTClib.h"
#include "SerialTransfer.h"
RTC_PCF8523 rtc;


SerialTransfer intf_stf;
SerialTransfer data_stf;


// Program states:
int ref_time  = 0;
int data_time = 0;
int data_interval = 10;

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
int  short_period   = 0;
int  time_stamp     = 0;
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




// interface packet
struct PKT {
  char cmd;
  bool flag;
  int  value;
} pkt;




// Data packet
struct DATA {
  int I_ref;
  int V_ref;
  int P_ref;
  int V_fc;
  int V_fc_spike;
  int I_monitor;
  int I_clamp;
  int op_mode;
  int time_stamp;
  bool alarm_1;
  bool alarm_2;
  bool alarm_ext;
  bool load_status;
} data;





// Pin specificaton
#define SDA_0_pin         0
#define SCL_0_pin         1
#define CLK_pin           2
#define MOSI_pin          3
#define MISO_pin          4
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
  while (!Serial) {
    ;  // wait for serial port to connect. Needed for native USB
  }
  USB_flag = true;
  Serial.println("Load controller connected");

  // Init communication to interface
  Serial1.setRX(RX_0_pin);
  Serial1.setTX(TX_0_pin);
  Serial1.setPollingMode(true);
  Serial1.begin(115200);
  intf_stf.begin(Serial1);

  //Set pins for I2C
  Wire.setSDA(SDA_0_pin);
  Wire.setSCL(SCL_0_pin);

  // PWM frequency
  analogWriteFreq(PWM_freq);
  analogWriteRange(PWM_range);

  // Pinmodes
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

  // initial outputs
  digitalWrite(set_alarm_pin, HIGH); // Alarm is activated by pulling low
  digitalWrite(clear_pin, HIGH);
  digitalWrite(CS_pin, HIGH);        // Set CS pins high while ADC's not being read

  // Init communication to data logger
  Serial2.setRX(RX_1_pin);
  Serial2.setTX(TX_1_pin);
  Serial2.setPollingMode(true);
  Serial2.begin(115200);
  data_stf.begin(Serial2);
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
  
  // read alarms
  statusAlarm1();
  statusAlarm2();

  // write to interface if alarms are triggered
  if (alarm_1) {
    msgWrite(intf_stf, 'A', true, 1);
  }
  if (alarm_2) {
    msgWrite(intf_stf, 'A', true, 2);
  }

  // read msg's from serial
  msgRead();

  // Collect data
  aggData();

  // send data every 10 ms
  if (millis() - data_time > data_interval) {
    dataWrite();
    data_time = millis();
  }
}


void loop1() {
  // check every iteration if a short-circuit has been requested
  if (short_flag) {
    executeShort(short_period);
  }
}
