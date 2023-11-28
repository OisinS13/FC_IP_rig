#include <SPI.h>
#include "RTClib.h"
#include "SerialTransfer.h"
RTC_PCF8523 rtc;

SerialTransfer DataLogger;
SerialTransfer CellVoltageMonitor;


// Program states:
uint32_t ref_time = 0;
uint32_t data_time = 0;
uint32_t data_interval = 10;

// Set maximum values
// Units: mA, mV, W, uS
uint32_t max_current = 200000;
uint32_t max_voltage = 24000;
uint32_t max_power = 3000;
uint32_t max_short_time = 200000;

// Reference variables:
// refs: Current, voltage, power
// units: mA, mV,  W
uint32_t V_ref = 24;
uint32_t I_ref = 0;
uint32_t P_ref = 0;

// split uf reference tracking to cc, cv cp

// Operating mode
// Available:
// 1 Reference tracking           (REF)
// 2 Maximum power point tracking (MPPT)
// 3 Maximum efficiency tracking  (MET)
uint8_t op_mode = 1;

// Short circuit parameters
uint32_t short_period = 0;
uint32_t time_stamp = 0;
uint8_t igbt_selected = 1;

// PWM Settings
uint32_t PWM_range = 24000;
uint32_t PWM_freq = 10000;

// flags
bool alarm_1 = false;
bool alarm_2 = false;
bool alarm_ext = false;
bool load_flag = false;
bool short_flag = false;
bool USB_flag = false;

// ADC channels
uint16_t voltage_spike = 0;
uint16_t voltage = 0;
uint16_t bank_current = 0;



// CVM packet
struct PKT {
  char cmd;
  bool flag;
  uint16_t value;
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

  uint8_t op_mode;

  bool alarm_1;
  bool alarm_2;
  bool alarm_ext;
  bool load_status;
} data;

#define Cycles_per_step 5
struct Optimisation_parameters {
  uint32_t Parameters[2] = { 50000, 30000000 };  //0=duration, 1=period, in uS
  bool Parameter_flag = 0;                       //if 0, optimising duration, if 1, optimising period
  double Mean_V_per_cycle[Cycles_per_step];
  double Mean_V_per_step[2] = { 0, 0 };
  uint8_t Cycle_counter = 0;
  // uint8_t Cycles_per_step=5;
  int32_t Step_size[2] = { 10000, 10000000 };       //(signed)
  uint32_t Minimum_step_size[2] = { 500, 500000 };  //0 element is for duration, 1 element is for period
  uint8_t Step_size_multiplier = 5;                 //After switching parameters, increase step size to start re-optimising parameter
} OptimisationParameters;



// Pin specificaton
#define SDA_0_pin 0
#define SCL_0_pin 1
#define ISO_ADC_CLK_pin 2
#define ISO_ADC_MOSI_pin 3  // (TX)
#define ISO_ADC_MISO_pin 4  // (RX)
#define in_flag_pin 5
#define out_flag_pin 6
#define igbt_1_pin 7
#define TX_1_pin 8
#define RX_1_pin 9
#define igbt_2_pin 10
#define COUT_1_pin 11
#define TX_0_pin 12
#define RX_0_pin 13
#define ISO_ADC_CS_pin 14
#define COUT_2_pin 15
#define CV_pin 16
#define CC_pin 17
#define set_alarm_pin 18
#define load_pin 19
#define clear_pin 20
#define alarm_2_pin 21
#define alarm_1_pin 22
#define SDA_1_pin 26
#define SCL_1_pin 27



// boot core 0
void setup() {
  delay(1000);
  USB_flag = USB_setup(115200, 1000);  //Iniitalise the USB, and set flag if present

  // // Init communication to interface
  // Serial1.setRX(RX_0_pin);
  // Serial1.setTX(TX_0_pin);
  // Serial1.setPollingMode(true);
  // Serial1.begin(115200);
  // CellVoltageMonitor.begin(Serial1);

  //Set pins for I2C
  //  Wire.setSDA(SDA_0_pin);
  //  Wire.setSCL(SCL_0_pin);



  // Pinmodes
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(load_pin, OUTPUT);
  pinMode(CC_pin, OUTPUT);
  pinMode(CV_pin, OUTPUT);
  pinMode(clear_pin, OUTPUT);
  pinMode(alarm_1_pin, INPUT);
  pinMode(alarm_2_pin, INPUT);
  pinMode(load_pin, OUTPUT);
  pinMode(ISO_ADC_CS_pin, OUTPUT_12MA);
  pinMode(in_flag_pin, INPUT);
  pinMode(out_flag_pin, OUTPUT_12MA);
  pinMode(set_alarm_pin, OUTPUT);
  pinMode(igbt_1_pin, OUTPUT_12MA);
  pinMode(igbt_2_pin, OUTPUT_12MA);

  digitalWrite(igbt_1_pin, LOW);  // Close igbt 1
  digitalWrite(igbt_2_pin, LOW);  // Close igbt 2

  // PWM frequency
  analogWriteFreq(PWM_freq);
  analogWriteRange(PWM_range);

  // SPI setup
  //Set the correct SPI pins for the SPI
  ISO_ADC_SPI.setSCK(ISO_ADC_CLK_pin);
  ISO_ADC_SPI.setTX(ISO_ADC_MOSI_pin);
  ISO_ADC_SPI.setRX(ISO_ADC_MISO_pin);
  ISO_ADC_SPI.setCS(LED_BUILTIN);
  digitalWrite(ISO_ADC_CS_pin, HIGH);

  ISO_ADC_SPI.begin(0);
  ISO_ADC_SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
  ISO_ADC_SPI.endTransaction();


  // initial outputs
  digitalWrite(set_alarm_pin, HIGH);  // Alarm is activated by pulling low
  digitalWrite(clear_pin, HIGH);

  // Init communication to data logger
  Serial2.setRX(RX_1_pin);
  Serial2.setTX(TX_1_pin);
  Serial2.setPollingMode(true);
  Serial2.begin(115200);
  DataLogger.begin(Serial2);

  // running
  digitalWrite(LED_BUILTIN, HIGH);
}



// boot core 1
void setup1() {

}



void loop() {

    if (!USB_flag) {
    USB_flag = USB_setup(115200, 1);  //Iniitalise the USB, and set flag if present
  }
}


void loop1() {
  // check every iteration if a short-circuit has been requested
  if (short_flag) {
    executeShort(short_period);
  }
}
