#include <SPI.h>
#include <Adafruit_MAX31856.h>
#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// you can also call it with a different address you want
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);
// you can also call it with a different address and I2C interface
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire);

//Pin definitions
#define CTHD_FLW_SIG_PIN A0
#define P_1_SIG_PIN A1
#define P_2_SIG_PIN A2
#define P_3_SIG_PIN A3
#define P_4_SIG_PIN A4
#define P_5_SIG_PIN A5
#define P_6_SIG_PIN A6
#define P_7_SIG_PIN A7
#define P_8_SIG_PIN A8
#define AND_FLW_SIG_PIN A9
#define AND_FLW_SIG2_PIN A10

#define CTHD_HTR_SFTY_TC1_DRDY 22
#define CTHD_HTR_SFTY_TC1_FLT 23
#define CTHD_HTR_SFTY_TC1_CS 24
#define CTHD_HTR_SFTY_TC2_DRDY 25
#define CTHD_HTR_SFTY_TC2_FLT 26
#define CTHD_HTR_SFTY_TC2_CS 27
#define CTHD_HTR_SFTY_TC3_DRDY 28
#define CTHD_HTR_SFTY_TC3_FLT 29
#define CTHD_HTR_SFTY_TC3_CS 30

const uint8_t CTHD_HTR_SFTY_DRDY[3] = { CTHD_HTR_SFTY_TC1_DRDY, CTHD_HTR_SFTY_TC2_DRDY, CTHD_HTR_SFTY_TC3_DRDY };
const uint8_t CTHD_HTR_SFTY_FLT[3] = { CTHD_HTR_SFTY_TC1_FLT, CTHD_HTR_SFTY_TC2_FLT, CTHD_HTR_SFTY_TC3_FLT };
const uint8_t CTHD_HTR_SFTY_CS[3] = { CTHD_HTR_SFTY_TC1_CS, CTHD_HTR_SFTY_TC2_CS, CTHD_HTR_SFTY_TC3_CS };

#define AND_TRC_HTR 31
#define CTHD_TRC_HTR 32
#define CTHD_HTR 34
#define HMD_FEED_PUMP_PIN 35
#define CTHD_HMD_FEED_VLV 36
#define AND_HMD_FEED_VLV 37
#define AND_FLW_PURGE 38
#define AND_FLW_VLV_OFF 39
#define CTHD_SLND 40
#define AND_H2_SLND 41
#define AND_N2_SLND 42
#define AND_PURGE_SLND 43
#define DATA_SPI_CS 47
#define IPU_FLG 3

//PWM driver definitions
#define CLNT_PUMP_PWM 0
#define AND_FLW_CONT_PWM 1
#define AND_RECIRC_PWM 2
#define CTHD_VLV_PWM 3
const uint8_t CLNT_FN_PWMS[6] = { 4, 5, 6, 7, 8, 9 };

const uint8_t P_sig_pins[8] = { P_1_SIG_PIN, P_2_SIG_PIN, P_3_SIG_PIN, P_4_SIG_PIN, P_5_SIG_PIN, P_6_SIG_PIN, P_7_SIG_PIN, P_8_SIG_PIN };  //Array of Pressure signal inputs
uint8_t P_sig_flags = 0;                                                                                                                   //Flags representing if pressure signals are detected. 1 means sensor detected on that pin, 0 means no sensor detected
uint32_t P_readings[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };

uint8_t HIH6120_address[10] = { 0x27 };  //Array of expected adresses of Humidity/Temperature sensors
uint16_t HIH6120_flags = 0;              //Flags to indicate if devices were found at relevant addresses
uint16_t HIH6120_data_flags = 0;         //Flags to indicate of data read is "stale" (1) or "normal" (0)
uint16_t HIH_RH_raw[10] = 0;             //Array to store RH raw data in
uint16_t HIH_T_raw[10] = 0;              //Array to store T raw data in
float HIH_RH[i] = 0;                     //Array to store converted RH data (as %)
float HIH_T[i] = 0;                      //Array to store converted T data (deg C)
uint16_t HIH_interval = 50;              //Time(mS) required for HIH to do data conversion, therefore time between readings
uint32_t HIH_last_read_time = 0;


Adafruit_MAX31856 HTR_SFTY_TC[3] = {
  Adafruit_MAX31856(CTHD_HTR_SFTY_CS[0]),  //Create TC amplifier objects using built in SPI by passing only the CS pins
  Adafruit_MAX31856(CTHD_HTR_SFTY_CS[1]),
  Adafruit_MAX31856(CTHD_HTR_SFTY_CS[2])
};

uint8_t HTR_SFTY_TC_fault[3] = 0;  //Create bytes to store TC amp faults

float TC_low_temp_thresh = 0.0;
float TC_high_temp_thresh = 100.0;
float CTHD_HTR_SFTY_TC_readings = { 0, 0, 0 };

uint16_t CTHD_FLW_reading = 0;
uint16_t AND_FLW_reading[2] = 0;
uint16_t AND_FLW_value[2] = 0;
uint8_t H2N2_flag = 0;  //0 means N2, 1 means H2

uint8_t Mode = 0;  //Mode of operation.
/*
0 = No mode, will only perform safety reactions
1 = Startup procedure
2 = shutdown procedure
3 = Normal power delivery
4 = Steady state low power
5 = Soft emergency stop
6 = Hard emergency stop
*/

uint16_t Safety_mode_flags = 0;  //Flags to indicate FMEA errors that are occuring. Bitwise array of flags
/*
0x0001 = Soft shutdown from external message
0x0002 = Hard shutdown form external message
0x0004 = Anode flooding
0x0008 = Cathode flooding
0x0010 = Anode dry out
0x0020 = Cathode dry out
0x0040 = Anode flow over temperature
0x0080 = Cathode flow/heater over temperature
0x0100 = Coolant over temperature
0x0200 = Anode flow under temperature
0x0400 = Cathode flow under temperature
0x0800 = Coolant under temperature
0x1000 = Coolant under flow
0x2000 = Under voltage of stack (from external message)
0x4000 = Over current of stack (from external message)
0x8000 = IPU stuck on
*/
uint8_t Cathode_flow_sensor_minimum_threshold = 190;  //Minimum expected value if sensor is connected and operating correctly
uint8_t Pressure_sensor_minimum_threshold = 81;       //Minimum expected value if sensor is connected and operating correctly


void setup() {
  Serial.begin(115200);  //Connect to computer over USB, opional
  delay(100);            //Give USB time to connect EDITME to shortest reliable time
  if (Serial) {
    // USB_flag = 1;  //Used so that when connected via USB, verbose status and error messages can be sent, but will still run if not connected
    Serial.println("Cell Voltage monitor connected");
  } else {
    Serial.end();
  }

  Serial1.begin(115200);  //Connect to SFTY UART
  while (!Serial1) {
    ;  // wait for serial port to connect to SFTY
  }
  if (Serial) {
    Serial.println("Safety system connected");
  }

  Serial2.begin(115200);  //Connect to Power system UART
  while (!Serial2) {
    ;  // wait for serial port to connect to Power system
  }
  if (Serial) {
    Serial.println("Load controller connected");
  }

  Serial3.begin(115200);  //Connect to Data UART
  while (!Serial3) {
    ;  // wait for serial port to connect to Data
  }
  if (Serial) {
    Serial.println("Data logger connected");
  }

  for (int i = 0; i < 3; i++) {
    pinMode(CTHD_HTR_SFTY_DRDY[i], INPUT);  //Set the TC amp DRDY pins to inputs
    pinMode(CTHD_HTR_SFTY_FLT[i], INPUT);   //Set the TC amp FLT pins to inputs
    if (!HTR_SFTY_TC[i].begin()) {          //Initialise thermocouple amps over SPI
      //EDITME write error throwing code
      if (Serial) {
        Serial.print("Could not initialize cathode heater safety thermocouple ");
        Serial.println(i);
      }
    } else {
      HTR_SFTY_TC[i].setThermocoupleType(MAX31856_TCTYPE_K);                            //Set the TC type to type k
      HTR_SFTY_TC[i].setConversionMode(MAX31856_CONTINUOUS);                            //Set the amplifiers to be continually reading
      HTR_SFTY_TC[i].setTempFaultThreshholds(TC_low_temp_thresh, TC_high_temp_thresh);  //Set the TC amp internal low and high temperature thresholds to trigger faults automatically
      HTR_SFTY_TC_fault[i] = HTR_SFTY_TC[i].readFault();                                //Read any stored faults on startup
      if (HTR_SFTY_TC_fault[i]) {
        if (HTR_SFTY_TC_fault[i] & MAX31856_FAULT_CJRANGE) {
          Serial.print("Cold Junction Range Fault, TC");
          Serial.println(i);
        }
        if (HTR_SFTY_TC_fault[i] & MAX31856_FAULT_TCRANGE) {
          Serial.print("Thermocouple Range Fault, TC1");
          Serial.println(i);
        }
        if (HTR_SFTY_TC_fault[i] & MAX31856_FAULT_CJHIGH) {
          Serial.print("Cold Junction High Fault, TC1");
          Serial.println(i);
        }
        if (HTR_SFTY_TC_fault[i] & MAX31856_FAULT_CJLOW) {
          Serial.print("Cold Junction Low Fault, TC1");
          Serial.println(i);
        }
        if (HTR_SFTY_TC_fault[i] & MAX31856_FAULT_TCHIGH) {
          Serial.print("Thermocouple High Fault, TC1");
          Serial.println(i);
        }
        if (HTR_SFTY_TC_fault[i] & MAX31856_FAULT_TCLOW) {
          Serial.print("Thermocouple Low Fault, TC1");
          Serial.println(i);
        }
        if (HTR_SFTY_TC_fault[i] & MAX31856_FAULT_OVUV) {
          Serial.print("Over/Under Voltage Fault, TC1");
          Serial.println(i);
        }
        if (HTR_SFTY_TC_fault[i] & MAX31856_FAULT_OPEN) {
          Serial.print("Thermocouple Open Fault, TC1");
          Serial.println(i);
        }
      }
    }
  }

  if (analogRead(CTHD_FLW_SIG_PIN) < Cathode_flow_sensor_minimum_threshold) {  //Flow sensor should output a minimum of 4mA, which should be read as 1V. Threshold of 190=0.928V
    if (Serial) {
      Serial.println("Could not read Cathode flow sensor");
    }
    //EDITME write error throwing code
  }

  Wire.begin();  //Initialise I2C bus as a master
  for (int i = 0; i < 10; i++) {
    Wire.beginTransmission(HIH6120_address[i]);  //Send measurement request to HIH6120 address
    int error = Wire.endTransmission();          //stop transmitting
    if (error == 0) {                            //No error, therefore I2C device successfully acknowledged measurement request
      HIH6120_flags |= 1 << i;                   //Set flag indicating this address value can be read from
    } else {                                     // I2C device did not successfully acknowledge the request
      if (Serial) {
        Serial.print("Could not find T+H sensor ");
        Serial.print(i);
        Serial.print(" at address 0x");
        Serial.println(HIH6120_address[i], HEX);
      }
    }
  }
  delay(50);  //Give HIH6120's time to convert readings
  for (int i = 0; i < 10; i++) {
    Wire.requestFrom(HIH6120_address[i], 4);  //Read data from I2C address, 4 bytes for RH and T
    if (Wire.available()) {
      uint8_t a = Wire.read();  //Store data in seperate temporary bytes
      uint8_t b = Wire.read();
      uint8_t c = Wire.read();
      uint8_t d = Wire.read();

      if (a & 0x80) {  //Check first bit to see if HIH61xx is in command mode
        if (Serial) {
          Serial.print("T+H sensor ");
          Serial.print(i);
          Serial.println(" in command mode");
        }
      } else if (a & 0x40) {           //Check second bit to see if data is "stale"
        HIH6120_data_flags |= 1 << i;  //Set data flag to 1 if stale
      } else {
        HIH6120_data_flags &= ~1 << i;          //set data flag to 0 if normal data
        HIH_RH_raw[i] = ((a & 0x3f) << 8) | b;  //Compile bytes into relevant data
        HIH_T_raw[i] = (c << 6) | (d >> 2);
        HIH_RH[i] = (HIH_RH_raw[i] / 163.82);
        HIH_T[i] = ((HIH_T_raw[i] / 16382.0) * 165.0) - 40.0;
      }

    } else {
      if (Serial) {
        Serial.print("Could not read from T+H sensor ");
        Serial.print(i);
        Serial.print(" at address 0x");
        Serial.println(HIH6120_address[i], HEX);
      }
    }
  }



  for (int i = 0; i < 8; i++) {
    if (analogRead(P_sig_pins[i]) > Cathode_flow_sensor_minimum_threshold) {  //Flow sensor should output a minimum of 10% V_in, which should be read as 0.5V. Threshold of 81=0.396
      P_sig_flags |= 1 << i;
    }
  }
  if (Serial) {
    Serial.print("Pressure sensors connected = ");
    Serial.println(P_sig_flags, BIN);
  }

  pwm.begin();           //Connect to the I2C PWM driver
  pwm.setPWMFreq(1600);  // This is the maximum PWM frequency
  //EDITME add code to test PWM outs one by one?

  //EDITME add code to test other outputs one by one?

  pinMode(AND_FLW_PURGE, OUTPUT);
  digitalWrite(AND_FLW_PURGE, HIGH);  //Purge line high = normal operation, low = purge
  pinMode(AND_FLW_VLV_OFF, OUTPUT);
  digitalWrite(AND_FLW_VLV_OFF, LOW);                             //Off line high = normal operation, low = closed valve
  AND_FLW_reading[1] = analogRead(AND_FLW_SIG_PIN);               //Voltage output from flow controller
  AND_FLW_reading[2] = analogRead(AND_FLW_SIG2_PIN);              //Current output from flow controller
  AND_FLW_value[1] = map(AND_FLW_reading[1], 0, 1023, 0, 50000);  //Convert analogue reading to mL per minute
  AND_FLW_value[2] = map(AND_FLW_reading[2], 0, 1023, 0, 50000);  //Convert analogue reading to mL per minute
  //EDITME Write some code to test if the flow controller is working
}

void loop() {

  for (int i = 0; i < 3; i++) {
    if (digitalRead(CTHD_HTR_SFTY_DRDY[i])) {                                       //Check if a reading from the Cathode heater safety thermocouple is ready
      CTHD_HTR_SFTY_TC_readings[i] = HTR_SFTY_TC[i].readThermocoupleTemperature();  //Read Cathode heater safety thermocouple
    }
  }


  if (CTHD_HTR_SFTY_TC_readings[1] > TC_high_temp_thresh) {
    //EDITME move to cathode heater control section, and copy for other safety TC's
  }

  CTHD_FLW_reading = map(analogRead(CTHD_FLW_SIG_PIN), 204, 1023, 0, 500);


  if (millis() - HIH_last_read_time > HIH_interval) {  //HIH takes up to 50 mS to convert, so for speed/non blocking, cycle triggers next conversion immediately after reading
    for (int i = 0; i < 10; i++) {
      if (HIH6120_flags & (1 << i)) {             //Only interact with HIH61xx's that have been detected
        Wire.requestFrom(HIH6120_address[i], 4);  //Read data from I2C address, 4 bytes for RH and T
        if (Wire.available()) {
          uint8_t a = Wire.read();  //Store data in seperate temporary bytes
          uint8_t b = Wire.read();
          uint8_t c = Wire.read();
          uint8_t d = Wire.read();

          if (a & 0x80) {  //Check first bit to see if HIH61xx is in command mode
            if (Serial) {
              Serial.print("T+H sensor ");
              Serial.print(i);
              Serial.println(" in command mode");
            }
          } else if (a & 0x40) {           //Check second bit to see if data is "stale"
            HIH6120_data_flags |= 1 << i;  //Set data flag to 1 if stale //EDITME do somethign about stale data
          } else {
            HIH6120_data_flags &= ~1 << i;          //set data flag to 0 if normal data
            HIH_RH_raw[i] = ((a & 0x3f) << 8) | b;  //Compile bytes into relevant data
            HIH_T_raw[i] = (c << 6) | (d >> 2);
            HIH_RH[i] = (HIH_RH_raw[i] / 163.82);  //Convert data to SI units as floats
            HIH_T[i] = ((HIH_T_raw[i] / 16382.0) * 165.0) - 40.0;
          }

        } else {
          if (Serial) {
            Serial.print("Could not read from T+H sensor ");
            Serial.print(i);
            Serial.print(" at address 0x");
            Serial.println(HIH6120_address[i], HEX);
          }
        }
        Wire.beginTransmission(HIH6120_address[i]);  //Send measurement request to HIH6120 address for next cycle
        int error = Wire.endTransmission();          //stop transmitting
        if (error != 0) {                            // I2C device did not successfully acknowledge the request
          if (Serial) {
            Serial.print("Could not find T+H sensor ");
            Serial.print(i);
            Serial.print(" at address 0x");
            Serial.println(HIH6120_address[i], HEX);
          }
        }
      }
    }
    HIH_last_read_time = millis();
  }

  for (int i = 0; i < 8; i++) {
    if (P_sig_flags & (1 << i)) {
      P_readings[i] = ((analogRead(P_sig_pins[i]) * 1000000) - 102400000) / 1024  //Convert analog reading to pressure in Pa
    }
  }

  AND_FLW_reading[1] = analogRead(AND_FLW_SIG_PIN);               //Voltage output from flow controller
  AND_FLW_reading[2] = analogRead(AND_FLW_SIG2_PIN);              //Current output from flow controller
  AND_FLW_value[1] = map(AND_FLW_reading[1], 0, 1023, 0, 50000);  //Convert analogue reading to mL per minute
  AND_FLW_value[2] = map(AND_FLW_reading[2], 0, 1023, 0, 50000);  //Convert analogue reading to mL per minute

  
}