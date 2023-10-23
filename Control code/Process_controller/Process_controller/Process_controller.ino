#include <SPI.h>
#include <Adafruit_MAX31856.h>
#include <Adafruit_PWMServoDriver.h>
#include "SerialTransfer.h"
#include <AutoPID.h>
#include <Thermistor.h>
#include <AverageThermistor.h>
#include <NTC_Thermistor.h>


SerialTransfer SafetySystem;
SerialTransfer Interface;
SerialTransfer DataLogger;

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

//PWM driver definitions
#define CLNT_PUMP_PWM 0
#define AND_FLW_CONT_PWM 1
#define AND_RECIRC_PWM 2
#define CTHD_VLV_PWM 3
const uint8_t CLNT_FN_PWMS[6] = { 4, 5, 6, 7, 8, 9 };

#define CTHD_FLW_SIG_PIN A0
#define P_0_SIG_PIN A1
#define P_1_SIG_PIN A2
#define P_2_SIG_PIN A3
#define P_3_SIG_PIN A4
#define P_4_SIG_PIN A5
#define P_5_SIG_PIN A6
#define P_6_SIG_PIN A7
#define P_7_SIG_PIN A8
#define AND_FLW_SIG_PIN A9
#define AND_FLW_SIG2_PIN A10

#define CTHD_HTR_SFTY_TC0_DRDY 22
#define CTHD_HTR_SFTY_TC0_FLT 23
#define CTHD_HTR_SFTY_TC0_CS 24
#define CTHD_HTR_SFTY_TC1_DRDY 25
#define CTHD_HTR_SFTY_TC1_FLT 26
#define CTHD_HTR_SFTY_TC1_CS 27
#define CTHD_HTR_SFTY_TC2_DRDY 28
#define CTHD_HTR_SFTY_TC2_FLT 29
#define CTHD_HTR_SFTY_TC2_CS 30

#define Number_of_CTHD_HTR_SFTY_TCs 2
const uint8_t CTHD_HTR_SFTY_DRDY[3] = { CTHD_HTR_SFTY_TC0_DRDY, CTHD_HTR_SFTY_TC1_DRDY, CTHD_HTR_SFTY_TC2_DRDY };
const uint8_t CTHD_HTR_SFTY_FLT[3] = { CTHD_HTR_SFTY_TC0_FLT, CTHD_HTR_SFTY_TC1_FLT, CTHD_HTR_SFTY_TC2_FLT };
const uint8_t CTHD_HTR_SFTY_CS[3] = { CTHD_HTR_SFTY_TC0_CS, CTHD_HTR_SFTY_TC1_CS, CTHD_HTR_SFTY_TC2_CS };

Adafruit_MAX31856 HTR_SFTY_TC[Number_of_CTHD_HTR_SFTY_TCs] = {
  Adafruit_MAX31856(CTHD_HTR_SFTY_CS[0]),  //Create TC amplifier objects using built in SPI by passing only the CS pins
  Adafruit_MAX31856(CTHD_HTR_SFTY_CS[1])
};

uint8_t HTR_SFTY_TC_fault[Number_of_CTHD_HTR_SFTY_TCs] = { 0 };  //Create bytes to store TC amp faults

#define AND_TRC_HTR_PIN 31
#define CTHD_TRC_HTR_PIN 32
#define CTHD_HTR_PIN 34
#define HMD_FEED_PUMP_PIN 35
#define CTHD_HMD_FEED_VLV_PIN 36
#define AND_HMD_FEED_VLV_PIN 37
#define AND_FLW_PURGE_PIN 38
#define AND_FLW_VLV_OFF_PIN 39
#define CTHD_SLND_PIN 40
#define AND_H2_SLND_PIN 41
#define AND_N2_SLND_PIN 42
#define AND_PURGE_SLND_PIN 43
#define DATA_SPI_CS 47
#define IPU_FLG 3

#define Number_of_pressure_signals 5
const uint8_t P_sig_pins[Number_of_pressure_signals] = { P_0_SIG_PIN, P_1_SIG_PIN, P_2_SIG_PIN, P_3_SIG_PIN, P_4_SIG_PIN };  //Array of Pressure signal inputs

Thermistor* Coolant_T[3];
const uint8_t Coolant_T_pins[3] = { P_5_SIG_PIN, P_6_SIG_PIN, P_7_SIG_PIN };

#define Coolant_T_REFERENCE_RESISTANCE 4700
#define Coolant_T_NOMINAL_RESISTANCE 10000
#define Coolant_T_NOMINAL_TEMPERATURE 25
#define Coolant_T_B_VALUE 3984

uint8_t HIH6120_data_flags = 0;  //Flags to indicate of data read is "stale" (1) or "normal" (0)
uint16_t HIH_RH_raw[8] = { 0 };  //Array to store RH raw data in
uint16_t HIH_T_raw[8] = { 0 };   //Array to store T raw data in
uint32_t HIH_last_read_time = 0;

uint16_t AND_FLW_reading[2] = { 0 };
double CTHD_FLW_reading_PID = 0;
double CTHD_FLW_target_PID = 0;  //in SLPM
double CTHD_FLW_output_PID = 0;
double CTHD_FLW_PID_KP = 30;
double CTHD_FLW_PID_KI = 10;
double CTHD_FLW_PID_KD = 0.025;
const double CTHD_PWM_min = 0;
const double CTHD_PWM_max = 4000;

// double AND_P_reading_PID = 0;
// double AND_P_target_PID = 80000;  //in Pa
// double AND_FLW_output_PID = 0;
// double AND_FLW_PID_KP = 30;  //EDITME needs tuning
// double AND_FLW_PID_KI = 10;
// double AND_FLW_PID_KD = 0.025;
// const double AND_PWM_min = 0;
// const double AND_PWM_max = 4000;

AutoPID CTHD_FLW_PID(&CTHD_FLW_reading_PID, &CTHD_FLW_target_PID, &CTHD_FLW_output_PID, CTHD_PWM_min, CTHD_PWM_max, CTHD_FLW_PID_KP, CTHD_FLW_PID_KI, CTHD_FLW_PID_KD);
// AutoPID AND_FLW_PID(&AND_P_reading_PID, &AND_P_target_PID, &AND_FLW_output_PID, AND_PWM_min, AND_PWM_max, AND_FLW_PID_KP, AND_FLW_PID_KI, AND_FLW_PID_KD);


//Definitions for calculations
#define num_cells 24
#define Moles_per_litre_x_F_const_x_electrons_per_mole 8615

struct Current_data {
  uint16_t set = 20;
  uint16_t sense = 20;
} Current;

struct Fault_message {
  uint8_t Fault_code = 0;
  uint8_t Fault_detail = 0;
};
struct Fault_message Incoming_fault;

struct DataFrameProcess {
  float CTHD_HTR_SFTY_TC[2] = { 0, 0 };       //DegC
  double CTHD_FLW = 0;                        //SLPM
  double CTHD_FLW_target = 0;                 //in SLPM
  uint16_t CTHD_FLW_output = 0;               //12 bit PWM value
  uint16_t AND_FLW_value[2] = { 0 };          //SmLPM
  uint16_t AND_FLW_target = 0;                //SmLPM
  uint16_t AND_FLW_output = 0;                //12 bit PWM value
  uint16_t AND_recirc_PWM = 4090;             //12 bit PWM value
  uint16_t CLNT_FLW_output = 4095;            //Coolant pump 12 bit PWM value
  double Coolant_T_readings[3];               //
  uint16_t Fan_setting = 0;                   //12 bit PWM setting for collant system fan speed
  float HIH_RH[8] = { 0 };                    //Array to store converted RH data (as %)
  float HIH_T[8] = { 0 };                     //Array to store converted T data (deg C)
  int32_t P_readings[5] = { 0, 0, 0, 0, 0 };  //Pa
  double O2_conc = 0.21;                      //
  bool CTHD_HTR = 0;                          //is the cathode heater on/off
  uint8_t Valves = 0;                         //Bit flags for what valves are set. Order is anode humidifier feed, cathode humidifier feed, anode flow controller purge, anode flow controller valve, anode purge, H2 in, N2 in //EDITME got hrough and change valve actions to prevent overwrite
  uint32_t Process_controller_timestamp = 0;  //timestamp in mS that data was sent out
} Outgoing_data;

struct SettingsProcess {
  float TC_low_temp_thresh = 10.0;
  float TC_high_temp_thresh = 100.0;
  uint8_t TC_high_temp_thresh_warning_buffer = 5;
  float Cathode_target_T = 77.0;                                        //target temperature for cathode inlet
  float Cathode_T_Hysteresis = 1;                                       //variation from target before action is taken
  uint8_t Cathode_flow_sensor_minimum_threshold = 190;                  //Minimum expected value if sensor is connected and operating correctly
  float CTHD_FLW_stoich = 2;                                            //
  uint16_t CTHD_FLW_min = 65;                                           //Minimum flow rate allowed, SLPM
  double Coolant_T_target = 35;                                         //Target temperature for coolant at stack inlet
  double Coolant_T_Hysteresis = 1;                                      //Variation from target before action is taken
  double Coolant_high_temp_thresh = 80;                                 //Coolant overtemperature limit
  double Coolant_high_temp_warning_buffer = 5;                          //
  uint8_t HIH6120_address[8] = { 0x29, 0x31, 0x32, 0x34, 0x35, 0x36 };  //Array of expected adresses of Humidity/Temperature sensors. Must be <0x39 for fault code to have space
  uint8_t HIH6120_flags = 0;                                            //Flags to indicate if devices were found at relevant addresses
  uint16_t HIH_interval = 50;                                           //Time(mS) required for HIH to do data conversion, therefore time between readings
  uint8_t P_sig_flags = 0;                                              //Flags representing if pressure signals are detected. 1 means sensor detected on that pin, 0 means no sensor detected
  uint8_t Pressure_sensor_minimum_threshold = 81;                       //Minimum expected value if sensor is connected and operating correctly
  uint8_t H2N2_flag = 0;                                                //0 means N2, 1 means H2
  uint16_t Data_interval = 200;                                         //time in mS between reporting data to logger
  uint32_t AND_P_target = 800;                                          //Target anode pressure mBar (reduced flow above this threshold)
  float AND_FLW_stoich = 1.2;                                           //Oversupply to maintain max pressure
  uint16_t Reading_interval = 100;                                      //Time in mS between reading sensor data
  float Flow_controller_K[2] = { 1, 1.35 };                             // Gas conversion factors for flow rate. {N2, H2}
  uint8_t P_offsets[5]={98,98,98,98,98};
} Settings;

uint32_t Last_reading_time = 0;
uint8_t Startup_procedure_counter = 0;
//T+H pointers
float* Cathode_inlet_T = &Outgoing_data.HIH_T[3];
double* Coolant_inlet_T = &Outgoing_data.Coolant_T_readings[1];
int32_t* AND_stack_inlet_P = &Outgoing_data.P_readings[4];
int32_t* AND_inlet_P = &Outgoing_data.P_readings[4];
int32_t* AND_recirc_P = &Outgoing_data.P_readings[5];


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

void setup() {

  Serial.begin(115200);  //Connect to computer over USB, optional
  delay(100);            //Give USB time to connect EDITME to shortest reliable time
  if (Serial) {
    // USB_flag = 1;  //Used so that when connected via USB, verbose status and error messages can be sent, but will still run if not connected
    Serial.println("Process controller connected");
  } else {
    Serial.end();
  }

  // Serial1.begin(115200);  //Connect to SFTY UART
  // while (!Serial1) {
  //   ;  // wait for serial port to connect to SFTY
  // }
  // SafetySystem.begin(Serial1);
  // if (Serial) {
  //   Serial.println("Safety system connected");
  // }

  // Serial2.begin(115200);  //Connect to Interface UART
  // while (!Serial2) {
  //   ;  // wait for serial port to connect to Interface
  // }
  // Interface.begin(Serial2);
  // if (Serial) {
  //   Serial.println("Interface connected");
  // }

  // Serial3.begin(115200);  //Connect to Data UART
  // while (!Serial3) {
  //   ;  // wait for serial port to connect to Data
  // }
  // DataLogger.begin(Serial3);
  // if (Serial) {
  //   Serial.println("Data logger connected");
  // }

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(500);  //Check max allowable with hardware

  CTHD_HTR_TC_setup();

  Wire.begin();  //Initialise I2C bus as a master
  HIH6120_setup();

  // Cathode flow meter setup
  if (analogRead(CTHD_FLW_SIG_PIN) < Settings.Cathode_flow_sensor_minimum_threshold) {  //Flow sensor should output a minimum of 4mA, which should be read as 1V. Threshold of 190=0.928V
    // FaultSend(Interface, 'f', 0x21, 0x01);                                              //if reading is outside nominal, send out fault code
    // FaultSend(DataLogger, 'f', 0x21, 0x01);
    if (Serial) {
      Serial.println("Could not read Cathode flow sensor");
    }
  }

  //Pressure sensor setup
  for (int i = 0; i < 5; i++) {
    if (analogRead(P_sig_pins[i]) > Settings.Pressure_sensor_minimum_threshold) {  //Flow sensor should output a minimum of 10% V_in, which should be read as 0.5V. Threshold of 81=0.396
      Settings.P_sig_flags |= 1 << i;
    }
  }
  if (Serial) {
    Serial.print("Pressure sensors connected = ");
    Serial.println(Settings.P_sig_flags, BIN);
  }
  if (Settings.P_sig_flags != 0b00011111) {
    // FaultSend(Interface, 'f', 0x41, Settings.P_sig_flags);  //Fault code for P sensor under-reading
    // FaultSend(DataLogger, 'f', 0x41, Settings.P_sig_flags);
  }

  pwm.setPWM(CLNT_PUMP_PWM, 0, Outgoing_data.CLNT_FLW_output);
  pinMode(AND_H2_SLND_PIN, OUTPUT);
  digitalWrite(AND_H2_SLND_PIN, LOW);  //LOW = closed
  pinMode(AND_N2_SLND_PIN, OUTPUT);
  digitalWrite(AND_N2_SLND_PIN, LOW);  //LOW = closed
  pinMode(AND_PURGE_SLND_PIN, OUTPUT);
  digitalWrite(AND_PURGE_SLND_PIN, LOW);  //LOW = closed
  pinMode(AND_FLW_PURGE_PIN, OUTPUT);
  digitalWrite(AND_FLW_PURGE_PIN, HIGH);                                                                                         //Purge line high = normal operation, low = purge
  pinMode(AND_FLW_VLV_OFF_PIN, OUTPUT);                                                                                          //
  digitalWrite(AND_FLW_VLV_OFF_PIN, LOW);                                                                                        //Off line high = normal operation, low = closed valve
  AND_FLW_reading[1] = analogRead(AND_FLW_SIG_PIN);                                                                              //Voltage output from flow controller
  AND_FLW_reading[2] = analogRead(AND_FLW_SIG2_PIN);                                                                             //Current output from flow controller
  Outgoing_data.AND_FLW_value[0] = map(AND_FLW_reading[0] * Settings.Flow_controller_K[Settings.H2N2_flag], 0, 1023, 0, 50000);  //Convert analogue reading to mL per minute
  Outgoing_data.AND_FLW_value[1] = map(AND_FLW_reading[1] * Settings.Flow_controller_K[Settings.H2N2_flag], 0, 1023, 0, 50000);  //Convert analogue reading to mL per minute
  //EDITME Write some code to test if the flow controller is working

  //Coolant thermistor setup
  for (int i = 0; i < 3; i++) {
    Coolant_T[i] = new NTC_Thermistor(
      Coolant_T_pins[i],
      Coolant_T_REFERENCE_RESISTANCE,
      Coolant_T_NOMINAL_RESISTANCE,
      Coolant_T_NOMINAL_TEMPERATURE,
      Coolant_T_B_VALUE);
  }

  // pinMode(AND_TRC_HTR, OUTPUT);
  // pinMode(CTHD_TRC_HTR, OUTPUT);
  pinMode(CTHD_HTR, OUTPUT);
  digitalWrite(CTHD_HTR, LOW); 
  if (Serial) {
    Serial.println("Setup complete");
  }
}

void loop() {
  //EDITME add time averaging function for outgoing data?
  Outgoing_data.CTHD_FLW = map(analogRead(CTHD_FLW_SIG_PIN), 204, 1023, 0, 500);

  for (int i = 0; i < 5; i++) {
    // Outgoing_data.P_readings[i] = ((analogRead(P_sig_pins[i])-99)*5.0 /1024.0)*250000;  //Convert analog reading to pressure in Pa
    Outgoing_data.P_readings[i] = ((analogRead(P_sig_pins[i]) - Settings.P_offsets[i]) * 5.0 / 1024.0) * 2500;  //Convert analog reading to pressure in mBar
  }

  // // if ((*AND_stack_inlet_P > Settings.AND_P_target) || (*AND_inlet_P > Settings.AND_P_target) || (*AND_recirc_P > Settings.AND_P_target)) {
  // if ((Outgoing_data.P_readings[3] > Settings.AND_P_target) || (Outgoing_data.P_readings[4] > Settings.AND_P_target)) {
  //   digitalWrite(AND_FLW_VLV_OFF_PIN, LOW);  //Off line high = normal operation, low = closed valve
  // } else {
  //   digitalWrite(AND_FLW_VLV_OFF_PIN, HIGH);  //Off line high = normal operation, low = closed valve
  // }
  // if ((Outgoing_data.P_readings[3] > Settings.AND_P_target + 150) || (Outgoing_data.P_readings[4] > Settings.AND_P_target + 150)) {
  //   digitalWrite(AND_N2_SLND_PIN, LOW);  //Off line high = normal operation, low = closed valve
  //   digitalWrite(AND_H2_SLND_PIN, LOW);  //Off line high = normal operation, low = closed valve
  // } else {
  //   if (Settings.H2N2_flag) {
  //     digitalWrite(AND_H2_SLND_PIN, HIGH);  //Off line high = normal operation, low = closed valve
  //   } else {
  //     digitalWrite(AND_N2_SLND_PIN, HIGH);  //Off line high = normal operation, low = closed valve
  //   }
  // }
  // if ((Outgoing_data.P_readings[3] > Settings.AND_P_target + 250) || (Outgoing_data.P_readings[4] > Settings.AND_P_target + 250)) {
  //   digitalWrite(AND_PURGE_SLND_PIN, HIGH);  //LOW = closed
  // } else {
  //   digitalWrite(AND_PURGE_SLND_PIN, LOW);  //LOW = closed
  // }

  for (int i = 0; i < 3; i++) {
    Outgoing_data.Coolant_T_readings[i] = Coolant_T[i]->readCelsius();
  }

  AND_FLW_reading[0] = analogRead(AND_FLW_SIG_PIN);                             //Voltage output from flow controller
  AND_FLW_reading[1] = analogRead(AND_FLW_SIG2_PIN);                            //Current output from flow controller
  Outgoing_data.AND_FLW_value[0] = map(AND_FLW_reading[0] * Settings.Flow_controller_K[Settings.H2N2_flag], 0, 1023, 0, 50000);  //Convert analogue reading to mL per minute
  Outgoing_data.AND_FLW_value[1] = map(AND_FLW_reading[1] * Settings.Flow_controller_K[Settings.H2N2_flag], 0, 1023, 0, 50000);  //Convert analogue reading to mL per minute

   Anode_pressure_control();

  if (millis() - Last_reading_time > Settings.Reading_interval) {
    Last_reading_time = millis();

    Read_CTHD_SFTY_TC();
    HIH6120_read_all();
  }

  //EDITME move these functions into the mode sections
  // Cathode_heater_control();
  // pwm.setPWM(CTHD_VLV_PWM, 0, Outgoing_data.CTHD_FLW_output);  //write output value to PWM controller

  // if (DataLogger.available()) {  //EDITME change to while?
  //   char ID = DataLogger.currentPacketID();

  //   if (ID == 'I') {  //Current data
  //     DataLogger.rxObj(Current);
  //   } else if (ID == 'f') {              //Fault code
  //     DataLogger.rxObj(Incoming_fault);  //EDITME write code to deal with faults
  //   } else {
  //     FaultSend(DataLogger, 'f', 0x01, ID);
  //   }
  // }

  // if (SafetySystem.available()) {  //EDITME add safety system passthrough functions
  //   char ID = SafetySystem.currentPacketID();

  //   if (ID == 'f') {                       //fault code
  //     SafetySystem.rxObj(Incoming_fault);  //EDITME write code to deal with faults
  //   } else {
  //     FaultSend(SafetySystem, 'f', 0x01, ID);
  //   }
  // }

  // if (Interface.available()) {
  //   char ID = Interface.currentPacketID();

  //   if (ID == 'f') {                    //fault code
  //     Interface.rxObj(Incoming_fault);  //EDITME write code to deal with faults
  //   } else {
  //     FaultSend(Interface, 'f', 0x01, ID);
  //   }
  // }

  if (Mode == 0) {
    // FaultSend(Interface, 'f', 0x02, 0);
    // FaultSend(DataLogger, 'f', 0x02, 0);
    digitalWrite(CTHD_HTR_PIN, 0);           //turn off cathode heater for safety
    digitalWrite(AND_N2_SLND_PIN, LOW);      //Off line high = normal operation, low = closed valve
    digitalWrite(AND_H2_SLND_PIN, LOW);      //Off line high = normal operation, low = closed valve
    digitalWrite(AND_PURGE_SLND_PIN, HIGH);  //Open purge valve
    pwm.setPWM(CTHD_VLV_PWM, 0, 4095);       //Open cathode valve
    pwm.setPWM(AND_RECIRC_PWM, 0, 2);        //Recirc off
    for (int i = 0; i < 6; i++) {
      pwm.setPWM(CLNT_FN_PWMS[i], 0, 2);  //Coolant fans off
    }
    pwm.setPWM(CLNT_PUMP_PWM, 0, 2);  //Coolant pump off
  } else if (Mode == 1) {             //Startup procedure


  } else if (Mode == 2) {  //Shutdown procedure

  } else if (Mode == 3) {  //Normal power delivery
    // digitalWrite(AND_N2_SLND_PIN, HIGH);
    Cathode_heater_control();
    // Outgoing_data.CTHD_FLW_target = (Current.set * num_cells * 60 * Settings.CTHD_FLW_stoich) / (Outgoing_data.O2_conc * Moles_per_litre_x_F_const_x_electrons_per_mole * 2);  //Calculate cathode flowrate in SLPM //EDITME check units, and deal with potential floating point problems
    // if (Outgoing_data.CTHD_FLW_target < Settings.CTHD_FLW_min) { Outgoing_data.CTHD_FLW_target = Settings.CTHD_FLW_min; }                                                      //Ensure flow rate stays above minimum threshold at low current
    // CTHD_FLW_reading_PID = Outgoing_data.CTHD_FLW;
    // CTHD_FLW_target_PID = Outgoing_data.CTHD_FLW_target;
    // CTHD_FLW_PID.run();  //run PID calcs to find output value
    // Outgoing_data.CTHD_FLW_output = CTHD_FLW_output_PID;
    // pwm.setPWM(CTHD_VLV_PWM, 0, Outgoing_data.CTHD_FLW_output);  //write output value to PWM controller
    Cathode_flow_control();
    Outgoing_data.AND_FLW_target = (Current.set * num_cells * 60 * Settings.AND_FLW_stoich * 1000) / (Moles_per_litre_x_F_const_x_electrons_per_mole);  //Calculate anode flowrate in SmLPM //EDITME check units, and deal with potential floating point problems
    // Outgoing_data.AND_FLW_target = 10000;
    Outgoing_data.AND_FLW_output = map(Outgoing_data.AND_FLW_target, 0, 50000, 0, 4096);
    pwm.setPWM(AND_FLW_CONT_PWM, 0, Outgoing_data.AND_FLW_output);  //write output value to PWM controller



    //EDITME add recirc pump control
    pwm.setPWM(AND_RECIRC_PWM, 0, Outgoing_data.AND_recirc_PWM);  //Recirc on full

    //EDITME add purge interval/reaction code
    Coolant_T_control();
    pwm.setPWM(CLNT_PUMP_PWM, 0, Outgoing_data.CLNT_FLW_output);

  }

  else if (Mode == 4) {  //Low power (stack idle)
    Cathode_heater_control();

    Outgoing_data.CTHD_FLW_output = 1;
    pwm.setPWM(CTHD_VLV_PWM, 0, Outgoing_data.CTHD_FLW_output);  //write output value to PWM controller


    //EDITME add recirc pump control
    pwm.setPWM(AND_RECIRC_PWM, 0, 2000);  //Recirc on low

    //EDITME add purge interval/reaction code

    Coolant_T_control();
    pwm.setPWM(CLNT_PUMP_PWM, 0, Outgoing_data.CLNT_FLW_output);
    //EDITME write code to check humidities and offer feedwater injection

    //EDITME write code to detect if power is above idle threshold
  } else if (Mode == 5) {                //soft e-stop
    digitalWrite(CTHD_HTR_PIN, 0);       //turn off cathode heater for safety
    pwm.setPWM(CTHD_VLV_PWM, 0, 4095);   //Open cathode valve
    Settings.H2N2_flag = 0;              //Set to N2
    digitalWrite(AND_H2_SLND_PIN, LOW);  //Close H2 valve
    Outgoing_data.AND_FLW_target = 30000;
    Outgoing_data.AND_FLW_output = map(Outgoing_data.AND_FLW_target, 0, 50000, 0, 4096);

    pwm.setPWM(AND_FLW_CONT_PWM, 0, Outgoing_data.AND_FLW_output);  //write output value to PWM controller
    pwm.setPWM(AND_RECIRC_PWM, 0, 2);                               //Recirc off
    for (int i = 0; i < 6; i++) {
      pwm.setPWM(CLNT_FN_PWMS[i], 0, 4095);  //Coolant fans on full
    }
    pwm.setPWM(CLNT_PUMP_PWM, 0, Outgoing_data.CLNT_FLW_output);  //Coolant pump on
    digitalWrite(AND_PURGE_SLND_PIN, HIGH);
  } else if (Mode == 6) {                //Hard e-stop
    digitalWrite(CTHD_HTR_PIN, 0);       //turn off cathode heater for safety
    pwm.setPWM(CTHD_VLV_PWM, 0, 4095);   //Open cathode valve
    digitalWrite(AND_N2_SLND_PIN, LOW);  //Off line high = normal operation, low = closed valve
    digitalWrite(AND_H2_SLND_PIN, LOW);  //Off line high = normal operation, low = closed valve
        for (int i = 0; i < 6; i++) {
      pwm.setPWM(CLNT_FN_PWMS[i], 0, 2);  //Coolant fans off
    }
    pwm.setPWM(CLNT_PUMP_PWM, 0, 2);  //Coolant pump off
  } else {                               //Unknown mode
    // FaultSend(Interface, 'f', 0x03, 0);
    // FaultSend(DataLogger, 'f', 0x03, 0);
  }

  if (millis() - Outgoing_data.Process_controller_timestamp > Settings.Data_interval) {
    Outgoing_data.Process_controller_timestamp = millis();

    if (Serial) {
      // Serial.print("/*");
      Serial.print(Outgoing_data.CTHD_HTR_SFTY_TC[0]);
      Serial.print(",");
      Serial.print(Outgoing_data.CTHD_HTR_SFTY_TC[1]);
      Serial.print(",");
      Serial.print(Outgoing_data.CTHD_FLW);
      Serial.print(",");
      Serial.print(Outgoing_data.CTHD_FLW_target);
      Serial.print(",");
      Serial.print(Outgoing_data.CTHD_FLW_output);
      Serial.print(",");
      Serial.print(Outgoing_data.AND_FLW_value[0]);
      Serial.print(",");
      Serial.print(Outgoing_data.AND_FLW_value[1]);
      Serial.print(",");
      Serial.print(Outgoing_data.AND_FLW_target);
      Serial.print(",");
      Serial.print(Outgoing_data.AND_FLW_output);
      Serial.print(",");
      Serial.print(Outgoing_data.AND_recirc_PWM);
      Serial.print(",");
      Serial.print(Outgoing_data.CLNT_FLW_output);
      Serial.print(",");
      Serial.print(Outgoing_data.Coolant_T_readings[0]);
      Serial.print(",");
      Serial.print(Outgoing_data.Coolant_T_readings[1]);
      Serial.print(",");
      Serial.print(Outgoing_data.Coolant_T_readings[2]);
      Serial.print(",");
      Serial.print(Outgoing_data.Fan_setting);
      Serial.print(",");
      Serial.print(Outgoing_data.HIH_RH[0]);
      Serial.print(",");
      Serial.print(Outgoing_data.HIH_RH[1]);
      Serial.print(",");
      Serial.print(Outgoing_data.HIH_RH[2]);
      Serial.print(",");
      Serial.print(Outgoing_data.HIH_RH[3]);
      Serial.print(",");
      Serial.print(Outgoing_data.HIH_RH[4]);
      Serial.print(",");
      Serial.print(Outgoing_data.HIH_RH[5]);
      Serial.print(",");
      Serial.print(Outgoing_data.HIH_RH[6]);
      Serial.print(",");
      Serial.print(Outgoing_data.HIH_RH[7]);
      Serial.print(",");
      Serial.print(Outgoing_data.HIH_T[0]);
      Serial.print(",");
      Serial.print(Outgoing_data.HIH_T[1]);
      Serial.print(",");
      Serial.print(Outgoing_data.HIH_T[2]);
      Serial.print(",");
      Serial.print(Outgoing_data.HIH_T[3]);
      Serial.print(",");
      Serial.print(Outgoing_data.HIH_T[4]);
      Serial.print(",");
      Serial.print(Outgoing_data.HIH_T[5]);
      Serial.print(",");
      Serial.print(Outgoing_data.HIH_T[6]);
      Serial.print(",");
      Serial.print(Outgoing_data.HIH_T[7]);
      Serial.print(",");
      Serial.print(Outgoing_data.P_readings[0]);
      Serial.print(",");
      Serial.print(Outgoing_data.P_readings[1]);
      Serial.print(",");
      Serial.print(Outgoing_data.P_readings[2]);
      Serial.print(",");
      Serial.print(Outgoing_data.P_readings[3]);
      Serial.print(",");
      Serial.print(Outgoing_data.P_readings[4]);
      Serial.print(",");
      Serial.print(Outgoing_data.CTHD_HTR);
      Serial.print(",");
      Serial.println(Outgoing_data.Process_controller_timestamp);
      // Serial.println("*/");
    }


    //   uint16_t sendSize = 0;
    //   sendSize = DataLogger.txObj(Outgoing_data, sendSize);
    //   DataLogger.sendData(sendSize, 'd');
  }
}
