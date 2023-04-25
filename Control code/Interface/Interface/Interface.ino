#include <Wire.h>
#include "rgb_lcd.h"
#include <SoftwareSerial.h>
#include "SerialTransfer.h"
#include <Adafruit_NeoPixel.h>
#include <RotaryEncoder.h>

#define PIN_ENCODER_A 6
#define PIN_ENCODER_B 7
#define COM_A    6
#define COM_B    6
#define BUTTON_CENTRE 8
#define BUTTON_DOWN 9
#define BUTTON_RIGHT 10
#define BUTTON_UP 11
#define BUTTON_LEFT 12

RotaryEncoder encoder(PIN_ENCODER_A, PIN_ENCODER_B, RotaryEncoder::LatchMode::TWO03);
// This interrupt will do our encoder reading/checking!
void checkPosition() {
  encoder.tick(); // just call tick() to check the state.
}
int last_rotary = 0;

#define NUMPIXELS 16
Adafruit_NeoPixel pixels(NUMPIXELS, 0, NEO_RGBW + NEO_KHZ800);

SerialPIO UART2(14, 15, 32);
SerialPIO UART3(16, 17, 32);

rgb_lcd MenuLCD;
rgb_lcd AlertLCD;

SerialTransfer CVM;
SerialTransfer DataLogger;
SerialTransfer LoadController;
SerialTransfer ProcessController;

// struct Fault_message {
//   uint8_t Fault_code = 0;
//   uint8_t Fault_detail = 0;
// };
// struct Fault_message Incoming_fault;

bool Core0_boot_flag = 0;  //Flag to tell Core1 that Core0 has successfully booted
bool Core1_boot_flag = 0;  //Flag to tell Core0 that Core1 has successfully booted

struct SettingsProcess {
  float TC_low_temp_thresh = 10.0;
  float TC_high_temp_thresh = 100.0;
  uint8_t TC_high_temp_thresh_warning_buffer = 5;
  float Cathode_target_T = 77.0;                                              //target temperature for cathode inlet
  float Cathode_T_Hysteresis = 1;                                             //variation from target before action is taken
  uint8_t Cathode_flow_sensor_minimum_threshold = 190;                        //Minimum expected value if sensor is connected and operating correctly
  float CTHD_FLW_stoich = 2;                                                  //
  uint16_t CTHD_FLW_min = 65;                                                 //Minimum flow rate allowed, SLPM
  double Coolant_T_target = 72;                                               //Target temperature for coolant at stack inlet
  double Coolant_T_Hysteresis = 1;                                            //Variation from target before action is taken
  double Coolant_high_temp_thresh = 80;                                       //Coolant overtemperature limit
  double Coolant_high_temp_warning_buffer = 5;                                //
  uint8_t HIH6120_address[8] = { 0x27, 0x28, 0x29, 0x30, 0x31, 0x32, 0x33 };  //Array of expected adresses of Humidity/Temperature sensors. Must be <0x39 for fault code to have space
  uint8_t HIH6120_flags = 0;                                                  //Flags to indicate if devices were found at relevant addresses
  uint16_t HIH_interval = 50;                                                 //Time(mS) required for HIH to do data conversion, therefore time between readings
  uint8_t P_sig_flags = 0;                                                    //Flags representing if pressure signals are detected. 1 means sensor detected on that pin, 0 means no sensor detected
  uint8_t Pressure_sensor_minimum_threshold = 81;                             //Minimum expected value if sensor is connected and operating correctly
  uint8_t H2N2_flag = 0;                                                      //0 means N2, 1 means H2
  uint16_t Data_interval = 200;                                               //time in mS between reporting data to logger
} Settings;

void setup() {

  Serial.begin(115200);  //Connect to computer over USB, opional
  delay(100);            //Give USB time to connect EDITME to shortest reliable time
  if (Serial) {
    // USB_flag = 1;  //Used so that when connected via USB, verbose status and error messages can be sent, but will still run if not connected
    Serial.println("Interface connected");
  } else {
    Serial.end();
  }

  Wire.setSDA(20);  //Set pins for RTC I2C
  Wire.setSCL(21);

  Wire1.setSDA(2);  //Set pins for RTC I2C
  Wire1.setSCL(3);

  MenuLCD.begin(16, 2);
  MenuLCD.begin(16, 2);
  //EDITME add boot message?


  Serial1.setRX(1);  //Set UART pins for UART0, to be used for CVM connection
  Serial1.setTX(0);
  // Serial1.setFIFOSize(128);      //May be required for stable UART comms
  Serial1.setPollingMode(true);  //Ensure UART0 port is listening for messages
  Serial1.begin(115200);         //Initialise UART0 port
  CVM.begin(Serial1);            //Initialise Serial Transfer object for CVM connection

  Serial2.setRX(5);  //Set UART pins for UART1, to be used for Interface connection
  Serial2.setTX(4);
  // Serial2.setFIFOSize(size_t 128); //May be required for stable UART comms
  Serial2.setPollingMode(true);  //Ensure UART1 port is listening for messages
  Serial2.begin(115200);         //Initialise UART1 port
  DataLogger.begin(Serial2);     //Initialise Serial Transfer object for Interface connection

  UART2.begin(115200);
  LoadController.begin(UART2);

  UART3.begin(115200);
  ProcessController.begin(UART3);

  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_A), checkPosition, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_B), checkPosition, CHANGE);

  pinMode(BUTTON_UP, INPUT_PULLUP);
  pinMode(BUTTON_DOWN, INPUT_PULLUP);
  pinMode(BUTTON_LEFT, INPUT_PULLUP);
  pinMode(BUTTON_RIGHT, INPUT_PULLUP);
  pinMode(BUTTON_IN, INPUT_PULLUP);
  pixels.begin();
  pixels.setBrightness(30);
  pixels.show();

}

void loop() {

    int curr_rotary = encoder.getPosition();
  RotaryEncoder::Direction direction = encoder.getDirection();
  //EDITME write code to connect rotary encoder to menu inputs
}
