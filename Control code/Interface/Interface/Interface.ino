#include <Wire.h>
#include "rgb_lcd.h"
#include <SoftwareSerial.h>
#include "SerialTransfer.h"
// #include <Adafruit_NeoPixel.h> //EDITME uncomment after testing
#include <RotaryEncoder.h>
#include "CMBMenu.hpp"

#define PIN_ENCODER_A 6
#define PIN_ENCODER_B 7
#define COM_A 6
#define COM_B 6
#define BUTTON_CENTRE 8
#define BUTTON_DOWN 9
#define BUTTON_RIGHT 10
#define BUTTON_UP 11
#define BUTTON_LEFT 12

RotaryEncoder encoder(PIN_ENCODER_A, PIN_ENCODER_B, RotaryEncoder::LatchMode::TWO03);
int last_rotary = 0;

// #define NUMPIXELS 16
// Adafruit_NeoPixel pixels(NUMPIXELS, 0, NEO_RGBW + NEO_KHZ800); //EDITME uncomment after testing

SerialPIO UART2(14, 15, 32);
SerialPIO UART3(16, 17, 32);

rgb_lcd MenuLCD;
rgb_lcd AlertLCD;

SerialTransfer CVM;
SerialTransfer DataLogger;
SerialTransfer LoadController;
SerialTransfer ProcessController;

struct Button_states {
  bool states[5] = { 0, 0, 0, 0, 0 };                  //Centre, down, right, up, left
  bool readings[5] = { 0, 0, 0, 0, 0 };                //Centre, down, right, up, left
  uint32_t Last_debounce_time[5] = { 0, 0, 0, 0, 0 };  //Centre, down, right, up, left
  uint8_t Debounce_delay = 100;
} ButtonStates;

struct Fault_message {
  uint8_t Fault_code = 0;
  uint8_t Fault_detail = 0;
};
struct Fault_message Incoming_fault;

bool Core0_boot_flag = 0;  //Flag to tell Core1 that Core0 has successfully booted
bool Core1_boot_flag = 0;  //Flag to tell Core0 that Core1 has successfully booted

// define text to display
const char g_Menu1_pc[] PROGMEM = { "1. Procedures" };
const char g_Menu1_1_pc[] PROGMEM = { "1.1 Start up" };
const char g_Menu1_2_pc[] PROGMEM = { "1.2 Shutdown" };
const char g_Menu1_3_pc[] PROGMEM = { "1.3 Calibrate" };
const char g_Menu1_4_pc[] PROGMEM = { "1.4 Idle" };
const char g_Menu2_pc[] PROGMEM = { "2. Settings" };
const char g_Menu2_1_pc[] PROGMEM = { "2.1 Cell Voltage Monitor" };
const char g_Menu2_2_pc[] PROGMEM = { "2.2 Data Logger" };
const char g_Menu2_3_pc[] PROGMEM = { "2.3 Interface" };
const char g_Menu2_4_pc[] PROGMEM = { "2.4 Load Controller" };
const char g_Menu2_5_pc[] PROGMEM = { "2.5 Process Controller" };
const char g_Menu2_5_1_pc[] PROGMEM = { "2.5.1 Fetch Settings" };
const char g_Menu2_5_2_pc[] PROGMEM = { "2.5.2 FTC_low_temp_thresh" };
const char g_Menu2_5_3_pc[] PROGMEM = { "2.5.3 TC_high_temp_thresh" };
const char g_Menu2_5_4_pc[] PROGMEM = { "2.5.4 TC_high_temp_thresh_warning_buffer" };
const char g_Menu2_5_5_pc[] PROGMEM = { "2.5.5 Cathode_target_T" };
const char g_Menu2_5_6_pc[] PROGMEM = { "2.5.6 Cathode_T_Hysteresis" };
const char g_Menu2_5_7_pc[] PROGMEM = { "2.5.7 Cathode_flow_sensor_minimum_thresh" };
const char g_Menu2_5_8_pc[] PROGMEM = { "2.5.8 CTHD_FLW_stoich" };
const char g_Menu2_5_9_pc[] PROGMEM = { "2.5.9 CTHD_FLW_min" };
const char g_Menu2_5_10_pc[] PROGMEM = { "2.5.10 Coolant_T_target" };
const char g_Menu2_5_11_pc[] PROGMEM = { "2.5.11 Coolant_T_Hysteresis" };
const char g_Menu2_5_12_pc[] PROGMEM = { "2.5.12 Coolant_high_temp_thresh" };
const char g_Menu2_5_13_pc[] PROGMEM = { "2.5.13 Coolant_high_temp_warning_buffer" };
const char g_Menu2_5_14_pc[] PROGMEM = { "2.5.14 HIH6120_address" };
const char g_Menu2_5_14_0_pc[] PROGMEM = { "2.5.14.0 HIH6120_address 0" };
const char g_Menu2_5_14_1_pc[] PROGMEM = { "2.5.14.1 HIH6120_address 0" };
const char g_Menu2_5_14_2_pc[] PROGMEM = { "2.5.14.2 HIH6120_address 0" };
const char g_Menu2_5_14_3_pc[] PROGMEM = { "2.5.14.3 HIH6120_address 0" };
const char g_Menu2_5_14_4_pc[] PROGMEM = { "2.5.14.4 HIH6120_address 0" };
const char g_Menu2_5_14_5_pc[] PROGMEM = { "2.5.14.5 HIH6120_address 0" };
const char g_Menu2_5_14_6_pc[] PROGMEM = { "2.5.14.6 HIH6120_address 0" };
const char g_Menu2_5_14_7_pc[] PROGMEM = { "2.5.14.7 HIH6120_address 0" };
const char g_Menu2_5_15_pc[] PROGMEM = { "2.5.15 HIH6120_flags" };
const char g_Menu2_5_16_pc[] PROGMEM = { "2.5.16 HIH_interval" };
const char g_Menu2_5_17_pc[] PROGMEM = { "2.5.17 P_sig_flags" };
const char g_Menu2_5_18_pc[] PROGMEM = { "2.5.18 Pressure_sensor_minimum_threshold" };
const char g_Menu2_5_19_pc[] PROGMEM = { "2.5.19 H2N2_flag" };
const char g_Menu2_5_20_pc[] PROGMEM = { "2.5.20 Data_interval" };
const char g_Menu2_5_21_pc[] PROGMEM = { "2.5.21 AND_P_target" };
const char g_Menu2_5_22_pc[] PROGMEM = { "2.5.22 AND_FLW_stoich" };
const char g_Menu2_6_pc[] PROGMEM = { "2.6 Safety Controller" };

// define function IDs
enum MenuFID {
  Menu1,
  Menu1_1,
  Menu1_2,
  Menu1_3,
  Menu1_4,
  Menu2,
  Menu2_1,
  Menu2_2,
  Menu2_3,
  Menu2_4,
  Menu2_5,
  Menu2_5_1,
  Menu2_5_2,
  Menu2_5_3,
  Menu2_5_4,
  Menu2_5_5,
  Menu2_5_6,
  Menu2_5_7,
  Menu2_5_8,
  Menu2_5_9,
  Menu2_5_10,
  Menu2_5_11,
  Menu2_5_12,
  Menu2_5_13,
  Menu2_5_14,
  Menu2_5_14_0,
  Menu2_5_14_1,
  Menu2_5_14_2,
  Menu2_5_14_3,
  Menu2_5_14_4,
  Menu2_5_14_5,
  Menu2_5_14_6,
  Menu2_5_14_7,
  Menu2_5_15,
  Menu2_5_16,
  Menu2_5_17,
  Menu2_5_18,
  Menu2_5_19,
  Menu2_5_20,
  Menu2_5_21,
  Menu2_5_22,
  Menu2_6,
};

// define key types
enum KeyType {
  KeyNone,  // no key is pressed
  KeyLeft,
  KeyRight,
  KeyEnter,
  KeyExit,
  EncoderUp,
  EncoderDown
};



// ** menu **
// create global CMBMenu instance
// (here for maximum 100 menu entries)
CMBMenu<100> g_Menu;

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
  uint32_t AND_P_target = 80000;                                              //Target anode pressure (reduced flow above this threshold)
  float AND_FLW_stoich = 1.2;                                                 //Oversupply to maintain max pressure
} Settings_Process;

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
  // AlertLCD.begin(16, 2); //EDITME uncomment after testing
  //EDITME add boot message?

  MenuLCD.setRGB(255, 255, 255);
  MenuLCD.clear();
  MenuLCD.print("mblib example");
  delay(1000);


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
  pinMode(BUTTON_CENTRE, INPUT_PULLUP);

  Serial.println("===========================");
  Serial.println("mblib - example for CMBMenu");
  Serial.println("===========================");

  // pixels.begin();
  // pixels.setBrightness(30);
  // pixels.show(); //EDITME uncomment after testing

  g_Menu.addNode(0, g_Menu1_pc, Menu1);
  g_Menu.addNode(1, g_Menu1_1_pc, Menu1_1);
  g_Menu.addNode(1, g_Menu1_2_pc, Menu1_2);
  g_Menu.addNode(1, g_Menu1_3_pc, Menu1_3);
  g_Menu.addNode(1, g_Menu1_4_pc, Menu1_4);
  g_Menu.addNode(0, g_Menu2_pc, Menu2);
  g_Menu.addNode(1, g_Menu2_1_pc, Menu2_1);
  g_Menu.addNode(1, g_Menu2_2_pc, Menu2_2);
  g_Menu.addNode(1, g_Menu2_3_pc, Menu2_3);
  g_Menu.addNode(1, g_Menu2_4_pc, Menu2_4);
  g_Menu.addNode(1, g_Menu2_5_pc, Menu2_5);
  g_Menu.addNode(2, g_Menu2_5_1_pc, Menu2_5_1);
  g_Menu.addNode(2, g_Menu2_5_2_pc, Menu2_5_2);
  g_Menu.addNode(2, g_Menu2_5_3_pc, Menu2_5_3);
  g_Menu.addNode(2, g_Menu2_5_4_pc, Menu2_5_4);
  g_Menu.addNode(2, g_Menu2_5_5_pc, Menu2_5_5);
  g_Menu.addNode(2, g_Menu2_5_6_pc, Menu2_5_6);
  g_Menu.addNode(2, g_Menu2_5_7_pc, Menu2_5_7);
  g_Menu.addNode(2, g_Menu2_5_8_pc, Menu2_5_8);
  g_Menu.addNode(2, g_Menu2_5_9_pc, Menu2_5_9);
  g_Menu.addNode(2, g_Menu2_5_10_pc, Menu2_5_10);
  g_Menu.addNode(2, g_Menu2_5_11_pc, Menu2_5_11);
  g_Menu.addNode(2, g_Menu2_5_12_pc, Menu2_5_12);
  g_Menu.addNode(2, g_Menu2_5_13_pc, Menu2_5_13);
  g_Menu.addNode(2, g_Menu2_5_14_pc, Menu2_5_14);
  g_Menu.addNode(3, g_Menu2_5_14_0_pc, Menu2_5_14_0);
  g_Menu.addNode(3, g_Menu2_5_14_1_pc, Menu2_5_14_1);
  g_Menu.addNode(3, g_Menu2_5_14_2_pc, Menu2_5_14_2);
  g_Menu.addNode(3, g_Menu2_5_14_3_pc, Menu2_5_14_3);
  g_Menu.addNode(3, g_Menu2_5_14_4_pc, Menu2_5_14_4);
  g_Menu.addNode(3, g_Menu2_5_14_5_pc, Menu2_5_14_5);
  g_Menu.addNode(3, g_Menu2_5_14_6_pc, Menu2_5_14_6);
  g_Menu.addNode(3, g_Menu2_5_14_7_pc, Menu2_5_14_7);
  g_Menu.addNode(2, g_Menu2_5_15_pc, Menu2_5_15);
  g_Menu.addNode(2, g_Menu2_5_16_pc, Menu2_5_16);
  g_Menu.addNode(2, g_Menu2_5_17_pc, Menu2_5_17);
  g_Menu.addNode(2, g_Menu2_5_18_pc, Menu2_5_18);
  g_Menu.addNode(2, g_Menu2_5_19_pc, Menu2_5_19);
  g_Menu.addNode(2, g_Menu2_5_20_pc, Menu2_5_20);
  g_Menu.addNode(2, g_Menu2_5_20_pc, Menu2_5_21);
  g_Menu.addNode(2, g_Menu2_5_20_pc, Menu2_5_22);
  g_Menu.addNode(1, g_Menu2_6_pc, Menu2_6);

  // ** menu **
  // build menu and print menu
  // (see terminal for output)
  const char* info;
  g_Menu.buildMenu(info);
  g_Menu.printMenu();

  // ** menu **
  // print current menu entry
  printMenuEntry(info, 0);
}

void loop() {

  // function ID
  int fid = 0;

  // info text from menu
  const char* info;

  // go to deeper or upper layer?
  bool layerChanged = false;

  // determine pressed key
  KeyType key = getKey();

  // ** menu **
  // call menu methods regarding pressed key
  switch (key) {
    case KeyExit:
      g_Menu.exit();
      break;
    case KeyEnter:
      g_Menu.enter(layerChanged);
      break;
    case KeyRight:
      g_Menu.right();
      break;
    case KeyLeft:
      g_Menu.left();
      break;
    case EncoderUp:
      g_Menu.left();  //For encoder, anticlockwise is positive
      break;
    case EncoderDown:
      g_Menu.right();
      break;
    default:
      break;
  }

  // ** menu **
  // pint/update menu when key was pressed
  // and get current function ID "fid"
  if (KeyNone != key) {
    fid = g_Menu.getInfo(info);
    printMenuEntry(info, fid);
  }
  char DegC_text[2];
  sprintf(&DegC_text[0], "%cC", 223);

  if ((0 != fid) && (KeyEnter == key) && (!layerChanged)) {
    switch (fid) {
      case Menu1:
        break;
      case Menu1_1:
        break;
      case Menu1_2:
        break;
      case Menu1_3:
        break;
      case Menu1_4:
        break;
      case Menu2:
        break;
      case Menu2_1:
        break;
      case Menu2_2:
        break;
      case Menu2_3:
        break;
      case Menu2_4:
        break;
      case Menu2_5:
        break;
      case Menu2_5_1:
        Serial.println("Fetching settings from Process Controller");
        MenuLCD.setCursor(0, 1);
        MenuLCD.print("Fetching...");  //EDITME add fetch message code //EDITME add push settings node, save settings (SD) node, retrieve settings (SD) node
        break;
      case Menu2_5_2:
        MenuLCD.setCursor(14, 1);
        MenuLCD.print(DegC_text);
        Settings_Process.TC_low_temp_thresh = Increment_float(Settings_Process.TC_low_temp_thresh, -10, 70);
        MenuLCD.setCursor(14, 1);
        MenuLCD.print("  ");
        break;
      case Menu2_5_3:
        MenuLCD.setCursor(14, 1);
        MenuLCD.print(DegC_text);
        Settings_Process.TC_high_temp_thresh = Increment_float(Settings_Process.TC_high_temp_thresh, 60, 150);
        MenuLCD.setCursor(14, 1);
        MenuLCD.print("  ");
        break;
      case Menu2_5_4:
        MenuLCD.setCursor(14, 1);
        MenuLCD.print(DegC_text);
        Settings_Process.TC_high_temp_thresh_warning_buffer = Increment_int(Settings_Process.TC_high_temp_thresh_warning_buffer, 0, 20);
        MenuLCD.setCursor(14, 1);
        MenuLCD.print("  ");
        break;
      case Menu2_5_5:
        MenuLCD.setCursor(14, 1);
        MenuLCD.print(DegC_text);
        Settings_Process.Cathode_target_T = Increment_float(Settings_Process.Cathode_target_T, 10, 85);
        MenuLCD.setCursor(14, 1);
        MenuLCD.print("  ");
        break;
      case Menu2_5_6:
        MenuLCD.setCursor(14, 1);
        MenuLCD.print(DegC_text);
        Settings_Process.Cathode_T_Hysteresis = Increment_float(Settings_Process.Cathode_T_Hysteresis, 0.1, 10);
        MenuLCD.setCursor(14, 1);
        MenuLCD.print("  ");
        break;
      case Menu2_5_7:
        Settings_Process.Cathode_flow_sensor_minimum_threshold = Increment_int(Settings_Process.Cathode_flow_sensor_minimum_threshold, 0, 1024);
        break;
      case Menu2_5_8:
        Settings_Process.CTHD_FLW_stoich = Increment_float(Settings_Process.CTHD_FLW_stoich, 1, 10);
        break;
      case Menu2_5_9:
        MenuLCD.setCursor(0, 1);
        MenuLCD.print("            SLPM");
        Settings_Process.CTHD_FLW_min = Increment_int(Settings_Process.CTHD_FLW_min, 0, 200);
        MenuLCD.setCursor(12, 1);
        MenuLCD.print("    ");
        break;
      case Menu2_5_10:
        MenuLCD.setCursor(14, 1);
        MenuLCD.print(DegC_text);
        Settings_Process.Coolant_T_target = Increment_float(Settings_Process.Coolant_T_target, 10, 90);
        MenuLCD.setCursor(14, 1);
        MenuLCD.print("  ");
        break;
      case Menu2_5_11:
        MenuLCD.setCursor(14, 1);
        MenuLCD.print(DegC_text);
        Settings_Process.Coolant_T_Hysteresis = Increment_float(Settings_Process.Coolant_T_Hysteresis, 0.1, 20);
        MenuLCD.setCursor(14, 1);
        MenuLCD.print("  ");
        break;
      case Menu2_5_12:
        MenuLCD.setCursor(14, 1);
        MenuLCD.print(DegC_text);
        Settings_Process.Coolant_high_temp_thresh = Increment_float(Settings_Process.Coolant_high_temp_thresh, 50, 90);
        MenuLCD.setCursor(14, 1);
        MenuLCD.print("  ");
        break;
      case Menu2_5_13:
        MenuLCD.setCursor(14, 1);
        MenuLCD.print(DegC_text);
        Settings_Process.Coolant_high_temp_warning_buffer = Increment_float(Settings_Process.Coolant_high_temp_warning_buffer, 0, 30);
        MenuLCD.setCursor(14, 1);
        MenuLCD.print("  ");
        break;
      case Menu2_5_14_0:
        Settings_Process.HIH6120_address[0] = Increment_address(Settings_Process.HIH6120_address[0], 0, 0x39);
        break;
      case Menu2_5_14_1:
        Settings_Process.HIH6120_address[0] = Increment_address(Settings_Process.HIH6120_address[1], 0, 0x39);
        break;
      case Menu2_5_14_2:
        Settings_Process.HIH6120_address[0] = Increment_address(Settings_Process.HIH6120_address[2], 0, 0x39);
        break;
      case Menu2_5_14_3:
        Settings_Process.HIH6120_address[0] = Increment_address(Settings_Process.HIH6120_address[3], 0, 0x39);
        break;
      case Menu2_5_14_4:
        Settings_Process.HIH6120_address[0] = Increment_address(Settings_Process.HIH6120_address[4], 0, 0x39);
        break;
      case Menu2_5_14_5:
        Settings_Process.HIH6120_address[0] = Increment_address(Settings_Process.HIH6120_address[5], 0, 0x39);
        break;
      case Menu2_5_14_6:
        Settings_Process.HIH6120_address[0] = Increment_address(Settings_Process.HIH6120_address[6], 0, 0x39);
        break;
      case Menu2_5_14_7:
        Settings_Process.HIH6120_address[0] = Increment_address(Settings_Process.HIH6120_address[7], 0, 0x39);
        break;
      case Menu2_5_16:
        MenuLCD.setCursor(0, 1);
        MenuLCD.print("              mS");
        Settings_Process.HIH_interval = Increment_int(Settings_Process.HIH_interval, 0, 500);
        MenuLCD.setCursor(14, 1);
        MenuLCD.print("  ");
        break;
      case Menu2_5_18:
        Settings_Process.Pressure_sensor_minimum_threshold = Increment_int(Settings_Process.Pressure_sensor_minimum_threshold, 0, 500);
        break;
      case Menu2_5_20:
        MenuLCD.setCursor(0, 1);
        MenuLCD.print("              mS");
        Settings_Process.Data_interval = Increment_int(Settings_Process.Data_interval, 0, 500);
        MenuLCD.setCursor(14, 1);
        MenuLCD.print("  ");
        break;
      case Menu2_5_21:
        MenuLCD.setCursor(0, 1);
        MenuLCD.print("              Pa");
        Settings_Process.AND_P_target = Increment_int(Settings_Process.AND_P_target, 10000, 100000);
        MenuLCD.setCursor(14, 1);
        MenuLCD.print("  ");
        break;
      case Menu2_5_22:
        Settings_Process.AND_FLW_stoich = Increment_float(Settings_Process.AND_FLW_stoich, 0.8, 3);
        break;
      case Menu2_6:
        break;
      default:
        break;
    }
  }
}

KeyType getKey() {
  KeyType key = KeyNone;

  // // here for demonstration: get "pressed" key from terminal
  // // replace code when using push buttons
  // while(Serial.available() > 0) {
  //   String Key_s = Serial.readString();
  //   Key_s.trim();
  //   Serial.println("");
  //   if(Key_s.equals("l")) { // left
  //     key = KeyLeft;
  //     Serial.println("<left>");
  //   } else if (Key_s.equals("r")) { // right
  //     key = KeyRight;
  //     Serial.println("<right>");
  //   } else if (Key_s.equals("e")) { // enter
  //     key = KeyEnter;
  //     Serial.println("<enter>");
  //   } else if (Key_s.equals("x")) { // exit
  //     key = KeyExit;
  //     Serial.println("<exit>");
  //   } else if (Key_s.equals("m")) { // print menu
  //     g_Menu.printMenu();
  //   }
  // }

  int curr_rotary = encoder.getPosition();
  RotaryEncoder::Direction direction = encoder.getDirection();

  // if (curr_rotary > last_rotary) {
  //   key = KeyRight;
  // } else if (curr_rotary < last_rotary) {
  //   key = KeyLeft;
  // } else if (!digitalRead(BUTTON_IN)) {
  //   key = KeyEnter;
  // }

  if (curr_rotary > last_rotary) {
    key = EncoderUp;
    Serial.println(curr_rotary);
  } else if (curr_rotary < last_rotary) {
    key = EncoderDown;
    Serial.println(curr_rotary);
  } else if (debounce(BUTTON_CENTRE)) {
    key = KeyEnter;
    Serial.println("Enter");
  } else if (debounce(BUTTON_LEFT)) {
    key = KeyLeft;
    Serial.println("Left");
  } else if (debounce(BUTTON_RIGHT)) {
    key = KeyRight;
    Serial.println("Right");
  } else if (debounce(BUTTON_UP)) {
    key = KeyExit;
    Serial.println("Exit");
  } else if (debounce(BUTTON_DOWN)) {
    g_Menu.printMenu();
    Serial.println("Menu");
  }

  last_rotary = curr_rotary;

  return key;
}

// This interrupt will do our encoder reading/checking!
void checkPosition() {
  encoder.tick();  // just call tick() to check the state.
}


// ********************************************
// ** menu **
// printMenuEntry
// ********************************************
void printMenuEntry(const char* f_Info, int FID) {
  String info_s;
  MBHelper::stringFromPgm(f_Info, info_s);
  char Value_to_print[16];

  // when using LCD: add/replace here code to
  // display info on LCD
  Serial.println("----------------");
  Serial.println(info_s);
  Serial.println("----------------");

  // print on LCD
  MenuLCD.clear();
  MenuLCD.setCursor(0, 0);
  MenuLCD.print(info_s);

  switch (FID) {
    case Menu1:
      break;
    case Menu1_1:
      break;
    case Menu1_2:
      break;
    case Menu1_3:
      break;
    case Menu1_4:
      break;
    case Menu2:
      break;
    case Menu2_1:
      break;
    case Menu2_2:
      break;
    case Menu2_3:
      break;
    case Menu2_4:
      break;
    case Menu2_5:
      break;
    case Menu2_5_1:
      break;
    case Menu2_5_2:
      sprintf(&Value_to_print[0], "%.2f%cC", Settings_Process.TC_low_temp_thresh, 223);
      break;
    case Menu2_5_3:
      sprintf(&Value_to_print[0], "%.2f%cC", Settings_Process.TC_high_temp_thresh, 223);
      break;
    case Menu2_5_4:
      sprintf(&Value_to_print[0], "%u%cC", Settings_Process.TC_high_temp_thresh_warning_buffer, 223);
      break;
    case Menu2_5_5:
      sprintf(&Value_to_print[0], "%.2f%cC", Settings_Process.Cathode_target_T, 223);
      break;
    case Menu2_5_6:
      sprintf(&Value_to_print[0], "%.2f%cC", Settings_Process.Cathode_T_Hysteresis, 223);
      break;
    case Menu2_5_7:
      sprintf(&Value_to_print[0], "%u", Settings_Process.Cathode_flow_sensor_minimum_threshold);
      break;
    case Menu2_5_8:
      sprintf(&Value_to_print[0], "%.2f", Settings_Process.CTHD_FLW_stoich);
      break;
    case Menu2_5_9:
      sprintf(&Value_to_print[0], "%u SLPM", Settings_Process.CTHD_FLW_min);
      break;
    case Menu2_5_10:
      sprintf(&Value_to_print[0], "%.2f%cC", Settings_Process.Coolant_T_target, 223);
      break;
    case Menu2_5_11:
      sprintf(&Value_to_print[0], "%.2f%cC", Settings_Process.Coolant_T_Hysteresis, 223);
      break;
    case Menu2_5_12:
      sprintf(&Value_to_print[0], "%.2f%cC", Settings_Process.Coolant_high_temp_thresh, 223);
      break;
    case Menu2_5_13:
      sprintf(&Value_to_print[0], "%.2f%cC", Settings_Process.Coolant_high_temp_warning_buffer, 223);
      break;
    case Menu2_5_14:
      {
        int j = 0;
        for (int i = 0; i < 8; i++) {
          j += sprintf(&Value_to_print[j], "%X", Settings_Process.HIH6120_address[i]);
        }
      }
      break;
    case Menu2_5_14_0:
      sprintf(&Value_to_print[0], "0x%X", Settings_Process.HIH6120_address[0]);
      break;
    case Menu2_5_14_1:
      sprintf(&Value_to_print[0], "0x%X", Settings_Process.HIH6120_address[1]);
      break;
    case Menu2_5_14_2:
      sprintf(&Value_to_print[0], "0x%X", Settings_Process.HIH6120_address[2]);
      break;
    case Menu2_5_14_3:
      sprintf(&Value_to_print[0], "0x%X", Settings_Process.HIH6120_address[3]);
      break;
    case Menu2_5_14_4:
      sprintf(&Value_to_print[0], "0x%X", Settings_Process.HIH6120_address[4]);
      break;
    case Menu2_5_14_5:
      sprintf(&Value_to_print[0], "0x%X", Settings_Process.HIH6120_address[5]);
      break;
    case Menu2_5_14_6:
      sprintf(&Value_to_print[0], "0x%X", Settings_Process.HIH6120_address[6]);
      break;
    case Menu2_5_14_7:
      sprintf(&Value_to_print[0], "0x%X", Settings_Process.HIH6120_address[7]);
      break;
    case Menu2_5_15:
      sprintf(&Value_to_print[0], "%p", Settings_Process.HIH6120_flags);
      break;
    case Menu2_5_16:
      sprintf(&Value_to_print[0], "%u mS", Settings_Process.HIH_interval);
      break;
    case Menu2_5_17:
      sprintf(&Value_to_print[0], "%p", Settings_Process.P_sig_flags);
      break;
    case Menu2_5_18:
      sprintf(&Value_to_print[0], "%u", Settings_Process.Pressure_sensor_minimum_threshold);
      break;
    case Menu2_5_19:
      // sprintf(&Value_to_print[0], "%p", Settings_Process.H2N2_flag);
      if (Settings_Process.H2N2_flag) {
        Value_to_print[0] = 'H';
      } else {
        Value_to_print[0] = 'N';
      }
      Value_to_print[1] = '2';
      break;
    case Menu2_5_20:
      sprintf(&Value_to_print[0], "%u mS", Settings_Process.Data_interval);
      break;
    case Menu2_6:
      break;
    default:
      break;
  }

  // you can print here additional infos into second line of LCD
  MenuLCD.setCursor(0, 1);
  MenuLCD.print(Value_to_print);
}

bool debounce(int Pin) {
  bool reading = !digitalRead(Pin);

  // If the switch changed, due to noise or pressing:
  if (reading != ButtonStates.readings[Pin - 8]) {
    // reset the debouncing timer
    ButtonStates.Last_debounce_time[Pin - 8] = millis();
  }
  ButtonStates.readings[Pin - 8] = reading;

  if ((millis() - ButtonStates.Last_debounce_time[Pin - 8]) > ButtonStates.Debounce_delay) {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:

    // if the button state has changed:
    if (reading != ButtonStates.states[Pin - 8]) {
      ButtonStates.states[Pin - 8] = reading;
    }
  }
  // bool temp=ButtonStates.states[Pin - 8];
  // return ButtonStates.states[Pin - 8];
  if (ButtonStates.states[Pin - 8]) {
    ButtonStates.states[Pin - 8] = 0;
    ButtonStates.Last_debounce_time[Pin - 8] = millis();
    return 1;
  }
  // return temp;
  return ButtonStates.states[Pin - 8];
}

float Increment_float(float Value_in, float Min, float Max) {
  float New_value = Value_in;
  float Increment_size = 1;
  uint8_t Cursor_position = 3;
  int Exit_flag = 0;
  char Value_to_print[16];

  dtostrf(Value_in, 7, 2, Value_to_print);
  // sprintf(&Value_to_print[0], "%05.2f", Value_in);
  MenuLCD.setCursor(0, 1);
  MenuLCD.print(Value_to_print);
  MenuLCD.setCursor(Cursor_position, 1);
  MenuLCD.blink();

  while (!Exit_flag) {
    // determine pressed key
    KeyType key = getKey();

    // ** menu **
    // call menu methods regarding pressed key
    switch (key) {
      case KeyExit:
        MenuLCD.noBlink();
        return Value_in;
        break;
      case KeyEnter:
        Exit_flag = 1;
        break;
      case KeyRight:
        Increment_size *= 0.1;
        Cursor_position++;
        if (Cursor_position == 4) { Cursor_position = 5; }
        MenuLCD.setCursor(Cursor_position, 1);
        // MenuLCD.setCursor(0, 1);
        // MenuLCD.print(Value_to_print);
        break;
      case KeyLeft:
        Increment_size *= 10;
        Cursor_position--;
        if (Cursor_position == 4) {
          Cursor_position = 3;
        } else if (Cursor_position > 15) {
          Cursor_position = 0;
        }
        MenuLCD.setCursor(Cursor_position, 1);
        break;
      case EncoderUp:
        New_value -= Increment_size;
        if (New_value < Min) { New_value = Min; }
        // sprintf(&Value_to_print[0], "%05.2f", New_value);
        dtostrf(New_value, 7, 2, Value_to_print);
        MenuLCD.setCursor(0, 1);
        MenuLCD.print(Value_to_print);
        MenuLCD.setCursor(Cursor_position, 1);
        MenuLCD.blink();
        break;
      case EncoderDown:
        New_value += Increment_size;
        if (New_value > Max) { New_value = Max; }
        // sprintf(&Value_to_print[0], "%05.2f", New_value);
        dtostrf(New_value, 7, 2, Value_to_print);
        MenuLCD.setCursor(0, 1);
        MenuLCD.print(Value_to_print);
        MenuLCD.setCursor(Cursor_position, 1);
        MenuLCD.blink();
        break;
      default:
        break;
    }
  }
  MenuLCD.noBlink();
  return New_value;
}

float Increment_int(int Value_in, int Min, int Max) {
  int New_value = Value_in;
  int Increment_size = 1;
  uint8_t Cursor_position = 4;
  int Exit_flag = 0;
  char Value_to_print[16];

  // dtostrf(Value_in, 7, 2, Value_to_print);
  sprintf(&Value_to_print[0], "%5d", Value_in);
  MenuLCD.setCursor(0, 1);
  MenuLCD.print(Value_to_print);
  MenuLCD.setCursor(Cursor_position, 1);
  MenuLCD.blink();

  while (!Exit_flag) {
    // determine pressed key
    KeyType key = getKey();

    // ** menu **
    // call menu methods regarding pressed key
    switch (key) {
      case KeyExit:
        MenuLCD.noBlink();
        return Value_in;
        break;
      case KeyEnter:
        Exit_flag = 1;
        break;
      case KeyRight:
        Increment_size /= 10;
        Cursor_position++;
        if (Cursor_position == 5) {
          Cursor_position = 4;
          Increment_size = 1;
        }
        MenuLCD.setCursor(Cursor_position, 1);
        // MenuLCD.setCursor(0, 1);
        // MenuLCD.print(Value_to_print);
        break;
      case KeyLeft:
        Increment_size *= 10;
        Cursor_position--;
        if (Cursor_position > 15) {
          Cursor_position = 0;
        }
        MenuLCD.setCursor(Cursor_position, 1);
        break;
      case EncoderUp:
        New_value -= Increment_size;
        if (New_value < Min) { New_value = Min; }
        sprintf(&Value_to_print[0], "%5d", New_value);
        // dtostrf(New_value, 7, 2, Value_to_print);
        MenuLCD.setCursor(0, 1);
        MenuLCD.print(Value_to_print);
        MenuLCD.setCursor(Cursor_position, 1);
        MenuLCD.blink();
        break;
      case EncoderDown:
        New_value += Increment_size;
        if (New_value > Max) { New_value = Max; }
        sprintf(&Value_to_print[0], "%5d", New_value);
        // dtostrf(New_value, 7, 2, Value_to_print);
        MenuLCD.setCursor(0, 1);
        MenuLCD.print(Value_to_print);
        MenuLCD.setCursor(Cursor_position, 1);
        MenuLCD.blink();
        break;
      default:
        break;
    }
  }
  MenuLCD.noBlink();
  return New_value;
}

float Increment_address(int Value_in, int Min, int Max) {
  int New_value = Value_in;
  int Increment_size = 1;
  uint8_t Cursor_position = 3;
  int Exit_flag = 0;
  char Value_to_print[16];

  // dtostrf(Value_in, 7, 2, Value_to_print);
  sprintf(&Value_to_print[0], "0x%X", Value_in);
  MenuLCD.setCursor(0, 1);
  MenuLCD.print(Value_to_print);
  MenuLCD.setCursor(Cursor_position, 1);
  MenuLCD.blink();

  while (!Exit_flag) {
    // determine pressed key
    KeyType key = getKey();

    // ** menu **
    // call menu methods regarding pressed key
    switch (key) {
      case KeyExit:
        MenuLCD.noBlink();
        return Value_in;
        break;
      case KeyEnter:
        Exit_flag = 1;
        break;
      case KeyRight:
        Increment_size /= 16;
        Cursor_position++;
        if (Cursor_position == 4) {
          Cursor_position = 3;
          Increment_size = 1;
        }
        MenuLCD.setCursor(Cursor_position, 1);
        // MenuLCD.setCursor(0, 1);
        // MenuLCD.print(Value_to_print);
        break;
      case KeyLeft:
        Increment_size *= 16;
        Cursor_position--;
        if (Cursor_position < 2) {
          Cursor_position = 2;
          Increment_size = 16;
        }
        MenuLCD.setCursor(Cursor_position, 1);
        break;
      case EncoderUp:
        New_value -= Increment_size;
        if (New_value < Min) { New_value = Min; }
        sprintf(&Value_to_print[0], "0x%X", New_value);
        // dtostrf(New_value, 7, 2, Value_to_print);
        MenuLCD.setCursor(0, 1);
        MenuLCD.print(Value_to_print);
        MenuLCD.setCursor(Cursor_position, 1);
        MenuLCD.blink();
        break;
      case EncoderDown:
        New_value += Increment_size;
        if (New_value > Max) { New_value = Max; }
        sprintf(&Value_to_print[0], "0x%X", New_value);
        // dtostrf(New_value, 7, 2, Value_to_print);
        MenuLCD.setCursor(0, 1);
        MenuLCD.print(Value_to_print);
        MenuLCD.setCursor(Cursor_position, 1);
        MenuLCD.blink();
        break;
      default:
        break;
    }
  }
  MenuLCD.noBlink();
  return New_value;
}