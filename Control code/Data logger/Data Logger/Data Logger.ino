#include <SPI.h>
#include <SoftwareSerial.h>
#include "SdFat.h"
#include "sdios.h"
#include "FreeStack.h"
#include "RTClib.h"
#include "SerialTransfer.h"
#include <CircularBuffer.h>

SerialPIO UART2(10, 11, 64);
SerialPIO UART3(14, 15, 32);
SerialPIO UART4(19, 20, 32);

SerialTransfer CVM;
SerialTransfer Interface;
SerialTransfer LoadController;
SerialTransfer SafetySystem;
SerialTransfer ProcessController;

const uint8_t SD_CS_PIN = 5;  //EDITME check CS pin
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, DEDICATED_SPI, SPI_CLOCK)
#define SPI_CLOCK SD_SCK_MHZ(50)
SdFat32 SD;

char Filename[27];  //Char array to determine filename

struct Fault_message {
  uint8_t Fault_code = 0;
  uint8_t Fault_detail = 0;
};
struct Fault_message Incoming_fault;

bool Core0_boot_flag = 0;  //Flag to tell Core1 that Core0 has successfully booted
bool Core1_boot_flag = 0;  //Flag to tell Core0 that Core1 has successfully booted
bool USB_flag = 0;         //Flag to indicate if USB communications are active
bool SD_boot_flag = 0;     //Flag to indicate SD successfully booted
bool RTC_flag = 0;         //Flag to indicate RTC successfully booted

struct DataFrameCVM {
  uint32_t Cell_V[28] = 0;     //Cell voltages in uV
  uint32_t CVM_timestamp = 0;  // timestamp in uS
};

struct DataFrameProcess {
  float CTHD_HTR_SFTY_TC[3] = { 0, 0, 0 };              //DegC
  double CTHD_FLW = 0;                                  //SLPM
  float HIH_RH[10] = { 0 };                             //Array to store converted RH data (as %)
  float HIH_T[10] = { 0 };                              //Array to store converted T data (deg C)
  uint32_t P_readings[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };  //Pa
  uint16_t AND_FLW_value[2] = { 0 };                    //SmLPM
  uint16_t CTHD_VLV_setpoint = 0;                       //12 bit PWM value
  uint16_t AND_FLW_target = 0;                          //SmLPM
  uint16_t AND_recirc_PWM = 0;                          //12 bit PWM value
  uint16_t CLNT_FLW_output = 0;                         //Coolant pump 12 bit PWM value
  bool CTHD_HTR = 0;                                    //is the cathode heater on/off
  uint8_t Valves = 0;                                   //Bit flags for what valves are set. Order is anode humidifier feed, cathode humidifier feed, anode flow controller purge, anode flow controller valve, anode purge, H2 in, N2 in
  uint32_t Process_controller_timestamp = 0;            //timestamp in mS that data was sent out
};

struct DataFrameLoad {
  //EDITME put in Load controller stuff
};

struct DataFrameSafety {
  //EDITME put in safety system stuff
};

struct DataFrameStruct {
  uint8_t Data_source_flags;      //Flags are "To-be-saved" flag, Process controller, Safety system, Load controller, CVM
  DataFrameCVM CVM_data;          //CVM data
  DataFrameProcess Process_data;  //Process controller data
  DataFrameLoad Load_data;        //Load controller data
  DataFrameSafety Safety_data;    //Safety system data
};

#define QueueLength 5
CircularBuffer<DataFrameStruct, QueueLength> DataQueue;


void setup() {
  Serial.begin(115200);

  Serial2.setRX(12);  //Set UART pins for UART1, to be used for Interface connection
  Serial2.setTX(11);
  // Serial2.setFIFOSize(size_t 128); //May be required for stable UART comms
  Serial2.setPollingMode(true);  //Ensure UART1 port is listening for messages
  Serial2.begin(115200);         //Initialise UART1 port
  Interface.begin(Serial2);      //Initialise Serial Transfer object for Interface connection

  Core0_boot_flag = 1;
  if (Serial) {
    USB_flag = 1;  //Used so that when connected via USB, verbose status and error messages can be sent, but will still run if not connected
    Serial.println("Cell Voltage monitor connected");
  } else {
    Serial.end();
  }
  while (!Core1_boot_flag) {  //Wait for Core1 to finish its setup
    delay(10);
  }
}

void setup1() {

  while (!Core0_boot_flag) {  //Wait for Core0 to finish its setup
    delay(10);
  }

  Serial1.setRX(2);  //Set UART pins for UART0, to be used for CVM connection
  Serial1.setTX(1);
  Serial1.setFIFOSize(128);      //May be required for stable UART comms
  Serial1.setPollingMode(true);  //Ensure UART0 port is listening for messages
  Serial1.begin(115200);         //Initialise UART0 port
  CVM.begin(Serial1);            //Initialise Serial Transfer object for CVM connection

  UART2.begin(115200);
  LoadController.begin(UART2);

  UART3.begin(115200);
  SafetySystem.begin(UART3);

  UART4.begin(115200);
  ProcessController.begin(UART4);

  SPI.setSCK(2);  //Set the correct SPI pins for the SD SPI
  SPI.setTX(3);
  SPI.setRX(4);
  SPI.setCS(SD_CS_PIN);
  if (USB_flag) {
    Serial.print("Initializing SD card...");
  }

  if (!SD.begin(SD_CONFIG)) {
    FaultSend(Interface, 'f', 0x71, 0);  //Send fault for SD initialisation fault
    if (USB_flag) {
      Serial.println("initialization failed!");
    }
    //EDITME write error throwing code
  } else {
    SD_boot_flag = 1;
    if (USB_flag) {
      Serial.println("initialization done.");
    }
  }

  Wire.setSDA(20);  //Set pins for RTC I2C
  Wire.setSCL(21);

  if (!rtc.begin()) {
    if (USB_flag) {
      Serial.println("Couldn't find RTC");
      Serial.flush();
    }
  } else {
    RTC_flag = 1;
  }

  //EDITME Write code to throw error and ask for manual reboot of Core1 if SD or RTC boot fails

  DateTime boot_time = rtc.now();

  while (!file_ready_flag) {  //Open a write file to test writing
    file_ready_flag = Create_logfile(boot_time, &Filename[0]);
  }
}

void loop() {
  while (!Core1_boot_flag) {}  //Check if Core1 has booted. Required in case of reboot over UART

  if (CVM.available()) {
    char ID = CVM.currentPacketID;

    if (ID == 'd') {  //Data frame
      for (int i = 0; i < QueueLength; i++) {
        if (!DataQueue[i].Data_source_flags & 0x01) {  //Check if the structure in position 'i' of the queue already has this data
          CVM.rxObj(DataQueue[i].CVM_data);            //Put the incomign packet into the 'i' position of the queue
          DataQueue[i].Data_source_flags |= 0x01;      //Set the data read flag bit for the CVM in the relevant member of the queue
          if (i > 2) {
            DataQueue[i - 2].Data_source_flags |= (1 << 4);  //Set the "To-be-saved" flag of the data frame 2 up in the queue
                                                             //EDITME add stale data warning?
          }
        }else {
          //EDITME throw queue overflow warning?
        }
      }
    }

    //EDITME put in other messages here
  }


  CVM;
  Interface;
  LoadController;
  SafetySystem;
  ProcessController