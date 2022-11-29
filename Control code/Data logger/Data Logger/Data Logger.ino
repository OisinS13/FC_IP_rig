#include <SPI.h>
#include <SoftwareSerial.h>
#include "SdFat.h"
#include "sdios.h"
#include "FreeStack.h"
#include "RTClib.h"
#include "SerialTransfer.h"

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
RTC_PCF8523 rtc;
File32 logfile;

#define Num_CVM_Channels 28

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
bool file_ready_flag = 0;
char Error_to_file[] = ",";     //Char array for the errors to be written to the file
int Error_to_file_counter = 1;  //Counter for the length of the error to file array

struct DataFrameCVM {
  uint32_t Cell_V[28] = { 0 };  //Cell voltages in uV
  uint32_t CVM_timestamp = 0;   // timestamp in uS
};

struct DataFrameProcess {
  float CTHD_HTR_SFTY_TC[3] = { 0, 0, 0 };              //DegC
  double CTHD_FLW = 0;                                  //SLPM
  float HIH_RH[10] = { 0 };                             //Array to store converted RH data (as %)
  float HIH_T[10] = { 0 };                              //Array to store converted T data (deg C)
  uint32_t P_readings[8] = { 0, 0, 0, 0, 0 };  //Pa
  uint16_t AND_FLW_value[2] = { 0 };                    //SmLPM
  uint16_t CTHD_FLW_output = 0;                       //12 bit PWM value
  uint16_t AND_FLW_target = 0;                          //SmLPM
  uint16_t AND_recirc_PWM = 0;                          //12 bit PWM value
  uint16_t CLNT_FLW_output = 0;                         //Coolant pump 12 bit PWM value
  bool CTHD_HTR = 0;                                    //is the cathode heater on/off
  uint8_t Valves = 0;                                   //Bit flags for what valves are set. Order is anode humidifier feed, cathode humidifier feed, anode flow controller purge, anode flow controller valve, anode purge, H2 in, N2 in
  uint32_t Process_controller_timestamp = 0;            //timestamp in mS that data was sent out
};

struct DataFrameLoad {
  //EDITME put in Load controller stuff
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

#define BufferSize 4
DataFrameStruct *DataPtr[BufferSize], DataFrame[BufferSize];



void setup() {
  Serial.begin(115200);

  for (int i = 0; i < BufferSize; i++) {
    DataPtr[i] = &DataFrame[i];
  }

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
    char ID = CVM.currentPacketID();

    if (ID == 'd') {  //Data frame
      for (int i = 0; i < BufferSize; i++) {
        if (!(*DataPtr[i]).Data_source_flags & 0x01) {  //Check if the structure in position 'i' of the queue already has this data
                                                        // DataFrameStruct Temporary=DataQueue[i]; //Create a temporary structure to work on, and non-destructively read the 'i'th member onto it
          CVM.rxObj((*DataPtr[i]).CVM_data);            //Put the incoming packet into the 'i' position of the queue
          (*DataPtr[i]).Data_source_flags |= 0x01;      //Set the data read flag bit for the CVM in the relevant member of the queue
          if (i > 2) {
            (*DataPtr[i - 2]).Data_source_flags |= 0x10;  //Set the "To-be-saved" flag of the data frame 2 up in the queue //EDITME can't use circular biffer like this, just make a normal buffer array
                                                          //EDITME add stale data warning?
          }
          i = BufferSize;  //exit for loop once data has been written once
        } else {
          //EDITME throw queue overflow warning?
        }
      }
    }

    if (ID == 'f') {  //Fault message
      //EDITME finish fault reading
      CVM.rxObj(Incoming_fault);                                                                                                                                //Put the fault in a structure
      Error_to_file_counter += sprintf(&Error_to_file[Error_to_file_counter], "%u-%u-%lu;", Incoming_fault.Fault_code, Incoming_fault.Fault_detail, millis());  //Append a fault, timestamp, and then a delimeter
      FaultSend(Interface, 'f', Incoming_fault.Fault_code, Incoming_fault.Fault_detail);                                                                        //Forward fault to Interface for log
    }

    if (ID == 'R') {                                                                      //CVM manual reboot request
      CVM.rxObj(Incoming_fault);                                                          //Put the fault in a structure
      FaultSend(Interface, 'R', Incoming_fault.Fault_code, Incoming_fault.Fault_detail);  //Forward request to Interface to be manually acknowledged
    }

    if (ID == 'U') {  //Cell undervoltage error
      uint32_t UV_error_flags = 0;
      CVM.rxObj(UV_error_flags);                                                                                               //Put the fault in a structure
      Error_to_file_counter += sprintf(&Error_to_file[Error_to_file_counter], "%u-%lu-%lu;", 0xF1, UV_error_flags, millis());  //Append a fault, timestamp, and then a delimeter
      FaultSend(Interface, 'U', 0xF1, 0);                                                                                      //Forward error to other systems for relevant actions to be taken
      FaultSend(LoadController, 'U', 0xF1, 0);                                                                                 //Forward error to other systems for relevant actions to be taken
      FaultSend(SafetySystem, 'U', 0xF1, 0);                                                                                   //Forward error to other systems for relevant actions to be taken
      FaultSend(ProcessController, 'U', 0xF1, 0);                                                                              //Forward error to other systems for relevant actions to be taken
    }

    if (ID == 'u') {                                  //Cell undervoltage error clear
      CVM.rxObj(Incoming_fault);                      //Put the fault in a structure
      FaultSend(Interface, 'u', 0xF1, 0xFF);          //Forward error clear to other systems for relevant actions to be taken
      FaultSend(LoadController, 'u', 0xF1, 0xFF);     //Forward error clear to other systems for relevant actions to be taken
      FaultSend(SafetySystem, 'u', 0xF1, 0xFF);       //Forward error clear to other systems for relevant actions to be taken
      FaultSend(ProcessController, 'u', 0xF1, 0xFF);  //Forward error clear to other systems for relevant actions to be taken
    }

    if (ID == 't') {                                                                                                                                            //Fault message
      CVM.rxObj(Incoming_fault);                                                                                                                                //Put the fault in a structure
      Error_to_file_counter += sprintf(&Error_to_file[Error_to_file_counter], "%u-%u-%lu;", Incoming_fault.Fault_code, Incoming_fault.Fault_detail, millis());  //Append a fault, timestamp, and then a delimeter
      FaultSend(Interface, 'f', Incoming_fault.Fault_code, Incoming_fault.Fault_detail);                                                                        //Forward fault to Interface for log
    }


    //EDITME put in other messages here
  }

  if (ProcessController.available()) {
    char ID = ProcessController.currentPacketID();

    if (ID == 'd') {  //Data frame
      for (int i = 0; i < BufferSize; i++) {
        if (!(*DataPtr[i]).Data_source_flags & 0x08) {          //Check if the structure in position 'i' of the queue already has this data
                                                                // DataFrameStruct Temporary=DataQueue[i]; //Create a temporary structure to work on, and non-destructively read the 'i'th member onto it
          ProcessController.rxObj((*DataPtr[i]).Process_data);  //Put the incoming packet into the 'i' position of the queue
          (*DataPtr[i]).Data_source_flags |= 0x08;              //Set the data read flag bit for the Process controller in the relevant member of the queue
          if (i > 2) {
            (*DataPtr[i - 2]).Data_source_flags |= 0x10;  //Set the "To-be-saved" flag of the data frame 2 up in the queue //EDITME can't use circular biffer like this, just make a normal buffer array
                                                          //EDITME add stale data warning?
          }
          i = BufferSize;  //exit for loop once data has been written once
        } else {
          //EDITME throw queue overflow warning?
        }
      }
    }

    if (ID == 'f') {  //Fault message
      //EDITME finish fault reading
      ProcessController.rxObj(Incoming_fault);                                                                                                                  //Put the fault in a structure
      Error_to_file_counter += sprintf(&Error_to_file[Error_to_file_counter], "%u-%u-%lu;", Incoming_fault.Fault_code, Incoming_fault.Fault_detail, millis());  //Append a fault, timestamp, and then a delimeter
    }

    if (ID == 'T') {                                                                                                                                                     //Temperature out of bounds
      ProcessController.rxObj(Incoming_fault);                                                                                                                           //Put the fault in a structure
      Error_to_file_counter += sprintf(&Error_to_file[Error_to_file_counter], "%u-%u-%u-%lu;", 0xF2, Incoming_fault.Fault_code, Incoming_fault.Fault_detail, millis());  //Append a fault, timestamp, and then a delimeter
      //EDITME add FMEA reactions to extreme scenarios
    }

    if (ID == 'F') {                                                                                                                                                     //Temperature out of bounds
      ProcessController.rxObj(Incoming_fault);                                                                                                                           //Put the fault in a structure
      Error_to_file_counter += sprintf(&Error_to_file[Error_to_file_counter], "%u-%u-%u-%lu;", 0xF3, Incoming_fault.Fault_code, Incoming_fault.Fault_detail, millis());  //Append a fault, timestamp, and then a delimeter
      //EDITME add FMEA reactions to extreme scenarios
    }

    if (ID == 'P') {                                                                                                                                                     //Temperature out of bounds
      ProcessController.rxObj(Incoming_fault);                                                                                                                           //Put the fault in a structure
      Error_to_file_counter += sprintf(&Error_to_file[Error_to_file_counter], "%u-%u-%u-%lu;", 0xF4, Incoming_fault.Fault_code, Incoming_fault.Fault_detail, millis());  //Append a fault, timestamp, and then a delimeter
      //EDITME add FMEA reactions to extreme scenarios
    }
    //EDITME put in other messages here
  }

  if (LoadController.available()) {
    char ID = LoadController.currentPacketID();

    if (ID == 'd') {  //Data frame
      for (int i = 0; i < BufferSize; i++) {
        if (!(*DataPtr[i]).Data_source_flags & 0x02) {    //Check if the structure in position 'i' of the queue already has this data
                                                          // DataFrameStruct Temporary=DataQueue[i]; //Create a temporary structure to work on, and non-destructively read the 'i'th member onto it
          LoadController.rxObj((*DataPtr[i]).Load_data);  //Put the incoming packet into the 'i' position of the queue
          (*DataPtr[i]).Data_source_flags |= 0x02;        //Set the data read flag bit for the Load controller in the relevant member of the queue
          if (i > 2) {
            (*DataPtr[i - 2]).Data_source_flags |= 0x10;  //Set the "To-be-saved" flag of the data frame 2 up in the queue //EDITME can't use circular biffer like this, just make a normal buffer array
                                                          //EDITME add stale data warning?
          }
          i = BufferSize;  //exit for loop once data has been written once
        } else {
          //EDITME throw queue overflow warning?
        }
      }
    }

    if (ID == 'f') {  //Fault message
      //EDITME finish fault reading
      LoadController.rxObj(Incoming_fault);                                                                                                                     //Put the fault in a structure
      Error_to_file_counter += sprintf(&Error_to_file[Error_to_file_counter], "%u-%u-%lu;", Incoming_fault.Fault_code, Incoming_fault.Fault_detail, millis());  //Append a fault, timestamp, and then a delimeter
    }

    //EDITME put in other messages here
  }

  if (SafetySystem.available()) {
    char ID = SafetySystem.currentPacketID();

    if (ID == 'd') {  //Data frame
      for (int i = 0; i < BufferSize; i++) {
        if (!(*DataPtr[i]).Data_source_flags & 0x04) {    //Check if the structure in position 'i' of the queue already has this data
                                                          // DataFrameStruct Temporary=DataQueue[i]; //Create a temporary structure to work on, and non-destructively read the 'i'th member onto it
          SafetySystem.rxObj((*DataPtr[i]).Safety_data);  //Put the incoming packet into the 'i' position of the queue
          (*DataPtr[i]).Data_source_flags |= 0x04;        //Set the data read flag bit for the Load controller in the relevant member of the queue
          if (i > 2) {
            (*DataPtr[i - 2]).Data_source_flags |= 0x10;  //Set the "To-be-saved" flag of the data frame 2 up in the queue //EDITME can't use circular biffer like this, just make a normal buffer array
                                                          //EDITME add stale data warning?
          }
          i = BufferSize;  //exit for loop once data has been written once
        } else {
          //EDITME throw queue overflow warning?
        }
      }
    }

    if (ID == 'f') {  //Fault message
      //EDITME finish fault reading
      SafetySystem.rxObj(Incoming_fault);                                                                                                                       //Put the fault in a structure
      Error_to_file_counter += sprintf(&Error_to_file[Error_to_file_counter], "%u-%u-%lu;", Incoming_fault.Fault_code, Incoming_fault.Fault_detail, millis());  //Append a fault, timestamp, and then a delimeter
    }

    //EDITME put in other messages here
  }

  if ((*DataPtr[0]).Data_source_flags == 0b1111) {  //if all of the data read flag bits are set, all the data for this frame has been read from sources, so set "To-be-saved" flag
    (*DataPtr[0]).Data_source_flags |= 0x10;
  }

  if ((*DataPtr[0]).Data_source_flags & 0x10) {  //If the "To-be-saved" bit is set

    char Data_to_file[] = "\n";  //Initialise char array, beginning with a new line character

    int j = 1;  //starts at 1 to account for newline chracter

    //Append CVM data
    for (int i = 0; i < Num_CVM_Channels; i++) {
      j += sprintf(&Data_to_file[j], "%lu,", (*DataPtr[0]).CVM_data.Cell_V[i]);  //Append a reading, and then a delimeter
    }
    j += sprintf(&Data_to_file[j], "%lu,", (*DataPtr[0]).CVM_data.CVM_timestamp);  //Append a timestamp, and then a delimeter

    //Append process controller data
    for (int i = 0; i < 3; i++) {
      j += sprintf(&Data_to_file[j], "%f,", (*DataPtr[0]).Process_data.CTHD_HTR_SFTY_TC[i]);  //Append a reading, and then a delimeter
    }
    j += sprintf(&Data_to_file[j], "%f,", (*DataPtr[0]).Process_data.CTHD_FLW);  //Append a reading, and then a delimeter
    for (int i = 0; i < 10; i++) {
      j += sprintf(&Data_to_file[j], "%f,", (*DataPtr[0]).Process_data.HIH_RH[i]);  //Append a reading, and then a delimeter
    }
    for (int i = 0; i < 10; i++) {
      j += sprintf(&Data_to_file[j], "%f,", (*DataPtr[0]).Process_data.HIH_T[i]);  //Append a reading, and then a delimeter
    }
    for (int i = 0; i < 8; i++) {
      j += sprintf(&Data_to_file[j], "%lu,", (*DataPtr[0]).Process_data.P_readings[i]);  //Append a reading, and then a delimeter
    }
    for (int i = 0; i < 2; i++) {
      j += sprintf(&Data_to_file[j], "%u,", (*DataPtr[0]).Process_data.AND_FLW_value[i]);  //Append a reading, and then a delimeter
    }
    j += sprintf(&Data_to_file[j], "%u,", (*DataPtr[0]).Process_data.CTHD_VLV_setpoint);              //Append a reading, and then a delimeter
    j += sprintf(&Data_to_file[j], "%u,", (*DataPtr[0]).Process_data.AND_FLW_target);                 //Append a reading, and then a delimeter
    j += sprintf(&Data_to_file[j], "%u,", (*DataPtr[0]).Process_data.AND_recirc_PWM);                 //Append a reading, and then a delimeter
    j += sprintf(&Data_to_file[j], "%u,", (*DataPtr[0]).Process_data.CLNT_FLW_output);                //Append a reading, and then a delimeter
    j += sprintf(&Data_to_file[j], "%u,", (*DataPtr[0]).Process_data.CTHD_HTR);                       //Append a reading, and then a delimeter
    j += sprintf(&Data_to_file[j], "%u,", (*DataPtr[0]).Process_data.Valves);                         //Append a reading, and then a delimeter
    j += sprintf(&Data_to_file[j], "%lu,", (*DataPtr[0]).Process_data.Process_controller_timestamp);  //Append a reading, and then a delimeter

    //Append Load controller data
    j += sprintf(&Data_to_file[j], "%lu,", (*DataPtr[0]).Load_data.I_ref);       //Append a reading, and then a delimeter
    j += sprintf(&Data_to_file[j], "%lu,", (*DataPtr[0]).Load_data.V_ref);       //Append a reading, and then a delimeter
    j += sprintf(&Data_to_file[j], "%lu,", (*DataPtr[0]).Load_data.P_ref);       //Append a reading, and then a delimeter
    j += sprintf(&Data_to_file[j], "%lu,", (*DataPtr[0]).Load_data.V_fc);        //Append a reading, and then a delimeter
    j += sprintf(&Data_to_file[j], "%lu,", (*DataPtr[0]).Load_data.V_fc_spike);  //Append a reading, and then a delimeter
    j += sprintf(&Data_to_file[j], "%lu,", (*DataPtr[0]).Load_data.I_monitor);   //Append a reading, and then a delimeter
    j += sprintf(&Data_to_file[j], "%lu,", (*DataPtr[0]).Load_data.I_clamp);     //Append a reading, and then a delimeter
    j += sprintf(&Data_to_file[j], "%lu,", (*DataPtr[0]).Load_data.time_stamp);  //Append a reading, and then a delimeter
    j += sprintf(&Data_to_file[j], "%u,", (*DataPtr[0]).Load_data.op_mode);      //Append a reading, and then a delimeter
    j += sprintf(&Data_to_file[j], "%u,", (*DataPtr[0]).Load_data.alarm_1);      //Append a reading, and then a delimeter
    j += sprintf(&Data_to_file[j], "%u,", (*DataPtr[0]).Load_data.alarm_2);      //Append a reading, and then a delimeter
    j += sprintf(&Data_to_file[j], "%u,", (*DataPtr[0]).Load_data.alarm_ext);    //Append a reading, and then a delimeter
    j += sprintf(&Data_to_file[j], "%u,", (*DataPtr[0]).Load_data.load_status);  //Append a reading, and then a delimeter


    //EDITME add safety controller data append

    logfile.write(&Data_to_file, j);                       //Write the whole data string to the file
    logfile.sync();                                        //Save data to disc
    logfile.write(&Error_to_file, Error_to_file_counter);  //Write the whole error string to the file
    logfile.sync();                                        //Save data to disc
    memset(Error_to_file, 0, Error_to_file_counter);
    Error_to_file[0] = ',';
    Error_to_file_counter = 1;
    // DataQueue.shift();
    DataFrameStruct Temporary;
    (*DataPtr[0]) = Temporary;
    for (int i = 0; i < (BufferSize - 1); i++) {
      DataPtr[i] = DataPtr[i + 1];
    }
    DataPtr[BufferSize] = DataPtr[0];
  }
}

void loop1() {
}
