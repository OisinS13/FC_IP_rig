#include <SPI.h>
#include "SdFat.h"
#include "sdios.h"
#include "FreeStack.h"
#include "RTClib.h"
#include "SerialTransfer.h"
SerialTransfer DataLogger;
SerialTransfer LoadController;
RTC_PCF8523 rtc;
File32 logfile;

const uint8_t SD_CS_PIN = 5;
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, DEDICATED_SPI, SPI_CLOCK)
#define SPI_CLOCK SD_SCK_MHZ(50)
SdFat32 SD;



#define In_flag_pin 19
#define Out_flag_pin 18

unsigned int TimeOut = 10000;  //Time in mS until a time out is triggered

const unsigned int Num_samples = 500;  //Number of samples per buffer
const unsigned int Num_channels = 28;  //Number of channels to read (Here, it is 24 cells, plus measurements at cells 6,12,18 and 24)
const unsigned int Num_buffs = 8;      //Number of data buffers
const uint8_t CS_pins[Num_channels / 4] = { 19, 20, 9, 10, 16, 17, 26 };
uint16_t V_in[Num_buffs][Num_samples][Num_channels + 3] = { 0 };  //Initialise array to store data, including 4 bytes of timestamp and 0xFFFF byte to denote line ending in HEX file
bool V_in_flag[Num_buffs] = { 0, 0, 0, 0, 0, 0, 0, 0 };           //Array of flags which indicate if a buffer has been written to (1) or saved to file (0)
unsigned int V_write_counter = 0;                                 //Counter to indicate which buffer should be written to file next
bool file_ready_flag = 0;
uint32_t UV_flags = 0;           //Cell undervoltage flags
uint32_t UV_threshold = 400000;  //Cell undervoltage threshold in uV

uint16_t V_in_UART[Num_channels] = { 0 };                                                                                                                                                           //Initialise array to for low frequency acquisition including 4 bytes of timestamp
uint32_t V_out_UART[Num_channels];                                                                                                                                                                  //Initialise uint32_t array to send uV data over UART, and save to SD
uint32_t StartTimefast;                                                                                                                                                                             //Timer start for high frequency acquisition
uint32_t StartTimeslow;                                                                                                                                                                             //Timer start for low frequency acquisition
uint16_t PD_R1_values[Num_channels] = { 0, 1000, 3000, 8060, 0, 1000, 3000, 8060, 0, 1000, 3000, 8060, 0, 1000, 3000, 8060, 0, 1000, 3000, 8060, 0, 1000, 3000, 8060, 8870, 21500, 33000, 45300 };  //R1 values for the potential dividers dropping the stacked voltages to below the ADC Vref values
                                                                                                                                                                                                    //R2 values are all 3k

bool Core0_boot_flag = 0;  //Flag to tell Core1 that Core0 has successfully booted
bool Core1_boot_flag = 0;  //Flag to tell Core0 that Core1 has successfully booted
bool USB_flag = 0;         //Flag to indicate if USB communications are active
bool SD_boot_flag = 0;     //Flag to indicate SD successfully booted
bool RTC_flag = 0;         //Flag to indicate RTC successfully booted
//EDITME consider reassigning these flags to bits in a single byte
//EDITME Write reboot over UART function using rp2040.restartCore1()

char Filenamefast[27];  //Char array to determine filename
char Filenameslow[31];  //Char array to determine filename
int ReadFreqfast = 5000;
int intervalfast = 1000000 / ReadFreqfast;  //interval in uS

int ReadFreqslow = 10;
int intervalslow = 1000 / ReadFreqslow;  //interval in mS

uint8_t Burst_read_flags = 0b00000000;  //Flags are Software (UART) request, Software Ack, Hardware In, Hardware Out, Null, Null, TimeOut(Hardware In), TimeOut(Data acquisition)
unsigned long Flag_time_Hardware_in = 0;

// bool stringComplete1 = 0;  //Flag to show incoming UART0 message is complete
// String Command1 = "";
// bool stringComplete2 = 0;  //Flag to show incoming UART1 message is complete
// String Command2 = "";

struct DATA {  //Structure to send out CVM data
  char CMD = d;
  uint32_t V[28] = 0;
  uint32_t Timestamp = 0;
} data_struct;

struct Fault_message {
  uint8_t Fault_code = 0;
  uint8_t Fault_detail = 0;
};
struct Fault_message Incoming_fault;

void setup() {
  Serial.begin(115200);
  // while (!Serial) {
  //   ;  // wait for serial port to connect. Needed for native USB
  // }

  Serial1.setRX(2);  //Set UART pins for UART0, to be used for Data logger connection
  Serial1.setTX(1);
  Serial1.setFIFOSize(128);  //May be required for stable UART comms
  Serial1.setPollingMode(true);     //Ensure UART0 port is listening for messages
  Serial1.begin(115200);            //Initialise UART0 port
  DataLogger.begin(Serial1);        //Initialise Serial Transfer object for Data logger connection

  Serial2.setRX(12);  //Set UART pins for UART1, to be used for Load controller connection
  Serial2.setTX(11);
  // Serial2.setFIFOSize(128); //May be required for stable UART comms
  Serial2.setPollingMode(true);   //Ensure UART1 port is listening for messages
  Serial2.begin(115200);          //Initialise UART1 port
  LoadController.begin(Serial2);  //Initialise Serial Transfer object for Load controller connection

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
  if (USB_flag) {
    Serial.println("Cell Voltage monitor booting");
  }

  SPI1.setRX(12);  //Set the correct SPI pins for the ADC SPI
  SPI1.setCS(13);  //Dummy CS pin, do not use for anything else
  SPI1.setSCK(10);
  SPI1.setTX(11);

  for (int i = 0; i < sizeof(CS_pins); i++) {
    pinMode(CS_pins[i], OUTPUT_12MA);  //Set the CS pins to high current outputs
    digitalWrite(CS_pins[i], HIGH);    // Set CS pins high while ADC's not being read
  }

  pinMode(In_flag_pin, INPUT);  //EDITME may need use of an internal pullup/pulldown
  pinMode(Out_flag_pin, OUTPUT_12MA);

  SPI1.begin(0);                                                      //Initialise ADC SPI port
  SPI1.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));  //Set the ADC SPI settings
  SPI1.endTransaction();

  while (!ID_check(0x81, CS_pins)) {}  //Check that the ADC's all read the correct address to determine if SPI lines are functional. ID of chip should be 0x81 at register 0
  //EDITME write error throwing code


  for (int i = 0; i < Num_buffs; i++) {
    for (int j = 0; j < Num_samples; j++) {
      V_in[i][j][Num_channels + 2] = 0xFFFF;  //Set line ending byte in data buffers once at beginning for speed
    }
  }
  if (USB_flag) {
    Serial.print("Data buffers formatted");
  }

  SPI.setSCK(2);  //Set the correct SPI pins for the SD SPI
  SPI.setTX(3);
  SPI.setRX(4);
  SPI.setCS(SD_CS_PIN);



  if (USB_flag) {
    Serial.print("Initializing SD card...");
  }

  if (!SD.begin(SD_CONFIG)) {
    FaultSend(DataLogger, 'f', 0x61, 0);  //Send fault for SD initialisation fault
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
  //FaultSend(DataLogger, 'R', 0, 0);  //Send reboot request

  DateTime boot_time = rtc.now();

  while (!file_ready_flag) {  //Open a write file to test writing
    file_ready_flag = Create_logfile(boot_time, &Filenamefast[0], 1);
  }
  Fill_data_buf();                       //Fill all data buffers once
  for (int i = 0; i < Num_buffs; i++) {  //Write all data buffers to test write file
    Write_buf_to_SD();
  }
  logfile.close();  //Close the test write file
  file_ready_flag = 0;
  for (byte i = 0; i < sizeof(Filenamefast) - 1; i++) {
    Filenamefast[i] = 0;  //Clear out the Filenamefast char array
  }

  if (!Create_logfile(boot_time, &Filenameslow[0], 0)) {
    ;  //EDITME Add error throwing code?
  }
  logfile.write("Cell 1(uV), Cell 2(uV), Cell 3(uV), Cell 4(uV), Cell 5(uV), Cell 6(uV), Cell 7(uV), Cell 8(uV), Cell 9(uV), Cell 10(uV), Cell 11(uV), Cell 12(uV), Cell 13(uV), Cell 14(uV), Cell 15(uV), Cell 16(uV), Cell 17(uV), Cell 18(uV), Cell 19(uV), Cell 20(uV), Cell 21(uV), Cell 22(uV), Cell 23(uV), Cell 24(uV), Stack 6(uV), Stack 12(uV), Stack 18(uV), Stack 24(uV), Time(uS)");
  logfile.close();  //close long term log file

  Core1_boot_flag = 1;
}

void loop() {
 
  if (V_in_flag[V_write_counter]) {  //Check if there is a full data buffer to be written
    Write_buf_to_SD();               //Write full data buffer to SD. For speed reasons, this is a priority
  } else {                           //Other code goes in here so that write is never delayed

    if (DataLogger.available()) {
      char ID = DataLogger.currentPacketID;

      if (ID == 'r') {  //Command to reboot core 1
        //EDITME write reboot code
      }
    }

    if (LoadController.available()) {
      char ID = LoadController.currentPacketID;

      //EDITME write High frequency voltage monitoring recieve code
    }

    //EDITME add all messages

    // while (Serial1.available()) {          //Detect incoming UART messages from Data logger
    //   char inChar = (char)Serial1.read();  //Put incoming data into a char byte
    //   if (inChar == '!') {                 //Check if the command string is complete by looking for end character
    //     stringComplete1 = true;            //raise flag to show command is complete
    //   } else Command1 += inChar;           //put incoming byte on end of command string
    //   if (stringComplete1) {               //check if incoming command is complete
    //     //EDITME Code triggered by UART0 (Data logger) commands goes in here
    //     Command1 = "";  //clear command string
    //   }
    //   stringComplete1 = false;
    // }
    // while (Serial2.available()) {          //Detect incoming UART messages from Data logger
    //   char inChar = (char)Serial2.read();  //Put incoming data into a char byte
    //   if (inChar == '!') {                 //Check if the command string is complete by looking for end character
    //     stringComplete2 = true;            //raise flag to show command is complete
    //   } else Command2 += inChar;           //put incoming byte on end of command string
    //   if (stringComplete2) {               //check if incoming command is complete
    //     //EDITME Code triggered by UART1 (Power controller) commands goes in here
    //     if (Command2 == "Begin HF CVM") {  //Command to start high frequency read mode
    //       Burst_read_flags |= 0x80;        //Set the flag requesting high frequency data acquisition
    //     }
    //     Command2 = "";  //clear command string
    //   }
    //   stringComplete2 = false;
    // }
    //EDITME write code to deal with extra characters that might be generated on boot?
  }
}

void loop1() {
  if ((Burst_read_flags & 0x80)) {     //If high frequency data acquisition has been requested over UART
    if (!(Burst_read_flags & 0x40)) {  //Check if acknowledgement has been sent out over UART

      DateTime log_time = rtc.now();  //Get the time to name the file
      while (!file_ready_flag) {      //Open a write file for high frequency data acquisition
        file_ready_flag = Create_logfile(log_time, &Filenamefast[0], 1);
      }
      Serial2.write("HF CVM go!");       //Send acknowledgment to Power controller that high frequency acquisiition is prepped
      Burst_read_flags |= 0x40;          //Set UART Out flag to 1
      Flag_time_Hardware_in = millis();  //Set TimeOut timer start for hardware In
    }

    if (millis() - Flag_time_Hardware_in > TimeOut) { Burst_read_flags |= 0x2; }  //If it has been longer than the Time Out interval, set the TimeOut bit to 1
    Burst_read_flags |= digitalRead(In_flag_pin) << 5;                            //Read the hardware flag

    if ((Burst_read_flags & 0x20)) {                //Look for hardware flag to trigger
      Burst_read_flags |= 0x10;                     //Set Hardware Out flag to 1
      Fill_data_buf();                              //Fill all of the data buffers once, so that we have some lead in data
      digitalWrite(Out_flag_pin, HIGH);             //Signal that high frequency data acquisition is occuring
      unsigned long Flag_time_data_acq = millis();  //Set TimeOut timer start

      while ((Burst_read_flags & 0x20) && !(Burst_read_flags & 0x1)) {             //Wait for Hardware flag to drop, signalling end of high frequency acquisition, or for timeout to occur
        if (millis() - Flag_time_data_acq > TimeOut) { Burst_read_flags |= 0x1; }  //If it has been longer than the Time Out interval, set the TimeOut bit to 1
        Burst_read_flags ^= (!(digitalRead(In_flag_pin) << 5) & 0x20);             //Read the hardware in flag, and write it to the relevant flag bit
        Fill_data_buf();
      }
      Burst_read_flags &= ~0x10;        //Reset the Hardware out bit to 0
      digitalWrite(Out_flag_pin, LOW);  //Signal that high frequency data acquisition has stopped

      if ((Burst_read_flags & 0x1)) {          //If the TimeOut bit is 1
                                               // Burst_read_flags &= ~0x1;   //Reset the TimeOut bit to 0
        FaultSend(DataLogger, 't', 2, 0);      //Tell Data Logger that the reading time has timedout
        FaultSend(LoadController, 't', 2, 0);  //Tell Load controller that the reading time has timedout
      }
      Burst_read_flags = 0b00000000;  // Reset all flags
      logfile.close();                //Close logfile
      file_ready_flag = 0;
      for (byte i = 0; i < sizeof(Filenamefast) - 1; i++) {
        Filenamefast[i] = 0;  //Clear out the Filenamefast char array
      }
    } else if ((Burst_read_flags & 0x2)) {   //Look for TimeOut trigger
                                             // Burst_read_flags &= ~0x2;         //Reset the TimeOut bit to 0
      FaultSend(DataLogger, 't', 1, 0);      //Tell Data Logger that the request has timedout
      FaultSend(LoadController, 't', 1, 0);  //Tell Load controller that the request has timedout
      Burst_read_flags = 0b00000000;         // Reset all flags
      logfile.close();                       //Close logfile
      file_ready_flag = 0;
      for (byte i = 0; i < sizeof(Filenamefast) - 1; i++) {
        Filenamefast[i] = 0;  //Clear out the Filenamefast char array
      }
    }
  } else {
    //EDITME put any other Core1 code here, so high speed data acquisition is always prioritised
    if (micros() - StartTimeslow > intervalslow) {  //Read data at slow acquisition rate, and send it over UART
      StartTimeslow = micros();
      for (int l = 0; l < (Num_channels / 4); l++) {
        digitalWrite(CS_pins[l], LOW);
        SPI1.transfer(0x5);  //Burst read of non moving average data
        V_in_UART[l * 4] = SPI1.transfer16(0x00);
        V_in_UART[(l * 4) + 1] = SPI1.transfer16(0x00);
        V_in_UART[(l * 4) + 2] = SPI1.transfer16(0x00);
        V_in_UART[(l * 4) + 3] = SPI1.transfer16(0x00);
        digitalWrite(CS_pins[l], HIGH);
      }
      //EDITME write code to throw cell undervoltage errors?
      for (int i = 0; i < Num_channels; i++) {
        //Total equation here is (V_in_UART[i] * 1.8) / (4096.0) * ((PD_R1_values[i] + 3000.0) / 3000.0)*1000000;
        data_struct.V[i] = (V_in_UART[i] * 18 * (PD_R1_values[i] + 3000) * 1000) / (4096 * 3 * 10);  //Calculate voltages (in uV) as uint32_t using R values from board pot divs
        if (data_struct.V[i] < UV_threshold) {
          UV_flags |= 1 << i;  //Set relevant under voltage flag
        }
      }
      data_struct.Timestamp = StartTimeslow;


      if (UV_flags) {  //If an undervoltage flag has been tripped
        DataLogger.txObj(UV_flags);
        DataLogger.sendData(4, 'U');  //Send UV_flags with UV command code
        UV_flags = 0;                 //Reset UV_flags
      }

      uint16_t n_byt = 0;
      n_byt = DataLogger.txObj(data_struct, n_byt);
      DataLogger.sendData(n_byt, 'd');  //send data with command code //EDITME check buffer size


      // Serial1.write("Data:");  //Inform Data logger that this is a data command
      // for (int i = 0; i < Num_channels; i++) {
      //   Serial1.write((byte *)&V_out_UART[i], 4);  //Send Voltages as uint32_t uV values
      //   Serial1.write(",");                        //send delimeter
      //   // Serial1.write(&V_in_UART, ((Num_channels + 2) * 2));  //Send uint16_t data out as binary bytes
      // }
      // Serial1.write((byte *)&StartTimeslow, 4);  //Send uS timestamp
      // Serial1.write("!");

      char Data_to_file[] = "\n";  //Initialise char array, beginning with a new line character
      if (!logfile.open(Filenameslow, O_RDWR | O_APPEND)) {
        ;
      }           //EDITME Write error throwing code
      int j = 1;  //starts at 1 to account for newline chracter
      for (int i = 0; i < Num_channels; i++) {
        j += sprintf(&Data_to_file[j], "%lu,", V_out_UART[i]);  //Append a reading, and then a delimeter
        // logfile.write(&V_out_UART[i], 4);  //write Voltages as uint32_t uV values
        // logfile.write(",");                //write delimeter
      }
      j += sprintf(&Data_to_file[j], "%lu,", StartTimeslow);  //Append timestamp, then a delimeter (in case error codes need to be added)
      // Serial1.write(StartTimeslow,4); //write timestamp in uS
      // Serial1.write(",");//write delimeter so error codes can be appended
      logfile.write(&Data_to_file, j);  //Write the whole string to the file
      logfile.sync();                   //Save data to disc
      logfile.close();                  //Close logfile
      //EDITME Write code to save errors to logfile at end of data frame
    }
  }
}