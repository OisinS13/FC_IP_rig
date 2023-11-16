#include <SPI.h>

#include "RTClib.h"
RTC_PCF8523 rtc;

#include "SdFat.h"
#include "sdios.h"
#include "FreeStack.h"
File32 logfile;

#include "SerialTransfer.h"
SerialTransfer DataLogger;
SerialTransfer LoadController;

const uint8_t SD_CS_PIN = 5;
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, DEDICATED_SPI, SPI_CLOCK)
#define SPI_CLOCK SD_SCK_MHZ(50)
SdFat32 SD;

#define Num_channels 24  //Number of channels to read (Here, it is 24 cells, plus measurements at cells 6,12,18 and 24)

#define In_flag_pin 19
#define Out_flag_pin 18
unsigned int TimeOut = 4000;  //Time in mS until a time out is triggered
#define Num_samples 500       //Number of samples per buffer
#define Num_buffs 8           //Number of data buffers
const uint8_t CS_pins[Num_channels / 4] = { 14, 15, 6, 7, 16, 17 };
uint16_t V_in[Num_buffs][Num_samples][Num_channels + 3] = { 0 };  //Initialise array to store data, including 4 bytes of timestamp and 0xFFFF byte to denote line ending in HEX file
bool V_in_flag[Num_buffs] = { 0, 0, 0, 0, 0, 0, 0, 0 };           //Array of flags which indicate if a buffer has been written to (1) or saved to file (0)
uint8_t V_write_counter = 0;                                      //Counter to indicate which buffer should be written to file next
bool file_ready_flag = 0;
uint16_t V_in_UART[Num_channels] = { 0 };

uint16_t PD_R1_values[Num_channels] = { 0, 1000, 3000, 8060, 0, 1000, 3000, 8060, 0, 1000, 3000, 8060, 0, 1000, 3000, 8060, 0, 1000, 3000, 8060, 0, 1000, 3000, 8060 };  //R1 values for the potential dividers dropping the stacked voltages to below the ADC Vref values
                                                                                                                                                                         //R2 values are all 3k

bool Core0_boot_flag = 0;  //Flag to tell Core1 that Core0 has successfully booted
bool Core1_boot_flag = 0;  //Flag to tell Core0 that Core1 has successfully booted
bool USB_flag = 0;         //Flag to indicate if USB communications are active
bool RTC_flag = 0;         //Flag to indicate RTC successfully booted
bool SD_boot_flag = 0;     //Flag to indicate SD successfully booted


char Filenamefast[27];  //Char array to determine filename
char Filenameslow[33];  //Char array to determine filename

uint8_t Burst_read_flags = 0b00000000;  //Flags are Software (UART) request, Software Ack, Hardware In, Hardware Out, Null, Null, TimeOut(Hardware In), TimeOut(Data acquisition)
unsigned long Flag_time_Hardware_in = 0;

uint32_t StartTimefast;
int ReadFreqfast = 5000;
int intervalfast = 1000000 / ReadFreqfast;  //interval in uS
uint32_t StartTimeslow;
int ReadFreqslow = 10;
int intervalslow = 1000000 / ReadFreqslow;  //interval in mS


struct DATA {  //Structure to send out CVM data
  // char CMD = d;
  uint32_t V[Num_channels];
  uint32_t Timestamp = 0;
} data_struct;


void setup() {
  USB_flag = USB_setup(115200, 1000);  //Iniitalise the USB, and set flag if present

  Serial2.setRX(9);  //Set UART pins for UART1, to be used for Load controller connection
  Serial2.setTX(8);
  // Serial2.setFIFOSize(128); //May be required for stable UART comms
  Serial2.setPollingMode(true);   //Ensure UART1 port is listening for messages
  Serial2.begin(115200);          //Initialise UART1 port
  LoadController.begin(Serial2);  //Initialise Serial Transfer object for Load controller connection

  Core0_boot_flag = 1;

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

  pinMode(In_flag_pin, INPUT);
  pinMode(Out_flag_pin, OUTPUT_12MA);
  SPI1.setRX(12);  //Set the correct SPI pins for the ADC SPI
  SPI1.setCS(13);  //Dummy CS pin, do not use for anything else
  SPI1.setSCK(10);
  SPI1.setTX(11);

  for (int i = 0; i < sizeof(CS_pins); i++) {
    pinMode(CS_pins[i], OUTPUT_12MA);  //Set the CS pins to high current outputs
    digitalWrite(CS_pins[i], HIGH);    // Set CS pins high while ADC's not being read
  }

  SPI1.begin(0);                                                      //Initialise ADC SPI port
  SPI1.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));  //Set the ADC SPI settings
  SPI1.endTransaction();

  while (!ID_check(0x81, CS_pins)) {
    // delay(200);
  }  //Check that the ADC's all read the correct address to determine if SPI lines are functional. ID of chip should be 0x81 at register 0


  RTC_setup(20, 21);
  DateTime boot_time = rtc.now();

  Initialise_SD(2, 3, 4, SD_CS_PIN);

  while (!Create_logfile(boot_time, &Filenameslow[0], 0)) {
  }
  logfile.write("Cell 1(uV), Cell 2(uV), Cell 3(uV), Cell 4(uV), Cell 5(uV), Cell 6(uV), Cell 7(uV), Cell 8(uV), Cell 9(uV), Cell 10(uV), Cell 11(uV), Cell 12(uV), Cell 13(uV), Cell 14(uV), Cell 15(uV), Cell 16(uV), Cell 17(uV), Cell 18(uV), Cell 19(uV), Cell 20(uV), Cell 21(uV), Cell 22(uV), Cell 23(uV), Cell 24(uV), Time(uS)");
  logfile.close();  //close long term log file


  Core1_boot_flag = 1;
}

void loop() {
  if (V_in_flag[V_write_counter]) {  //Check if there is a full data buffer to be written
    Write_buf_to_SD();               //Write full data buffer to SD. For speed reasons, this is a priority
  } else {                           //Other code goes in here so that write is never delayed

    // if (DataLogger.available()) {
    //   char ID = DataLogger.currentPacketID();

    //   if (ID == 'r') {  //Command to reboot core 1
    //     //EDITME write reboot code
    //   }
    // }

    if (LoadController.available()) {
      char ID = LoadController.currentPacketID();

      //EDITME write High frequency voltage monitoring recieve code
      if (ID == 'B') {
        Burst_read_flags |= 0x80;
        uint8_t temporary = 0;
        LoadController.rxObj(temporary);  //EDITME throw error if wrong number sent?
      }
    }
    if (!USB_flag) {
      USB_flag = USB_setup(115200, 0);  //Iniitalise the USB, and set flag if present
    }
  }
}


void loop1() {
  if ((Burst_read_flags & 0x80)) {     //If high frequency data acquisition has been requested over UART
    if (!(Burst_read_flags & 0x40)) {  //Check if acknowledgement has been sent out over UART

      DateTime log_time = rtc.now();  //Get the time to name the file
      while (!file_ready_flag) {      //Open a write file for high frequency data acquisition
        file_ready_flag = Create_logfile(log_time, &Filenamefast[0], 1);
      }
      LoadController.txObj(1);
      LoadController.sendData(1, 'B');
      if (USB_flag) {
        Serial.println("Burst read acknowledge");
      }
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
        // Burst_read_flags ^= ((digitalRead(In_flag_pin) << 5) & 0x20);             //Read the hardware in flag, and write it to the relevant flag bit
        if (!(digitalRead(In_flag_pin))) {
          Burst_read_flags &= ~0x20;
        }
        Fill_data_buf();
      }
      Burst_read_flags &= ~0x10;        //Reset the Hardware out bit to 0
      digitalWrite(Out_flag_pin, LOW);  //Signal that high frequency data acquisition has stopped

      if ((Burst_read_flags & 0x1)) {  //If the TimeOut bit is 1
                                       // FaultSend(DataLogger, 't', 2, 0);      //Tell Data Logger that the reading time has timedout
                                       // FaultSend(LoadController, 't', 2, 0);  //Tell Load controller that the reading time has timedout
        if (USB_flag) {
          Serial.println("Burst read data acquisition timeout");
        }
      }
      Burst_read_flags = 0b00000000;  // Reset all flags
      logfile.close();                //Close logfile
      file_ready_flag = 0;
      for (byte i = 0; i < sizeof(Filenamefast) - 1; i++) {
        Filenamefast[i] = 0;  //Clear out the Filenamefast char array
      }
    } else if ((Burst_read_flags & 0x2)) {  //Look for TimeOut trigger
                                            // FaultSend(DataLogger, 't', 1, 0);      //Tell Data Logger that the request has timedout
                                            // FaultSend(LoadController, 't', 1, 0);  //Tell Load controller that the request has timedout
      if (USB_flag) {
        Serial.println("Burst read request timeout");
      }
      Burst_read_flags = 0b00000000;  // Reset all flags
      logfile.close();                //Close logfile
      file_ready_flag = 0;
      for (byte i = 0; i < sizeof(Filenamefast) - 1; i++) {
        Filenamefast[i] = 0;  //Clear out the Filenamefast char array
      }
    }
  } else {
    if (micros() - StartTimeslow > intervalslow) {  //Read data at slow acquisition rate, and send it over UART
      StartTimeslow = micros();
      for (int l = 0; l < (Num_channels / 4); l++) {
        digitalWrite(CS_pins[l], LOW);
        SPI1.transfer(0x5);  //Burst read of non moving average data
        V_in_UART[l * 4] = SPI1.transfer16(0x00);
        V_in_UART[(l * 4) + 1] = SPI1.transfer16(0x00);
        V_in_UART[(l * 4) + 2] = SPI1.transfer16(0x00);
        V_in_UART[(l * 4) + 3] = SPI1.transfer16(0x00) - V_in_UART[(l * 4) + 2];
        V_in_UART[(l * 4) + 2] -= V_in_UART[(l * 4) + 1];
        V_in_UART[(l * 4) + 1] -= V_in_UART[l * 4];
        digitalWrite(CS_pins[l], HIGH);
      }

      for (int i = 0; i < Num_channels; i++) {
        //Total equation here is (V_in_UART[i] * 1.8) / (4096.0) * ((PD_R1_values[i] + 3000.0) / 3000.0)*1000000;
        data_struct.V[i] = (V_in_UART[i] * 18.0 * (PD_R1_values[i] + 3000.0) * 1000.0) / (4096.0 * 3.0 * 10.0);  //Calculate voltages (in uV) as uint32_t using R values from board pot divs
                                                                                                                 //EDITME add undervoltage code
      }
      data_struct.Timestamp = StartTimeslow;

      // for (int i = 0; i < Num_channels; i++) {
      //   Serial.print((data_struct.V[i] / 1000000.00), 4);
      //   // Serial.print(V_in_UART[i]);
      //   Serial.print(',');
      // }
      // Serial.println(data_struct.Timestamp);
      // Serial.println(" ");
      Log_to_file();
    }
  }
}
