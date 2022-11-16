#include <SPI.h>
#include "SdFat.h"
#include "sdios.h"
#include "FreeStack.h"
#include "RTClib.h"
#include "SerialTransfer.h"

SerialTransfer CVM;
SerialTransfer Interface;
SerialTransfer LoadController;
SerialTransfer SafetySystem;
SerialTransfer ProcessController;

const uint8_t SD_CS_PIN = 5; //EDITME check CS pin
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, DEDICATED_SPI, SPI_CLOCK)
#define SPI_CLOCK SD_SCK_MHZ(50)
SdFat32 SD;

char Filename[27];  //Char array to determine filename

struct Fault_message {
  uint8_t Fault_code = 0;
  uint8_t Fault_detail = 0;
};

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
