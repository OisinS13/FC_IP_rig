#include "SerialTransfer.h"

SerialTransfer ProcessController;

#define H2_VLV_pin 48
#define N2_VLV_pin 47

const int H2Sense_pins[10] = { A0, A1, A2, A3, A4, A5, A6, A7, A8, A9 };

uint16_t H2Sense_readings[10];
uint16_t H2Sensor_flags = 0;              //Flags to indicate if sensor is found. 1=present, 0=not found
uint16_t H2Sense_minor_threshold;         //EDITME add minor threshold
uint16_t H2Sense_major_threshold;         //EDITME add Major threshold
uint16_t H2fault_sent = 0;                //0=no fault sent, 1=minor fault sent, 2=major fault sent
uint32_t H2fault_last_sent = 0;           //Timestamp
uint16_t H2_minor_flags = 0;              // Flags to indicate minor leak detected
uint16_t H2_major_flags = 0;              // Flags to indicate majpr leak detected
uint16_t H2fault_update_interval = 5000;  //Time in mS for safety system to repeat messages

#define Interlock_pin 2
bool Interlock_signal = 0;

struct Fault_message {
  uint8_t Fault_code = 0;
  uint8_t Fault_detail = 0;
};

void setup() {
  Serial.begin(115200);  //Connect to computer over USB, opional
  delay(100);            //Give USB time to connect EDITME to shortest reliable time
  if (Serial) {
    Serial.println("Safety system connected");
  } else {
    Serial.end();
  }

  Serial1.begin(115200);  //Connect to SFTY UART
  while (!Serial1) {
    ;  // wait for serial port to connect to SFTY
  }
  ProcessController.begin(Serial1);
  if (Serial) {
    Serial.println("Process Controller connected");
  }

  pinMode(H2_VLV_pin, OUTPUT);
  digitalWrite(H2_VLV_pin, LOW);
  pinMode(N2_VLV_pin, OUTPUT);
  digitalWrite(N2_VLV_pin, LOW);
  pinMode(Interlock_pin, INPUT);
}

void loop() {

  for (int i = 0; i < 10; i++) {
    H2Sense_readings[i] = analogRead(H2Sense_pins[i]);
    if (H2Sense_readings[i] > H2Sense_major_threshold) {
      H2_major_flags |= (1 << i);
      H2_minor_flags &= ~(1 << i);
    } else if (H2Sense_readings[i] > H2Sense_minor_threshold) {
      H2_minor_flags |= (1 << i);
      H2_major_flags &= ~(1 << i);
    } else {
      H2_major_flags &= ~(1 << i);
      H2_minor_flags &= ~(1 << i);
    }
  }

  if (H2fault_sent == 2) {
    if (!H2_major_flags) {
      FaultSend(ProcessController, 'H', 0x80, 0);  //Clear major leak message
      if (H2_minor_flags) {
        uint8_t a = (8 + (H2_minor_flags >> 8));
        uint8_t b = H2_minor_flags & 0xFF;
        FaultSend(ProcessController, 'h', a, b);  //Send minor leak message with Sensor ID's
        H2fault_sent = 1;
        H2fault_last_sent = millis();
      } else {
        FaultSend(ProcessController, 'h', 0x80, 0);  //Clear minor leak message
        H2fault_sent = 0;
      }
    } else if (millis() > (H2fault_last_sent + H2fault_update_interval)) {
      uint8_t a = (8 + (H2_major_flags >> 8));
      uint8_t b = H2_major_flags & 0xFF;
      FaultSend(ProcessController, 'H', a, b);  //Send major leak message with Sensor ID's
      H2fault_sent = 2;
      H2fault_last_sent = millis();
    }
  } else if (H2fault_sent == 1) {
    if (H2_major_flags) {
      uint8_t a = (8 + (H2_major_flags >> 8));
      uint8_t b = H2_major_flags & 0xFF;
      FaultSend(ProcessController, 'H', a, b);  //Send major leak message with Sensor ID's
      H2fault_sent = 2;
      H2fault_last_sent = millis();
    } else if (!H2_minor_flags) {
      FaultSend(ProcessController, 'h', 0x80, 0);  //Clear minor leak message
      H2fault_sent = 0;
    } else if (millis() > (H2fault_last_sent + H2fault_update_interval)) {
      uint8_t a = (8 + (H2_minor_flags >> 8));
      uint8_t b = H2_minor_flags & 0xFF;
      FaultSend(ProcessController, 'h', a, b);  //Send major leak message with Sensor ID's
      H2fault_sent = 1;
      H2fault_last_sent = millis();
    }
  } else {
    if (H2_major_flags) {
      uint8_t a = (8 + (H2_major_flags >> 8));
      uint8_t b = H2_major_flags & 0xFF;
      FaultSend(ProcessController, 'H', a, b);  //Send major leak message with Sensor ID's
      H2fault_sent = 2;
      H2fault_last_sent = millis();
    } else if (H2_minor_flags) {
      uint8_t a = (8 + (H2_minor_flags >> 8));
      uint8_t b = H2_minor_flags & 0xFF;
      FaultSend(ProcessController, 'h', a, b);  //Send minor leak message with Sensor ID's
      H2fault_sent = 1;
      H2fault_last_sent = millis();
    }
  }

  if (!Interlock_signal) {
    Interlock_signal = digitalRead(Interlock_pin);
    if (Interlock_signal) {
      FaultSend(ProcessController, 'S', 0, 0);
    }
  } else if (Interlock_signal) {
    Interlock_signal = digitalRead(Interlock_pin);
    if (!Interlock_signal) {
      FaultSend(ProcessController, 'S', 1, 0);
    }
  }

  if (H2_major_flags) {
    digitalWrite(H2_VLV_pin, LOW);
  } else if (Interlock_signal) {
    digitalWrite(H2_VLV_pin, LOW);
  }

  //EDITME add code to recieve all clear message and reset H2 and N2 valves
}

void FaultSend(SerialTransfer& stf, char ID, uint8_t FaultCode, uint8_t FaultDetail) {
  struct Fault_message Fault;
  Fault.Fault_code = FaultCode;
  Fault.Fault_detail = FaultDetail;
  stf.txObj(Fault);
  stf.sendData(2, ID);
}
