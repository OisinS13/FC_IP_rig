bool SystemCheck() {

  for (int i = 0; i < 3; i++) {
    if (!HTR_SFTY_TC[i].begin()) {  //Initialise thermocouple amps over SPI //EDITME chekc if you can re-initialise like this, otherwise, find workaround
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

  for (int i = 0; i < 3; i++) {
    if (digitalRead(CTHD_HTR_SFTY_DRDY[i])) {                                       //Check if a reading from the Cathode heater safety thermocouple is ready
      CTHD_HTR_SFTY_TC_readings[i] = HTR_SFTY_TC[i].readThermocoupleTemperature();  //Read Cathode heater safety thermocouple
    }
  }  //EDITME add else statement

  if (analogRead(CTHD_FLW_SIG_PIN) < Cathode_flow_sensor_minimum_threshold) {  //Flow sensor should output a minimum of 4mA, which should be read as 1V. Threshold of 190=0.928V
    if (Serial) {
      Serial.println("Could not read Cathode flow sensor");
    }
    //EDITME write error throwing code
  }

  
}

void FaultSend(SerialTransfer &stf, char ID, uint8_t FaultCode, uint8_t FaultDetail){
      struct Fault_message Fault;
      Fault.Fault_code = FaultCode;
      Fault.Fault_detail = FaultDetail;
      stf.txObj(Fault);
      stf.sendData(2,ID);
}



