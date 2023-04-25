void CTHD_HTR_TC_setup() {
  pinMode(CTHD_HTR_PIN, OUTPUT);
  for (int i = 0; i < Number_of_CTHD_HTR_SFTY_TCs; i++) {
    pinMode(CTHD_HTR_SFTY_DRDY[i], INPUT);       //Set the TC amp DRDY pins to inputs
    pinMode(CTHD_HTR_SFTY_FLT[i], INPUT);        //Set the TC amp FLT pins to inputs
    if (!HTR_SFTY_TC[i].begin()) {               //Initialise thermocouple amps over SPI
      FaultSend(Interface, 'f', (0x14 + i), 0);  //If they don't initialise, send out fault code over UART
      FaultSend(DataLogger, 'f', (0x14 + i), 0);
      if (Serial) {
        Serial.print("Could not initialize cathode heater safety thermocouple ");
        Serial.println(i);
      }
    } else {
      HTR_SFTY_TC[i].setThermocoupleType(MAX31856_TCTYPE_K);                                              //Set the TC type to type k
      HTR_SFTY_TC[i].setConversionMode(MAX31856_CONTINUOUS);                                              //Set the amplifiers to be continually reading
      HTR_SFTY_TC[i].setTempFaultThreshholds(Settings.TC_low_temp_thresh, Settings.TC_high_temp_thresh);  //Set the TC amp internal low and high temperature thresholds to trigger faults automatically
      HTR_SFTY_TC_fault[i] = HTR_SFTY_TC[i].readFault();                                                  //Read any stored faults on startup
      if (HTR_SFTY_TC_fault[i]) {
        FaultSend(Interface, 'f', (0x11 + i), HTR_SFTY_TC_fault[i]);  //If fault detected, send out over UART
        FaultSend(DataLogger, 'f', (0x11 + i), HTR_SFTY_TC_fault[i]);
        if (Serial) {
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
  }
}

void HIH6120_setup() {
  for (int i = 0; i < 8; i++) {
    Wire.beginTransmission(Settings.HIH6120_address[i]);  //Send measurement request to HIH6120 address
    int error = Wire.endTransmission();                   //stop transmitting
    if (error == 0) {                                     //No error, therefore I2C device successfully acknowledged measurement request
      Settings.HIH6120_flags |= 1 << i;                   //Set flag indicating this address value can be read from
    } else {                                              // I2C device did not successfully acknowledge the request
      if (Serial) {
        Serial.print("Could not find T+H sensor ");
        Serial.print(i);
        Serial.print(" at address 0x");
        Serial.println(Settings.HIH6120_address[i], HEX);
      }
      FaultSend(Interface, 'f', 0x31, Settings.HIH6120_address[i]);  //Fault for can't be read
      FaultSend(DataLogger, 'f', 0x31, Settings.HIH6120_address[i]);
    }
  }

  delay(Settings.HIH_interval);  //Give HIH6120's time to convert readings
  HIH6120_read_all();
}

void HIH6120_read_all() {
  for (int i = 0; i < 8; i++) {
    if (Settings.HIH6120_flags & (1 << i)) {             //Only interact with HIH61xx's that have been detected
      Wire.requestFrom(Settings.HIH6120_address[i], 4);  //Read data from I2C address, 4 bytes for RH and T
      if (Wire.available()) {
        uint8_t a = Wire.read();  //Store data in seperate temporary bytes
        uint8_t b = Wire.read();
        uint8_t c = Wire.read();
        uint8_t d = Wire.read();

        if (a & 0x80) {  //Check first bit to see if HIH61xx is in command mode
          if (Serial) {
            Serial.print("T+H sensor ");
            Serial.print(i);
            Serial.print(" at address 0x");
            Serial.print(Settings.HIH6120_address[i], HEX);
            Serial.println(" in command mode");
          }
          FaultSend(Interface, 'f', 0x31, Settings.HIH6120_address[i] + (1 << 6));  //if in command mode
          FaultSend(DataLogger, 'f', 0x31, Settings.HIH6120_address[i] + (1 << 6));

        } else if (a & 0x40) {           //Check second bit to see if data is "stale"
          HIH6120_data_flags |= 1 << i;  //Set data flag to 1 if stale
        } else {
          HIH6120_data_flags &= ~1 << i;          //set data flag to 0 if normal data
          HIH_RH_raw[i] = ((a & 0x3f) << 8) | b;  //Compile bytes into relevant data
          HIH_T_raw[i] = (c << 6) | (d >> 2);
          Outgoing_data.HIH_RH[i] = (HIH_RH_raw[i] / 163.82);  //Convert data to SI units as floats
          Outgoing_data.HIH_T[i] = ((HIH_T_raw[i] / 16382.0) * 165.0) - 40.0;
        }

      } else {
        if (Serial) {
          Serial.print("Could not find T+H sensor ");
          Serial.print(i);
          Serial.print(" at address 0x");
          Serial.println(Settings.HIH6120_address[i], HEX);
        }
        FaultSend(Interface, 'f', 0x31, Settings.HIH6120_address[i]);  //Fault for can't be read
        FaultSend(DataLogger, 'f', 0x31, Settings.HIH6120_address[i]);
      }
      Wire.beginTransmission(Settings.HIH6120_address[i]);  //Send measurement request to HIH6120 address for next cycle
      int error = Wire.endTransmission();                   //stop transmitting
      if (error != 0) {
        if (Serial) {  // I2C device did not successfully acknowledge the request
          Serial.print("Could not find T+H sensor ");
          Serial.print(i);
          Serial.print(" at address 0x");
          Serial.println(Settings.HIH6120_address[i], HEX);
        }
        FaultSend(Interface, 'f', 0x31, Settings.HIH6120_address[i]);  //Fault for can't be read
        FaultSend(DataLogger, 'f', 0x31, Settings.HIH6120_address[i]);
      }
    }
  }
  HIH_last_read_time = millis();
  //EDITME add stale data batch faultcode
  if (HIH6120_data_flags) {
    if (Serial) {  // I2C device did not successfully acknowledge the request
      Serial.print("TT+H stale data on sensors");
      Serial.println(HIH6120_data_flags, BIN);
    }
    FaultSend(Interface, 'f', 0x32, HIH6120_data_flags);  //Fault for stale data
    FaultSend(DataLogger, 'f', 0x32, HIH6120_data_flags);
  }
}

void FaultSend(SerialTransfer& stf, char ID, uint8_t FaultCode, uint8_t FaultDetail) {
  struct Fault_message Fault;
  Fault.Fault_code = FaultCode;
  Fault.Fault_detail = FaultDetail;
  stf.txObj(Fault);
  stf.sendData(2, ID);
}

void Read_CTHD_SFTY_TC() {
  for (int i = 0; i < 2; i++) {
    if (digitalRead(CTHD_HTR_SFTY_DRDY[i])) {                                            //Check if a reading from the Cathode heater safety thermocouple is ready
      Outgoing_data.CTHD_HTR_SFTY_TC[i] = HTR_SFTY_TC[i].readThermocoupleTemperature();  //Read Cathode heater safety thermocouple
    }
  }  //EDITME add else statement
}

void Cathode_heater_control() {
  if (*Cathode_inlet_T < (Settings.Cathode_target_T - Settings.Cathode_T_Hysteresis)) {  //Set Cathode heater on/off based on hysteretic control //EDITME Add heater outlet safety limit to protect sensors?
    Outgoing_data.CTHD_HTR = 1;
  } else if (*Cathode_inlet_T > (Settings.Cathode_target_T + Settings.Cathode_T_Hysteresis)) {
    Outgoing_data.CTHD_HTR = 0;
  }
  for (int i = 0; i < 2; i++) {
    if (Outgoing_data.CTHD_HTR_SFTY_TC[i] > Settings.TC_high_temp_thresh) {
      Outgoing_data.CTHD_HTR = 0;
      FaultSend(Interface, 'f', (0x1A + i), 0);  //if reading is outside nominal, send out fault code
      FaultSend(DataLogger, 'f', (0x1A + i), 0);

    } else if (Outgoing_data.CTHD_HTR_SFTY_TC[i] > Settings.TC_high_temp_thresh - Settings.TC_high_temp_thresh_warning_buffer) {
      FaultSend(Interface, 'f', (0x17 + i), 0);  //if reading is outside nominal, send out fault code
      FaultSend(DataLogger, 'f', (0x17 + i), 0);
    }
  }
  digitalWrite(CTHD_HTR_PIN, Outgoing_data.CTHD_HTR);
}

void Coolant_T_control() {
  if (*Coolant_inlet_T < (Settings.Coolant_T_target - Settings.Coolant_T_Hysteresis)) {  //Set fan speed based on hysteretic control //EDITME change to PID if time to tune?
    Outgoing_data.Fan_setting = 4095;
  } else if (*Coolant_inlet_T > (Settings.Coolant_T_target + Settings.Coolant_T_Hysteresis)) {
    Outgoing_data.Fan_setting = 3;
  }
  for (int i = 0; i < 3; i++) {  //EDITME add time limit to prevent errors being produced every loop
    if (Outgoing_data.Coolant_T_readings[i] > Settings.Coolant_high_temp_thresh) {
      Outgoing_data.Fan_setting = 4095;
      FaultSend(Interface, 'T', (11 + i), 3);  //Temperature 11,12,13 is over and requires immediate action
      FaultSend(DataLogger, 'T', (11 + i), 3);

    } else if (Outgoing_data.Coolant_T_readings[i] > Settings.Coolant_high_temp_thresh - Settings.Coolant_high_temp_warning_buffer) {
      FaultSend(Interface, 'T', (11 + i), 1);  //Temperature 11,12,13 is approaching overtemp limit
      FaultSend(DataLogger, 'T', (11 + i), 1);
    }
  }
  for (int i = 0; i < 6; i++) {
    pwm.setPWM(CLNT_FN_PWMS[i], 0, Outgoing_data.Fan_setting);
  }
}