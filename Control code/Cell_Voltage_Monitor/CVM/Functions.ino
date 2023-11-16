
int Read_register(uint8_t Address, uint8_t Chip_select) {
  byte inByte = 0;          // incoming byte from the SPI
  unsigned int result = 0;  // result to return
  byte frame = Address << 2;
  // Serial.print("frame to send: ");
  // Serial.println(frame, BIN);
  digitalWrite(Chip_select, LOW);
  SPI1.transfer(frame);
  result = SPI1.transfer16(0x0000);
  digitalWrite(Chip_select, HIGH);
  // Serial.print("Data recieved: ");
  // Serial.print(result,HEX);
  // Serial.print("   ");
  // Serial.println(result,BIN);
  return result;
}

bool ID_check(uint16_t ID, const uint8_t Chip_select[6]) {
  bool ID_flag = 1;
  uint8_t ADC_error_ID = 0;
  for (int i = 0; i < 6; i++) {
    if (ID != Read_register(0, Chip_select[i])) {  //ID address of chip is stored at register 0
      Serial.print("ADC chip ");
      Serial.print(i);
      Serial.println(" ID check error");
      ID_flag = 0;
      ADC_error_ID |= 1 << i;
    }
  }
  // if (ADC_error_ID) {  //If a bit in the ADC ID byte has been set, there's an error, so send it out
  // FaultSend(DataLogger, 'f', 0x71, ADC_error_ID); //Send fault for ADC ID check fault
  // }
  return ID_flag;
}

bool USB_setup(uint32_t Baud_rate, uint16_t Wait_time_mS) {
  Serial.begin(Baud_rate);
  delay(Wait_time_mS);  //Give USB time to connect EDITME to shortest reliable time
  if (Serial) {
    // USB_flag = 1;  //Used so that when connected via USB, verbose status and error messages can be sent, but will still run if not connected
    Serial.println("Cell Voltage monitor connected");
    Serial.println("V.0.1 Oisin Shaw Sep 2023");

    return 1;
  } else {
    Serial.end();
    return 0;
  }
}

void RTC_setup(uint8_t SDA, uint8_t SCL) {

  Wire.setSDA(SDA);  //Set pins for RTC I2C
  Wire.setSCL(SCL);

  if (!rtc.begin()) {
    if (USB_flag) {
      Serial.println("Couldn't find RTC");
      Serial.flush();
    }
  } else {
    if (USB_flag) {
      Serial.println("RTC connected");
      Serial.flush();
    }
    RTC_flag = 1;
  }
}

void Initialise_SD(uint8_t SCK, uint8_t COPI, uint8_t CIPO, uint8_t CS) {
  SPI.setSCK(SCK);  //Set the correct SPI pins for the SD SPI
  SPI.setTX(COPI);
  SPI.setRX(CIPO);
  SPI.setCS(CS);
  if (USB_flag) {
    Serial.print("Initializing SD card...");
  }

  if (!SD.begin(SD_CONFIG)) {
    if (USB_flag) {
      Serial.println("initialization failed!");
    }
    return;
  } else {
    SD_boot_flag = 1;
    if (USB_flag) {
      Serial.println("initialization done.");
    }
  }
}

bool Create_logfile(DateTime Log_time, char *Filename_array, bool fast) {
  String filename_string = String(Log_time.year(), DEC) + '_';
  if (Log_time.month() < 10) {
    filename_string += '0' + String(Log_time.month(), DEC) + "_";
  } else {
    filename_string += String(Log_time.month(), DEC) + "_";
  }
  if (Log_time.day() < 10) {
    filename_string += '0' + String(Log_time.day(), DEC) + "_";
  } else {
    filename_string += String(Log_time.day(), DEC) + "_";
  }
  if (Log_time.hour() < 10) {
    filename_string += '0' + String(Log_time.hour(), DEC) + "_";
  } else {
    filename_string += String(Log_time.hour(), DEC) + "_";
  }
  if (Log_time.minute() < 10) {
    filename_string += '0' + String(Log_time.minute(), DEC) + "_";
  } else {
    filename_string += String(Log_time.minute(), DEC) + "_";
  }
  if (Log_time.second() < 10) {
    filename_string += '0' + String(Log_time.second(), DEC);
  } else {
    filename_string += String(Log_time.second(), DEC);
  }
  if (!fast) {
    filename_string += "_log.txt";
  } else {
    filename_string += ".txt";
  }
  filename_string.toCharArray(Filename_array, 28);
  if (USB_flag) {
    Serial.print("Filename generated: ");
    Serial.println(Filename_array);
  }

  if (!logfile.open(Filename_array, O_RDWR | O_CREAT | O_TRUNC)) {
    // FaultSend(DataLogger, 'f', 0x62, 0);  //Send fault for SD create logfile fault
    if (USB_flag) {
      Serial.println("File open failed");
    }
    return 0;
  } else {
    return 1;
  }
}

void Log_to_file() {
  char Data_to_file[210] = "\n";  //Initialise char array, beginning with a new line character
  while (!logfile.open(Filenameslow, O_RDWR | O_APPEND)) {
    // FaultSend(DataLogger, 'f', 0x62, 0);  //Send fault for SD create/open logfile fault
    if (USB_flag) {
      Serial.println("Error opening logfile");
    }
  }
  int j = 1;  //starts at 1 to account for newline chracter
  for (int i = 0; i < Num_channels; i++) {
    j += sprintf(&Data_to_file[j], "%lu,", data_struct.V[i]);  //Append a reading, and then a delimeter
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

  if (USB_flag) {
    Serial.print(Data_to_file);
  }
}

void Fill_data_buf() {
  for (int i = 0; i < Num_buffs; i++) {
    if (!V_in_flag[i]) {
      for (int j = 0; j < Num_samples;) {
        if (micros() - StartTimefast > intervalfast) {
          StartTimefast = micros();
          for (int l = 0; l < (Num_channels / 4); l++) {
            digitalWrite(CS_pins[l], LOW);
            SPI1.transfer(0x5);  //Burst read of non moving average data
            V_in[i][j][l * 4] = SPI1.transfer16(0x00);
            V_in[i][j][(l * 4) + 1] = SPI1.transfer16(0x00);
            V_in[i][j][(l * 4) + 2] = SPI1.transfer16(0x00);
            V_in[i][j][(l * 4) + 3] = SPI1.transfer16(0x00)-V_in[i][j][(l * 4) + 2];
            V_in[i][j][(l * 4) + 2] -= V_in[i][j][(l * 4) + 1];
            V_in[i][j][(l * 4) + 1] -= V_in[i][j][l * 4];

            digitalWrite(CS_pins[l], HIGH);
          }
          V_in[i][j][Num_channels] = (StartTimefast >> 16);
          V_in[i][j][Num_channels + 1] = (StartTimefast);
          // V_in[i][j][Num_channels + 2] = 0xFFFF;  //This is now done once for all buffers during setup for speed/efficiency
          j++;
        }
      }
      V_in_flag[i] = true;
    } else {
      // EDITME Put in error throwing code here
      // FaultSend(DataLogger, 'f', 0x04, 0);  //Send fault code for buffer overflow fault
  if (USB_flag) {
      Serial.println("Data buffer overflow!");
  }
    }
  }
}

void Write_buf_to_SD() {
  // StartTime1 = micros(); //For checking write speed if necessary
  logfile.write(&V_in[V_write_counter][0][0], ((Num_channels + 3) * 2) * Num_samples);
  logfile.sync();
  V_in_flag[V_write_counter] = false;
  V_write_counter++;
  if (V_write_counter > Num_buffs - 1) {
    V_write_counter = 0;
  }
  // StartTime1 = micros() - StartTime1; //For checking write speed if necessary
  // Serial.println(StartTime1);
}