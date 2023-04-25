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
  //Serial.print("Data recieved: ");
  //Serial.print(result,HEX);
  //Serial.print("   ");
  //Serial.println(result,BIN);
  return result;
}

void ErrorSend(SerialTransfer &stf, ERROR Error_struct_to_send) {
  uint16_t n_byt = 0;

  // Stuff buffer with struct
  n_byt = stf.txObj(Error_struct_to_send, n_byt);

  // Send buffer
  stf.sendData(n_byt);
}

bool ID_check(uint16_t ID, const uint8_t Chip_select[Num_channels / 4]) {
  bool ID_flag = 1;
  uint8_t ADC_error_ID=0;
  for (int i = 0; i < sizeof(CS_pins); i++) {
    if (ID != Read_register(0, Chip_select[i])) {  //ID address of chip is stored at register 0
      Serial.print("ADC chip ");
      Serial.print(i);
      Serial.println(" ID check error");
      ID_flag = 0;
      ADC_error_ID |= 1 << i;
    }
  }
  if (ADC_error_ID) {  //If a bit in the ADC ID byte has been set, there's an error, so send it out
  FaultSend(DataLogger, 'f', 0x71, ADC_error_ID); //Send fault for ADC ID check fault
  }
  return ID_flag;
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
  filename_string.toCharArray(Filename_array, 26);
  if (USB_flag) {
  Serial.print("Filename generated: ");
  Serial.println(Filename_array);
  }

  if (!logfile.open(Filename_array, O_RDWR | O_CREAT | O_TRUNC)) {
    FaultSend(DataLogger, 'f', 0x62, 0); //Send fault for SD create logfile fault
    if (USB_flag) {
    Serial.println("File open failed");
    }
    return 0;
  } else {
    return 1;
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
            V_in[i][j][(l * 4) + 3] = SPI1.transfer16(0x00);
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
FaultSend(DataLogger, 'f', 0x04, 0); //Send fault code for buffer overflow fault

      // Serial.println("Data buffer overflow!");
      // delay(10);
    }
  }
}

void FaultSend(SerialTransfer &stf, char ID, uint8_t FaultCode, uint8_t FaultDetail){
      struct Fault_message Fault;
      Fault.Fault_code = FaultCode;
      Fault.Fault_detail = FaultDetail;
      stf.txObj(Fault);
      stf.sendData(2,ID);
}