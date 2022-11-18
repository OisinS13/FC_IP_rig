bool Create_logfile(DateTime Log_time, char *Filename_array) {
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
    filename_string += ".txt";
  filename_string.toCharArray(Filename_array, 26);
  if (USB_flag) {
  Serial.print("Filename generated: ");
  Serial.println(Filename_array);
  }

  if (!logfile.open(Filename_array, O_RDWR | O_CREAT | O_TRUNC)) {
    FaultSend(Interface, 'f', 0x72, 0); //Send fault for SD create logfile fault
    if (USB_flag) {
    Serial.println("File open failed");
    }
    return 0;
  } else {
    return 1;
  }
}

void FaultSend(SerialTransfer &stf, char ID, uint8_t FaultCode, uint8_t FaultDetail){
      struct Fault_message Fault;
      Fault.Fault_code = FaultCode;
      Fault.Fault_detail = FaultDetail;
      stf.txObj(Fault);
      stf.sendData(2,ID);
}