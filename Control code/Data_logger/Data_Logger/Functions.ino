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
    FaultSend(Interface, 'f', 0x72, 0);  //Send fault for SD create logfile fault
    if (USB_flag) {
      Serial.println("File open failed");
    }
    return 0;
  } else {
    return 1;
  }
}

void FaultSend(SerialTransfer &stf, char ID, uint8_t FaultCode, uint8_t FaultDetail) {
  struct Fault_message Fault;
  Fault.Fault_code = FaultCode;
  Fault.Fault_detail = FaultDetail;
  stf.txObj(Fault);
  stf.sendData(2, ID);
}

void Receive_data_frame(uint8_t Source_flag_position) {
  for (int i = 0; i < BufferSize; i++) {
    if (!(*DataPtr[i]).Data_source_flags & Source_flag_position) {  //Check if the structure in position 'i' of the queue already has this data
                                                                    // DataFrameStruct Temporary=DataQueue[i]; //Create a temporary structure to work on, and non-destructively read the 'i'th member onto it
      CVM.rxObj((*DataPtr[i]).CVM_data);                            //Put the incoming packet into the 'i' position of the queue
      (*DataPtr[i]).Data_source_flags |= Source_flag_position;      //Set the data read flag bit for the CVM in the relevant member of the queue
      if (i > 2) {
        (*DataPtr[i - 2]).Data_source_flags |= 0x10;  //Set the "To-be-saved" flag of the data frame 2 up in the queue
                                                      //EDITME add stale data warning?
      }
      i = BufferSize;  //exit for loop once data has been written once
    } else {
      //EDITME throw queue overflow warning?
    }
  }
}

