// read serial
void msgRead(SerialTransfer &stf) {
  // check messages from interface
  while (stf.available()) {

    // read packet ID
    uint8_t ID = stf.currentPacketID();
    uint16_t n_byt = 0;


    // if the received package is a simple message
    if (ID == 0x67) {
      n_byt = stf.rxObj(rx_msg, n_byt);
      msgParse(stf, ID, rx_msg.cmd, rx_msg.flag, rx_msg.value);
      Serial.println("Ping response received");
    }



    // if the received package is a transfer of data 
    if (ID == 0x68) {
      n_byt = stf.rxObj(data, n_byt);
      Serial.println(data.time_stamp);
    }

    // clead ID
    ID = 0;


  }
}


void msgWrite(SerialTransfer &stf, byte ID, char cmd, bool flag, uint32_t value) {
  uint16_t n_byt = 0;

  // Set struct values
  tx_msg.cmd   = cmd;
  tx_msg.flag  = flag;
  tx_msg.value = value;

  // Stuff buffer with struct
  n_byt = stf.txObj(tx_msg, n_byt);

  // Send buffer
  stf.sendData(n_byt, ID);
}



void msgParse(SerialTransfer &stf, byte ID, char cmd, bool flag, uint32_t value) {

  // Handle packets of ID = 0x67
  if (ID == 0x67) {
    // listen for ping
    if (cmd == 'P') {
      
    }
  }
}
